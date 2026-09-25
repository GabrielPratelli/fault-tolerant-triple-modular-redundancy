#include <Arduino.h>
#include <string.h>
#include <avr/wdt.h>          /* hardware watchdog (independent reset)   */

/* ---- pins / timing ---- */
#define LINK_BAUD         115200 /* hardware UART (pins 0/1), full-duplex */
#define ACTUATOR_PIN      7      /* problem LED (1 = fault/lock, 0 = healthy) */
#define RESET_ESP_PIN     10
#define RESTART_PIN       2

#define HEARTBEAT_PERIOD_MS    1000  /* how often we tell the ESP we're alive */
#define ESP_SILENT_TIMEOUT_MS  3000  /* ESP silence -> safe state             */
#define FRAME_TIMEOUT_MS       100   /* mid-frame stall -> reset the parser   */
#define ESP_BOOT_GRACE_MS      8000  /* after resetting the ESP, wait this long before declaring it silent again */

/* ---- fault injection (enable to exercise the all-three-differ case) ---- */
/* #define INJECT_C */
#define INJECT_EVERY  5
#define INJECT_BIT    0x00010000UL    /* flip bit 16 (A/B on the ESP32)  */
#define INJECT_C_BIT  0x01000000UL    /* flip bit 24 — MUST differ from INJECT_BIT
                                         so A and C do NOT coincide when both are
                                         corrupted (all-three-differ case) */

/* ============================================================================
 *  APPLICATION SPEC — MUST match the ESP32 exactly
 * ============================================================================
 *  f(samples[8]) = sum(samples[i]^2)  (mod 2^32)
 *  Channel C (this file): 256-entry lookup table — no multiply in the hot
 *  path, so a fault class that hits the multiplier cannot hit this channel.
 */
#define N_SAMPLES 8
static void make_samples(uint8_t *out, uint16_t event_id){
    //generating samples from the same seed
    uint32_t s = 0x12345678UL ^ ((uint32_t)event_id << 16);

    for (int i = 0; i < N_SAMPLES; i++) {
        s = s * 1664525UL + 1013904223UL;
        out[i] = (uint8_t)(s >> 24);
    }
}

static uint32_t g_squares[256];

static void build_lut(void){
    for (int i = 0; i < 256; i++) {
        g_squares[i] = (uint32_t)i * (uint32_t)i;
    }
}

/* Channel C — table lookup, no multiplication in the hot path. */
static uint32_t compute_action_C(const uint8_t *s){
    uint32_t acc = 0;

    for (int i = 0; i < N_SAMPLES; i++) {
        acc += g_squares[s[i]];
    }
    return acc;
}

/* ============================================================================
 *  UART PROTOCOL — identical to the ESP32 side
 * ============================================================================
 *  Frame:  0xAA 0x55 | TYPE | SEQ | LEN | PAYLOAD | CRC16(LE)
 *  CRC16-CCITT (poly 0x1021, init 0xFFFF) over TYPE..PAYLOAD.
 *  This is shared infrastructure, so it MUST match the ESP32 bit-for-bit —
 *  diversity applies only to the voted f(), never to the protocol.
 */
#define FRAME_SYNC0       0xAA
#define FRAME_SYNC1       0x55
#define FRAME_MAX_PAYLOAD 64

enum {
    MSG_RESULT     = 0x01,   /* ESP -> Arduino: {event_id(2), a(4), b(4)} */
    MSG_HEARTBEAT  = 0x02,   /* both directions: {counter(4), flags(1)}   */
    MSG_ACK        = 0x03,   /* Arduino -> ESP: {event_id(2), verdict(1), decided(4)} */
    MSG_SAFE       = 0x04,   /* Arduino -> ESP: {event_id(2), reason(1)}  */
    MSG_LOCK       = 0x05,   /* both: enter system lock {reason(1)}       */
    MSG_UNLOCK     = 0x06,   /* both: button pressed, recover {}          */
    MSG_REQ_REPEAT = 0x07,   /* ESP -> Arduino: re-send cached verdict {event_id(2)} */
};

/* verdict byte in ACK (informational; the ESP only needs the value) */
enum {
    VERDICT_AGREE       = 0,  /* 3/3 agree                        */
    VERDICT_MAJORITY_A  = 1,  /* c == a, channel b was healed     */
    VERDICT_MAJORITY_B  = 2,  /* c == b, channel a was healed     */
};

/* reason byte in SAFE */
enum {
    REASON_COMMON_MODE  = 1,  /* a == b != c: correlated channels dissent from C */
    REASON_ALL_DIFFER   = 2,  /* a, b, c all differ: double fault               */
};

static uint8_t s_seq = 0;   /* TX sequence number (wraps at 256) */

// CRC16-CCITT step byte identical to the ESP32 implementation
static uint16_t crc16_update(uint16_t crc, uint8_t c){
    crc ^= (uint16_t)c << 8;
    for (int b = 0; b < 8; b++) {
        if (crc & 0x8000) {crc = (uint16_t)((crc << 1) ^ 0x1021);} //check if the MSB is one
        else {crc = (uint16_t)(crc << 1);} // if MSB = 0 we shif left crc
    }
    return crc;
}

/* Decode a 4-byte little-endian field. */
static uint32_t read_u32(const uint8_t *p){
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ---------------------------------------------------------------------------
 *  Link: the Uno's only hardware UART (pin 0 = RX, pin 1 = TX) carries the
 *  link to the ESP32.  There is no PC-facing serial on this board any more —
 *  the ESP32's USB/UART0 console is the single monitoring point.
 * ------------------------------------------------------------------------- */
Stream &link = Serial;

/* Wrap a payload into a synced, CRC-protected frame and send it. */
static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[2 + 3 + FRAME_MAX_PAYLOAD + 2];

    buf[0] = FRAME_SYNC0;
    buf[1] = FRAME_SYNC1;
    buf[2] = type;
    buf[3] = s_seq++;
    buf[4] = len;
    if (len > 0) {
        memcpy(&buf[5], payload, len);
    }

    uint16_t crc = 0xFFFF;
    for (int i = 2; i < 5 + len; i++) {
        crc = crc16_update(crc, buf[i]);
    }
    buf[5 + len]     = crc & 0xFF;
    buf[5 + len + 1] = crc >> 8;

    link.write(buf, 5 + len + 2);
}

/* ---- cached verdict: re-send it when the ESP retransmits a RESULT ---- */
static uint16_t s_last_event_id;
static bool     s_has_last_reply = false;
static uint8_t  s_last_reply_type;
static uint8_t  s_last_reply_payload[7];
static uint8_t  s_last_reply_len;

static void cache_reply(uint16_t event_id, uint8_t type, const uint8_t *payload, uint8_t len){
    s_last_event_id = event_id;
    s_last_reply_type = type;
    s_last_reply_len = len;
    memcpy(s_last_reply_payload, payload, len);
    s_has_last_reply = true;
}

static void resend_cached_reply(void){
    if (s_has_last_reply) {
        send_frame(s_last_reply_type, s_last_reply_payload, s_last_reply_len);
    }
}

static void send_ack(uint16_t event_id, uint8_t verdict, uint32_t decided)
{
    //Splicing the message in bytes for UART
    uint8_t p[7];
    p[0] = event_id & 0xFF;
    p[1] = (event_id >> 8) & 0xFF;
    p[2] = verdict;
    p[3] = decided & 0xFF;
    p[4] = (decided >> 8) & 0xFF;
    p[5] = (decided >> 16) & 0xFF;
    p[6] = (decided >> 24) & 0xFF;
    send_frame(MSG_ACK, p, 7);
    cache_reply(event_id, MSG_ACK, p, 7);
}

static void send_safe(uint16_t event_id, uint8_t reason)
{
    uint8_t p[3];
    p[0] = event_id & 0xFF;
    p[1] = (event_id >> 8) & 0xFF;
    p[2] = reason;
    send_frame(MSG_SAFE, p, 3);
    cache_reply(event_id, MSG_SAFE, p, 3);
}

static void send_heartbeat(uint32_t counter, bool locked)
{
    uint8_t p[5];
    p[0] = counter & 0xFF;
    p[1] = (counter >> 8) & 0xFF;
    p[2] = (counter >> 16) & 0xFF;
    p[3] = (counter >> 24) & 0xFF;
    p[4] = locked ? 0x01 : 0x00;   /* flags: bit0 = locked */
    send_frame(MSG_HEARTBEAT, p, 5);
}

/* Propagate the local system-lock decision to the ESP32. */
static void send_lock(uint8_t reason)
{
    uint8_t p[1];
    p[0] = reason;
    send_frame(MSG_LOCK, p, 1);
}

/* Propagate a button-press recovery to the ESP32. */
static void send_unlock(void)
{
    send_frame(MSG_UNLOCK, NULL, 0);
}

/* ============================================================================
 *  THE 5-CASE VOTE — the intellectual core
 * ============================================================================ */
static bool vote_5_cases(uint32_t a, uint32_t b, uint32_t c, uint32_t *decided, uint8_t *verdict, uint8_t *reason){
    if (a == b) {
        if (a == c) {
            *decided = a;
            *verdict = VERDICT_AGREE;    // 3/3 agrees
            return true;
        }
        *reason = REASON_COMMON_MODE;
        return false;
    }

    // a != b: the independent Arduino breaks the tie
    if (c == a) { *decided = a; *verdict = VERDICT_MAJORITY_A; return true; }
    if (c == b) { *decided = b; *verdict = VERDICT_MAJORITY_B; return true; }

    *reason = REASON_ALL_DIFFER;         // double fault 
    return false;
}

/* ============================================================================
 *  ACTIONS
 * ============================================================================ */
static volatile uint8_t s_fault_bucket = 0;
static volatile bool s_system_lock = false;
static volatile bool s_unlock_requested = false;   /* set by the restart ISR */
static uint32_t s_unlock_grace_until_ms = 0;       /* ignore stale peer-lock heartbeats until then */
static uint32_t s_last_lock_advertise_ms = 0;      /* rate-limit LOCK re-advertise while locked */
#define MAX_FAULT_BUCKET 10
#define BUCKET_RECOVERY 1
#define BUCKET_PENALTY 2
#define LOCK_REASON_BUCKET 1   /* local bucket reached its capacity */
#define LOCK_REASON_MIRROR 2   /* locked because the peer advertised lock in its heartbeat */
#define UNLOCK_GRACE_MS  3000  /* after unlocking, ignore stale peer-lock heartbeats for this long */

/* ACTUATOR_PIN drives a problem LED (1 = fault/lock, 0 = healthy). */

static void apply_action(uint32_t action){
    (void)action;   /* nothing to actuate here: the ESP applies the action */
    if(s_system_lock){
        return;   /* locked: never apply an action */
    }
    if(s_fault_bucket > 0){
        s_fault_bucket = s_fault_bucket - BUCKET_RECOVERY;
    }
    digitalWrite(ACTUATOR_PIN, LOW);   /* problem LED off: healthy */
}

static void system_unlock(void){
    s_fault_bucket = 0;
    s_system_lock = false;
    digitalWrite(ACTUATOR_PIN, LOW);   /* problem LED off */
    s_unlock_grace_until_ms = millis() + UNLOCK_GRACE_MS;  /* ignore stale peer-lock heartbeats */
}

static void safe_state(void){    
    if(!s_system_lock){
        s_fault_bucket = s_fault_bucket + BUCKET_PENALTY;

        if(s_fault_bucket >= MAX_FAULT_BUCKET){
            s_system_lock = true;
            digitalWrite(ACTUATOR_PIN, HIGH);   /* problem LED on */
            send_lock(LOCK_REASON_BUCKET);      /* make the ESP32 lock too */
        }
    }
}
/* ============================================================================
 *  INTERRUPT SERVICE ROUTINE (ISR)
 * ============================================================================ */
void isr_unlock_system() {
    if(!s_system_lock) return;   /* button only acts while the system is locked */
    s_unlock_requested = true;   /* defer the unlock + UART notify to loop() */
}

/* ============================================================================
 *  RECEIVE STATE MACHINE — rebuilds frames from the raw UART byte stream
 * ============================================================================ */
static uint8_t  s_rx_type;
static uint8_t  s_rx_len;
static uint8_t  s_rx_idx;
static uint8_t  s_rx_hdr[3];
static uint8_t  s_rx_payload[FRAME_MAX_PAYLOAD];
static uint8_t  s_rx_crc_bytes[2];
static uint16_t s_rx_running;

typedef enum { WAIT_SYNC0, WAIT_SYNC1, READ_HDR, READ_PAY, READ_CRC } rx_state_t;
static rx_state_t s_rx_state = WAIT_SYNC0;

static uint8_t  s_last_seq;      /* last SEQ seen from the ESP32 */
static bool     s_seq_valid = false;

/* Timestamps (millis) for the reciprocal liveness checks. */
static uint32_t s_last_esp_ms;   /* last valid frame from the ESP32   */
static uint32_t s_last_hb_ms;    /* last time we sent our heartbeat   */
static uint32_t s_last_byte_ms;  /* last byte received (frame stall)  */
static uint32_t g_hb_counter = 0;
static uint32_t s_esp_boot_deadline_ms = 0;  /* suppress ESP-silent reset until this time */

static void handle_result(const uint8_t *p);

/* Handle a fully received, CRC-valid frame. */
static void handle_rx_frame(uint8_t type, const uint8_t *p, uint8_t len){
    s_last_esp_ms = millis();    //updating ESP32 interaction timer

    if (type == MSG_RESULT && len == 10) {
        handle_result(p);
    }
    else if (type == MSG_LOCK && len == 1) {
        if (!s_system_lock) {
            s_system_lock = true;
            digitalWrite(ACTUATOR_PIN, HIGH);   /* problem LED on */
        }
    }
    else if (type == MSG_UNLOCK && len == 0) {
        bool was_locked = s_system_lock;
        system_unlock();
        if (was_locked) {
            send_unlock();   /* re-propagate so recovery wins any race */
        }
    }
    else if (type == MSG_REQ_REPEAT && len == 2) {
        /* The ESP asks us to re-send our cached verdict: its ACK/SAFE was
         * lost on the way back.  If we are locked we are not arbitrating,
         * so tell the ESP why right away instead. */
        uint16_t event_id = p[0] | ((uint16_t)p[1] << 8);
        if (s_system_lock) {
            send_lock(LOCK_REASON_BUCKET);
        }
        else if (s_has_last_reply && event_id == s_last_event_id) {
            resend_cached_reply();
        }
    }
    else if (type == MSG_HEARTBEAT && len == 5) {
        /* Mirror the peer's lock so a rebooted board converges.  After a
         * local unlock we briefly ignore stale peer-lock heartbeats. */
        if ((p[4] & 0x01) && !s_system_lock && (int32_t)(millis() - s_unlock_grace_until_ms) >= 0) {
            s_system_lock = true;
            digitalWrite(ACTUATOR_PIN, HIGH);   /* problem LED on */
            send_lock(LOCK_REASON_MIRROR);
        }
    }
}

static void handle_result(const uint8_t *p){
    /* A locked supervisor must not arbitrate: advertise the lock (rate-limited)
     * and drop the RESULT so the ESP does not apply the action. */
    if (s_system_lock) {
        if (millis() - s_last_lock_advertise_ms >= 1000) {
            s_last_lock_advertise_ms = millis();
            send_lock(LOCK_REASON_BUCKET);
        }
        return;
    }

    //piecing together the received bytes
    uint16_t event_id = p[0] | ((uint16_t)p[1] << 8);
    uint32_t a = read_u32(&p[2]);   
    uint32_t b = read_u32(&p[6]);   

    /* Retransmitted RESULT for the same cycle: re-send the cached verdict
     * instead of re-voting, so the fault bucket is penalised only once. */
    if (s_has_last_reply && event_id == s_last_event_id) {
        resend_cached_reply();
        return;
    }

    //recalculating the result
    uint8_t samples[N_SAMPLES];
    make_samples(samples, event_id);
    uint32_t c = compute_action_C(samples);

#ifdef INJECT_C
    if ((event_id % INJECT_EVERY) == 0) {
        c ^= INJECT_C_BIT;   /* different bit from the ESP32's INJECT_BIT */
    }
#endif

    uint32_t decided = 0;
    uint8_t verdict = 0, reason = 0;

    if (vote_5_cases(a, b, c, &decided, &verdict, &reason)) {
        apply_action(decided);
        send_ack(event_id, verdict, decided);
    } 
    else {
        safe_state();
        send_safe(event_id, reason);
    }
}

static void feed_rx_byte(uint8_t c){
    switch (s_rx_state) {

    case WAIT_SYNC0: // waiting for first sync byte
        if (c == FRAME_SYNC0) s_rx_state = WAIT_SYNC1;
        break;

    case WAIT_SYNC1: // waiting for second sync byte
        if (c == FRAME_SYNC1) {
            s_rx_state = READ_HDR;      //setting the next stage
            s_rx_idx = 0;
            s_rx_running = 0xFFFF;      //set the initial value of CRC
        } 
        else if (c == FRAME_SYNC0) {
            s_rx_state = WAIT_SYNC1;    // if the frame contains the first sync frame
        }                               // we stay on this state
        else {
            s_rx_state = WAIT_SYNC0;    //if everything fails we go back to the starting frame
        }
        break;

    case READ_HDR: //reading the header of the message
        s_rx_running = crc16_update(s_rx_running, c);   //calculating CRC for the header
        s_rx_hdr[s_rx_idx++] = c; //saving the header info

        if (s_rx_idx == 3) {
            s_rx_type = s_rx_hdr[0];
            s_rx_len  = s_rx_hdr[2];

            if (s_rx_len > sizeof(s_rx_payload)) { //length check
                s_rx_state = WAIT_SYNC0;
            } 
            else if (s_rx_len == 0) {
                s_rx_idx = 0;            //zero-length payload: skip READ_PAY
                s_rx_state = READ_CRC;
            }
            else {
                s_rx_idx = 0;
                s_rx_state = READ_PAY;  //if the informations are sensical we start reading the payload
            }
        }
        break;

    case READ_PAY:
        s_rx_running = crc16_update(s_rx_running, c);   //calculating CRC for payload
        s_rx_payload[s_rx_idx++] = c;   //saving payload bytes

        if (s_rx_idx == s_rx_len) {     //when we reach the final byte of the payload
            s_rx_idx = 0;               
            s_rx_state = READ_CRC;      //start reading CRC
        }
        break;

    case READ_CRC:
        s_rx_crc_bytes[s_rx_idx++] = c; //saving CRC bytes

        if (s_rx_idx == 2) {
            s_rx_state = WAIT_SYNC0;    //reset to the original state

            uint16_t crc = (uint16_t)s_rx_crc_bytes[0] | ((uint16_t)s_rx_crc_bytes[1] << 8);  //piecing together recived CRC bytes

            if (crc == s_rx_running) { // checking if received CRC = calculated CRC
                bool accept = true;
                if (s_seq_valid) {
                    uint8_t delta = (uint8_t)(s_rx_hdr[1] - s_last_seq);
                    if (delta == 0) {
                        accept = false;   // duplicate SEQ: drop
                    } else if (delta > 1) {
                        /* SEQ gap: one or more frames were lost */
                    }
                } else {
                    s_seq_valid = true;  //first valid frame: baseline
                }
                if (accept) {
                    s_last_seq = s_rx_hdr[1];
                    handle_rx_frame(s_rx_type, s_rx_payload, s_rx_len);
                }
            } 
            else {
                /* CRC mismatch: drop */
            }
        }
        break;
    }
}

/* ============================================================================
 *  SETUP / LOOP
 * ============================================================================ */
void setup(){
    Serial.begin(LINK_BAUD);     // hardware UART (pins 0/1) <-> ESP32

    pinMode(ACTUATOR_PIN, OUTPUT);  
    pinMode(RESET_ESP_PIN,OUTPUT);
    pinMode(RESTART_PIN, INPUT);    /* button network pulls N low at rest (R8), high on press */

    attachInterrupt(digitalPinToInterrupt(RESTART_PIN), isr_unlock_system, RISING); //active-high: edge on press (0 -> 1)

    digitalWrite(RESET_ESP_PIN,LOW);
    digitalWrite(ACTUATOR_PIN, LOW);   /* problem LED off at boot */

    build_lut();

    // Starting timers
    s_last_esp_ms  = millis(); //interaction with ESP32
    s_last_hb_ms   = millis(); //hearbeat sent
    s_last_byte_ms = millis(); //recived bytes

    wdt_enable(WDTO_2S); // enabling watch-dog-timer, triggering if not resetted every 2s
}

void loop(){
    /* Button pressed while locked: recover locally and tell the ESP32. */
    if (s_unlock_requested) {
        s_unlock_requested = false;
        system_unlock();
        send_unlock();
    }

    while (link.available()) {                            //while is active until there are bytes on the UART <-> ESP32
        s_last_byte_ms = millis();                        //update bytes timer
        feed_rx_byte((uint8_t)link.read());             //takes the oldest bytes out of the buffer and feeds it to the frame stat machine
    }

    /* 2) If a frame stalled mid-way, reset the parser. */
    if (s_rx_state != WAIT_SYNC0 && millis() - s_last_byte_ms > FRAME_TIMEOUT_MS) {
        s_rx_state = WAIT_SYNC0;
    }

    /* 3) ESP32 silent too long -> it is dead -> safe state.
     *    After a reset the ESP needs time to boot: defer the next silence
     *    check by ESP_BOOT_GRACE_MS so we don't reset it again mid-boot. */
    if (millis() - s_last_esp_ms > ESP_SILENT_TIMEOUT_MS &&
        (int32_t)(millis() - s_esp_boot_deadline_ms) >= 0) {
        //ESP32 reset sequence
        digitalWrite(RESET_ESP_PIN, HIGH);
        delay(50);
        digitalWrite(RESET_ESP_PIN, LOW);

        s_last_esp_ms = millis();
        s_esp_boot_deadline_ms = millis() + ESP_BOOT_GRACE_MS;
        safe_state();
    }

    /* 4) Send our own heartbeat so the ESP32 does not declare US silent
     *    (the ESP declares "supervisor silent" after 3 s of radio silence). */
    if (millis() - s_last_hb_ms >= HEARTBEAT_PERIOD_MS) {
        s_last_hb_ms = millis();
        send_heartbeat(g_hb_counter++, s_system_lock);
    }

    wdt_reset();   /* reached the end of loop(): still alive */
}
