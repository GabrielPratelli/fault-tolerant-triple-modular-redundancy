/* ---- standard / platform headers ---- */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"      /* GPIO config + ISR service            */
#include "driver/uart.h"      /* UART1 driver (link to the Arduino)   */
#include "esp_log.h"          /* ESP_LOGx macros                      */
#include "esp_task_wdt.h"     /* per-task watchdog (TWDT)             */
#include "esp_timer.h"        /* microsecond timestamps               */

/* ============================================================================
 *  HARDWARE MAP + TIMING
 * ============================================================================ */
#define ACTUATOR_PIN    18  /* problem LED (1 = fault/lock, 0 = healthy) */
#define RESTART_PIN     19  /* input: button to exit system lock (active-high) */
#define ARDUINO_RESET_PIN 22 /**/
#define LINK_UART       1   /* UART0 is the PC log console            */
#define LINK_TX_PIN     17           /* ESP TX -> Arduino RX                   */
#define LINK_RX_PIN     16           /* ESP RX <- Arduino TX                   */
#define LINK_BAUD       115200       /* hardware UART on the Arduino (pins 0/1) */

#define TWDT_TIMEOUT_S  3            /* task watchdog period (seconds)         */
#define CYCLE_PERIOD_MS 50           /* control loop period (continuous run)   */
#define FRAME_TIMEOUT_MS 100         /* mid-frame stall -> reset the parser (match Arduino) */
#define ARDUINO_BOOT_GRACE_US (8000u * 1000u)  /* after resetting the Arduino, wait this long before the next silence check */

/* FreeRTOS task priorities (higher = preempts lower).                      */
#define CTRL_PRIO    10              /* control: the critical path             */
#define VERIFY_PRIO  9               /* verify : channel B                     */
#define RX_PRIO      8               /* comm_rx: UART receive state machine    */
#define TELEM_PRIO   2               /* telemetry + heartbeat                  */
#define TASK_STACK   4096            /* stack bytes per task                   */

/* ============================================================================
 *  FAULT INJECTION (enable ONE directive at a time)
 * ============================================================================
 *  These force a single bit-flip every INJECT_EVERY cycles to exercise the
 *  vote.  Leave all three undefined for the no-fault baseline run.
 *
 *    INJECT_A    flip a bit in channel A only  -> A != B, Arduino arbitrates
 *    INJECT_B    flip a bit in channel B only  -> symmetric case
 *    INJECT_BOTH flip the SAME bit in A and B  -> A == B (wrong), Arduino
 *                                                 dissents = common-mode
 *
 *  A and B share INJECT_BIT on purpose: for INJECT_BOTH the two corrupted
 *  channels must agree (A == B).  The all-three-differ case comes from
 *  INJECT_A here + INJECT_C on the Arduino, whose INJECT_C_BIT is a
 *  DIFFERENT bit: if A and C were corrupted on the same bit they would
 *  coincide and the vote would return a majority (ACK) instead of case 5.
 */
/* #define INJECT_A */
/* #define INJECT_B */
/* #define INJECT_BOTH */
#define INJECT_EVERY 5              /* inject on every Nth event               */
#define INJECT_BIT   0x00010000u    /* flip bit 16 of the 32-bit result        */

static const char *TAG = "fault_tolerant";   /* ESP_LOG tag */

/* ============================================================================
 *  APPLICATION SPEC — the function being voted on
 * ============================================================================
 *  f(samples[8]) = sum(samples[i]^2)  (mod 2^32)
 *
 *  Three deliberately DIFFERENT implementations (N-version diversity):
 *    A (this file, core 1): forward for-loop, hardware multiply
 *    B (this file, core 0): no multiply at all — x^2 built from the odd
 *                           number series 1 + 3 + 5 + ... + (2x-1)
 *    C (Arduino)          : 256-entry lookup table
 *
 *  Diversity matters: if all three used the same code, one systematic bug
 *  would corrupt them identically and the vote would confirm the wrong
 *  answer with a "3:0 majority".  The odd-number series and the LUT share
 *  no multiply logic, so a fault class that hits the multiplier cannot hit
 *  all three channels the same way.
 */
#define N_SAMPLES 8

static void make_samples(uint8_t *out, uint16_t event_id)
{
    uint32_t seed = 0x12345678u ^ ((uint32_t)event_id << 16);

    for (int i = 0; i < N_SAMPLES; i++) {
        seed = seed * 1664525u + 1013904223u;   /* Hull-Dobell constants */
        out[i] = (uint8_t)(seed >> 24);      /* take the top byte       */
    }
}

/* Channel A — forward loop, hardware multiply. */
static uint32_t compute_action_A(const uint8_t *s)
{
    uint32_t acc = 0;

    for (int i = 0; i < N_SAMPLES; i++) {
        acc += (uint32_t)s[i] * (uint32_t)s[i];
    }
    return acc;
}

/* Channel B — no multiplication: x^2 = 1 + 3 + 5 + ... + (2x - 1). */
static uint32_t compute_action_B(const uint8_t *s)
{
    uint32_t acc = 0;

    for (int i = 0; i < N_SAMPLES; i++) {
        uint32_t sq = 0;                 /* running square of s[i] */
        uint32_t odd = 1;                /* 1st odd number          */
        for (int k = 0; k < s[i]; k++) { /* add s[i] odd numbers    */
            sq += odd;
            odd += 2;
        }
        acc += sq;
    }
    return acc;
}

/* ============================================================================
 *  TMR — triple modular redundancy on the critical state (last_action)
 * ============================================================================
 *  The applied action is stored three times so a single bit-flip in RAM can
 *  be detected and healed on the next read (2-of-3 majority).
 *
 *  seq / seq_check give a second line of defense: seq is bumped before the
 *  three copies are written and seq_check is set after.  If a reader ever
 *  sees seq != seq_check, the write was interrupted ("torn") — the three
 *  copies are not guaranteed coherent, so the read is flagged even though
 *  the portENTER_CRITICAL section below normally already makes the write
 *  atomic on this platform.
 */

#define TMR_UNRECOVERABLE 0xFFFFFFFFu   /* sentinel: all 3 copies disagree */

typedef struct {
    uint32_t seq;        /* write counter, bumped before the copies      */
    uint32_t c1;         /* redundant copy 1                             */
    uint32_t c2;         /* redundant copy 2                             */
    uint32_t c3;         /* redundant copy 3                             */
    uint32_t seq_check;  /* shadow of seq, set after the copies          */
} tmr_uint32_t;

static tmr_uint32_t last_action;                                /* TMR state */
static portMUX_TYPE tmr_lock = portMUX_INITIALIZER_UNLOCKED;    /* spinlock  */

/* Atomically overwrite the TMR value with three fresh copies. */
static void tmr_write(tmr_uint32_t *v, uint32_t value)
{
    portENTER_CRITICAL(&tmr_lock);   /* atomic w.r.t. both cores + IRQs */
    v->seq++;
    v->c1 = value;
    v->c2 = value;
    v->c3 = value;
    v->seq_check = v->seq;
    portEXIT_CRITICAL(&tmr_lock);
}

/*
 * Read the TMR value, vote 2-of-3, and heal the odd copy if one is bad.
 * Returns TMR_UNRECOVERABLE if all three copies disagree (double fault).
 * *torn is set true if the seq/seq_check pair reveals an interrupted write.
 */
static uint32_t tmr_read_and_heal(tmr_uint32_t *v, bool *torn)
{
    uint32_t r;

    portENTER_CRITICAL(&tmr_lock);           //entering critical section

    *torn = (v->seq != v->seq_check);            //check if write was interrupted

    if(v->c1 == v->c2 && v->c2 == v->c3) {       // 3/3 agree
        r = v->c1;
    }
    else if (v->c1 == v->c2) {                   // c3 is the odd one
        v->c3 = v->c1;
        r = v->c1;
    }
    else if (v->c1 == v->c3) {                   // c2 is the odd one
        v->c2 = v->c1;
        r = v->c1;
    }
    else if (v->c2 == v->c3) {                   // c1 is the odd one
        v->c1 = v->c2;
        r = v->c2;
    }
    else {                                       // all disagree
        portEXIT_CRITICAL(&tmr_lock);
        return TMR_UNRECOVERABLE;
    }

    portEXIT_CRITICAL(&tmr_lock);
    return r;
}

/* ============================================================================
 *  UART PROTOCOL — frame layout + CRC16
 * ============================================================================
 *  Frame (all multi-byte fields little-endian):
 *
 *    0xAA 0x55 | TYPE | SEQ | LEN | PAYLOAD (LEN bytes) | CRC16(LE)
 *    ─ sync ─   ───────── header ────────   ─ payload ─  ─ CRC ─
 *
 *  CRC16-CCITT (poly 0x1021, init 0xFFFF) covers TYPE..PAYLOAD only.
 *  It must be byte-identical to the Arduino implementation — the protocol
 *  is shared infrastructure, so it is the ONE thing that must NOT differ
 *  between the two platforms (diversity applies only to the voted f()).
 *
 *  Message types:
 *    RESULT     ESP->Arduino  { event_id(2), a(4), b(4) }   = 10 B
 *    HEARTBEAT  both          { counter(4), flags(1) }      =  5 B
 *    ACK        Arduino->ESP  { event_id(2), verdict(1), decided(4) } = 7 B
 *    SAFE       Arduino->ESP  { event_id(2), reason(1) }    =  3 B
 *    LOCK       both          { reason(1) }                 =  1 B
 *    UNLOCK     both          { }                           =  0 B
 *    REQ_REPEAT ESP->Arduino  { event_id(2) }               =  2 B
 */
#define FRAME_SYNC0       0xAA          /* first sync byte                          */
#define FRAME_SYNC1       0x55          /* second sync byte                         */
#define FRAME_MAX_PAYLOAD 64            /* largest payload we accept (bounds check) */

typedef enum {
    MSG_RESULT     = 0x01,   /* ESP -> Arduino: both channel results        */
    MSG_HEARTBEAT  = 0x02,   /* ESP -> Arduino: liveness tick                */
    MSG_ACK        = 0x03,   /* Arduino -> ESP: supervisor agreed           */
    MSG_SAFE       = 0x04,   /* Arduino -> ESP: supervisor dissented        */
    MSG_LOCK       = 0x05,   /* both: enter system lock {reason(1)}         */
    MSG_UNLOCK     = 0x06,   /* both: button pressed, recover {}            */
    MSG_REQ_REPEAT = 0x07,   /* ESP -> Arduino: re-send cached verdict      */
} msg_type_t;

static uint8_t s_seq = 0;              /* TX sequence number (wraps at 256) */
static portMUX_TYPE seq_lock = portMUX_INITIALIZER_UNLOCKED;   /* guards s_seq++ */

/* One CRC16-CCITT step.  Shared with the Arduino (must match exactly). */
static uint16_t crc16_update(uint16_t crc, uint8_t c)
{
    crc ^= (uint16_t)c << 8;
    for (int b = 0; b < 8; b++) {
        if (crc & 0x8000) {crc = (uint16_t)((crc << 1) ^ 0x1021);} //check if the MSB is one
        else {crc = (uint16_t)(crc << 1);} // if MSB = 0 we shif left crc
    }
    return crc;
}

/* Decode a 4-byte little-endian field (used for the ACK "decided" value). */
static uint32_t read_u32(const uint8_t *p){
    //Rebuilds a 32bit message in single 32bit variable
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ---------------------------------------------------------------------------
 *  UART1 setup — independent from UART0 (the PC log console).
 * ------------------------------------------------------------------------- */
static void uart_init(void)
{
    //defining UART cominication parameters
    uart_config_t cfg = {
        .baud_rate  = LINK_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(LINK_UART, 1024, 1024, 0, NULL, 0); //installing UART drivers
    uart_param_config(LINK_UART, &cfg); // configuring UART with cfg parameters
    uart_set_pin(LINK_UART, LINK_TX_PIN, LINK_RX_PIN,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/* ---------------------------------------------------------------------------
 *  Frame builder: wrap a payload into a synced, CRC-protected frame and send.
 * ------------------------------------------------------------------------- */
static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    /* 2 sync + 3 header + max payload + 2 CRC */
    uint8_t buf[2 + 3 + FRAME_MAX_PAYLOAD + 2];

    buf[0] = FRAME_SYNC0;
    buf[1] = FRAME_SYNC1;
    buf[2] = type;                       /* TYPE                    */
    portENTER_CRITICAL(&seq_lock);
    buf[3] = s_seq++;                    /* SEQ (increments)        */
    portEXIT_CRITICAL(&seq_lock);
    buf[4] = len;                        /* LEN                     */
    if (len > 0) {
        memcpy(&buf[5], payload, len);   /* PAYLOAD                 */
    }

    uint16_t crc = 0xFFFF;
    for (int i = 2; i < 5 + len; i++) {  /* CRC over TYPE..PAYLOAD  */
        crc = crc16_update(crc, buf[i]);
    }
    buf[5 + len]     = crc & 0xFF;       /* CRC low byte  (LE)      */
    buf[5 + len + 1] = crc >> 8;         /* CRC high byte (LE)      */

    uart_write_bytes(LINK_UART, (const char *)buf, 5 + len + 2);
}

/* Serialize channel results into a RESULT frame. */
static void send_result(uint16_t event_id, uint32_t a, uint32_t b){
    uint8_t p[10];
    int o = 0;

    p[o++] = event_id & 0xFF; // low half of event_id
    p[o++] = (event_id >> 8) & 0xFF; //high half of event_id
    p[o++] = a & 0xFF; // 1/4 of a
    p[o++] = (a >> 8) & 0xFF; //2/4 of a
    p[o++] = (a >> 16) & 0xFF; //3/4 of a
    p[o++] = (a >> 24) & 0xFF; //4/4 of a
    p[o++] = b & 0xFF; //1/4 of b
    p[o++] = (b >> 8) & 0xFF; //2/4 of b
    p[o++] = (b >> 16) & 0xFF; //3/4 of b
    p[o++] = (b >> 24) & 0xFF; //4/4 of b

    send_frame(MSG_RESULT, p, sizeof(p));
}

/* Ask the Arduino to re-send its cached ACK/SAFE verdict for one event. */
static void send_req_repeat(uint16_t event_id)
{
    uint8_t p[2];
    p[0] = event_id & 0xFF;
    p[1] = (event_id >> 8) & 0xFF;
    send_frame(MSG_REQ_REPEAT, p, 2);
}

/* Serialize the liveness counter + lock state into a HEARTBEAT frame. */
static void send_heartbeat(uint32_t counter, bool locked)
{
    uint8_t p[5];
    // splicing in bytes the message
    p[0] = counter & 0xFF;
    p[1] = (counter >> 8) & 0xFF;
    p[2] = (counter >> 16) & 0xFF;
    p[3] = (counter >> 24) & 0xFF;
    p[4] = locked ? 0x01 : 0x00;   /* flags: bit0 = locked */

    send_frame(MSG_HEARTBEAT, p, 5);
}

/* Propagate the local system-lock decision to the Arduino. */
static void send_lock(uint8_t reason)
{
    uint8_t p[1];
    p[0] = reason;
    send_frame(MSG_LOCK, p, 1);
}

/* Propagate a button-press recovery to the Arduino. */
static void send_unlock(void)
{
    send_frame(MSG_UNLOCK, NULL, 0);
}

/* ============================================================================
 *  INTER-TASK PLUMBING
 * ============================================================================
 *  work_queue    : control -> verify, carries the event_id to compute B for
 *  result_queue  : verify  -> control, carries {event_id, result_b}
 *  verdict_queue : comm_rx -> control, carries the Arduino's verdict
 *
 *  CONVENTION: both verify_result_t and verdict_t store event_id as their
 *  FIRST field.  This lets wait_for_event_id() read the id uniformly.
 */

typedef struct { uint16_t event_id; uint32_t result_b; } verify_result_t;
typedef struct { uint16_t event_id; bool ok; uint32_t value; } verdict_t;

static QueueHandle_t     work_queue;      /* control -> verify: cycle to compute */
static QueueHandle_t     result_queue;    /* verify -> control: channel B        */
static QueueHandle_t     verdict_queue;   /* comm_rx -> control: Arduino verdict */

/* Timestamp of the last valid frame from the Arduino (liveness check). */
static volatile uint32_t s_arduino_last_us = 0;
/* Until this time, suppress the "Arduino silent" reset (boot grace after reset). */
static volatile int64_t s_arduino_boot_deadline_us = 0;

/* Diagnostic counters — these summarise fault coverage at runtime. */
static uint32_t g_events = 0;             /* total events processed              */
static uint32_t g_disagreements = 0;      /* local A != B cases                  */
static uint32_t g_safe = 0;               /* times safe state was entered        */

/*
 * Drain a queue until an item whose event_id matches arrives (or the deadline
 * passes).  Each attempt blocks up to 10 ms, so `attempts` * 10 ms is the
 * total budget.  Stale items (different event_id) are logged and discarded.
 */
static bool wait_for_event_id(QueueHandle_t q, void *out,uint16_t event_id, int attempts)
{
    for (int i = 0; i < attempts; i++) {
        if (xQueueReceive(q, out, pdMS_TO_TICKS(10)) != pdTRUE) {
            continue;                              /* queue empty in window */
        }
        uint16_t got_id = *(uint16_t *)out;        /* event_id is 1st field  */
        if (got_id == event_id) {
            return true;                           /* matching item found    */
        }
        ESP_LOGW(TAG, "dropped stale item ev=%u", got_id);
    }
    return false;                                  /* deadline hit           */
}

/* ============================================================================
 *  GPIO — actuator + Arduino reset as outputs, restart button as input.
 * ============================================================================ */
static void restart_isr_handler(void *arg);   /* defined below with IRAM_ATTR */

static void gpio_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << ACTUATOR_PIN) | (1ULL << ARDUINO_RESET_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    /* Restart button (active-HIGH): the node N of the button network
     * is pulled LOW at rest by R8 and rises to ~3.3 V
     * (divider R4-R5) when the button is pressed, so the useful edge is the
     * RISING one.  No internal pull-up: R8 defines the idle level. */
    io.pin_bit_mask = (1ULL << RESTART_PIN);
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_POSEDGE;

    gpio_config(&io);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(RESTART_PIN, restart_isr_handler, NULL);

    gpio_set_level(ACTUATOR_PIN, 0);   /* problem LED off at boot */
}
/* ============================================================================
 *  ACTIONS
 * ============================================================================ */
static volatile uint8_t s_fault_bucket = 0;
static volatile bool s_system_lock = false;
static volatile bool s_unlock_requested = false;   /* set by the restart ISR */
static volatile int64_t s_unlock_grace_until_us = 0;  /* ignore stale peer-lock heartbeats until then (us, monotonic, wrap-safe with int64) */
#define MAX_FAULT_BUCKET 10
#define BUCKET_RECOVERY 1
#define BUCKET_PENALTY 2
#define LOCK_REASON_BUCKET 1   /* local bucket reached its capacity */
#define LOCK_REASON_MIRROR 2   /* locked because the peer advertised lock in its heartbeat */
#define UNLOCK_GRACE_US  3000000u  /* after unlocking, ignore stale peer-lock heartbeats for this long */

/* ACTUATOR_PIN drives a problem LED (1 = fault/lock, 0 = healthy). */
static portMUX_TYPE fault_lock = portMUX_INITIALIZER_UNLOCKED;   /* guards lock state */

/* ---- lock-state accessors: all s_system_lock / s_fault_bucket reads and
 *      writes go through these so comm_rx (core 0), control (core 1) and
 *      telemetry (core 1) never race on the shared state. ---- */
static bool system_is_locked(void)
{
    bool locked;
    portENTER_CRITICAL(&fault_lock);
    locked = s_system_lock;
    portEXIT_CRITICAL(&fault_lock);
    return locked;
}

static void set_system_lock(bool locked)
{
    portENTER_CRITICAL(&fault_lock);
    s_system_lock = locked;
    portEXIT_CRITICAL(&fault_lock);
}

/* Add one penalty; returns the new bucket value (atomically). */
static uint8_t bucket_add_penalty(void)
{
    uint8_t v;
    portENTER_CRITICAL(&fault_lock);
    s_fault_bucket = s_fault_bucket + BUCKET_PENALTY;
    v = s_fault_bucket;
    portEXIT_CRITICAL(&fault_lock);
    return v;
}

static void bucket_recover(void)
{
    portENTER_CRITICAL(&fault_lock);
    if (s_fault_bucket > 0) {
        s_fault_bucket = s_fault_bucket - BUCKET_RECOVERY;
    }
    portEXIT_CRITICAL(&fault_lock);
}

/* Enter system lock locally, light the LED and propagate to the Arduino. */
static void system_lock_now(uint8_t reason)
{
    if (system_is_locked()) {
        return;
    }
    set_system_lock(true);
    gpio_set_level(ACTUATOR_PIN, 1);        /* problem LED on */
    ESP_LOGE(TAG, "SYSTEM LOCKED (reason=%u)", reason);
    send_lock(reason);                      /* make the Arduino lock too */
}

/* Recover locally (button or remote UNLOCK); the caller re-broadcasts. */
static void system_unlock(void)
{
    set_system_lock(false);
    portENTER_CRITICAL(&fault_lock);
    s_fault_bucket = 0;
    portEXIT_CRITICAL(&fault_lock);
    gpio_set_level(ACTUATOR_PIN, 0);        /* problem LED off */
    s_unlock_grace_until_us = esp_timer_get_time() + UNLOCK_GRACE_US;
    ESP_LOGI(TAG, "SYSTEM UNLOCKED");
}

static void safe_state_local(void)
{
    g_safe++;
    ESP_LOGE(TAG, "SAFE STATE (local)");

    if (system_is_locked()) {
        return;
    }

    uint8_t bucket = bucket_add_penalty();
    if (bucket >= MAX_FAULT_BUCKET) {
        system_lock_now(LOCK_REASON_BUCKET);
    } else {
        ESP_LOGE(TAG, "SAFE STATE LOCAL: fault bucket increased to %d", (int)bucket);
    }
}

static void apply_action(uint32_t action)
{
    if (system_is_locked()) {
        ESP_LOGE(TAG, "SYSTEM ENTERED IN CRITICAL MODE, NO ACTION WILL BE APPLIED UNTIL RESET");
        return;
    }

    bucket_recover();

    tmr_write(&last_action, action);         /* persist via TMR            */
    gpio_set_level(ACTUATOR_PIN, 0);         /* problem LED off: healthy   */
    ESP_LOGI(TAG, "action applied: %u", (unsigned)action);
}
/* ============================================================================
 *  INTERRUPT SERVICE ROUTINE (ISR)
 * ============================================================================ */
static void IRAM_ATTR restart_isr_handler(void* arg) {
    // Button only acts while the system is locked; defer the unlock + UART
    // notify to a task (UART and spinlocks are NOT ISR-safe here — no ESP_LOGx!).
    if (!s_system_lock) return;
    s_unlock_requested = true;
}

/* ============================================================================
 *  CONTROL TASK (core 1) — channel A + local vote + supervisor interaction
 * ============================================================================ */
static void control_task(void *arg)
{
    esp_task_wdt_add(NULL);   //Subscribe this task to the WDT
    uint16_t event_id = 0;

    while (1) {
        esp_task_wdt_reset();            /* always running: feed the dog */

        /* Button pressed while locked: recover locally and tell the Arduino. */
        if (s_unlock_requested) {
            s_unlock_requested = false;
            ESP_LOGI(TAG, "restart button pressed: unlocking");
            system_unlock();
            send_unlock();
        }

        /* While locked there is nothing to arbitrate: the Arduino drops every
         * RESULT and only re-advertises its own lock, so generating events here
         * would just waste an ~800ms timeout per cycle and spam the UART. */
        if (system_is_locked()) {
            vTaskDelay(pdMS_TO_TICKS(CYCLE_PERIOD_MS));
            continue;
        }

        /* Discard verdicts left over from a previous cycle (a late ACK/SAFE
         * arriving after our wait expired). Without this the queue fills up
         * and a later verdict would be dropped by the 0-timeout send. */
        verdict_t stale_v;
        while (xQueueReceive(verdict_queue, &stale_v, 0) == pdTRUE) { }

        event_id++;
        g_events++;

        /* 1) Kick the verify channel FIRST, so B runs concurrently with A. */
        if (xQueueSend(work_queue, &event_id, pdMS_TO_TICKS(50)) != pdTRUE) {
            ESP_LOGE(TAG, "verify channel unreachable");
            safe_state_local();
            vTaskDelay(pdMS_TO_TICKS(CYCLE_PERIOD_MS));
            continue;
        }

        /* 2) Compute A over our OWN copy of the input. */
        uint8_t samples_a[N_SAMPLES];
        make_samples(samples_a, event_id);
        uint32_t a = compute_action_A(samples_a);

        /* Fault injection on channel A (compiled only if enabled). */
#if defined(INJECT_A) || defined(INJECT_BOTH)
        if ((event_id % INJECT_EVERY) == 0) {
            a ^= INJECT_BIT;
            ESP_LOGW(TAG, "injected bit-flip in channel A");
        }
#endif

        verify_result_t r;
        if (!wait_for_event_id(result_queue, &r, event_id, 10)) {
            ESP_LOGE(TAG, "verify channel timeout");
            safe_state_local();
            vTaskDelay(pdMS_TO_TICKS(CYCLE_PERIOD_MS));
            continue;
        }
        uint32_t b = r.result_b;

        /* 4) Report both results to the supervisor and ALWAYS wait for its
         * verdict before acting — even when a == b. Channel C is the only
         * channel that can spot a common-mode fault (a == b != c), so we
         * must not apply the local vote ahead of its verdict. */
        if (a != b) {
            g_disagreements++;
            ESP_LOGW(TAG, "A != B locally, deferring to supervisor");
        }

        send_result(event_id, a, b);

        /* A lost frame would strand us, so re-send the RESULT a few times
         * before giving up. */
        verdict_t v;
        bool got_verdict = false;
        const int retry_attempts = 3;
        const int wait_attempts  = 20;   /* 20 * 10 ms = 200 ms per try */

        for (int attempt = 0; attempt < retry_attempts && !got_verdict; attempt++) {
            if (attempt > 0) {
                ESP_LOGW(TAG, "no verdict yet, re-sending RESULT (attempt %d/%d)",
                         attempt + 1, retry_attempts);
                send_result(event_id, a, b);
            }
            got_verdict = wait_for_event_id(verdict_queue, &v, event_id, wait_attempts);
        }

        /* Final explicit pull: ask the Arduino to re-send its cached verdict.
         * Covers the case where the ACK/SAFE was lost on the way back and the
         * RESULT retransmissions above also failed to elicit a fresh reply. */
        if (!got_verdict) {
            ESP_LOGW(TAG, "no verdict after retries, sending REQ_REPEAT ev=%u", event_id);
            send_req_repeat(event_id);
            got_verdict = wait_for_event_id(verdict_queue, &v, event_id, wait_attempts);
        }

        if (!got_verdict) {
            ESP_LOGE(TAG, "supervisor silent");
            safe_state_local();
            vTaskDelay(pdMS_TO_TICKS(CYCLE_PERIOD_MS));
            continue;
        }
        if (v.ok) {apply_action(v.value);}
        else {safe_state_local();}

        vTaskDelay(pdMS_TO_TICKS(CYCLE_PERIOD_MS));  /* pace the continuous run */
    }
}

/* ============================================================================
 *  VERIFY TASK (core 0) — channel B
 * ============================================================================ */
static void verify_task(void *arg)
{
    esp_task_wdt_add(NULL);

    while (1) {
        esp_task_wdt_reset();                /* feed the dog every loop */
        uint16_t event_id;
        if (xQueueReceive(work_queue, &event_id, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;
        }

        /* Own copy of the input — never share a buffer with channel A. */
        uint8_t samples_b[N_SAMPLES];
        make_samples(samples_b, event_id);
        uint32_t b = compute_action_B(samples_b);

        /* Fault injection on channel B (compiled only if enabled). */
#if defined(INJECT_B) || defined(INJECT_BOTH)
        if ((event_id % INJECT_EVERY) == 0) {
            b ^= INJECT_BIT;
            ESP_LOGW(TAG, "injected bit-flip in channel B");
        }
#endif

        verify_result_t r = { .event_id = event_id, .result_b = b };
        xQueueSend(result_queue, &r, portMAX_DELAY);
    }
}

/* ============================================================================
 *  COMM_RX TASK (core 0) — UART receive state machine
 * ============================================================================
 *  UART delivers a raw byte stream with no framing, so the receiver rebuilds
 *  frames byte by byte through a small state machine.  States:
 *
 *    WAIT_SYNC0 -> WAIT_SYNC1 -> READ_HDR -> READ_PAY -> READ_CRC -> back
 *
 *  The CRC is accumulated incrementally over the same bytes the sender
 *  covered (TYPE..PAYLOAD), and checked once the two CRC bytes arrive.
 */
static uint8_t  s_rx_type;                          /* frame TYPE              */
static uint8_t  s_rx_len;                           /* frame LEN               */
static uint8_t  s_rx_idx;                           /* position in current stage */
static uint8_t  s_rx_hdr[3];                        /* TYPE/SEQ/LEN staging     */
static uint8_t  s_rx_payload[FRAME_MAX_PAYLOAD];    /* payload staging          */
static uint8_t  s_rx_crc_bytes[2];                  /* CRC staging (LE)         */
static uint16_t s_rx_running;                       /* incremental CRC          */
static uint8_t  s_last_seq;                         /* last SEQ seen (RX)       */
static bool     s_seq_valid = false;                /* first valid SEQ received? */

typedef enum {
    WAIT_SYNC0,
    WAIT_SYNC1,
    READ_HDR,
    READ_PAY,
    READ_CRC,
} rx_state_t;

static rx_state_t s_rx_state = WAIT_SYNC0;
static volatile uint32_t s_last_byte_us;    /* last byte received (frame stall) */

/* Handle a fully received, CRC-valid frame. */
static void handle_rx_frame(uint8_t type, const uint8_t *p, uint8_t len){
    s_arduino_last_us = (uint32_t)esp_timer_get_time(); //update arduino interaction timer

    if (type == MSG_ACK && len == 7) {
        uint16_t event_id = p[0] | (p[1] << 8); //Rebuilding envent_id following little-endian formatting
        uint32_t decided  = read_u32(&p[3]);  //Correct value
        verdict_t v = { .event_id = event_id, .ok = true, .value = decided }; //populating v fields
        xQueueSend(verdict_queue, &v, 0);  //sending verditc to queue
        ESP_LOGI(TAG, "ACK ev=%u verdict=%u decided=%u", event_id, p[2], (unsigned)decided);
    }
    else if (type == MSG_SAFE && len == 3) {
        uint16_t event_id = p[0] | (p[1] << 8); //Rebuilding event_id following little-endian formatting
        ESP_LOGE(TAG, "SAFE ev=%u reason=%u", event_id, p[2]);

        /* Any dissent (common-mode OR all-differ) is queued to control_task,
         * which is now always waiting for the verdict. It calls
         * safe_state_local() exactly once — no double penalty, and no direct
         * cross-core write to s_system_lock from this RX task. */
        verdict_t v = { .event_id = event_id, .ok = false, .value = 0 };
        xQueueSend(verdict_queue, &v, 0);
    }
    else if (type == MSG_LOCK && len == 1) {
        if (!system_is_locked()) {
            set_system_lock(true);
            gpio_set_level(ACTUATOR_PIN, 1);   /* problem LED on */
            ESP_LOGE(TAG, "SYSTEM LOCKED (remote, reason=%u)", p[0]);
        }
    }
    else if (type == MSG_UNLOCK && len == 0) {
        bool was_locked = system_is_locked();
        system_unlock();
        if (was_locked) {
            send_unlock();   /* re-propagate so recovery wins any race */
        }
    }
    else if (type == MSG_HEARTBEAT && len == 5) {
        /* Mirror the peer's lock so a rebooted board converges.  After a
         * local unlock we briefly ignore stale peer-lock heartbeats. */
        if ((p[4] & 0x01) && !system_is_locked() &&
            esp_timer_get_time() >= s_unlock_grace_until_us) {
            system_lock_now(LOCK_REASON_MIRROR);
        }
    }
}

static void comm_rx_task(void *arg){
    esp_task_wdt_add(NULL);
    s_last_byte_us = (uint32_t)esp_timer_get_time();

    while (1) {
        esp_task_wdt_reset();                /* feed the dog every loop */
        uint8_t c;
        if (uart_read_bytes(LINK_UART, &c, 1, pdMS_TO_TICKS(100)) <= 0) {
            /* Frame stalled mid-way: realign the parser (same as the Arduino). */
            if (s_rx_state != WAIT_SYNC0 &&
                (uint32_t)esp_timer_get_time() - s_last_byte_us > (uint32_t)FRAME_TIMEOUT_MS * 1000u) {
                s_rx_state = WAIT_SYNC0;
            }
            continue;
        }
        s_last_byte_us = (uint32_t)esp_timer_get_time();

        switch (s_rx_state){

        case WAIT_SYNC0:
            /* Look for the first sync byte; anything else is line noise. */
            if (c == FRAME_SYNC0) s_rx_state = WAIT_SYNC1;
            break;

        case WAIT_SYNC1:
            /* Second sync byte confirms the frame start and resets the CRC. */
            if (c == FRAME_SYNC1) {
                s_rx_state = READ_HDR;
                s_rx_idx = 0;
                s_rx_running = 0xFFFF;           /* CRC16 init value */
            }
            else if (c == FRAME_SYNC0) {s_rx_state = WAIT_SYNC1;} /* 0xAA 0xAA: keep waiting */
            else {s_rx_state = WAIT_SYNC0;}         /* noise: start over */
            break;

        case READ_HDR:
            /* Header bytes: TYPE, SEQ (checked in READ_CRC), LEN. */
            s_rx_running = crc16_update(s_rx_running, c);
            s_rx_hdr[s_rx_idx++] = c;

            if (s_rx_idx == 3) {
                s_rx_type = s_rx_hdr[0];
                s_rx_len  = s_rx_hdr[2];

                /* Bounds-check LEN: a corrupted length field must not be
                 * allowed to overflow the payload buffer. */
                if (s_rx_len > sizeof(s_rx_payload)) {
                    ESP_LOGW(TAG, "invalid frame length %u, dropped", s_rx_len);
                    s_rx_state = WAIT_SYNC0;
                }
                else if (s_rx_len == 0) {
                    s_rx_idx = 0;              /* zero-length payload: skip READ_PAY */
                    s_rx_state = READ_CRC;
                }
                else {
                    s_rx_idx = 0;
                    s_rx_state = READ_PAY;
                }
            }
            break;

        case READ_PAY:
            s_rx_running = crc16_update(s_rx_running, c);
            s_rx_payload[s_rx_idx++] = c;

            if (s_rx_idx == s_rx_len) {
                s_rx_idx = 0;
                s_rx_state = READ_CRC;
            }
            break;

        case READ_CRC:
            /* Two CRC bytes arrive low byte first (little-endian). */
            s_rx_crc_bytes[s_rx_idx++] = c;

            if (s_rx_idx == 2) {
                s_rx_state = WAIT_SYNC0;         /* ready for next frame */

                uint16_t crc = (uint16_t)s_rx_crc_bytes[0] | ((uint16_t)s_rx_crc_bytes[1] << 8);

                if (crc == s_rx_running) {
                    bool accept = true;
                    if (s_seq_valid) {
                        uint8_t delta = (uint8_t)(s_rx_hdr[1] - s_last_seq);
                        if (delta == 0) {
                            ESP_LOGW(TAG, "duplicate SEQ, frame dropped");
                            accept = false;
                        } else if (delta > 1) {
                            ESP_LOGW(TAG, "SEQ gap: lost %u frame(s)", (unsigned)(delta - 1));
                        }
                    } else {
                        s_seq_valid = true;    /* first valid frame: baseline */
                    }
                    if (accept) {
                        s_last_seq = s_rx_hdr[1];
                        handle_rx_frame(s_rx_type, s_rx_payload, s_rx_len);
                    }
                }
                else {ESP_LOGW(TAG, "CRC mismatch, frame dropped");}
            }
            break;
        }
    }
}

/* ============================================================================
 *  TELEMETRY TASK (core 1) — TMR audit, stats, heartbeat, supervisor timeout
 * ============================================================================ */
static void telemetry_task(void *arg)
{
    esp_task_wdt_add(NULL); //initializing WDT
    uint32_t hb = 0;

    while (1) {
        /* Audit the TMR copy of last_action: detect + heal silent damage. */
        bool torn = false;
        uint32_t a = tmr_read_and_heal(&last_action, &torn);

        if (a == TMR_UNRECOVERABLE) {
            ESP_LOGE(TAG, "TMR unrecoverable (3 copies disagree)"); //TRM unrecoverable -> entering safe state
            safe_state_local();
        }
        else if (torn) {
            ESP_LOGW(TAG, "torn write detected + healed, value=%u", (unsigned)a); // torn during write detecter
        }
        else {
            ESP_LOGI(TAG, "telemetry: last_action=%u", (unsigned)a); //all fine
        }

        ESP_LOGI(TAG, "stats: events=%u disagreements=%u safe=%u",
                 (unsigned)g_events, (unsigned)g_disagreements, (unsigned)g_safe);
        send_heartbeat(hb++, system_is_locked()); //sending heartbeat to the other board

        //checking if other board is still alive
        if ((uint32_t)esp_timer_get_time() - s_arduino_last_us > 3u * 1000000u &&
            esp_timer_get_time() >= s_arduino_boot_deadline_us) {
            ESP_LOGE(TAG, "supervisor silent > 3 s");
            ESP_LOGE(TAG, "sending arduino reset signal");
            gpio_set_level(ARDUINO_RESET_PIN,1);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(ARDUINO_RESET_PIN,0);
            /* give the Arduino time to boot before the next silence check */
            s_arduino_last_us = (uint32_t)esp_timer_get_time();
            s_arduino_boot_deadline_us = esp_timer_get_time() + ARDUINO_BOOT_GRACE_US;
            safe_state_local();
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_task_wdt_reset(); // resetting WDT
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "boot");

    //configuring watchdog timer
    const esp_task_wdt_config_t twdt = {
        .timeout_ms     = TWDT_TIMEOUT_S * 1000,    //time before WDT triggers
        .idle_core_mask = (1 << 0) | (1 << 1),      //selecting the cores
        .trigger_panic  = true,                     //enabling panic_handler
    };
    /* TWDT is already initialized by CONFIG_ESP_TASK_WDT_INIT, so re-apply the
     * project's timeout/panic settings via reconfigure when init is refused. */
    if (esp_task_wdt_init(&twdt) != ESP_OK) {
        esp_task_wdt_reconfigure(&twdt);
    }

    gpio_init();   // initializing gpio pins
    uart_init();   // initializing uart
    s_arduino_last_us = (uint32_t)esp_timer_get_time(); //starts traking interaction time

    /* Create the inter-task primitives. */
    work_queue    = xQueueCreate(8, sizeof(uint16_t));
    result_queue  = xQueueCreate(8, sizeof(verify_result_t));
    verdict_queue = xQueueCreate(8, sizeof(verdict_t));

    /* Pin each task to its core so the two channels truly run in parallel:
     * control on core 1, verify on core 0. */
    xTaskCreatePinnedToCore(control_task,   "control",   TASK_STACK, NULL, CTRL_PRIO,   NULL, 1);
    xTaskCreatePinnedToCore(verify_task,    "verify",    TASK_STACK, NULL, VERIFY_PRIO, NULL, 0);
    xTaskCreatePinnedToCore(comm_rx_task,   "comm_rx",   TASK_STACK, NULL, RX_PRIO,     NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", TASK_STACK, NULL, TELEM_PRIO,  NULL, 1);
}