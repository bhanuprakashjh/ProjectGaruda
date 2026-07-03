/**
 * @file gsp_ak.c
 *
 * @brief GSP v1 engine + bridge for the high-speed sensorless trapezoidal
 * reference.  Protocol core adapted from the Garuda AK firmware (gsp.c);
 * the bridge maps this firmware's MC1APP/trapezoidal data into the exact
 * snapshot/info wire layout the garuda-gui decodes (decode.ts).
 *
 * This file is #included at the bottom of main.c (single translation unit)
 * so no MPLAB project-file edits are required.  See gsp_ak.h.
 *
 * Component: GSP (port)
 */

/* This file is registered in the MPLAB project ONLY so it shows up in the
 * project tree; the REAL compilation happens via #include from main.c,
 * which defines GSP_AK_IMPLEMENTATION first.  A standalone compile of this
 * file (what the IDE does for registered sources) intentionally produces
 * an empty object - no duplicate symbols. */
typedef int gsp_ak_registered_for_mplab_tree_t;   /* keep the TU non-empty */

#if defined(ENABLE_GSP) && defined(GSP_AK_IMPLEMENTATION)

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "gsp_ak.h"
#include "hal/uart1.h"
#include "mc1_init.h"
#include "mc1_service.h"
#include "mc1_calc_params.h"

/* ── Tuning ──────────────────────────────────────────────────────────── */

#define GSP_START_BYTE        0x02
#define GSP_RX_RING_SIZE      256
#define GSP_TX_RING_SIZE      256
#define GSP_RX_RING_MASK      (GSP_RX_RING_SIZE - 1)
#define GSP_TX_RING_MASK      (GSP_TX_RING_SIZE - 1)
#define GSP_MAX_PAYLOAD_LEN   249
#define GSP_PARSER_TIMEOUT_MS 100
#define GSP_BAUDRATE_DIVIDER  54          /* 100M/(16*54) ~ 115.7k (as Garuda) */
#define GSP_ISR_PER_MS        (PWMFREQUENCY_HZ / 1000u)   /* 40 @ 40 kHz */
#define GSP_FAILSAFE_MS       1000u       /* GSP throttle src: stop if silent */

/* Command IDs (garuda-gui contract) */
enum {
    GSP_CMD_PING            = 0x00,
    GSP_CMD_GET_INFO        = 0x01,
    GSP_CMD_GET_SNAPSHOT    = 0x02,
    GSP_CMD_START_MOTOR     = 0x03,
    GSP_CMD_STOP_MOTOR      = 0x04,
    GSP_CMD_CLEAR_FAULT     = 0x05,
    GSP_CMD_SET_THROTTLE    = 0x06,
    GSP_CMD_SET_THROTTLE_SRC= 0x07,
    GSP_CMD_HEARTBEAT       = 0x08,
    GSP_CMD_GET_PARAM       = 0x10,
    GSP_CMD_SET_PARAM       = 0x11,
    GSP_CMD_GET_PARAM_LIST  = 0x16,
    GSP_CMD_TELEM_START     = 0x14,
    GSP_CMD_TELEM_STOP      = 0x15,
    GSP_CMD_TELEM_FRAME     = 0x80,
    GSP_CMD_ERROR           = 0xFF
};

enum {
    GSP_ERR_UNKNOWN_CMD  = 0x01,
    GSP_ERR_BAD_LENGTH   = 0x02,
    GSP_ERR_WRONG_STATE  = 0x04,
    GSP_ERR_OUT_OF_RANGE = 0x05,
    GSP_ERR_UNKNOWN_PARAM= 0x06
};

/* Throttle sources (garuda-gui ControlPanel: ADC=0, GSP=1) */
enum { GSP_THR_SRC_ADC = 0, GSP_THR_SRC_GSP = 1 };

/* GUI ESC_STATES indices (types.ts):
 * ['IDLE','ARMED','DETECT','ALIGN','OL_RAMP','MORPH','CLOSED_LOOP',
 *  'BRAKING','RECOVERY','FAULT'] */
enum {
    GUI_ST_IDLE = 0, GUI_ST_ARMED = 1, GUI_ST_ALIGN = 3, GUI_ST_OL_RAMP = 4,
    GUI_ST_MORPH = 5, GUI_ST_CLOSED_LOOP = 6, GUI_ST_FAULT = 9
};

#define GSP_BOARD_MCLV48V300W  0x0001

/* ── State ───────────────────────────────────────────────────────────── */

typedef struct { uint8_t buf[GSP_RX_RING_SIZE]; uint16_t head, tail; } GSP_RXRING_T;
typedef struct { uint8_t buf[GSP_TX_RING_SIZE]; uint16_t head, tail; } GSP_TXRING_T;

typedef enum { GSP_WAIT_START, GSP_GOT_START, GSP_COLLECTING } GSP_PSTATE_T;

typedef struct {
    GSP_PSTATE_T state;
    uint8_t  pktBuf[1 + GSP_MAX_PAYLOAD_LEN];
    uint8_t  pktLen, pktIdx;
    uint8_t  crcBuf[2], crcIdx;
    bool     collectingCrc;
    uint32_t lastRxTick;
} GSP_PARSER_T;

static GSP_RXRING_T gspRx;
static GSP_TXRING_T gspTx;
static GSP_PARSER_T gspParser;

static volatile uint32_t gspMsTick;        /* 1 ms system tick               */
static volatile uint16_t gspIsrDiv;

static uint8_t  gspThrottleSrc = GSP_THR_SRC_ADC;
static volatile uint16_t gspThrottle;      /* 0..2000 (GSP source)           */
static uint32_t gspLastPacketMs;

static bool     gspTelemOn;
static uint16_t gspTelemIntervalMs = 20;
static uint32_t gspTelemLastMs;
static uint16_t gspTelemSeq;

static uint16_t gspIbusMaxRaw = 2048;      /* 2048-centered raw counts       */

extern MC1APP_DATA_T *pMC1Data;            /* mc1_service.c                  */
extern uint16_t runCmdMC1;                 /* mc1_service.c (button toggle)  */
extern uint16_t directionCmdMC1;           /* mc1_service.c (button toggle)  */

/* ── CRC-16-CCITT ────────────────────────────────────────────────────── */

static uint16_t GspCrc16(uint16_t crc, uint8_t b)
{
    crc ^= (uint16_t)b << 8;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                             : (uint16_t)(crc << 1);
    return crc;
}

/* ── Rings ───────────────────────────────────────────────────────────── */

static inline uint16_t GspRxCount(void){ return (gspRx.head - gspRx.tail) & GSP_RX_RING_MASK; }
static inline bool     GspRxFull(void) { return GspRxCount() == GSP_RX_RING_MASK; }
static inline void     GspRxPut(uint8_t b){ gspRx.buf[gspRx.head] = b; gspRx.head = (gspRx.head + 1) & GSP_RX_RING_MASK; }
static inline uint8_t  GspRxGet(void){ uint8_t b = gspRx.buf[gspRx.tail]; gspRx.tail = (gspRx.tail + 1) & GSP_RX_RING_MASK; return b; }
static inline uint16_t GspTxCount(void){ return (gspTx.head - gspTx.tail) & GSP_TX_RING_MASK; }
static inline uint16_t GspTxFree(void) { return GSP_TX_RING_MASK - GspTxCount(); }
static inline void     GspTxPut(uint8_t b){ gspTx.buf[gspTx.head] = b; gspTx.head = (gspTx.head + 1) & GSP_TX_RING_MASK; }
static inline uint8_t  GspTxGet(void){ uint8_t b = gspTx.buf[gspTx.tail]; gspTx.tail = (gspTx.tail + 1) & GSP_TX_RING_MASK; return b; }

/* ── Framed response ─────────────────────────────────────────────────── */

static bool GSP_SendResponse(uint8_t cmdId, const uint8_t *payload, uint8_t len)
{
    if (len > GSP_MAX_PAYLOAD_LEN) return false;
    if (len > 0 && payload == NULL) return false;
    if (GspTxFree() < (uint16_t)(5 + len)) return false;

    uint8_t pktLen = (uint8_t)(1 + len);
    uint16_t crc = GspCrc16(0xFFFF, pktLen);
    crc = GspCrc16(crc, cmdId);
    for (uint8_t i = 0; i < len; i++) crc = GspCrc16(crc, payload[i]);

    GspTxPut(GSP_START_BYTE);
    GspTxPut(pktLen);
    GspTxPut(cmdId);
    for (uint8_t i = 0; i < len; i++) GspTxPut(payload[i]);
    GspTxPut((uint8_t)(crc >> 8));
    GspTxPut((uint8_t)(crc & 0xFF));
    return true;
}

static void GspSendError(uint8_t err)
{
    GSP_SendResponse(GSP_CMD_ERROR, &err, 1);
}

/* ── Little-endian byte writers (immune to struct packing) ───────────── */

static inline void GspPut8 (uint8_t **p, uint8_t v) { *(*p)++ = v; }
static inline void GspPut16(uint8_t **p, uint16_t v){ *(*p)++ = (uint8_t)v; *(*p)++ = (uint8_t)(v >> 8); }
static inline void GspPut32(uint8_t **p, uint32_t v){ *(*p)++ = (uint8_t)v; *(*p)++ = (uint8_t)(v >> 8); *(*p)++ = (uint8_t)(v >> 16); *(*p)++ = (uint8_t)(v >> 24); }

/* ── Snapshot (68 bytes — exact decode.ts v1 offsets) ────────────────── */

static uint8_t GspGuiState(void)
{
    MCAPP_CONTROL_SCHEME_T *c = pMC1Data->pControlScheme;
    switch (MC1_GspAppState()) {
        case MCAPP_INIT:
        case MCAPP_CMD_WAIT:  return GUI_ST_IDLE;
        case MCAPP_OFFSET:    return GUI_ST_ARMED;
        case MCAPP_FAULT:     return GUI_ST_FAULT;
        case MCAPP_STOP:
        case MCAPP_DIRECTION_CHANGE: return GUI_ST_IDLE;
        case MCAPP_RUN:
            switch (c->controlState) {
                case ROTOR_LOCK:                 return GUI_ST_ALIGN;
                case FORCED_COMMUTATION_STARTUP: return GUI_ST_OL_RAMP;
                case CLOSED_LOOP_STARTING:       return GUI_ST_MORPH;
                case CONTROL_FAULT:              return GUI_ST_FAULT;
                default:                         return GUI_ST_CLOSED_LOOP;
            }
        default: return GUI_ST_IDLE;
    }
}

static uint16_t GspThrottleTelem(void)
{
    if (gspThrottleSrc == GSP_THR_SRC_GSP) return gspThrottle;
    float pot = pMC1Data->pMotorInputs->measurePot;         /* 0..4096 */
    if (pot < 0) pot = 0;
    return (uint16_t)(pot * 2000.0f / MAX_ADC_COUNT);
}

static void GspBuildSnapshot(uint8_t *buf)  /* buf[68] */
{
    MCAPP_CONTROL_SCHEME_T *c = pMC1Data->pControlScheme;
    MCAPP_MEASURE_T *m = pMC1Data->pMotorInputs;
    uint8_t *p = buf;

    /* eRPM from the estimator's mechanical speed */
    float rpm = c->estimator.calculateSpeed.speed;
    if (rpm < 0) rpm = -rpm;
    uint32_t erpm = (uint32_t)(rpm * (float)POLE_PAIRS);

    /* Host encodings: eRPM = 450000/stepPeriod (u16 fallback)
     *                 eRPM = 1e9/hwzcStepPeriodHR (preferred when hwzcEnabled) */
    uint16_t stepPeriod = 0;
    uint32_t stepHR = 0;
    if (erpm >= 7u) {
        uint32_t sp = 450000UL / erpm;
        stepPeriod = (sp > 0xFFFFu) ? 0xFFFFu : (uint16_t)sp;
        stepHR = 1000000000UL / erpm;
    }

    /* Ibus raw, 2048-centered 12-bit (GUI: amps = (raw-2048)/93).
     * measureCurrent.Ibus is (adc-2048)<<4 — undo the shift. */
    int16_t ibusQ = m->measureCurrent.Ibus;
    uint16_t ibusRaw = (uint16_t)(2048 + (ibusQ >> 4));
    if (ibusRaw > gspIbusMaxRaw) gspIbusMaxRaw = ibusRaw;

    uint16_t vbusRaw = (uint16_t)m->measureVdc.count;

    float neutral = c->estimator.reconstructedNeutralVoltage;
    if (neutral < 0) neutral = 0;

    bool synced = c->estimator.closedLoopFlag;

    GspPut8 (&p, GspGuiState());                              /*  0 state    */
    GspPut8 (&p, (uint8_t)c->faultStatus);                    /*  1 fault    */
    GspPut8 (&p, (uint8_t)c->commutationSector);              /*  2 step     */
    GspPut8 (&p, (uint8_t)c->directionCmd);                   /*  3 dir      */
    GspPut16(&p, GspThrottleTelem());                         /*  4 throttle */
    GspPut8 (&p, (uint8_t)(((uint64_t)c->pwmDuty * 100u) /
                           (c->pwmPeriod ? c->pwmPeriod : 1u)));/* 6 duty%   */
    GspPut8 (&p, 0);                                          /*  7 pad      */
    GspPut16(&p, vbusRaw);                                    /*  8 vbusRaw  */
    GspPut16(&p, ibusRaw);                                    /* 10 ibusRaw  */
    GspPut16(&p, gspIbusMaxRaw);                              /* 12 ibusMax  */
    GspPut16(&p, (uint16_t)m->measurePhaseVolt.Va);           /* 14 bemfRaw  */
    GspPut16(&p, (uint16_t)neutral);                          /* 16 zcThresh */
    GspPut16(&p, stepPeriod);                                 /* 18 stepPer  */
    GspPut16(&p, g_gspZcCount);                               /* 20 goodZc   */
    GspPut8 (&p, synced ? 1 : 0);                             /* 22 risingOk */
    GspPut8 (&p, synced ? 1 : 0);                             /* 23 fallingOk*/
    GspPut8 (&p, synced ? 1 : 0);                             /* 24 zcSynced */
    GspPut8 (&p, 0);                                          /* 25 pad      */
    GspPut16(&p, g_gspZcCount);                               /* 26 zcConf   */
    GspPut16(&p, 0);                                          /* 28 zcTimeout*/
    GspPut8 (&p, (erpm > 0) ? 1 : 0);                         /* 30 hwzcEn   */
    GspPut8 (&p, 0);                                          /* 31 hwzcPhase*/
    GspPut32(&p, 0);                                          /* 32 hwzcZc   */
    GspPut32(&p, 0);                                          /* 36 hwzcMiss */
    GspPut32(&p, stepHR);                                     /* 40 hwzcHR   */
    GspPut8 (&p, 0);                                          /* 44 latchDis */
    GspPut8 (&p, 0);                                          /* 45 pad      */
    GspPut8 (&p, c->estimator.closedLoopStartingFlag ? 1 : 0);/* 46 morphSub */
    GspPut8 (&p, 0);                                          /* 47 morphStep*/
    GspPut16(&p, 0);                                          /* 48 morphZc  */
    GspPut16(&p, 0);                                          /* 50 morphAlph*/
    GspPut32(&p, 0);                                          /* 52 clpciTrip*/
    GspPut32(&p, 0);                                          /* 56 fpciTrip */
    GspPut32(&p, gspMsTick);                                  /* 60 sysTick  */
    GspPut32(&p, gspMsTick / 1000u);                          /* 64 uptimeSec*/
}

#define GSP_SNAPSHOT_LEN 68

/* ── Command handlers ────────────────────────────────────────────────── */


/* ==== Live parameter system (garuda-gui ParamPanel) =====================
 * The GUI shows exactly the params this table lists (GET_PARAM_LIST drives
 * the panel), with names/units looked up by ID in gui/src/protocol/types.ts.
 * IDs 0xA0+ are reserved for this firmware (the Garuda AK firmware uses
 * 0x15..0x93, so the two name tables coexist in the GUI).
 * Values on the wire are u32; floats are scaled per the unit noted here. */

#define GSP_PTYPE_U8   0
#define GSP_PTYPE_U16  1
#define GSP_PTYPE_U32  2

typedef struct {
    uint16_t id;
    uint8_t  type;      /* GSP_PTYPE_* (GUI input sizing only)             */
    uint8_t  group;     /* GUI group label index                           */
    uint32_t min, max;
    bool     liveOk;    /* false = settable only in IDLE (CMD_WAIT/INIT)   */
    uint32_t (*get)(void);
    void     (*set)(uint32_t v);
} GSP_PARAM_DESC_T;

#define GSP_CTL   (pMC1Data->pControlScheme)

/* -- getters/setters (scaling noted per param) -- */
static uint32_t PgetLoopMode(void)      { return GSP_CTL->ctrlParam.controlLoop; }
static void     PsetLoopMode(uint32_t v){ GSP_CTL->ctrlParam.controlLoop = (MCAPP_CRTL_LOOP_T)v; }
static uint32_t PgetMaxRpm(void)        { return (uint32_t)GSP_CTL->motor.MaxSpeed; }
static void     PsetMaxRpm(uint32_t v)  { GSP_CTL->motor.MaxSpeed = (float)v; }
static uint32_t PgetRampRpmS(void)      { return (uint32_t)(GSP_CTL->ctrlParam.speedRampRate * 1000.0f); }
static void     PsetRampRpmS(uint32_t v){ GSP_CTL->ctrlParam.speedRampRate = (float)v / 1000.0f; }
static uint32_t PgetLockMs(void)        { return GSP_CTL->startup.rotorLockTimeLimit / GSP_ISR_PER_MS; }
static void     PsetLockMs(uint32_t v)  { GSP_CTL->startup.rotorLockTimeLimit = v * GSP_ISR_PER_MS; }
static uint32_t PgetLockMa(void)        { return (uint32_t)(GSP_CTL->startup.rotorLockCurrentRef * 1000.0f); }
static void     PsetLockMa(uint32_t v)  { GSP_CTL->startup.rotorLockCurrentRef = (float)v / 1000.0f; }
static uint32_t PgetStartMa(void)       { return (uint32_t)(GSP_CTL->startup.startupCurrent * 1000.0f); }
static void     PsetStartMa(uint32_t v) { GSP_CTL->startup.startupCurrent = (float)v / 1000.0f; }
static uint32_t PgetSpdKp(void)         { return (uint32_t)(GSP_CTL->piSpeed.param.kp * 1.0e6f + 0.5f); }
static void     PsetSpdKp(uint32_t v)   { GSP_CTL->piSpeed.param.kp = (float)v * 1.0e-6f; }
static uint32_t PgetSpdKi(void)         { return (uint32_t)(GSP_CTL->piSpeed.param.ki * 1.0e6f + 0.5f); }
static void     PsetSpdKi(uint32_t v)   { GSP_CTL->piSpeed.param.ki = (float)v * 1.0e-6f; }
static uint32_t PgetCurKp(void)         { return (uint32_t)(GSP_CTL->piCurrent.param.kp * 1.0e6f + 0.5f); }
static void     PsetCurKp(uint32_t v)   { GSP_CTL->piCurrent.param.kp = (float)v * 1.0e-6f; }
static uint32_t PgetCurKi(void)         { return (uint32_t)(GSP_CTL->piCurrent.param.ki * 1.0e6f + 0.5f); }
static void     PsetCurKi(uint32_t v)   { GSP_CTL->piCurrent.param.ki = (float)v * 1.0e-6f; }
static uint32_t PgetClBlank(void)       { return GSP_CTL->estimator.closedLoopBlankingTime; }
static void     PsetClBlank(uint32_t v) { GSP_CTL->estimator.closedLoopBlankingTime = (uint16_t)v; }
static uint32_t PgetOlBlank(void)       { return GSP_CTL->estimator.startupBlankingTime; }
static void     PsetOlBlank(uint32_t v) { GSP_CTL->estimator.startupBlankingTime = (uint16_t)v; }
static uint32_t PgetDir(void)           { return directionCmdMC1; }
static void     PsetDir(uint32_t v)     { directionCmdMC1 = (uint16_t)v; }
static uint32_t PgetPP(void)            { return POLE_PAIRS; }
static void     PsetPP(uint32_t v)      { (void)v; }

/* Groups reuse the GUI's existing labels:
 * 0 'Startup & Ramp', 1 'Closed-Loop Control', 3 'ZC Detection',
 * 7 'Motor Hardware' */
static const GSP_PARAM_DESC_T gspParams[] = {
 /*  id    type          grp  min   max         live   get          set        */
 { 0xA0, GSP_PTYPE_U8 ,  1,   1,    3,          false, PgetLoopMode, PsetLoopMode },
 { 0xA1, GSP_PTYPE_U32,  1,   500,  60000,      false, PgetMaxRpm,   PsetMaxRpm   },
 { 0xA2, GSP_PTYPE_U16,  0,   50,   60000,      true,  PgetRampRpmS, PsetRampRpmS },
 { 0xA3, GSP_PTYPE_U16,  0,   50,   5000,       false, PgetLockMs,   PsetLockMs   },
 { 0xA4, GSP_PTYPE_U16,  0,   100,  10000,      false, PgetLockMa,   PsetLockMa   },
 { 0xA5, GSP_PTYPE_U16,  0,   100,  20000,      false, PgetStartMa,  PsetStartMa  },
 { 0xA6, GSP_PTYPE_U32,  1,   1,    1000000000, true,  PgetSpdKp,    PsetSpdKp    },
 { 0xA7, GSP_PTYPE_U32,  1,   1,    1000000000, true,  PgetSpdKi,    PsetSpdKi    },
 { 0xA8, GSP_PTYPE_U32,  1,   1,    1000000000, true,  PgetCurKp,    PsetCurKp    },
 { 0xA9, GSP_PTYPE_U32,  1,   1,    1000000000, true,  PgetCurKi,    PsetCurKi    },
 { 0xAA, GSP_PTYPE_U16,  3,   1,    2000,       true,  PgetClBlank,  PsetClBlank  },
 { 0xAB, GSP_PTYPE_U16,  3,   1,    20000,      false, PgetOlBlank,  PsetOlBlank  },
 { 0xAC, GSP_PTYPE_U8 ,  7,   0,    1,          false, PgetDir,      PsetDir      },
 { 0xAD, GSP_PTYPE_U8 ,  7,   POLE_PAIRS, POLE_PAIRS, false, PgetPP, PsetPP       },
};
#define GSP_PARAM_COUNT (sizeof(gspParams)/sizeof(gspParams[0]))

static const GSP_PARAM_DESC_T *GspFindParam(uint16_t id)
{
    for (uint8_t i = 0; i < GSP_PARAM_COUNT; i++)
        if (gspParams[i].id == id) return &gspParams[i];
    return NULL;
}

static bool GspIsIdle(void)
{
    uint16_t st = MC1_GspAppState();
    return (st == MCAPP_CMD_WAIT) || (st == MCAPP_INIT);
}

static void GspHandleGetParamList(const uint8_t *payload, uint8_t len)
{
    uint8_t start = (len >= 1) ? payload[0] : 0;
    if (start > GSP_PARAM_COUNT) start = GSP_PARAM_COUNT;

    uint8_t remain = (uint8_t)(GSP_PARAM_COUNT - start);
    uint8_t count  = (remain > 20) ? 20 : remain;   /* 3+20*12 = 243 <= 249 */

    uint8_t buf[3 + 20 * 12];
    uint8_t *p = buf;
    GspPut8(&p, (uint8_t)GSP_PARAM_COUNT);
    GspPut8(&p, start);
    GspPut8(&p, count);
    for (uint8_t i = 0; i < count; i++) {
        const GSP_PARAM_DESC_T *d = &gspParams[start + i];
        GspPut16(&p, d->id);
        GspPut8 (&p, d->type);
        GspPut8 (&p, d->group);
        GspPut32(&p, d->min);
        GspPut32(&p, d->max);
    }
    GSP_SendResponse(GSP_CMD_GET_PARAM_LIST, buf, (uint8_t)(p - buf));
}

static void GspRespondParamValue(uint8_t cmdId, const GSP_PARAM_DESC_T *d)
{
    uint8_t buf[6];
    uint8_t *p = buf;
    GspPut16(&p, d->id);
    GspPut32(&p, d->get());
    GSP_SendResponse(cmdId, buf, sizeof(buf));
}

static void GspHandleGetParam(const uint8_t *payload, uint8_t len)
{
    if (len < 2) { GspSendError(GSP_ERR_BAD_LENGTH); return; }
    uint16_t id = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    const GSP_PARAM_DESC_T *d = GspFindParam(id);
    if (!d) { GspSendError(GSP_ERR_UNKNOWN_PARAM); return; }
    GspRespondParamValue(GSP_CMD_GET_PARAM, d);
}

static void GspHandleSetParam(const uint8_t *payload, uint8_t len)
{
    if (len < 6) { GspSendError(GSP_ERR_BAD_LENGTH); return; }
    uint16_t id = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    uint32_t v  = (uint32_t)payload[2] | ((uint32_t)payload[3] << 8)
                | ((uint32_t)payload[4] << 16) | ((uint32_t)payload[5] << 24);

    const GSP_PARAM_DESC_T *d = GspFindParam(id);
    if (!d)                       { GspSendError(GSP_ERR_UNKNOWN_PARAM); return; }
    if (v < d->min || v > d->max) { GspSendError(GSP_ERR_OUT_OF_RANGE);  return; }
    if (!d->liveOk && !GspIsIdle()){ GspSendError(GSP_ERR_WRONG_STATE);  return; }

    d->set(v);
    GspRespondParamValue(GSP_CMD_SET_PARAM, d);
}

static void GspHandleGetInfo(void)
{
    uint8_t buf[24];
    uint8_t *p = buf;
    GspPut8 (&p, 1);                        /* protocolVersion             */
    GspPut8 (&p, 0);                        /* fwMajor                     */
    GspPut8 (&p, 9);                        /* fwMinor                     */
    GspPut8 (&p, 0);                        /* fwPatch                     */
    GspPut16(&p, GSP_BOARD_MCLV48V300W);    /* boardId (AK decode path)    */
    GspPut8 (&p, 0);                        /* motorProfile                */
    GspPut8 (&p, POLE_PAIRS);               /* polePairs                   */
    GspPut32(&p, 0x00090001UL);             /* featureFlags: BEMF_CL|GSP|ADC_POT */
    GspPut32(&p, PWMFREQUENCY_HZ);          /* pwmFrequency                */
    GspPut32(&p, (uint32_t)(MAXIMUM_SPEED_RPM * (float)POLE_PAIRS)); /* maxErpm */
    GspPut32(&p, 0x48535053UL);             /* buildHash ("HSPS")          */
    GSP_SendResponse(GSP_CMD_GET_INFO, buf, sizeof(buf));
}

static void GspDispatch(uint8_t cmdId, const uint8_t *payload, uint8_t len)
{
    switch (cmdId) {

    case GSP_CMD_PING:
        GSP_SendResponse(GSP_CMD_PING, NULL, 0);
        break;

    case GSP_CMD_GET_INFO:
        GspHandleGetInfo();
        break;

    case GSP_CMD_GET_SNAPSHOT: {
        uint8_t snap[GSP_SNAPSHOT_LEN];
        GspBuildSnapshot(snap);
        GSP_SendResponse(GSP_CMD_GET_SNAPSHOT, snap, sizeof(snap));
        break;
    }

    case GSP_CMD_START_MOTOR:
        if (MC1_GspAppState() != MCAPP_CMD_WAIT) { GspSendError(GSP_ERR_WRONG_STATE); break; }
        gspIbusMaxRaw = 2048;
        runCmdMC1 = 1;
        GSP_SendResponse(GSP_CMD_START_MOTOR, NULL, 0);
        break;

    case GSP_CMD_STOP_MOTOR:
        runCmdMC1 = 0;
        GSP_SendResponse(GSP_CMD_STOP_MOTOR, NULL, 0);
        break;

    case GSP_CMD_CLEAR_FAULT:
        if (MC1_GspAppState() != MCAPP_FAULT) { GspSendError(GSP_ERR_WRONG_STATE); break; }
        MC1_GspClearFault();
        GSP_SendResponse(GSP_CMD_CLEAR_FAULT, NULL, 0);
        break;

    case GSP_CMD_SET_THROTTLE: {
        if (len < 2) { GspSendError(GSP_ERR_BAD_LENGTH); break; }
        if (gspThrottleSrc != GSP_THR_SRC_GSP) { GspSendError(GSP_ERR_WRONG_STATE); break; }
        uint16_t v = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
        if (v > 2000) { GspSendError(GSP_ERR_OUT_OF_RANGE); break; }
        gspThrottle = v;
        GSP_SendResponse(GSP_CMD_SET_THROTTLE, NULL, 0);
        break;
    }

    case GSP_CMD_SET_THROTTLE_SRC: {
        if (len < 1) { GspSendError(GSP_ERR_BAD_LENGTH); break; }
        if (MC1_GspAppState() != MCAPP_CMD_WAIT &&
            MC1_GspAppState() != MCAPP_INIT) { GspSendError(GSP_ERR_WRONG_STATE); break; }
        if (payload[0] == GSP_THR_SRC_ADC) {
            gspThrottleSrc = GSP_THR_SRC_ADC;
        } else if (payload[0] == GSP_THR_SRC_GSP) {
            gspThrottleSrc = GSP_THR_SRC_GSP;
            gspThrottle = 0;
            gspLastPacketMs = gspMsTick;
        } else { GspSendError(GSP_ERR_OUT_OF_RANGE); break; }
        GSP_SendResponse(GSP_CMD_SET_THROTTLE_SRC, NULL, 0);
        break;
    }

    case GSP_CMD_HEARTBEAT:
        GSP_SendResponse(GSP_CMD_HEARTBEAT, NULL, 0);
        break;

    case GSP_CMD_TELEM_START: {
        uint8_t rate = (len >= 1) ? payload[0] : 20;
        if (rate < 10)  rate = 10;
        if (rate > 100) rate = 100;
        gspTelemIntervalMs = (uint16_t)(1000u / rate);
        gspTelemOn = true;
        gspTelemLastMs = gspMsTick;
        gspTelemSeq = 0;
        uint8_t actual = (uint8_t)(1000u / gspTelemIntervalMs);
        GSP_SendResponse(GSP_CMD_TELEM_START, &actual, 1);
        break;
    }

    case GSP_CMD_TELEM_STOP:
        gspTelemOn = false;
        GSP_SendResponse(GSP_CMD_TELEM_STOP, NULL, 0);
        break;

    case GSP_CMD_GET_PARAM_LIST:
        GspHandleGetParamList(payload, len);
        break;

    case GSP_CMD_GET_PARAM:
        GspHandleGetParam(payload, len);
        break;

    case GSP_CMD_SET_PARAM:
        GspHandleSetParam(payload, len);
        break;

    default:
        GspSendError(GSP_ERR_UNKNOWN_CMD);
        break;
    }
}

/* ── Telemetry streaming ─────────────────────────────────────────────── */

static void GspTelemTick(void)
{
    if (!gspTelemOn) return;
    uint32_t now = gspMsTick;
    if ((now - gspTelemLastMs) < gspTelemIntervalMs) return;
    gspTelemLastMs = now;
    gspTelemSeq++;

    uint8_t buf[2 + GSP_SNAPSHOT_LEN];
    buf[0] = (uint8_t)(gspTelemSeq & 0xFF);
    buf[1] = (uint8_t)(gspTelemSeq >> 8);
    GspBuildSnapshot(&buf[2]);
    GSP_SendResponse(GSP_CMD_TELEM_FRAME, buf, sizeof(buf));
}

/* ── UART pumps + parser ─────────────────────────────────────────────── */

static void GspPumpRx(void)
{
    if (UART1_IsReceiveBufferOverFlowDetected())
        UART1_ReceiveBufferOverrunErrorFlagClear();

    while (UART1_IsReceiveBufferDataReady()) {
        uint8_t b = (uint8_t)UART1_DataRead();
        if (!GspRxFull()) GspRxPut(b);
    }
}

static void GspPumpTx(void)
{
    while (GspTxCount() > 0 && !UART1_StatusBufferFullTransmitGet())
        UART1_DataWrite(GspTxGet());
}

static void GspParserReset(void)
{
    gspParser.state = GSP_WAIT_START;
    gspParser.pktIdx = 0;
    gspParser.pktLen = 0;
    gspParser.crcIdx = 0;
    gspParser.collectingCrc = false;
}

static void GspParserProcess(void)
{
    uint8_t done = 0;

    if (gspParser.state != GSP_WAIT_START &&
        (gspMsTick - gspParser.lastRxTick) > GSP_PARSER_TIMEOUT_MS)
        GspParserReset();

    while (GspRxCount() > 0 && done < 1) {
        uint8_t b = GspRxGet();
        gspParser.lastRxTick = gspMsTick;

        switch (gspParser.state) {
        case GSP_WAIT_START:
            if (b == GSP_START_BYTE) gspParser.state = GSP_GOT_START;
            break;

        case GSP_GOT_START:
            if (b >= 1 && b <= GSP_MAX_PAYLOAD_LEN) {
                gspParser.pktLen = b;
                gspParser.pktIdx = 0;
                gspParser.crcIdx = 0;
                gspParser.collectingCrc = false;
                gspParser.state = GSP_COLLECTING;
            } else {
                GspParserReset();
            }
            break;

        case GSP_COLLECTING:
            if (!gspParser.collectingCrc) {
                gspParser.pktBuf[gspParser.pktIdx++] = b;
                if (gspParser.pktIdx >= gspParser.pktLen) {
                    gspParser.collectingCrc = true;
                    gspParser.crcIdx = 0;
                }
            } else {
                gspParser.crcBuf[gspParser.crcIdx++] = b;
                if (gspParser.crcIdx >= 2) {
                    uint16_t crc = GspCrc16(0xFFFF, gspParser.pktLen);
                    for (uint8_t i = 0; i < gspParser.pktLen; i++)
                        crc = GspCrc16(crc, gspParser.pktBuf[i]);
                    uint16_t rx = ((uint16_t)gspParser.crcBuf[0] << 8)
                                | gspParser.crcBuf[1];
                    if (crc == rx) {
                        gspLastPacketMs = gspMsTick;
                        uint8_t cmdId = gspParser.pktBuf[0];
                        uint8_t plen  = (uint8_t)(gspParser.pktLen - 1);
                        GspDispatch(cmdId,
                                    (plen > 0) ? &gspParser.pktBuf[1] : NULL,
                                    plen);
                        done++;
                    }
                    GspParserReset();
                }
            }
            break;
        }
    }
}

/* ── Public API ──────────────────────────────────────────────────────── */

void GSP_IsrTick(void)
{
    if (++gspIsrDiv >= GSP_ISR_PER_MS) {
        gspIsrDiv = 0;
        gspMsTick++;
    }
}

float GSP_ThrottleTarget(float potValue)
{
    if (gspThrottleSrc != GSP_THR_SRC_GSP)
        return potValue;

    /* GSP failsafe: host silent too long while running -> throttle to zero */
    if ((gspMsTick - gspLastPacketMs) > GSP_FAILSAFE_MS)
        return 0.0f;

    return ((float)gspThrottle * MAX_ADC_COUNT) / 2000.0f;
}

void GSP_Init(void)
{
    memset(&gspRx, 0, sizeof(gspRx));
    memset(&gspTx, 0, sizeof(gspTx));
    GspParserReset();
    gspParser.lastRxTick = 0;

    UART1_InterruptReceiveDisable();
    UART1_InterruptReceiveFlagClear();
    UART1_InterruptTransmitDisable();
    UART1_InterruptTransmitFlagClear();
    UART1_Initialize();
    UART1_BaudRateDividerSet(GSP_BAUDRATE_DIVIDER);
    UART1_SpeedModeStandard();
    UART1_ModuleEnable();
}

void GSP_Service(void)
{
    GspPumpRx();
    GspParserProcess();
    GspTelemTick();
    GspPumpTx();
}

#endif /* ENABLE_GSP && GSP_AK_IMPLEMENTATION */
