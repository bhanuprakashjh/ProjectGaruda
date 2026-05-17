/**
 * @file gsp_commands.c
 * @brief GSP v2 command handlers — info, motor control, telemetry.
 */

#include "../garuda_config.h"

#if FEATURE_GSP
#include <stdint.h>
#include <string.h>
#include "gsp_commands.h"
#include "gsp.h"
#include "../garuda_service.h"
#include "../garuda_config.h"
#include "../motor/sector_pi.h"
#include "../motor/motor_params.h"
#include "../hal/hal_ata6847.h"
#include "../hal/hal_capture.h"
#include "../hal/port_config.h"

extern volatile ESC_STATE_T gEscState;
extern volatile uint32_t gSystemTick;
extern volatile uint16_t gPotRaw;
extern volatile uint16_t gVbusRaw;
extern volatile int16_t  gIaRaw;
extern volatile int16_t  gIbRaw;

static bool     telemActive = false;
static uint16_t telemSeq = 0;
static uint32_t telemLastTick = 0;

void GSP_DispatchCommand(uint8_t cmdId, const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload; (void)payloadLen;

    switch (cmdId)
    {
        case GSP_CMD_PING:
            GSP_FlushTx();
            GSP_SendResponse(GSP_CMD_PING, NULL, 0);
            break;

        case GSP_CMD_GET_INFO:
        {
            /* GUI's decodeInfo expects 20 bytes (matches GSP_INFO_T):
             *   [0]    protocolVersion (u8)
             *   [1..3] fwMajor, fwMinor, fwPatch (u8 each)
             *   [4..5] boardId (u16 LE) — 0x0002 (was CK board, retained for GUI compat)
             *   [6]    motorProfile (u8)
             *   [7]    motorPolePairs (u8)
             *   [8..11]  featureFlags (u32 LE)
             *   [12..15] pwmFrequency (u32 LE)
             *   [16..19] maxErpm (u32 LE)
             * Sending fewer bytes makes the GUI's DataView throw, which
             * kills the read loop → disconnect → writes fail. */
            uint8_t info[20];
            memset(info, 0, sizeof(info));
            info[0] = GSP_PROTOCOL_VERSION;
            info[1] = 4;                    /* fwMajor */
            info[2] = 0;                    /* fwMinor */
            info[3] = 0;                    /* fwPatch */
            uint16_t boardId = GSP_BOARD_EV43F54A;
            memcpy(&info[4], &boardId, 2);
            info[6] = MOTOR_PROFILE;
            info[7] = 7;                    /* polePairs (placeholder) */
            uint32_t f = 0x80000000UL;      /* bit 31 = sector-PI marker */
            memcpy(&info[8],  &f, 4);
            uint32_t pwmHz = PWMFREQUENCY_HZ;
            memcpy(&info[12], &pwmHz, 4);
            uint32_t maxErpm = 200000UL;
            memcpy(&info[16], &maxErpm, 4);
            GSP_SendResponse(GSP_CMD_GET_INFO, info, 20);
            break;
        }

        case GSP_CMD_GET_SNAPSHOT:
        case GSP_CMD_TELEM_START:
        case GSP_CMD_TELEM_STOP:
            /* Handled below in TelemTick */
            if (cmdId == GSP_CMD_TELEM_START)
                telemActive = true;
            else if (cmdId == GSP_CMD_TELEM_STOP)
                telemActive = false;
            GSP_SendResponse(cmdId, NULL, 0);
            break;

        case GSP_CMD_START_MOTOR:
            if (gEscState == ESC_IDLE)
                GarudaService_StartMotor();
            GSP_SendResponse(GSP_CMD_START_MOTOR, NULL, 0);
            break;

        case GSP_CMD_STOP_MOTOR:
            GarudaService_StopMotor();
            GSP_SendResponse(GSP_CMD_STOP_MOTOR, NULL, 0);
            break;

        case GSP_CMD_CLEAR_FAULT:
            GarudaService_ClearFault();
            GSP_SendResponse(GSP_CMD_CLEAR_FAULT, NULL, 0);
            break;

        case GSP_CMD_HEARTBEAT:
            /* GUI sends HEARTBEAT every 200ms while telemetry is active
             * to keep the connection alive. firmware has no watchdog action —
             * just acknowledge so the GUI doesn't spam unknown-command
             * error toasts. */
            GSP_SendResponse(GSP_CMD_HEARTBEAT, NULL, 0);
            break;

        case GSP_CMD_SET_THROTTLE:
            /* firmware reads pot directly via ADC; ignore GUI throttle commands
             * silently rather than spam unknown-command errors. */
            GSP_SendResponse(GSP_CMD_SET_THROTTLE, NULL, 0);
            break;

        case GSP_CMD_GET_PARAM:
        {
            /* Payload: 2-byte param ID (LE).
             * Response: 6 bytes — id (u16) + value (u32). */
            if (payloadLen < 2) {
                uint8_t err = GSP_ERR_BAD_LENGTH;
                GSP_SendResponse(GSP_CMD_ERROR, &err, 1);
                break;
            }
            uint16_t id = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
            bool ok;
            uint32_t val = Params_Get(id, &ok);
            if (!ok) {
                uint8_t err = GSP_ERR_UNKNOWN_PARAM;
                GSP_SendResponse(GSP_CMD_ERROR, &err, 1);
                break;
            }
            uint8_t resp[6];
            resp[0] = (uint8_t)(id & 0xFF);
            resp[1] = (uint8_t)(id >> 8);
            resp[2] = (uint8_t)(val & 0xFF);
            resp[3] = (uint8_t)((val >> 8) & 0xFF);
            resp[4] = (uint8_t)((val >> 16) & 0xFF);
            resp[5] = (uint8_t)((val >> 24) & 0xFF);
            GSP_SendResponse(GSP_CMD_GET_PARAM, resp, 6);
            break;
        }

        case GSP_CMD_SET_PARAM:
        {
            /* Payload: 2-byte ID + 4-byte value (LE).
             * HOT params are change-while-running — no IDLE check.
             * Response: echo the 6-byte payload on success. */
            if (payloadLen < 6) {
                uint8_t err = GSP_ERR_BAD_LENGTH;
                GSP_SendResponse(GSP_CMD_ERROR, &err, 1);
                break;
            }
            uint16_t id = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
            uint32_t val = (uint32_t)payload[2]
                         | ((uint32_t)payload[3] << 8)
                         | ((uint32_t)payload[4] << 16)
                         | ((uint32_t)payload[5] << 24);
            if (!Params_Set(id, val)) {
                uint8_t err = GSP_ERR_OUT_OF_RANGE;
                GSP_SendResponse(GSP_CMD_ERROR, &err, 1);
                break;
            }
            GSP_SendResponse(GSP_CMD_SET_PARAM, payload, 6);
            break;
        }

        case GSP_CMD_GET_PARAM_LIST:
        {
            /* Paginated descriptor table. Payload: 1 byte = startIndex.
             * Response: [totalCount, startIndex, pageSize, entries...]
             * Each entry is 12 bytes: id(u16), type(u8), group(u8), min(u32), max(u32).
             * Page sized to fit GSP max payload (249 - 3 header = 246, /12 = 20). */
            uint8_t startIndex = (payloadLen >= 1) ? payload[0] : 0;
            uint8_t totalCount;
            const PARAM_DESC_T *table = Params_GetDescriptorTable(&totalCount);

            if (startIndex >= totalCount) {
                /* Tell GUI "no more pages" by returning empty page header
                 * rather than error — matches V3 convention from V3 code. */
                uint8_t hdr[3] = { totalCount, startIndex, 0 };
                GSP_SendResponse(GSP_CMD_GET_PARAM_LIST, hdr, 3);
                break;
            }
            uint8_t remaining = totalCount - startIndex;
            uint8_t pageSize = (remaining > 20) ? 20 : remaining;

            uint8_t buf[3 + 20 * 12];
            buf[0] = totalCount;
            buf[1] = startIndex;
            buf[2] = pageSize;
            uint8_t i;
            for (i = 0; i < pageSize; i++) {
                const PARAM_DESC_T *d = &table[startIndex + i];
                uint8_t off = 3 + i * 12;
                buf[off + 0]  = (uint8_t)(d->id & 0xFF);
                buf[off + 1]  = (uint8_t)(d->id >> 8);
                buf[off + 2]  = d->type;
                buf[off + 3]  = d->group;
                buf[off + 4]  = (uint8_t)(d->min & 0xFF);
                buf[off + 5]  = (uint8_t)((d->min >> 8) & 0xFF);
                buf[off + 6]  = (uint8_t)((d->min >> 16) & 0xFF);
                buf[off + 7]  = (uint8_t)((d->min >> 24) & 0xFF);
                buf[off + 8]  = (uint8_t)(d->max & 0xFF);
                buf[off + 9]  = (uint8_t)((d->max >> 8) & 0xFF);
                buf[off + 10] = (uint8_t)((d->max >> 16) & 0xFF);
                buf[off + 11] = (uint8_t)((d->max >> 24) & 0xFF);
            }
            GSP_SendResponse(GSP_CMD_GET_PARAM_LIST, buf, 3 + pageSize * 12);
            break;
        }

        case GSP_CMD_SAVE_CONFIG:
            /* Phase B will wire EEPROM. For now ack so GUI doesn't error. */
            GSP_SendResponse(GSP_CMD_SAVE_CONFIG, NULL, 0);
            break;

        case GSP_CMD_LOAD_DEFAULTS:
            Params_InitDefaults();
            GSP_SendResponse(GSP_CMD_LOAD_DEFAULTS, NULL, 0);
            break;

        case GSP_CMD_PI_LOG:
        {
            /* Diagnostic dump of first PI runs after CL entry.
             * Response: [count][30×8B entries] — total <=241 bytes. */
            uint8_t resp[1 + 30 * 8];
            uint8_t n = 0;
            SectorPI_GetCaptureLog(&resp[1], &n);
            resp[0] = n;
            GSP_SendResponse(GSP_CMD_PI_LOG, resp, (uint8_t)(1U + (uint16_t)n * 8U));
            break;
        }

        case GSP_CMD_BEMF_PROBE:
        {
            /* Real-time BEMF GPIO read + sector context. Safe in all states;
             * use to diagnose phase-wiring and comparator polarity by hand-
             * rotating the motor and watching the three BEMF lines toggle. */
            extern volatile uint8_t floatingPhase;
            extern volatile uint8_t ptgExpectedComp;
            uint8_t probe[7];
            probe[0] = BEMF_A_GetValue();
            probe[1] = BEMF_B_GetValue();
            probe[2] = BEMF_C_GetValue();
            probe[3] = floatingPhase;
            probe[4] = ptgExpectedComp;
            probe[5] = (uint8_t)SectorPI_GetPhase();
            probe[6] = HAL_Capture_IsRisingZc() ? 1u : 0u;
            GSP_SendResponse(GSP_CMD_BEMF_PROBE, probe, sizeof(probe));
            break;
        }

        case GSP_CMD_ATA_DIAG:
        {
            /* Bring-up only: 8 raw register bytes from the ATA6847L.
             * Order matches HAL_ATA6847_ReadDiag():
             *   [0] DSR1   (GDU status — GDUS bit 2 = ready)
             *   [1] DSR2
             *   [2] SIR1   (fault summary — latched)
             *   [3] SIR2
             *   [4] SIR3
             *   [5] SIR4
             *   [6] SIR5
             *   [7] GOPMCR (current operating mode)
             * Skip while motor is running (SPI shared with start-path). */
            extern volatile uint8_t  gAta_LastDsr1AtNormal;
            extern volatile uint16_t gAta_LastGduAttempts;
            extern volatile uint8_t  gAta_LastGduResult;

            uint8_t diag[12];
            if (gEscState != ESC_IDLE && gEscState != ESC_FAULT) {
                uint8_t err = GSP_ERR_WRONG_STATE;
                GSP_SendResponse(GSP_CMD_ERROR, &err, 1);
            } else {
                HAL_ATA6847_ReadDiag(diag);
                /* Append bring-up instrumentation: what EnterGduNormal saw */
                diag[8]  = gAta_LastDsr1AtNormal;
                diag[9]  = (uint8_t)(gAta_LastGduAttempts & 0xFF);
                diag[10] = (uint8_t)(gAta_LastGduAttempts >> 8);
                diag[11] = gAta_LastGduResult;
                GSP_SendResponse(GSP_CMD_ATA_DIAG, diag, sizeof(diag));
            }
            break;
        }

        default:
        {
            uint8_t err = GSP_ERR_UNKNOWN_CMD;
            GSP_SendResponse(GSP_CMD_ERROR, &err, 1);
            break;
        }
    }
}

void GSP_TelemTick(void)
{
    if (!telemActive) return;

    /* Rate limit: ~10 Hz (every 100ms) */
    uint32_t now = gSystemTick;
    if ((now - telemLastTick) < 100) return;
    telemLastTick = now;

    /* 252-byte telemetry snapshot consumed by pot_capture.py / GUI.
     * Wire layout (offsets are into `snap[]`, not `d[]`):
     *   snap[0..1]   = seq
     *   snap[2..249] = d[0..247] — fixed-offset diagnostic fields below
     *   snap[250..251] = fpStaleCount (d[248..249])
     * snap[252] would be OOB — keep this array sized strictly. */
    TELEM_T t;
    SectorPI_TelemGet(&t);

    uint8_t snap[252];
    memset(snap, 0, sizeof(snap));

    /* Seq counter (2 bytes) */
    snap[0] = (uint8_t)(telemSeq & 0xFF);
    snap[1] = (uint8_t)(telemSeq >> 8);
    telemSeq++;

    uint8_t *d = &snap[2];  /* snapshot starts at offset 2 */

    /* Core state (offset 0-7) */
    d[0] = (uint8_t)gEscState;               /* state */
    d[1] = 0;                                /* fault */
    d[2] = t.position;                       /* step */
    d[3] = 0;                                /* ataStatus */
    d[4] = (uint8_t)(gPotRaw & 0xFF);     /* potRaw L */
    d[5] = (uint8_t)(gPotRaw >> 8);       /* potRaw H */
    /* Duty% from the value the hardware actually wrote, not the upstream
     * commanded amplitude. g_pwmActualDuty is the post-clamp PG[123]DC
     * value and g_pwmPer is its denominator (LOOPTIME_TCY in normal mode,
     * sector-period in SP). Was previously `actualAmplitude * 100 / 32768`
     * which always showed 99% even when the per-200 (now per-100) clamp
     * pinned the gates at ~94-97%. */
    extern volatile uint16_t g_pwmActualDuty;
    extern volatile uint16_t g_pwmPer;
    extern volatile bool     g_blockCommActive;
    uint8_t dutyPct = 0;
    if (g_blockCommActive)
        dutyPct = 100;                          /* override path → solid ON */
    else if (g_pwmPer > 0U && t.actualAmplitude > 0U)
        dutyPct = (uint8_t)((uint32_t)g_pwmActualDuty * 100UL / g_pwmPer);
    d[6] = dutyPct;                          /* dutyPct */
    d[7] = t.commandEnabled ? 1 : 0;        /* zcSynced (repurpose as PI active) */

    /* Electrical (offset 8-17) — calibrated physical units.
     *   d[8..9]   vbus_mV   uint16   (0 .. ~52 V)
     *   d[10..11] ia_mA     int16    signed phase A current
     *   d[12..13] ib_mA     int16    signed phase B current
     *   d[14..15] ibus_mA   int16    signed DC bus current (direct from OA3)
     * All conversions happen in the ADC ISR using the Q8 constants in
     * garuda_config.h.  The host only divides by 1000 to display volts/amps;
     * it doesn't know the shunt, gain, or divider values. */
    {
        extern volatile uint16_t gVbus_mV;
        extern volatile int16_t  gIa_mA, gIb_mA, gIbus_mA;
        memcpy(&d[8],  &gVbus_mV, 2);
        memcpy(&d[10], &gIa_mA,   2);
        memcpy(&d[12], &gIb_mA,   2);
        memcpy(&d[14], &gIbus_mA, 2);
    }
    memcpy(&d[16], &t.actualAmplitude, 2);   /* duty (raw) */

    /* Speed/Timing (offset 18-31) */
    memcpy(&d[18], &t.timerPeriod, 2);       /* stepPeriod → timerPeriod */
    memcpy(&d[20], &t.timerPeriod, 2);       /* stepPeriodHR → same */
    uint32_t eRpm = SectorPI_ErpmGet();
    memcpy(&d[22], &eRpm, 4);               /* eRpm */
    memcpy(&d[26], &t.sectorCount, 2);      /* goodZc → sectorCount low */
    /* zcInterval, prevZcInterval = 0 */

    /* ZC diagnostics (offset 28-39) — sector-PI diagnostics */
    memcpy(&d[28], &t.diagLastCapValue, 2);  /* zcInterval → lastCapValue */
    memcpy(&d[30], &t.diagDelta, 2);         /* prevZcInterval → PI delta (signed) */
    memcpy(&d[32], &t.diagCaptures, 2);      /* icAccepted → captures accepted */
    memcpy(&d[34], &t.diagPiRuns, 2);        /* icFalse → PI run count */
    d[36] = t.stallCounter;                  /* filterLevel → stallCounter */
    /* Pack both SP bits: bit0 = active, bit1 = request.
     * 0 = no request, not active
     * 2 = request latched but SP not yet applied (boundary lag / error)
     * 3 = SP active and requested
     * 1 = stale/impossible active state */
    d[37] = (t.spMode ? 1U : 0U) | (t.spRequest ? 2U : 0U) | (g_blockCommActive ? 4U : 0U);
    { uint16_t erpm16 = (t.erpmNow > 0xFFFF) ? 0xFFFF : (uint16_t)t.erpmNow;
      memcpy(&d[38], &erpm16, 2); }          /* erpmNow from timerPeriod */

    /* System (offset 40-47) */
    memcpy(&d[40], &gSystemTick, 4);       /* systemTick */
    uint32_t uptime = gSystemTick / 1000;
    memcpy(&d[44], &uptime, 4);              /* uptime */

    /* Capture-rate diagnostics (offset 48-63). 32-bit because the BEMF
     * ISR fires at ~60 kHz and uint16 would wrap every ~1.1 s.
     *   adcBlankReject  : sample fired pre-blanking-end
     *   adcStateMismatch: past blanking, GPIO != expected post-ZC
     *   adcCaptureSet   : total captures (all 6 sectors)
     *   adcSetRising    : subset on rising-ZC sectors (0,2,4)
     * Wire slots d[64]/d[68] kept as zero pads for layout stability. */
    memcpy(&d[48], &t.adcBlankReject,   4);
    memcpy(&d[52], &t.adcStateMismatch, 4);
    memcpy(&d[56], &t.adcCaptureSet,    4);
    memcpy(&d[60], &t.adcSetRising,     4);
    { uint32_t zero = 0; memcpy(&d[64], &zero, 4); memcpy(&d[68], &zero, 4); }
    memcpy(&d[72], &t.ptgFires,         4);  /* PTG ISR fire count */
    memcpy(&d[76], &t.ptgRisingAcc,     4);  /* per-polarity shadow (unused) */
    memcpy(&d[80], &t.ptgRisingRej,     4);
    memcpy(&d[84], &t.ptgFallingAcc,    4);
    memcpy(&d[88], &t.ptgFallingRej,    4);

    /* Per-polarity PI-feed accounting (host computes pR/pF % from these).
     *   d[92]  = diagPiFedRising  (rising sector accepted into PI)
     *   d[96]  = diagPiMissRising (rising sector dropped/no-capture)
     *   d[100] = diagPiFedFalling (falling sector accepted into PI)
     *   d[104] = diagPiMissFalling(falling sector dropped/no-capture) */
    {
        extern volatile uint32_t diagPiFedRising, diagPiFedFalling;
        extern volatile uint32_t diagPiMissRising, diagPiMissFalling;
        memcpy(&d[92],  &diagPiFedRising,   4);
        memcpy(&d[96],  &diagPiMissRising,  4);
        memcpy(&d[100], &diagPiFedFalling,  4);
        memcpy(&d[104], &diagPiMissFalling, 4);
    }

    /* Measurement-PI tracker (smoothed tMeasHR). */
    memcpy(&d[108], &t.tMeasHR, 2);

    /* Full 32-bit sectorCount for host-side rate computation. */
    memcpy(&d[110], &t.sectorCount, 4);

    /* Phase + bus current peaks. Rolling window reset on snapshot read.
     * At-fault fields preserved across snapshots until motor restart
     * (valid==1). ibus reads OA3 shunt directly. */
    extern volatile int16_t gIaPkMax, gIaPkMin;
    extern volatile int16_t gIbPkMax, gIbPkMin;
    extern volatile int16_t gIbusPkMax, gIbusPkMin;
    extern volatile int16_t gIaAtFaultMax, gIaAtFaultMin;
    extern volatile int16_t gIbAtFaultMax, gIbAtFaultMin;
    extern volatile int16_t gIbusAtFaultMax, gIbusAtFaultMin;
    extern volatile int16_t gIaAtFaultInst, gIbAtFaultInst, gIbusAtFaultInst;
    extern volatile uint8_t gFaultSnapshotValid;
    extern volatile int16_t gIaRaw, gIbRaw, gIbusRaw;

    memcpy(&d[114], &gIaPkMax,        2);
    memcpy(&d[116], &gIaPkMin,        2);
    memcpy(&d[118], &gIbPkMax,        2);
    memcpy(&d[120], &gIbPkMin,        2);
    memcpy(&d[122], &gIbusPkMax,      2);
    memcpy(&d[124], &gIbusPkMin,      2);
    memcpy(&d[126], &gIaAtFaultMax,   2);
    memcpy(&d[128], &gIaAtFaultMin,   2);
    memcpy(&d[130], &gIbAtFaultMax,   2);
    memcpy(&d[132], &gIbAtFaultMin,   2);
    memcpy(&d[134], &gIbusAtFaultMax, 2);
    memcpy(&d[136], &gIbusAtFaultMin, 2);
    memcpy(&d[138], &gIaAtFaultInst,   2);
    memcpy(&d[140], &gIbAtFaultInst,   2);
    memcpy(&d[142], &gIbusAtFaultInst, 2);
    d[144] = gFaultSnapshotValid;
    d[145] = 0;  /* pad */

    /* Mechanism probe — per-polarity elapsed snapshots (10 bytes).
     * Read by pot_capture.py when len(data) >= 156.
     *   d[146] = diagElapsedAcceptRising
     *   d[148] = diagElapsedAcceptFalling
     *   d[150] = diagElapsedRejectRising
     *   d[152] = diagElapsedRejectFalling
     *   d[154] = diagFilterHRLast */
    {
        extern volatile uint16_t diagElapsedAcceptRising,  diagElapsedAcceptFalling;
        extern volatile uint16_t diagElapsedRejectRising,  diagElapsedRejectFalling;
        extern volatile uint16_t diagFilterHRLast;
        memcpy(&d[146], &diagElapsedAcceptRising,  2);
        memcpy(&d[148], &diagElapsedAcceptFalling, 2);
        memcpy(&d[150], &diagElapsedRejectRising,  2);
        memcpy(&d[152], &diagElapsedRejectFalling, 2);
        memcpy(&d[154], &diagFilterHRLast,         2);
    }

    /* Capture-layer probe — comp value × sector polarity, past blanking.
     * Distinguishes physics asymmetry (comp never leaves pre-ZC state)
     * from a software bug (accept logic discards a valid post-ZC sample). */
    {
        extern volatile uint32_t compRising_High,  compRising_Low;
        extern volatile uint32_t compFalling_High, compFalling_Low;
        memcpy(&d[156], &compRising_High,  4);
        memcpy(&d[160], &compRising_Low,   4);
        memcpy(&d[164], &compFalling_High, 4);
        memcpy(&d[168], &compFalling_Low,  4);
    }

    /* PTG postscale: count of fires that bypassed ProcessBemfSample().
     * Effective BEMF-sample rate = (ptgFires - ptgSkipped) / window. */
    {
        extern volatile uint32_t ptgSkipped;
        memcpy(&d[172], &ptgSkipped, 4);
    }

    /* Per-sector hit counters — 6 × uint32. Validates that all 6
     * commutation positions actually fire (an even/odd-only pattern
     * would mean position is incrementing by 2 somewhere). */
    {
        extern volatile uint32_t sectorHits[6];
        memcpy(&d[176], (const void *)sectorHits, 24);
    }

    /* Multi-phase BEMF tally:
     *   d[200..211]: bemfTallyTotal[6]    (6 × uint16)
     *   d[212..247]: bemfTally[6][3]      (6 × 3 × uint16)
     * Host computes comp=1/total ratio per (sector, phase) to detect
     * phase-mapping bugs (ratio at 0% or 100% on the supposed floating
     * phase = bug; (10..90)% = actually floating).
     *
     * Tallies are reset after copy so each frame is a fresh delta over
     * the ~50 ms inter-snapshot window. Without the reset, uint16 wraps
     * before one window completes at 60 kHz PWM. */
    {
        extern volatile uint16_t bemfTally[6][3];
        extern volatile uint16_t bemfTallyTotal[6];
        memcpy(&d[200], (const void *)bemfTallyTotal, 12);
        memcpy(&d[212], (const void *)bemfTally,      36);
        memset((void *)bemfTallyTotal, 0, sizeof(bemfTallyTotal));
        memset((void *)bemfTally,      0, sizeof(bemfTally));
    }

    /* Stale-floatingPhase diagnostic — uint16 at d[248..249], per-window
     * count of PTG fires (past blanking) where floatingPhase disagreed
     * with commutationTable[currentSector].floatingPhase. Reset
     * alongside the tally. */
    {
        extern volatile uint16_t fpStaleCount;
        memcpy(&d[248], (const void *)&fpStaleCount, 2);
        fpStaleCount = 0;
    }

    /* Reset rolling peaks for next 20 ms window. Seed with current
     * instantaneous sample so the window doesn't start at stale extrema.
     * Peaks track milliamps (same scale as the live mA values). */
    {
        extern volatile int16_t gIa_mA, gIb_mA, gIbus_mA;
        gIaPkMax   = gIaPkMin   = gIa_mA;
        gIbPkMax   = gIbPkMin   = gIb_mA;
        gIbusPkMax = gIbusPkMin = gIbus_mA;
    }

    GSP_SendResponse(GSP_CMD_TELEM_FRAME, snap, sizeof(snap));
}
#endif
