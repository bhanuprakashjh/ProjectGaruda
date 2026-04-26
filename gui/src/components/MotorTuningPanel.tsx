import { useState, useMemo, useEffect, useRef, useCallback } from 'react';
import { useEscStore } from '../store/useEscStore';
import { isFocEnabled, PROFILE_NAMES } from '../protocol/types';
import { serial } from './ConnectionBar';
import { buildPacket, CMD } from '../protocol/gsp';

const SQRT3 = 1.73205080757;
const TWO_PI = 6.28318530718;

// ESC state constants (matching garuda_types.h with ESC_DETECT at 2)
const ESC_IDLE = 0;
const ESC_ARMED = 1;
const ESC_DETECT = 2;
const ESC_FAULT = 9;

// FOC param IDs for readback
const PARAM_FOC_RS_MOHM = 0x70;
const PARAM_FOC_LS_UH = 0x71;
const PARAM_FOC_KE_UV = 0x72;
const PARAM_FOC_KP_DQ = 0x76;
const PARAM_FOC_KI_DQ = 0x77;
const PARAM_FOC_HANDOFF = 0x7E;
const PARAM_FOC_FAULT_OC = 0x7F;
const PARAM_POLE_PAIRS = 0x50;

interface MotorInput {
  kv: number;
  polePairs: number;
  resistance: number; // Ohm
  inductance: number; // mH
  vbus: number;
  pwmFreq: number;
  maxCurrentA: number;
}

interface FocCalc {
  lambdaPm: number;
  ke: number;
  ktFoc: number;
  currentBw: number;
  kpCurrent: number;
  kiCurrent: number;
  maxOmegaElec: number;
  maxRpmMech: number;
  maxOlRadS: number;
  handoffRadS: number;
  alignIq: number;
  rampIq: number;
  clampVdq: number;
  pllKp: number;
  pllKi: number;
}

type DetectStatus = 'idle' | 'running' | 'done' | 'failed';

interface DetectResult {
  rsMohm: number;
  lsUh: number;
  keUv: number;
  kpMilli?: number;
  kiDq?: number;
  handoffRadS?: number;
  faultOcCentiA?: number;
}

const PRESETS: Record<string, Partial<MotorInput>> = {
  a2212: { kv: 1400, polePairs: 7, resistance: 0.093, inductance: 0.022, vbus: 12, maxCurrentA: 20 },
  '5010': { kv: 750, polePairs: 14, resistance: 0.042, inductance: 0.015, vbus: 14.8, maxCurrentA: 35 },
  hurst: { kv: 583, polePairs: 5, resistance: 0.680, inductance: 0.310, vbus: 24, maxCurrentA: 5 },
};

function calcFoc(m: MotorInput): FocCalc {
  const L = m.inductance / 1000;
  const lambdaPm = 60 / (SQRT3 * TWO_PI * m.kv * m.polePairs);
  const ktFoc = 1.5 * m.polePairs * lambdaPm;
  const bw = TWO_PI * m.pwmFreq / 10;
  const kpCurrent = L * bw;
  const kiCurrent = m.resistance * bw;
  const maxOmegaElec = m.vbus / lambdaPm;
  const maxRpmMech = maxOmegaElec * 60 / (TWO_PI * m.polePairs);
  const maxOlRadS = maxOmegaElec * 0.85;
  const handoffRadS = maxOmegaElec * 0.07;
  const alignIq = Math.min(m.maxCurrentA * 0.3, 6);
  const rampIq = Math.min(m.maxCurrentA * 0.3, 6);
  const clampVdq = m.vbus * 0.95 / SQRT3;
  const pllBw = TWO_PI * 100;
  const pllKp = 2 * pllBw;
  const pllKi = pllBw * pllBw;
  return {
    lambdaPm, ke: lambdaPm, ktFoc, currentBw: bw,
    kpCurrent, kiCurrent,
    maxOmegaElec, maxRpmMech, maxOlRadS, handoffRadS,
    alignIq, rampIq, clampVdq, pllKp, pllKi,
  };
}

function InputRow({ label, value, unit, onChange, tooltip, min, max, step, disabled }: {
  label: string; value: number; unit: string; onChange: (v: number) => void;
  tooltip?: string; min?: number; max?: number; step?: number; disabled?: boolean;
}) {
  return (
    <div style={{
      display: 'flex', alignItems: 'center', gap: 8, padding: '6px 0',
      borderBottom: '1px solid var(--border-light)',
      opacity: disabled ? 0.5 : 1,
    }}>
      <div style={{ flex: 1, minWidth: 160 }}>
        <div style={{ fontSize: 12, color: 'var(--text-secondary)', fontWeight: 500 }}>{label}</div>
        {tooltip && <div style={{ fontSize: 10, color: 'var(--text-muted)' }}>{tooltip}</div>}
      </div>
      <div style={{ display: 'flex', alignItems: 'center', gap: 4 }}>
        <input
          type="number"
          value={value}
          min={min}
          max={max}
          step={step ?? 'any'}
          disabled={disabled}
          onChange={e => onChange(parseFloat(e.target.value) || 0)}
          style={{
            width: 100, padding: '4px 8px', borderRadius: 'var(--radius-sm)',
            border: '1px solid var(--border)', background: 'var(--bg-input)',
            color: 'var(--text-primary)', fontFamily: 'var(--font-mono)', fontSize: 13,
            textAlign: 'right',
          }}
        />
        <span style={{ fontSize: 11, color: 'var(--text-muted)', minWidth: 30 }}>{unit}</span>
      </div>
    </div>
  );
}

/** Row used in the SMO live-tune card.  Edit the value, then click Push
 *  to send a SET_PARAM with the corresponding ID.  Param takes effect
 *  within one PWM tick on the firmware side. */
function SmoParamRow({ label, value, unit, onChange, onPush, tooltip,
                      min, max, step, disabled }: {
  label: string; value: number; unit: string;
  onChange: (v: number) => void;
  onPush: () => void;
  tooltip?: string; min?: number; max?: number; step?: number; disabled?: boolean;
}) {
  return (
    <div style={{
      padding: '8px 10px', borderRadius: 'var(--radius-sm)',
      border: '1px solid var(--border-light)', background: 'var(--bg-secondary)',
      opacity: disabled ? 0.5 : 1,
    }}>
      <div style={{ fontSize: 12, fontWeight: 600, color: 'var(--text-primary)', marginBottom: 2 }}>
        {label}
      </div>
      {tooltip && (
        <div style={{ fontSize: 10, color: 'var(--text-muted)', whiteSpace: 'pre-line', marginBottom: 6 }}>
          {tooltip}
        </div>
      )}
      <div style={{ display: 'flex', gap: 6, alignItems: 'center' }}>
        <input
          type="number" value={value} min={min} max={max} step={step ?? 1}
          disabled={disabled}
          onChange={e => onChange(parseInt(e.target.value || '0', 10))}
          style={{
            flex: 1, padding: '6px 8px', borderRadius: 'var(--radius-sm)',
            border: '1px solid var(--border)', background: 'var(--bg-input)',
            color: 'var(--text-primary)', fontFamily: 'var(--font-mono)', fontSize: 13,
            textAlign: 'right',
          }}
        />
        <span style={{ fontSize: 11, color: 'var(--text-muted)', minWidth: 36 }}>{unit}</span>
        <button
          disabled={disabled}
          onClick={onPush}
          style={{
            padding: '6px 12px', borderRadius: 'var(--radius-sm)', border: 'none',
            background: disabled ? 'var(--bg-input)' : 'var(--accent-blue)',
            color: disabled ? 'var(--text-muted)' : '#fff',
            fontSize: 11, fontWeight: 700,
            cursor: disabled ? 'not-allowed' : 'pointer',
          }}
        >
          Push to ESC
        </button>
      </div>
    </div>
  );
}

function ResultRow({ label, value, unit, color, highlight, deprecated, deprecatedReason }: {
  label: string; value: string; unit: string; color?: string; highlight?: boolean;
  /** When set, row is rendered greyed out with strikethrough and a reason tooltip.
   *  Used to flag computed values that AN1078 doesn't actually consume. */
  deprecated?: boolean; deprecatedReason?: string;
}) {
  return (
    <div style={{
      display: 'flex', alignItems: 'center', justifyContent: 'space-between',
      padding: '5px 0', borderBottom: '1px solid var(--border-light)',
      background: highlight ? 'rgba(59,130,246,0.05)' : 'transparent',
      opacity: deprecated ? 0.45 : 1,
    }}>
      <span style={{
        fontSize: 12, color: 'var(--text-secondary)',
        textDecoration: deprecated ? 'line-through' : 'none',
      }} title={deprecatedReason ?? ''}>
        {label}
        {deprecated && (
          <span style={{ marginLeft: 6, fontSize: 10, color: 'var(--accent-yellow)' }}>
            (not used by AN1078)
          </span>
        )}
      </span>
      <span style={{
        fontSize: 13, fontWeight: 600, fontFamily: 'var(--font-mono)',
        color: color ?? 'var(--text-primary)',
        textDecoration: deprecated ? 'line-through' : 'none',
      }}>
        {value} <span style={{ fontSize: 10, color: 'var(--text-muted)', fontWeight: 400 }}>{unit}</span>
      </span>
    </div>
  );
}

async function sendCmd(cmdId: number, payload?: Uint8Array) {
  await serial.write(buildPacket(cmdId, payload));
}

async function readParam(paramId: number) {
  const buf = new Uint8Array(2);
  buf[0] = paramId & 0xFF;
  buf[1] = (paramId >> 8) & 0xFF;
  await sendCmd(CMD.GET_PARAM, buf);
}

async function setParam(paramId: number, value: number) {
  const buf = new Uint8Array(6);
  const v = new DataView(buf.buffer);
  v.setUint16(0, paramId, true);
  v.setUint32(2, value, true);
  await sendCmd(CMD.SET_PARAM, buf);
}

export function MotorTuningPanel() {
  const info = useEscStore(s => s.info);
  const connected = useEscStore(s => s.connected);
  const snapshot = useEscStore(s => s.snapshot);
  const params = useEscStore(s => s.params);
  const addToast = useEscStore(s => s.addToast);
  const focMode = info ? isFocEnabled(info.featureFlags) : false;
  const activeProfile = useEscStore(s => s.activeProfile);

  const [motor, setMotor] = useState<MotorInput>({
    kv: 1400, polePairs: 7, resistance: 0.093, inductance: 0.022,
    vbus: 12, pwmFreq: 24000, maxCurrentA: 20,
  });

  const [showHeader, setShowHeader] = useState(false);
  const [paramSource, setParamSource] = useState<'smo' | 'calculator' | 'autotune'>('smo');
  /* AN1078 SMO live-tune state (matches gsp_params.h IDs 0x90-0x93).
   * Initial values read from the store (populated by GET_PARAM_LIST at
   * connect, and refreshed on every SET_PARAM echo).  This keeps the
   * sliders in sync with whatever the firmware actually has after
   * navigating away and back, instead of resetting to a hardcoded
   * default that may differ from what was pushed. */
  const storeThetaBase = params.get(0x90)?.value;
  const storeThetaK    = params.get(0x91)?.value;
  const storeKslide    = params.get(0x92)?.value;
  const storeIdFwMax   = params.get(0x93)?.value;
  const [smoThetaBaseDegX10, setSmoThetaBaseDegX10] = useState(storeThetaBase ?? 200);
  const [smoThetaKE7,        setSmoThetaKE7]        = useState(storeThetaK    ?? 1000);
  const [smoKslideMv,        setSmoKslideMv]        = useState(storeKslide    ?? 2500);
  const [smoIdFwMaxDecia,    setSmoIdFwMaxDecia]    = useState(storeIdFwMax   ?? 120);
  const [smoStatus,          setSmoStatus]          = useState<string>('');
  const [showTuneGuide,      setShowTuneGuide]      = useState(false);

  /* Resync sliders when the store gets a fresh value.  Triggered:
   *  - On connect, when GET_PARAM_LIST populates the store
   *  - After every SET_PARAM, when the firmware echo updates the store
   *  - When user reads a param via GET_PARAM
   * Without this, slider state is local and "forgets" the pushed value
   * after tab navigation. */
  useEffect(() => {
    if (storeThetaBase !== undefined) setSmoThetaBaseDegX10(storeThetaBase);
  }, [storeThetaBase]);
  useEffect(() => {
    if (storeThetaK !== undefined) setSmoThetaKE7(storeThetaK);
  }, [storeThetaK]);
  useEffect(() => {
    if (storeKslide !== undefined) setSmoKslideMv(storeKslide);
  }, [storeKslide]);
  useEffect(() => {
    if (storeIdFwMax !== undefined) setSmoIdFwMaxDecia(storeIdFwMax);
  }, [storeIdFwMax]);

  // Auto-tune state
  const [detectStatus, setDetectStatus] = useState<DetectStatus>('idle');
  const [detectPhase, setDetectPhase] = useState(0);
  const [detectElapsed, setDetectElapsed] = useState(0);
  const [detectResult, setDetectResult] = useState<DetectResult | null>(null);
  const detectStartTime = useRef(0);
  const pollTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);

  const update = (key: keyof MotorInput, val: number) => {
    setMotor(prev => ({ ...prev, [key]: val }));
  };

  const loadPreset = (preset: Partial<MotorInput>) => {
    setMotor(prev => ({ ...prev, ...preset }));
  };

  const calc = useMemo(() => calcFoc(motor), [motor]);

  // Monitor snapshot for detect progress
  useEffect(() => {
    if (detectStatus !== 'running' || !snapshot) return;

    const elapsed = (Date.now() - detectStartTime.current) / 1000;
    setDetectElapsed(elapsed);

    if (snapshot.state === ESC_DETECT) {
      setDetectPhase(snapshot.focSubState);
    } else if (snapshot.state === ESC_ARMED) {
      // Detect completed — read back params
      setDetectStatus('done');
      setDetectPhase(6); // DETECT_DONE
      readDetectedParams();
    } else if (snapshot.state === ESC_FAULT) {
      setDetectStatus('failed');
      setDetectPhase(7); // DETECT_FAIL
      addToast(`Auto-detect failed (fault code: ${snapshot.faultCode})`, 'error');
    } else if (snapshot.state === ESC_IDLE && elapsed > 2) {
      setDetectStatus('failed');
      addToast('Auto-detect stopped unexpectedly', 'error');
    }
  }, [snapshot, detectStatus]);

  // Cleanup poll timer
  useEffect(() => {
    return () => {
      if (pollTimerRef.current) clearInterval(pollTimerRef.current);
    };
  }, []);

  const readDetectedParams = useCallback(async () => {
    // Read all FOC params that were written by detect
    await readParam(PARAM_FOC_RS_MOHM);
    await readParam(PARAM_FOC_LS_UH);
    await readParam(PARAM_FOC_KE_UV);
    await readParam(PARAM_FOC_KP_DQ);
    await readParam(PARAM_FOC_KI_DQ);
    await readParam(PARAM_FOC_HANDOFF);
    await readParam(PARAM_FOC_FAULT_OC);
  }, []);

  // Build detect result from store params whenever they update
  useEffect(() => {
    if (detectStatus !== 'done') return;
    const rs = params.get(PARAM_FOC_RS_MOHM)?.value;
    const ls = params.get(PARAM_FOC_LS_UH)?.value;
    const ke = params.get(PARAM_FOC_KE_UV)?.value;
    const kp = params.get(PARAM_FOC_KP_DQ)?.value;
    const ki = params.get(PARAM_FOC_KI_DQ)?.value;
    const ho = params.get(PARAM_FOC_HANDOFF)?.value;
    const oc = params.get(PARAM_FOC_FAULT_OC)?.value;
    if (rs !== undefined && ls !== undefined && ke !== undefined) {
      setDetectResult({
        rsMohm: rs, lsUh: ls, keUv: ke,
        kpMilli: kp, kiDq: ki,
        handoffRadS: ho, faultOcCentiA: oc,
      });
    }
  }, [detectStatus, params]);

  const startDetect = async () => {
    if (!connected || !snapshot || snapshot.state !== ESC_IDLE) {
      addToast('Motor must be IDLE to start auto-detect', 'error');
      return;
    }

    // Set pole pairs first if needed
    await setParam(PARAM_POLE_PAIRS, motor.polePairs);
    await new Promise(r => setTimeout(r, 50));

    // Start detect
    setDetectStatus('running');
    setDetectPhase(0);
    setDetectResult(null);
    detectStartTime.current = Date.now();

    await sendCmd(CMD.AUTO_DETECT);

    // Start polling snapshots if telemetry isn't active
    const telemActive = useEscStore.getState().telemActive;
    if (!telemActive) {
      if (pollTimerRef.current) clearInterval(pollTimerRef.current);
      pollTimerRef.current = setInterval(async () => {
        try { await sendCmd(CMD.GET_SNAPSHOT); } catch { /* ignore */ }
      }, 200);
    }
  };

  const saveDetectedParams = async () => {
    await sendCmd(CMD.SAVE_CONFIG);
    addToast('Detected parameters saved to EEPROM', 'success');
  };

  const stopPolling = useCallback(() => {
    if (pollTimerRef.current) {
      clearInterval(pollTimerRef.current);
      pollTimerRef.current = null;
    }
  }, []);

  // Stop polling when detect finishes
  useEffect(() => {
    if (detectStatus === 'done' || detectStatus === 'failed' || detectStatus === 'idle') {
      stopPolling();
    }
  }, [detectStatus, stopPolling]);

  // Generate C header
  const headerCode = useMemo(() => {
    const lines = [
      `/* Auto-generated FOC parameters for ${motor.kv}KV ${motor.polePairs}pp motor */`,
      `/* Bus voltage: ${motor.vbus}V, PWM: ${motor.pwmFreq}Hz */`,
      ``,
      `/* Motor Electrical Parameters */`,
      `#define MOTOR_R             ${motor.resistance.toFixed(4)}f    /* Phase resistance (Ohm) */`,
      `#define MOTOR_L             ${(motor.inductance / 1000).toExponential(3)}f    /* Phase inductance (H) */`,
      `#define MOTOR_LAMBDA_PM     ${calc.lambdaPm.toFixed(6)}f    /* Flux linkage (V.s/rad) */`,
      `#define MOTOR_POLE_PAIRS    ${motor.polePairs}`,
      ``,
      `/* Current Controller (BW = PWM_FREQ/10 = ${(calc.currentBw / TWO_PI).toFixed(0)} Hz) */`,
      `#define KP_ID               ${calc.kpCurrent.toFixed(4)}f`,
      `#define KI_ID               ${calc.kiCurrent.toFixed(4)}f`,
      `#define KP_IQ               ${calc.kpCurrent.toFixed(4)}f`,
      `#define KI_IQ               ${calc.kiCurrent.toFixed(4)}f`,
      ``,
      `/* Voltage Clamp */`,
      `#define CLAMP_VDQ           ${calc.clampVdq.toFixed(3)}f    /* 0.95*Vbus/sqrt(3) */`,
      ``,
      `/* PLL Estimator (100 Hz BW) */`,
      `#define PLL_KP              ${calc.pllKp.toFixed(1)}f`,
      `#define PLL_KI              ${calc.pllKi.toFixed(1)}f`,
      ``,
      `/* Speed Controller */`,
      `#define KP_SPD              0.005f`,
      `#define KI_SPD              0.5f`,
      ``,
      `/* I/f Startup */`,
      `#define STARTUP_ALIGN_IQ_A      ${calc.alignIq.toFixed(1)}f`,
      `#define STARTUP_RAMP_IQ_A       ${calc.rampIq.toFixed(1)}f`,
      `#define STARTUP_HANDOFF_RAD_S   ${calc.handoffRadS.toFixed(0)}f`,
      `#define STARTUP_MAX_OL_RAD_S    ${calc.maxOlRadS.toFixed(0)}f`,
      ``,
      `/* Fault Thresholds */`,
      `#define FAULT_OC_A          ${motor.maxCurrentA.toFixed(1)}f`,
    ];
    return lines.join('\n');
  }, [motor, calc]);

  const copyHeader = () => { navigator.clipboard.writeText(headerCode); };

  const cardStyle: React.CSSProperties = {
    background: 'var(--bg-card)', borderRadius: 'var(--radius)',
    padding: 16, border: '1px solid var(--border)',
  };

  const sectionTitle: React.CSSProperties = {
    fontSize: 11, color: 'var(--text-muted)', textTransform: 'uppercase',
    letterSpacing: '1px', marginBottom: 8, fontWeight: 600,
  };

  const tabBtn = (active: boolean): React.CSSProperties => ({
    padding: '8px 20px', borderRadius: 'var(--radius-sm)',
    border: `1px solid ${active ? 'var(--accent-blue)' : 'var(--border)'}`,
    background: active ? 'var(--accent-blue-dim)' : 'transparent',
    color: active ? 'var(--accent-blue)' : 'var(--text-muted)',
    fontSize: 12, fontWeight: 600, cursor: 'pointer',
  });

  const isIdle = !snapshot || snapshot.state === ESC_IDLE;
  const isDetecting = snapshot?.state === ESC_DETECT;

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 16 }}>
      {/* Mode indicator */}
      {!focMode && info && (
        <div style={{
          background: 'rgba(249,115,22,0.1)', border: '1px solid rgba(249,115,22,0.3)',
          borderRadius: 'var(--radius)', padding: '10px 16px',
          color: 'var(--accent-orange)', fontSize: 12,
        }}>
          Firmware is compiled in 6-step mode. Motor tuning parameters below are for reference —
          recompile with FEATURE_FOC=1 to use FOC.
        </div>
      )}

      {/* Presets */}
      <div style={cardStyle}>
        <div style={sectionTitle}>Motor Presets</div>
        <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap' }}>
          {Object.entries(PRESETS).map(([key, preset]) => (
            <button key={key} onClick={() => loadPreset(preset)} style={{
              padding: '8px 16px', borderRadius: 'var(--radius-sm)',
              border: '1px solid var(--border)', background: 'var(--bg-secondary)',
              color: 'var(--text-secondary)', fontSize: 12, fontWeight: 600,
              cursor: 'pointer',
            }}>
              {key === 'a2212' ? 'A2212 1400KV' : key === '5010' ? '5010 750KV' : 'Hurst DMB0224'}
            </button>
          ))}
          {info && (
            <span style={{ fontSize: 11, color: 'var(--text-muted)', alignSelf: 'center', marginLeft: 8 }}>
              Active: {PROFILE_NAMES[activeProfile] ?? `#${activeProfile}`}
            </span>
          )}
        </div>
      </div>

      {/* Parameter Source Tabs */}
      {focMode && (
        <div style={{ display: 'flex', gap: 8 }}>
          <button style={tabBtn(paramSource === 'smo')}
            onClick={() => setParamSource('smo')}>
            AN1078 SMO Tune
          </button>
          <button style={tabBtn(paramSource === 'calculator')}
            onClick={() => setParamSource('calculator')}>
            FOC Calculator (ref)
          </button>
          <button style={tabBtn(paramSource === 'autotune')}
            onClick={() => setParamSource('autotune')}>
            Auto-Detect
          </button>
        </div>
      )}

      {/* AN1078 SMO Live Tuning Panel */}
      {paramSource === 'smo' && focMode && (
        <div style={{ display: 'flex', flexDirection: 'column', gap: 16 }}>
          {/* Live tunables */}
          <div style={cardStyle}>
            <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 12 }}>
              <div style={sectionTitle}>AN1078 SMO Live Tunables</div>
              <button onClick={() => setShowTuneGuide(!showTuneGuide)} style={{
                padding: '4px 10px', borderRadius: 'var(--radius-sm)',
                border: '1px solid var(--accent-blue)', background: 'transparent',
                color: 'var(--accent-blue)', fontSize: 11, fontWeight: 600, cursor: 'pointer',
              }}>
                {showTuneGuide ? 'Hide' : 'Show'} Tuning Guide
              </button>
            </div>
            <div style={{ fontSize: 11, color: 'var(--text-muted)', marginBottom: 12, lineHeight: 1.5 }}>
              These four params take effect <strong>immediately</strong> while motor is in CL —
              no recompile, no flash, no motor stop. Watch <code>focVd</code> in the Scope tab
              (preset "AN1078 SMO Tune") to see alignment in real time.
            </div>

            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 16 }}>
              <SmoParamRow
                label="Theta Offset BASE"
                value={smoThetaBaseDegX10} unit="°×10"
                onChange={setSmoThetaBaseDegX10}
                onPush={async () => {
                  await setParam(0x90, smoThetaBaseDegX10);
                  setSmoStatus(`BASE → ${(smoThetaBaseDegX10/10).toFixed(1)}° pushed`);
                }}
                tooltip={`Compensates BEMF→rotor offset at low speed.
Watch focVd: 0=aligned, -V=lags, +V=leads.
Range 0-3600 (= 0°-360°). 200 = 20° (good for 2810).`}
                min={0} max={3600} step={5}
                disabled={!connected} />
              <SmoParamRow
                label="Theta Offset K (speed slope)"
                value={smoThetaKE7} unit="×1e7"
                onChange={setSmoThetaKE7}
                onPush={async () => {
                  await setParam(0x91, smoThetaKE7);
                  setSmoStatus(`K → ${(smoThetaKE7 * 1e-7).toExponential(2)} rad/(rad/s)`);
                }}
                tooltip={`Compensates LPF lag at high speed.
1000 = 1.0e-4 rad/(rad/s elec).
Tune AFTER BASE. K=0 if PLL already gives clean angle.`}
                min={0} max={1000} step={10}
                disabled={!connected} />
              <SmoParamRow
                label="Kslide (sliding gain)"
                value={smoKslideMv} unit="mV"
                onChange={setSmoKslideMv}
                onPush={async () => {
                  await setParam(0x92, smoKslideMv);
                  setSmoStatus(`Kslide → ${(smoKslideMv/1000).toFixed(1)} V`);
                }}
                tooltip={`SMC sliding signal magnitude.
Low-Rs motors (2810): 2000-3000 mV.
High-Rs motors (Hurst): 6000-20000 mV.
Too high → buries weak BEMF, observer loses lock.`}
                min={100} max={30000} step={100}
                disabled={!connected} />
              <SmoParamRow
                label="Field Weakening max |Id|"
                value={smoIdFwMaxDecia} unit="dA"
                onChange={setSmoIdFwMaxDecia}
                onPush={async () => {
                  await setParam(0x93, smoIdFwMaxDecia);
                  setSmoStatus(`FW max → ${(smoIdFwMaxDecia/10).toFixed(1)} A negative Id`);
                }}
                tooltip={`Field-weakening current cap.
0 = FW disabled (motor caps at voltage limit ~180k eRPM on 2810).
120 = 12A (allows ~210k eRPM on 2810 at 24V).`}
                min={0} max={200} step={5}
                disabled={!connected} />
            </div>

            {smoStatus && (
              <div style={{
                marginTop: 12, padding: '8px 12px',
                background: 'rgba(34,197,94,0.1)', borderRadius: 'var(--radius-sm)',
                color: 'var(--accent-green)', fontSize: 12, fontWeight: 500,
              }}>
                ✓ {smoStatus}
              </div>
            )}

            <div style={{
              marginTop: 12, padding: '8px 12px',
              background: 'rgba(234,179,8,0.05)', borderRadius: 'var(--radius-sm)',
              fontSize: 11, color: 'var(--text-muted)', lineHeight: 1.5,
            }}>
              <strong>Note:</strong> values are RAM-only (lost on reboot). Once dialed,
              copy them to the per-profile defaults in <code>gsp_params.c</code> and
              recompile to bake them in. SAVE_CONFIG persists everything else but not
              these four (V3 EEPROM schema doesn't carry them; V4 schema bump pending).
            </div>
          </div>

          {/* Inline tuning guide */}
          {showTuneGuide && (
            <div style={cardStyle}>
              <div style={sectionTitle}>SMO Tuning Procedure</div>
              <ol style={{ fontSize: 12, color: 'var(--text-secondary)', lineHeight: 1.7, paddingLeft: 20 }}>
                <li>
                  <strong>Verify observer tracks.</strong> Open Scope → preset
                  "AN1078 Angle Health". Spin motor low CL (10-30k eRPM). Drive Theta
                  and Observer Theta should rotate at same rate. If observer is stuck
                  or rotating wrong direction, lower <strong>Kslide</strong> (low-Rs
                  motors need 2-3 V; the AN1078 default 0.85·Vbus is too aggressive).
                </li>
                <li>
                  <strong>Tune <code>BASE</code> at low speed.</strong> Switch Scope
                  preset to "AN1078 SMO Tune". Hold motor at ~10× the OL handoff
                  speed. Read focVd average. Negative → observer lags → INCREASE
                  BASE. Positive → leads → DECREASE BASE. Step 10-20 units (1-2°).
                  Goal: focVd within ±0.5 V.
                </li>
                <li>
                  <strong>Tune <code>K</code> at high speed.</strong> Push motor to
                  half of max throttle. Read focVd again. Negative → INCREASE K
                  (step +100 = +1e-5). Positive → DECREASE K. Goal: focVd stays
                  within ±1.5 V across the whole speed range.
                </li>
                <li>
                  <strong>Field weakening.</strong> Push throttle to max. focModIndex
                  hits 0.95 and speed plateaus → set FW max in 10 dA (1 A) increments.
                  Stop when speed stops growing OR when motor heats up. Set 0 to
                  disable.
                </li>
                <li>
                  <strong>Bake in.</strong> Once values feel right, edit
                  <code>gspParams.c</code> per-profile defaults and recompile.
                </li>
              </ol>
            </div>
          )}
        </div>
      )}

      {/* Auto-Tune Panel */}
      {paramSource === 'autotune' && focMode && (
        <div style={{ display: 'flex', flexDirection: 'column', gap: 16 }}>
          {/* Setup */}
          <div style={cardStyle}>
            <div style={sectionTitle}>Auto-Tune Setup</div>
            <div style={{ fontSize: 12, color: 'var(--text-secondary)', marginBottom: 12, lineHeight: 1.5 }}>
              Auto-detect measures Rs, Ls, and back-EMF constant by injecting test currents.
              The motor will lock briefly, pulse, then spin. Ensure the motor is free to rotate
              and connected to the ESC.
            </div>

            <InputRow label="Pole Pairs" value={motor.polePairs} unit="pp"
              onChange={v => update('polePairs', v)}
              tooltip="Must be set manually — cannot be auto-detected"
              min={1} max={30} step={1}
              disabled={detectStatus === 'running'} />

            <div style={{ display: 'flex', gap: 12, marginTop: 16, alignItems: 'center' }}>
              <button
                disabled={!connected || !isIdle || detectStatus === 'running'}
                onClick={startDetect}
                style={{
                  padding: '10px 24px', borderRadius: 'var(--radius-sm)', border: 'none',
                  background: (connected && isIdle && detectStatus !== 'running')
                    ? 'var(--accent-blue)' : 'var(--bg-input)',
                  color: (connected && isIdle && detectStatus !== 'running')
                    ? '#fff' : 'var(--text-muted)',
                  fontSize: 13, fontWeight: 700, cursor:
                    (connected && isIdle && detectStatus !== 'running') ? 'pointer' : 'default',
                  letterSpacing: '0.3px',
                }}
              >
                {detectStatus === 'running' ? 'Detecting...' : 'Start Auto-Detect'}
              </button>

              {!connected && (
                <span style={{ fontSize: 11, color: 'var(--accent-red)' }}>
                  Connect to ESC first
                </span>
              )}
              {connected && !isIdle && detectStatus !== 'running' && (
                <span style={{ fontSize: 11, color: 'var(--accent-orange)' }}>
                  Motor must be IDLE
                </span>
              )}
            </div>
          </div>

          {/* Progress */}
          {detectStatus === 'running' && (
            <div style={cardStyle}>
              <div style={sectionTitle}>Detection Progress</div>
              <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
                {[1, 2, 3, 4, 5].map(phase => {
                  const names = ['Measuring Rs', 'Measuring Ls', 'Re-Aligning', 'Measuring Lambda', 'Auto-Tuning'];
                  const durations = ['~1.2s', '~3s', '~0.2s', '~4.6s', 'instant'];
                  const details = [
                    'Rotor locks at theta=0, measures voltage/current',
                    'Voltage pulses to measure inductance',
                    'Re-locks rotor at theta=0 with Id current',
                    'd\u2192q transition + I/f spin to measure back-EMF',
                    'Calculates PI gains and startup params',
                  ];
                  const isActive = detectPhase === phase;
                  const isDone = detectPhase > phase;
                  return (
                    <div key={phase} style={{
                      display: 'flex', alignItems: 'center', gap: 12, padding: '8px 12px',
                      borderRadius: 'var(--radius-sm)',
                      background: isActive ? 'rgba(59,130,246,0.1)' : 'transparent',
                      border: `1px solid ${isActive ? 'var(--accent-blue)' : 'var(--border-light)'}`,
                    }}>
                      <div style={{
                        width: 24, height: 24, borderRadius: '50%',
                        display: 'flex', alignItems: 'center', justifyContent: 'center',
                        background: isDone ? 'var(--accent-green)' : isActive ? 'var(--accent-blue)' : 'var(--bg-input)',
                        color: (isDone || isActive) ? '#fff' : 'var(--text-muted)',
                        fontSize: 11, fontWeight: 700,
                      }}>
                        {isDone ? '\u2713' : phase}
                      </div>
                      <div style={{ flex: 1 }}>
                        <div style={{
                          fontSize: 12, fontWeight: 600,
                          color: isActive ? 'var(--accent-blue)' : isDone ? 'var(--accent-green)' : 'var(--text-muted)',
                        }}>
                          {names[phase - 1]}
                          <span style={{ fontSize: 10, color: 'var(--text-muted)', marginLeft: 8, fontWeight: 400 }}>
                            {durations[phase - 1]}
                          </span>
                        </div>
                        <div style={{ fontSize: 10, color: 'var(--text-muted)' }}>{details[phase - 1]}</div>
                      </div>
                      {isActive && (
                        <div style={{
                          width: 8, height: 8, borderRadius: '50%',
                          background: 'var(--accent-blue)',
                          animation: 'pulse 1s ease-in-out infinite',
                        }} />
                      )}
                    </div>
                  );
                })}
              </div>

              {/* Live telemetry during detect */}
              {isDetecting && snapshot && (
                <div style={{
                  marginTop: 12, padding: '8px 12px',
                  background: 'var(--bg-secondary)', borderRadius: 'var(--radius-sm)',
                  fontFamily: 'var(--font-mono)', fontSize: 11, color: 'var(--text-secondary)',
                  display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: 4,
                }}>
                  <span>Id: {snapshot.focIdMeas.toFixed(3)}A</span>
                  <span>Iq: {snapshot.focIqMeas.toFixed(3)}A</span>
                  <span>\u03C9: {snapshot.focOmega.toFixed(1)} rad/s</span>
                  <span>Vbus: {snapshot.focVbus.toFixed(1)}V</span>
                </div>
              )}

              <div style={{ marginTop: 8, fontSize: 11, color: 'var(--text-muted)' }}>
                Elapsed: {detectElapsed.toFixed(1)}s
              </div>
            </div>
          )}

          {/* Results */}
          {detectStatus === 'done' && detectResult && (
            <div style={cardStyle}>
              <div style={sectionTitle}>Detected Motor Parameters</div>

              <ResultRow label="Phase Resistance (Rs)"
                value={`${detectResult.rsMohm} m\u03A9 (${(detectResult.rsMohm / 1000).toFixed(4)} \u03A9)`}
                unit="" color="var(--accent-cyan)" highlight />
              <ResultRow label="Phase Inductance (Ls)"
                value={`${detectResult.lsUh} \u00B5H (${(detectResult.lsUh / 1000).toFixed(3)} mH)`}
                unit="" color="var(--accent-cyan)" highlight />
              <ResultRow label="Back-EMF Constant (Ke)"
                value={`${detectResult.keUv} \u00B5V\u00B7s/rad (${(detectResult.keUv / 1e6).toFixed(6)} V\u00B7s/rad)`}
                unit="" color="var(--accent-cyan)" highlight />

              {detectResult.kpMilli !== undefined && (
                <ResultRow label="Current Loop Kp" value={(detectResult.kpMilli / 1000).toFixed(4)} unit="" />
              )}
              {detectResult.kiDq !== undefined && (
                <ResultRow label="Current Loop Ki" value={detectResult.kiDq.toFixed(1)} unit="1/s" />
              )}
              {detectResult.handoffRadS !== undefined && (
                <ResultRow label="Handoff Speed" value={detectResult.handoffRadS.toFixed(0)} unit="rad/s" />
              )}
              {detectResult.faultOcCentiA !== undefined && (
                <ResultRow label="OC Fault Threshold" value={(detectResult.faultOcCentiA / 100).toFixed(1)} unit="A" />
              )}

              {/* Derived KV estimate */}
              {detectResult.keUv > 0 && (
                <ResultRow label="Estimated KV"
                  value={(60 / (SQRT3 * TWO_PI * (detectResult.keUv / 1e6) * motor.polePairs)).toFixed(0)}
                  unit="RPM/V" color="var(--accent-yellow)" highlight />
              )}

              <div style={{ display: 'flex', gap: 8, marginTop: 16 }}>
                <button onClick={saveDetectedParams} style={{
                  padding: '8px 20px', borderRadius: 'var(--radius-sm)', border: 'none',
                  background: 'var(--accent-green)', color: '#000',
                  fontSize: 12, fontWeight: 700, cursor: 'pointer',
                }}>
                  Save to EEPROM
                </button>
                <button onClick={() => { setDetectStatus('idle'); setDetectResult(null); }} style={{
                  padding: '8px 20px', borderRadius: 'var(--radius-sm)',
                  border: '1px solid var(--border)', background: 'transparent',
                  color: 'var(--text-muted)', fontSize: 12, fontWeight: 600, cursor: 'pointer',
                }}>
                  Re-run
                </button>
              </div>
            </div>
          )}

          {/* Failure */}
          {detectStatus === 'failed' && (
            <div style={{
              ...cardStyle,
              borderColor: 'rgba(239,68,68,0.3)',
              background: 'rgba(239,68,68,0.05)',
            }}>
              <div style={{ ...sectionTitle, color: 'var(--accent-red)' }}>Detection Failed</div>
              <div style={{ fontSize: 12, color: 'var(--text-secondary)', lineHeight: 1.5 }}>
                {snapshot?.faultCode
                  ? `Fault code: ${snapshot.faultCode}`
                  : 'Motor returned to IDLE unexpectedly'}
                <br />
                Check: motor connected? Free to spin? Vbus OK?
              </div>
              <button onClick={() => setDetectStatus('idle')} style={{
                marginTop: 12, padding: '8px 20px', borderRadius: 'var(--radius-sm)',
                border: '1px solid var(--border)', background: 'transparent',
                color: 'var(--text-muted)', fontSize: 12, fontWeight: 600, cursor: 'pointer',
              }}>
                Try Again
              </button>
            </div>
          )}
        </div>
      )}

      {/* Calculator Panel (existing functionality) */}
      {paramSource === 'calculator' && (
        <>
          {/* Notice: this view is for V2/V3 reference + sanity-check math.
           * AN1078 live-tuning is on the SMO tab. */}
          <div style={{
            background: 'rgba(234,179,8,0.05)', border: '1px solid rgba(234,179,8,0.2)',
            borderRadius: 'var(--radius)', padding: '10px 16px',
            color: 'var(--text-secondary)', fontSize: 12, lineHeight: 1.5,
          }}>
            <strong style={{ color: 'var(--accent-yellow)' }}>Reference only.</strong>
            {' '}This calculator computes V2/V3-style FOC values.  AN1078 only
            uses the motor params (Rs/Ls/λ) and current-PI gains; rows that AN1078
            doesn't consume (Max OL Speed, PLL Kp/Ki) are greyed out.  For live
            tuning, switch to the <strong>AN1078 SMO Tune</strong> tab.  For adding
            a brand-new motor, see <code>docs/an1078_add_new_motor.md</code>.
          </div>
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 16 }}>
            {/* Input parameters */}
            <div style={cardStyle}>
              <div style={sectionTitle}>Motor Parameters</div>
              <InputRow label="KV Rating" value={motor.kv} unit="RPM/V"
                onChange={v => update('kv', v)} tooltip="No-load speed constant"
                min={100} max={10000} step={10} />
              <InputRow label="Pole Pairs" value={motor.polePairs} unit="pp"
                onChange={v => update('polePairs', v)} tooltip="Magnetic pole pairs (poles / 2)"
                min={1} max={30} step={1} />
              <InputRow label="Phase Resistance" value={motor.resistance} unit="Ohm"
                onChange={v => update('resistance', v)} tooltip="Line-to-line / 2 for wye-wound"
                min={0.001} max={10} step={0.001} />
              <InputRow label="Phase Inductance" value={motor.inductance} unit="mH"
                onChange={v => update('inductance', v)} tooltip="Line-to-line / 2 for wye-wound"
                min={0.001} max={100} step={0.001} />

              <div style={{ ...sectionTitle, marginTop: 16 }}>System Parameters</div>
              <InputRow label="Bus Voltage" value={motor.vbus} unit="V"
                onChange={v => update('vbus', v)} tooltip="DC bus voltage (battery)"
                min={3} max={60} step={0.1} />
              <InputRow label="PWM Frequency" value={motor.pwmFreq} unit="Hz"
                onChange={v => update('pwmFreq', v)} tooltip="Switching frequency"
                min={8000} max={48000} step={1000} />
              <InputRow label="Max Phase Current" value={motor.maxCurrentA} unit="A"
                onChange={v => update('maxCurrentA', v)} tooltip="Overcurrent fault threshold"
                min={1} max={100} step={1} />
            </div>

            {/* Calculated results */}
            <div style={cardStyle}>
              <div style={sectionTitle}>Calculated FOC Parameters</div>

              <ResultRow label="Flux Linkage (lambda_pm)" value={calc.lambdaPm.toFixed(6)} unit="V.s/rad"
                color="var(--accent-cyan)" highlight />
              <ResultRow label="Back-EMF Constant (Ke)" value={(calc.ke * 1000).toFixed(3)} unit="mV.s/rad"
                color="var(--accent-cyan)" />
              <ResultRow label="Torque Constant (Kt_FOC)" value={calc.ktFoc.toFixed(5)} unit="N.m/A"
                color="var(--accent-green)" highlight />

              <div style={{ ...sectionTitle, marginTop: 12 }}>Current Controller</div>
              <ResultRow label="Bandwidth" value={(calc.currentBw / TWO_PI).toFixed(0)} unit="Hz" />
              <ResultRow label="Kp (Id & Iq)" value={calc.kpCurrent.toFixed(4)} unit="" color="var(--accent-blue)" />
              <ResultRow label="Ki (Id & Iq)" value={calc.kiCurrent.toFixed(4)} unit="" color="var(--accent-blue)" />
              <ResultRow label="Voltage Clamp" value={calc.clampVdq.toFixed(2)} unit="V" />

              <div style={{ ...sectionTitle, marginTop: 12 }}>Speed / Limits</div>
              <ResultRow label="Max Elec Speed" value={calc.maxOmegaElec.toFixed(0)} unit="rad/s" />
              <ResultRow label="Max Mech RPM" value={calc.maxRpmMech.toFixed(0)} unit="RPM" color="var(--accent-yellow)" highlight />
              <ResultRow label="Handoff Speed" value={calc.handoffRadS.toFixed(0)} unit="rad/s" />
              <ResultRow label="Max OL Speed (85%)" value={calc.maxOlRadS.toFixed(0)} unit="rad/s"
                deprecated
                deprecatedReason="AN1078 ramps OL only to AN_END_SPEED (=AN_END_SPEED_RPM_MECH * polepairs * 2π/60), not 85% of max. This 85% rule is V2/V3-era." />

              <div style={{ ...sectionTitle, marginTop: 12 }}>Startup</div>
              <ResultRow label="Align Iq" value={calc.alignIq.toFixed(1)} unit="A" />
              <ResultRow label="Ramp Iq" value={calc.rampIq.toFixed(1)} unit="A" />
              <ResultRow label="Align Torque" value={(calc.ktFoc * calc.alignIq * 1000).toFixed(1)} unit="mN.m" />

              <div style={{ ...sectionTitle, marginTop: 12 }}>PLL Estimator</div>
              <ResultRow label="PLL Kp" value={calc.pllKp.toFixed(1)} unit=""
                deprecated
                deprecatedReason="AN1078 PLL uses garuda_foc_params.h:PLL_KP=628 (50 Hz BW). The 6283 shown here is a 100 Hz value from V2 era and is too aggressive for sensorless." />
              <ResultRow label="PLL Ki" value={calc.pllKi.toFixed(1)} unit=""
                deprecated
                deprecatedReason="AN1078 PLL uses garuda_foc_params.h:PLL_KI=98600 (50 Hz BW). The 395k shown here is a 100 Hz value from V2 era." />
            </div>
          </div>

          {/* Generated header */}
          <div style={cardStyle}>
            <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 8 }}>
              <button onClick={() => setShowHeader(!showHeader)} style={{
                display: 'flex', alignItems: 'center', gap: 6,
                background: 'none', border: 'none', color: 'var(--text-primary)',
                fontSize: 12, fontWeight: 600, padding: 0, cursor: 'pointer',
              }}>
                <svg width="12" height="12" viewBox="0 0 12 12" fill="none"
                  style={{ transform: showHeader ? 'rotate(90deg)' : 'rotate(0deg)', transition: 'transform 0.2s' }}>
                  <path d="M4 2L8 6L4 10" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
                </svg>
                <span style={{ textTransform: 'uppercase', letterSpacing: '0.5px', color: 'var(--text-muted)' }}>
                  Generated C Header (garuda_foc_params.h)
                </span>
              </button>
              {showHeader && (
                <button onClick={copyHeader} style={{
                  padding: '4px 12px', borderRadius: 'var(--radius-sm)',
                  border: '1px solid var(--accent-blue)', background: 'var(--accent-blue-dim)',
                  color: 'var(--accent-blue)', fontSize: 11, fontWeight: 600, cursor: 'pointer',
                }}>
                  Copy to Clipboard
                </button>
              )}
            </div>
            {showHeader && (
              <pre style={{
                background: 'var(--bg-input)', borderRadius: 'var(--radius-sm)',
                padding: 12, fontSize: 11, fontFamily: 'var(--font-mono)',
                color: 'var(--text-secondary)', lineHeight: 1.6,
                overflow: 'auto', maxHeight: 500,
                border: '1px solid var(--border-light)',
              }}>
                {headerCode}
              </pre>
            )}
          </div>
        </>
      )}

      {/* Quick Reference (show for both modes) */}
      <div style={cardStyle}>
        <div style={sectionTitle}>Quick Reference</div>
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: 12, fontSize: 11 }}>
          <div>
            <div style={{ color: 'var(--text-muted)', marginBottom: 4 }}>Phase Relationships</div>
            <div style={{ color: 'var(--text-secondary)' }}>
              R_ll = 2 x R_phase<br/>
              L_ll = 2 x L_phase<br/>
              Ke_line = sqrt(3) x Ke_phase
            </div>
          </div>
          <div>
            <div style={{ color: 'var(--text-muted)', marginBottom: 4 }}>KV to Lambda</div>
            <div style={{ color: 'var(--text-secondary)' }}>
              lambda = 60 / (sqrt(3) x 2pi x KV x pp)<br/>
              Kt_FOC = (3/2) x pp x lambda<br/>
              Torque = Kt_FOC x Iq
            </div>
          </div>
          <div>
            <div style={{ color: 'var(--text-muted)', marginBottom: 4 }}>Auto-Detect Phases</div>
            <div style={{ color: 'var(--text-secondary)' }}>
              1. Rs: DC current injection (~1.2s)<br/>
              2. Ls: Voltage step response (~3s)<br/>
              3. Re-align: Lock rotor + d→q transition (~0.3s)<br/>
              4. Lambda: Spin + measure BEMF (~4.5s)
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
