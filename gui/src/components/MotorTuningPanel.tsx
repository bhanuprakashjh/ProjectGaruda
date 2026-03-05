import { useState, useMemo } from 'react';
import { useEscStore } from '../store/useEscStore';
import { isFocEnabled, PROFILE_NAMES } from '../protocol/types';

const SQRT3 = 1.73205080757;
const TWO_PI = 6.28318530718;

interface MotorInput {
  kv: number;         // RPM/V
  polePairs: number;
  resistance: number; // Ohm
  inductance: number; // mH (display), stored as H internally
  vbus: number;       // V
  pwmFreq: number;    // Hz
  maxCurrentA: number; // A
}

interface FocCalc {
  lambdaPm: number;    // V.s/rad
  ke: number;          // same as lambda
  ktFoc: number;       // N.m/A
  currentBw: number;   // rad/s
  kpCurrent: number;
  kiCurrent: number;
  maxOmegaElec: number; // rad/s
  maxRpmMech: number;
  maxOlRadS: number;
  handoffRadS: number;
  alignIq: number;
  rampIq: number;
  clampVdq: number;
  pllKp: number;
  pllKi: number;
}

const PRESETS: Record<string, Partial<MotorInput>> = {
  a2212: { kv: 1400, polePairs: 7, resistance: 0.093, inductance: 0.022, vbus: 12, maxCurrentA: 20 },
  '5010': { kv: 750, polePairs: 14, resistance: 0.042, inductance: 0.015, vbus: 14.8, maxCurrentA: 35 },
  hurst: { kv: 583, polePairs: 5, resistance: 0.680, inductance: 0.310, vbus: 24, maxCurrentA: 5 },
};

function calcFoc(m: MotorInput): FocCalc {
  const L = m.inductance / 1000; // mH -> H
  const lambdaPm = 60 / (SQRT3 * TWO_PI * m.kv * m.polePairs);
  const ktFoc = 1.5 * m.polePairs * lambdaPm;

  // Current controller bandwidth: 1/10th of PWM frequency
  const bw = TWO_PI * m.pwmFreq / 10;
  const kpCurrent = L * bw;
  const kiCurrent = m.resistance * bw;

  // Speed limits
  const maxOmegaElec = m.vbus / lambdaPm;
  const maxRpmMech = maxOmegaElec * 60 / (TWO_PI * m.polePairs);
  const maxOlRadS = maxOmegaElec * 0.85;
  const handoffRadS = maxOmegaElec * 0.07; // ~7% of max

  // Startup currents
  const alignIq = Math.min(m.maxCurrentA * 0.3, 6);
  const rampIq = Math.min(m.maxCurrentA * 0.3, 6);

  // Voltage clamp
  const clampVdq = m.vbus * 0.95 / SQRT3;

  // PLL (100 Hz bandwidth)
  const pllBw = TWO_PI * 100;
  const pllKp = 2 * pllBw;
  const pllKi = pllBw * pllBw;

  return {
    lambdaPm, ke: lambdaPm, ktFoc, currentBw: bw,
    kpCurrent, kiCurrent,
    maxOmegaElec, maxRpmMech, maxOlRadS, handoffRadS,
    alignIq, rampIq, clampVdq,
    pllKp, pllKi,
  };
}

function InputRow({ label, value, unit, onChange, tooltip, min, max, step }: {
  label: string; value: number; unit: string; onChange: (v: number) => void;
  tooltip?: string; min?: number; max?: number; step?: number;
}) {
  return (
    <div style={{
      display: 'flex', alignItems: 'center', gap: 8, padding: '6px 0',
      borderBottom: '1px solid var(--border-light)',
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

function ResultRow({ label, value, unit, color, highlight }: {
  label: string; value: string; unit: string; color?: string; highlight?: boolean;
}) {
  return (
    <div style={{
      display: 'flex', alignItems: 'center', justifyContent: 'space-between',
      padding: '5px 0', borderBottom: '1px solid var(--border-light)',
      background: highlight ? 'rgba(59,130,246,0.05)' : 'transparent',
    }}>
      <span style={{ fontSize: 12, color: 'var(--text-secondary)' }}>{label}</span>
      <span style={{
        fontSize: 13, fontWeight: 600, fontFamily: 'var(--font-mono)',
        color: color ?? 'var(--text-primary)',
      }}>
        {value} <span style={{ fontSize: 10, color: 'var(--text-muted)', fontWeight: 400 }}>{unit}</span>
      </span>
    </div>
  );
}

export function MotorTuningPanel() {
  const info = useEscStore(s => s.info);
  const focMode = info ? isFocEnabled(info.featureFlags) : false;
  const activeProfile = useEscStore(s => s.activeProfile);

  const [motor, setMotor] = useState<MotorInput>({
    kv: 1400, polePairs: 7, resistance: 0.093, inductance: 0.022,
    vbus: 12, pwmFreq: 24000, maxCurrentA: 20,
  });

  const [showHeader, setShowHeader] = useState(false);

  const update = (key: keyof MotorInput, val: number) => {
    setMotor(prev => ({ ...prev, [key]: val }));
  };

  const loadPreset = (preset: Partial<MotorInput>) => {
    setMotor(prev => ({ ...prev, ...preset }));
  };

  const calc = useMemo(() => calcFoc(motor), [motor]);

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

  const copyHeader = () => {
    navigator.clipboard.writeText(headerCode);
  };

  const cardStyle: React.CSSProperties = {
    background: 'var(--bg-card)', borderRadius: 'var(--radius)',
    padding: 16, border: '1px solid var(--border)',
  };

  const sectionTitle: React.CSSProperties = {
    fontSize: 11, color: 'var(--text-muted)', textTransform: 'uppercase',
    letterSpacing: '1px', marginBottom: 8, fontWeight: 600,
  };

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
          <ResultRow label="Max OL Speed (85%)" value={calc.maxOlRadS.toFixed(0)} unit="rad/s" />

          <div style={{ ...sectionTitle, marginTop: 12 }}>Startup</div>
          <ResultRow label="Align Iq" value={calc.alignIq.toFixed(1)} unit="A" />
          <ResultRow label="Ramp Iq" value={calc.rampIq.toFixed(1)} unit="A" />
          <ResultRow label="Align Torque" value={(calc.ktFoc * calc.alignIq * 1000).toFixed(1)} unit="mN.m" />

          <div style={{ ...sectionTitle, marginTop: 12 }}>PLL Estimator</div>
          <ResultRow label="PLL Kp" value={calc.pllKp.toFixed(1)} unit="" />
          <ResultRow label="PLL Ki" value={calc.pllKi.toFixed(1)} unit="" />
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

      {/* Motor info reference */}
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
            <div style={{ color: 'var(--text-muted)', marginBottom: 4 }}>PI Tuning</div>
            <div style={{ color: 'var(--text-secondary)' }}>
              BW = f_pwm / 10 (conservative)<br/>
              Kp = L x omega_bw<br/>
              Ki = R x omega_bw
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
