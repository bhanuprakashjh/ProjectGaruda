import { useState } from 'react';
import { useEscStore } from '../store/useEscStore';
import { FEATURE_NAMES } from '../protocol/types';

interface FeatureSection {
  title: string;
  icon: string;
  description: string;
  details: string[];
  featureBit?: number;
}

const FEATURES: FeatureSection[] = [
  {
    title: '6-Step Trapezoidal BLDC Control',
    icon: '\u26A1',
    description: 'Classic 6-step commutation using back-EMF zero-crossing detection for sensorless brushless DC motor control.',
    details: [
      'Trapezoidal commutation drives two of three motor phases at a time while the third floats',
      'Zero-crossing events on the floating phase determine precise commutation timing',
      'Supports both software ADC polling and hardware ADC comparator zero-crossing methods',
      'Achieves reliable operation from low RPM (after startup) through maximum speed',
    ],
    featureBit: 0,
  },
  {
    title: 'Sine Startup with V/f Ramp',
    icon: '\u223F',
    description: 'Smooth motor starting using sine-modulated PWM with a voltage-per-frequency ramp, ensuring reliable spin-up even under load.',
    details: [
      'Three-phase alignment locks rotor to a known electrical angle before ramping',
      'V/f (voltage/frequency) ramp maintains constant flux during acceleration',
      'Sine modulation produces smooth, low-vibration torque during startup',
      'Configurable alignment duty, initial speed, and ramp acceleration rate',
    ],
    featureBit: 8,
  },
  {
    title: 'Morph Transition (Sine to Trapezoidal)',
    icon: '\u27A1',
    description: 'Gradual blending from sine drive to trapezoidal commutation, eliminating the current spike that would occur with a hard switchover.',
    details: [
      'Alpha blending factor smoothly crossfades between sine and trapezoidal waveforms',
      'Floating phase voltage is managed to prevent shoot-through current during transition',
      'Morph only exits after sufficient zero-crossing confirmation (sync threshold)',
      'Windowed Hi-Z insertion ensures clean BEMF sensing during the transition',
    ],
  },
  {
    title: 'Hardware Zero-Crossing Detection',
    icon: '\uD83D\uDD27',
    description: 'ADC digital comparators provide hardware-assisted zero-crossing detection at high speeds, offloading the CPU for more consistent timing.',
    details: [
      'Uses AD1CH5 (Phase B) and AD2CH1 (Phase A/C) digital comparators',
      'SCCP timer captures provide sub-microsecond ZC timestamps',
      'IIR-filtered step period tracking with noise floor rejection',
      'Configurable crossover speed for automatic SW-to-HW ZC transition',
    ],
    featureBit: 9,
  },
  {
    title: 'Commutation Timing Advance',
    icon: '\u23F1',
    description: 'Speed-dependent commutation advance improves motor efficiency and power output at high RPM.',
    details: [
      'Linearly interpolates advance angle from 0\u00b0 at ramp speed to configurable maximum',
      'Applies to both software and hardware ZC paths for consistent behavior',
      'Compensates for switching delays and winding inductance at high speeds',
      'Maximum advance is tunable per motor profile (typically 10-15\u00b0 for drone motors)',
    ],
    featureBit: 4,
  },
  {
    title: 'Duty Cycle Slew Rate Limiting',
    icon: '\u2197',
    description: 'Configurable ramp-up and ramp-down rates prevent sudden torque changes that could destabilize the motor or stress mechanical components.',
    details: [
      'Independent up and down slew rates (in %/ms)',
      'Post-sync settle period with reduced slew for smooth closed-loop entry',
      'Prevents regenerative braking overcurrent on rapid throttle-down',
      'Decoupled from PWM frequency \u2014 works at any PWM rate',
    ],
    featureBit: 3,
  },
  {
    title: 'Three-Tier Overcurrent Protection',
    icon: '\uD83D\uDEE1',
    description: 'Layered current protection with software monitoring, hardware comparator (CLPCI), and board-level PCI fault for comprehensive safety.',
    details: [
      'Software ADC monitoring: 24kHz sampling with soft limit (duty reduction) and hard fault threshold',
      'CMP3 hardware comparator (CLPCI): cycle-by-cycle current chopping via PWM PCI, with configurable trip level',
      'Board FPCI: DIM040 hardware fault input provides last-resort protection',
      'Separate startup current limit allows higher inrush during alignment without false trips',
      'Current gate holds ramp acceleration when bus current exceeds threshold',
    ],
    featureBit: 10,
  },
  {
    title: 'Voltage Protection',
    icon: '\u26A0',
    description: 'Overvoltage and undervoltage monitoring protects the power stage and prevents operation outside safe bus voltage range.',
    details: [
      'Configurable OV and UV thresholds in ADC counts',
      'UV startup threshold allows temporary sag during motor start',
      'Immediate fault on threshold violation \u2014 motor brakes to safe state',
      'ADC-based monitoring at 24kHz sample rate',
    ],
    featureBit: 1,
  },
  {
    title: 'Desync Recovery',
    icon: '\u21BB',
    description: 'Automatic detection and recovery from loss of synchronization, with configurable coast time and restart attempts.',
    details: [
      'Detects desync via missed zero-crossings and period instability',
      'Configurable coast-down time before restart attempt',
      'Maximum restart attempts before permanent fault (0 = no retries)',
      'Stall detection via HWZC noise reject limit and interval rejection',
    ],
    featureBit: 2,
  },
  {
    title: 'Motor Profiles',
    icon: '\u2699',
    description: 'Built-in profiles for common motors with all 31 parameters pre-tuned, plus a custom profile for user-defined motors.',
    details: [
      'Hurst DMB0224C10002: 10-pole, 24V, low-speed industrial motor',
      'A2212 1400KV: 14-pole, 12V, common drone motor with 8x4.5 prop',
      '5010 750KV: 28-pole, large multirotor motor',
      'Custom profile: user-tuned values saved to EEPROM',
      'Profile switching loads all 31 parameters atomically (motor must be idle)',
    ],
  },
  {
    title: 'GSP Serial Protocol',
    icon: '\uD83D\uDD0C',
    description: 'Binary serial protocol over UART for real-time telemetry streaming, parameter configuration, and motor control.',
    details: [
      'CRC-16-CCITT protected packets with framing (STX + length)',
      'Commands: ping, info, snapshot, start/stop motor, throttle control',
      'Parameter read/write with min/max validation and cross-checking',
      '50Hz streaming telemetry with heartbeat keepalive',
      'EEPROM persistence for all configuration parameters',
    ],
    featureBit: 16,
  },
  {
    title: 'Dynamic Demagnetization Blanking',
    icon: '\uD83E\uDDF2',
    description: 'Duty-dependent blanking window suppresses switching noise that could trigger false zero-crossing events.',
    details: [
      'Extra blanking percentage scales linearly with duty cycle above configurable threshold',
      'Prevents the PWM switching transients from corrupting BEMF sensing',
      'Separate configurable base blanking percentage (fraction of step period)',
      'Works with both SW and HW zero-crossing detection paths',
    ],
    featureBit: 5,
  },
];

const HW_INFO = {
  mcu: 'Microchip dsPIC33AK128MC106',
  arch: '32-bit, 200 MHz, Hardware FPU',
  flash: '128 KB',
  ram: '16 KB',
  pwm: '3-phase complementary, 24 kHz',
  adc: '12-bit SAR, up to 4.9 Msps per channel',
  board: 'MCLV-48V-300W Development Board',
  protocol: 'GSP v2 over UART1 @ 115200 baud',
};

function ChevronIcon({ open }: { open: boolean }) {
  return (
    <svg width="16" height="16" viewBox="0 0 16 16" fill="none"
      style={{ transform: open ? 'rotate(90deg)' : 'rotate(0deg)', transition: 'transform 0.2s ease' }}>
      <path d="M6 4L10 8L6 12" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
    </svg>
  );
}

function FeatureItem({ feature, info }: { feature: FeatureSection; info: ReturnType<typeof useEscStore.getState>['info'] }) {
  const [open, setOpen] = useState(false);
  const enabled = feature.featureBit !== undefined && info
    ? (info.featureFlags & (1 << feature.featureBit)) !== 0
    : undefined;

  return (
    <div style={{
      border: '1px solid var(--border)',
      borderRadius: 'var(--radius)',
      overflow: 'hidden',
      background: open ? 'var(--bg-card-alt)' : 'transparent',
      transition: 'background 0.2s',
    }}>
      <button
        onClick={() => setOpen(!open)}
        style={{
          width: '100%', display: 'flex', alignItems: 'center', gap: 10,
          padding: '10px 14px', border: 'none',
          background: 'transparent', color: 'var(--text-primary)',
          fontSize: 14, fontWeight: 500, textAlign: 'left',
        }}
      >
        <ChevronIcon open={open} />
        <span style={{ fontSize: 16 }}>{feature.icon}</span>
        <span style={{ flex: 1 }}>{feature.title}</span>
        {enabled !== undefined && (
          <span style={{
            fontSize: 10, fontWeight: 600, textTransform: 'uppercase',
            padding: '2px 8px', borderRadius: 10,
            background: enabled ? 'var(--accent-green-dim)' : 'rgba(239,68,68,0.15)',
            color: enabled ? 'var(--accent-green)' : 'var(--accent-red)',
            letterSpacing: '0.5px',
          }}>
            {enabled ? 'Enabled' : 'Disabled'}
          </span>
        )}
      </button>
      {open && (
        <div style={{ padding: '0 14px 14px 44px', animation: 'fadeIn 0.2s ease' }}>
          <p style={{ color: 'var(--text-secondary)', fontSize: 13, marginBottom: 10, lineHeight: 1.6 }}>
            {feature.description}
          </p>
          <ul style={{ listStyle: 'none', padding: 0 }}>
            {feature.details.map((d, i) => (
              <li key={i} style={{
                color: 'var(--text-muted)', fontSize: 12, padding: '3px 0',
                paddingLeft: 16, position: 'relative', lineHeight: 1.5,
              }}>
                <span style={{
                  position: 'absolute', left: 0, top: 6,
                  width: 5, height: 5, borderRadius: '50%',
                  background: 'var(--accent-blue)', opacity: 0.6,
                }} />
                {d}
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
}

export function HelpPanel() {
  const [showHelp, setShowHelp] = useState(false);
  const info = useEscStore(s => s.info);

  return (
    <div style={{ marginTop: 20 }}>
      <button
        onClick={() => setShowHelp(!showHelp)}
        style={{
          width: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center',
          gap: 8, padding: '10px 16px',
          background: 'var(--bg-card)', border: '1px solid var(--border)',
          borderRadius: 'var(--radius)', color: 'var(--text-secondary)',
          fontSize: 13, fontWeight: 500,
        }}
      >
        <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
          <circle cx="8" cy="8" r="7" stroke="currentColor" strokeWidth="1.5" />
          <path d="M6.5 6C6.5 5.2 7.2 4.5 8 4.5C8.8 4.5 9.5 5.2 9.5 6C9.5 6.8 8.5 7 8 7.5V8.5" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" />
          <circle cx="8" cy="10.5" r="0.75" fill="currentColor" />
        </svg>
        {showHelp ? 'Hide' : 'Show'} ESC Features & Documentation
        <ChevronIcon open={showHelp} />
      </button>

      {showHelp && (
        <div style={{
          marginTop: 8, padding: 20,
          background: 'var(--bg-card)', border: '1px solid var(--border)',
          borderRadius: 'var(--radius)', animation: 'fadeIn 0.3s ease',
        }}>
          {/* Hardware Info */}
          <div style={{ marginBottom: 20 }}>
            <h4 style={{ color: 'var(--accent-cyan)', fontSize: 14, marginBottom: 10, display: 'flex', alignItems: 'center', gap: 8 }}>
              <MicrochipIcon size={18} />
              Hardware Platform
            </h4>
            <div style={{
              display: 'grid', gridTemplateColumns: '1fr 1fr',
              gap: '6px 24px', fontSize: 12,
            }}>
              {Object.entries(HW_INFO).map(([key, val]) => (
                <div key={key} style={{ display: 'flex', gap: 8 }}>
                  <span style={{ color: 'var(--text-muted)', textTransform: 'capitalize', minWidth: 70 }}>{key}:</span>
                  <span style={{ color: 'var(--text-secondary)', fontFamily: 'var(--font-mono)', fontSize: 11 }}>{val}</span>
                </div>
              ))}
            </div>
          </div>

          {/* Active Feature Flags */}
          {info && (
            <div style={{ marginBottom: 20 }}>
              <h4 style={{ color: 'var(--accent-cyan)', fontSize: 14, marginBottom: 10 }}>
                Active Feature Flags
              </h4>
              <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6 }}>
                {Array.from({ length: 19 }, (_, i) => {
                  const enabled = (info.featureFlags & (1 << i)) !== 0;
                  const name = FEATURE_NAMES[i] ?? `BIT_${i}`;
                  return (
                    <span key={i} style={{
                      fontSize: 10, fontWeight: 500, padding: '3px 8px',
                      borderRadius: 4, fontFamily: 'var(--font-mono)',
                      background: enabled ? 'var(--accent-green-dim)' : 'rgba(71,85,105,0.3)',
                      color: enabled ? 'var(--accent-green)' : 'var(--text-muted)',
                      border: `1px solid ${enabled ? 'rgba(34,197,94,0.3)' : 'transparent'}`,
                    }}>
                      {name}
                    </span>
                  );
                })}
              </div>
            </div>
          )}

          {/* Features List */}
          <h4 style={{ color: 'var(--accent-cyan)', fontSize: 14, marginBottom: 10 }}>
            ESC Features Reference
          </h4>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 4 }}>
            {FEATURES.map((f, i) => (
              <FeatureItem key={i} feature={f} info={info} />
            ))}
          </div>

          {/* Protocol Reference */}
          <div style={{ marginTop: 20 }}>
            <h4 style={{ color: 'var(--accent-cyan)', fontSize: 14, marginBottom: 10 }}>
              Protocol Reference (GSP v2)
            </h4>
            <div style={{ fontSize: 12, color: 'var(--text-muted)', lineHeight: 1.7 }}>
              <p>Frame format: <code style={{ color: 'var(--accent-purple)', fontFamily: 'var(--font-mono)', fontSize: 11 }}>[0x02] [LEN] [CMD] [PAYLOAD...] [CRC16_H] [CRC16_L]</code></p>
              <p style={{ marginTop: 4 }}>CRC: CRC-16-CCITT (poly=0x1021, init=0xFFFF) over LEN+CMD+PAYLOAD</p>
              <p style={{ marginTop: 4 }}>Baud rate: 115200, 8N1 over UART1</p>
              <p style={{ marginTop: 4 }}>Max payload: 249 bytes. Telemetry: 50 Hz streaming with 200 ms heartbeat.</p>
              <p style={{ marginTop: 4 }}>31 runtime parameters across 8 groups, paginated descriptor list (20 entries/page).</p>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

export function MicrochipIcon({ size = 24 }: { size?: number }) {
  return (
    <svg width={size} height={size} viewBox="0 0 24 24" fill="none">
      {/* IC chip body */}
      <rect x="6" y="4" width="12" height="16" rx="1.5" stroke="var(--accent-cyan)" strokeWidth="1.5" fill="none" />
      {/* Pins left */}
      <line x1="3" y1="8" x2="6" y2="8" stroke="var(--accent-cyan)" strokeWidth="1.5" />
      <line x1="3" y1="12" x2="6" y2="12" stroke="var(--accent-cyan)" strokeWidth="1.5" />
      <line x1="3" y1="16" x2="6" y2="16" stroke="var(--accent-cyan)" strokeWidth="1.5" />
      {/* Pins right */}
      <line x1="18" y1="8" x2="21" y2="8" stroke="var(--accent-cyan)" strokeWidth="1.5" />
      <line x1="18" y1="12" x2="21" y2="12" stroke="var(--accent-cyan)" strokeWidth="1.5" />
      <line x1="18" y1="16" x2="21" y2="16" stroke="var(--accent-cyan)" strokeWidth="1.5" />
      {/* Pin 1 dot */}
      <circle cx="9" cy="7" r="1" fill="var(--accent-cyan)" />
    </svg>
  );
}
