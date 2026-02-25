import { useEscStore } from '../store/useEscStore';
import { serial } from './ConnectionBar';
import { buildPacket, CMD } from '../protocol/gsp';

type ThrottleSrc = 'ADC' | 'GSP' | 'PWM' | 'DSHOT' | 'AUTO';
const SRC_VALUES: Record<ThrottleSrc, number> = { ADC: 0, GSP: 1, PWM: 2, DSHOT: 3, AUTO: 4 };

export function ControlPanel() {
  const { connected, snapshot, info, throttleSource, setThrottleSource } = useEscStore();
  const isIdle = snapshot?.state === 0;
  const isFault = snapshot?.state === 8;
  const flags = info?.featureFlags ?? 0;
  const hasAdcPot = (flags & (1 << 19)) !== 0;
  const hasRxPwm = (flags & (1 << 20)) !== 0;
  const hasRxDshot = (flags & (1 << 21)) !== 0;
  const hasRxAuto = (flags & (1 << 22)) !== 0;

  const send = async (cmdId: number, payload?: Uint8Array) => {
    if (!connected) return;
    await serial.write(buildPacket(cmdId, payload));
  };

  const setSource = async (src: ThrottleSrc) => {
    await send(CMD.SET_THROTTLE_SRC, new Uint8Array([SRC_VALUES[src]]));
    setThrottleSource(src as 'ADC' | 'GSP');
  };

  const btn = (bg: string, color: string, active = true): React.CSSProperties => ({
    padding: '8px 18px', borderRadius: 'var(--radius-sm)', border: 'none',
    cursor: active ? 'pointer' : 'default', fontWeight: 600, fontSize: 13,
    background: active ? bg : 'var(--bg-input)',
    color: active ? color : 'var(--text-muted)',
    transition: 'all 0.15s',
  });

  const srcBtn = (src: ThrottleSrc): React.CSSProperties => ({
    padding: '5px 12px', borderRadius: 'var(--radius-sm)',
    border: `1px solid ${throttleSource === src ? 'var(--accent-blue)' : 'var(--border)'}`,
    background: throttleSource === src ? 'var(--accent-blue-dim)' : 'transparent',
    color: throttleSource === src ? 'var(--accent-blue)' : 'var(--text-muted)',
    fontSize: 12, fontWeight: 600, cursor: !isIdle || !connected ? 'default' : 'pointer',
  });

  return (
    <div style={{
      background: 'var(--bg-card)', borderRadius: 'var(--radius)',
      padding: 16, border: '1px solid var(--border)',
      display: 'flex', flexDirection: 'column', gap: 12,
    }}>
      <div style={{
        fontSize: 13, color: 'var(--text-muted)', textTransform: 'uppercase',
        letterSpacing: '1px',
      }}>
        Motor Control
      </div>
      <div style={{ display: 'flex', gap: 8, alignItems: 'center', flexWrap: 'wrap' }}>
        <button style={btn('var(--accent-green)', '#000', isIdle && connected)}
          disabled={!isIdle || !connected}
          onClick={() => send(CMD.START_MOTOR)}>
          Start
        </button>
        <button style={btn('var(--accent-red)', '#fff', connected)}
          disabled={!connected}
          onClick={() => send(CMD.STOP_MOTOR)}>
          Stop
        </button>
        <button style={btn('var(--accent-yellow)', '#000', isFault && connected)}
          disabled={!isFault || !connected}
          onClick={() => send(CMD.CLEAR_FAULT)}>
          Clear Fault
        </button>
        <div style={{ flex: 1 }} />
        <div style={{ display: 'flex', gap: 4, alignItems: 'center' }}>
          <span style={{ fontSize: 11, color: 'var(--text-muted)', marginRight: 4 }}>Throttle Src</span>
          {hasAdcPot && (
            <button style={srcBtn('ADC')} disabled={!isIdle || !connected}
              onClick={() => setSource('ADC')}>ADC</button>
          )}
          {connected && (
            <button style={srcBtn('GSP')} disabled={!isIdle || !connected}
              onClick={() => setSource('GSP')}>GSP</button>
          )}
          {hasRxPwm && (
            <button style={srcBtn('PWM')} disabled={!isIdle || !connected}
              onClick={() => setSource('PWM')}>PWM</button>
          )}
          {hasRxDshot && (
            <button style={srcBtn('DSHOT')} disabled={!isIdle || !connected}
              onClick={() => setSource('DSHOT')}>DSHOT</button>
          )}
          {hasRxAuto && (
            <button style={srcBtn('AUTO')} disabled={!isIdle || !connected}
              onClick={() => setSource('AUTO')}>AUTO</button>
          )}
        </div>
      </div>
    </div>
  );
}
