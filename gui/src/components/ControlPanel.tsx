import { useEscStore } from '../store/useEscStore';
import { serial } from './ConnectionBar';
import { buildPacket, CMD } from '../protocol/gsp';

export function ControlPanel() {
  const { connected, snapshot, throttleSource, setThrottleSource } = useEscStore();
  const isIdle = snapshot?.state === 0;
  const isFault = snapshot?.state === 8;

  const send = async (cmdId: number, payload?: Uint8Array) => {
    if (!connected) return;
    await serial.write(buildPacket(cmdId, payload));
  };

  const setSource = async (src: 'ADC' | 'GSP') => {
    const val = src === 'GSP' ? 1 : 0;
    await send(CMD.SET_THROTTLE_SRC, new Uint8Array([val]));
    setThrottleSource(src);
  };

  const btn = (bg: string, color: string, active = true): React.CSSProperties => ({
    padding: '8px 18px', borderRadius: 'var(--radius-sm)', border: 'none',
    cursor: active ? 'pointer' : 'default', fontWeight: 600, fontSize: 13,
    background: active ? bg : 'var(--bg-input)',
    color: active ? color : 'var(--text-muted)',
    transition: 'all 0.15s',
  });

  const srcBtn = (src: 'ADC' | 'GSP'): React.CSSProperties => ({
    padding: '5px 12px', borderRadius: 'var(--radius-sm)',
    border: `1px solid ${throttleSource === src ? 'var(--accent-blue)' : 'var(--border)'}`,
    background: throttleSource === src ? 'var(--accent-blue-dim)' : 'transparent',
    color: throttleSource === src ? 'var(--accent-blue)' : 'var(--text-muted)',
    fontSize: 12, fontWeight: 600, cursor: 'pointer',
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
          <button style={srcBtn('ADC')} onClick={() => setSource('ADC')}>ADC</button>
          <button style={srcBtn('GSP')}
            disabled={!isIdle || !connected}
            onClick={() => setSource('GSP')}>GSP</button>
        </div>
      </div>
    </div>
  );
}
