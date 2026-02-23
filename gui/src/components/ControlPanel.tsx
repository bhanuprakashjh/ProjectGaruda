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

  const btnStyle: React.CSSProperties = {
    padding: '8px 16px', borderRadius: 4, border: 'none',
    cursor: 'pointer', fontWeight: 600,
  };

  return (
    <div style={{ background: '#16213e', borderRadius: 8, padding: 16, marginTop: 16, display: 'flex', gap: 12, alignItems: 'center', flexWrap: 'wrap' }}>
      <button style={{ ...btnStyle, background: '#22c55e', color: '#000' }} disabled={!isIdle || !connected}
        onClick={() => send(CMD.START_MOTOR)}>Start Motor</button>
      <button style={{ ...btnStyle, background: '#ef4444', color: '#fff' }} disabled={!connected}
        onClick={() => send(CMD.STOP_MOTOR)}>Stop Motor</button>
      <button style={{ ...btnStyle, background: '#eab308', color: '#000' }} disabled={!isFault || !connected}
        onClick={() => send(CMD.CLEAR_FAULT)}>Clear Fault</button>
      <div style={{ marginLeft: 'auto', display: 'flex', gap: 8, alignItems: 'center' }}>
        <span>Throttle:</span>
        <button style={{ ...btnStyle, background: throttleSource === 'ADC' ? '#3b82f6' : '#333' }}
          onClick={() => setSource('ADC')}>ADC</button>
        <button style={{ ...btnStyle, background: throttleSource === 'GSP' ? '#3b82f6' : '#333' }}
          disabled={!isIdle || !connected} onClick={() => setSource('GSP')}>GSP</button>
      </div>
    </div>
  );
}
