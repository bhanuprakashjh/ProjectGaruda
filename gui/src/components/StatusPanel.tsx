import { useEscStore } from '../store/useEscStore';
import { ESC_STATES, FAULT_CODES } from '../protocol/types';

export function StatusPanel() {
  const { snapshot, info } = useEscStore();
  const state = snapshot ? ESC_STATES[snapshot.state] ?? 'UNKNOWN' : '—';
  const fault = snapshot ? FAULT_CODES[snapshot.faultCode] ?? 'UNKNOWN' : '—';
  const uptime = snapshot ? snapshot.uptimeSec : 0;
  const fw = info ? `${info.fwMajor}.${info.fwMinor}.${info.fwPatch}` : '—';

  const panelStyle: React.CSSProperties = {
    background: '#16213e', borderRadius: 8, padding: 16,
  };

  return (
    <div style={panelStyle}>
      <h3 style={{ marginBottom: 8 }}>Status</h3>
      <div>State: <strong>{state}</strong></div>
      <div>Fault: <strong>{fault}</strong></div>
      <div>Uptime: {uptime}s</div>
      <div>FW: {fw}</div>
    </div>
  );
}
