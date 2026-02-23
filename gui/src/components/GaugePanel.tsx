import { useEscStore } from '../store/useEscStore';

export function GaugePanel() {
  const snapshot = useEscStore(s => s.snapshot);
  if (!snapshot) return <div style={{ background: '#16213e', borderRadius: 8, padding: 16 }}>No data</div>;

  const eRPM = snapshot.stepPeriod > 0 ? Math.round(240000 / snapshot.stepPeriod) : 0;
  const vbus = (snapshot.vbusRaw * 3.3 / 4096 * 19.8).toFixed(1); // typical divider
  const ibus = ((snapshot.ibusRaw - 2048) / 93.0).toFixed(1); // OA3 biased

  const gaugeStyle: React.CSSProperties = {
    background: '#16213e', borderRadius: 8, padding: 16,
    display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 12,
  };
  const bigNum: React.CSSProperties = { fontSize: 28, fontWeight: 700, color: '#60a5fa' };

  return (
    <div style={gaugeStyle}>
      <div><div style={{ fontSize: 12, color: '#888' }}>eRPM</div><div style={bigNum}>{eRPM.toLocaleString()}</div></div>
      <div><div style={{ fontSize: 12, color: '#888' }}>Vbus</div><div style={bigNum}>{vbus} V</div></div>
      <div><div style={{ fontSize: 12, color: '#888' }}>Ibus</div><div style={bigNum}>{ibus} A</div></div>
      <div><div style={{ fontSize: 12, color: '#888' }}>Duty</div><div style={bigNum}>{snapshot.dutyPct}%</div></div>
    </div>
  );
}
