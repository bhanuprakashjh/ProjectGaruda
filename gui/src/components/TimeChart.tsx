import { useEscStore } from '../store/useEscStore';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';

export function TimeChart() {
  const history = useEscStore(s => s.history);
  const data = history.map((s, i) => ({
    t: i,
    eRPM: s.stepPeriod > 0 ? Math.round(240000 / s.stepPeriod) : 0,
    duty: s.dutyPct,
  }));

  return (
    <div style={{ background: '#16213e', borderRadius: 8, padding: 16, marginTop: 16 }}>
      <h3 style={{ marginBottom: 8 }}>Telemetry</h3>
      <ResponsiveContainer width="100%" height={200}>
        <LineChart data={data}>
          <CartesianGrid strokeDasharray="3 3" stroke="#333" />
          <XAxis dataKey="t" tick={false} />
          <YAxis yAxisId="left" stroke="#60a5fa" />
          <YAxis yAxisId="right" orientation="right" stroke="#f472b6" />
          <Tooltip contentStyle={{ background: '#1a1a2e', border: '1px solid #333' }} />
          <Line yAxisId="left" type="monotone" dataKey="eRPM" stroke="#60a5fa" dot={false} />
          <Line yAxisId="right" type="monotone" dataKey="duty" stroke="#f472b6" dot={false} />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
