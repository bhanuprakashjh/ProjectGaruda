import { useEscStore } from '../store/useEscStore';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Legend } from 'recharts';

export function TimeChart() {
  const history = useEscStore(s => s.history);
  const data = history.map((s, i) => ({
    t: (i * 0.02).toFixed(1),
    eRPM: s.stepPeriod > 0 ? Math.round(240000 / s.stepPeriod) : 0,
    duty: s.dutyPct,
    ibus: ((s.ibusRaw - 2048) / 93.0),
  }));

  return (
    <div style={{
      background: 'var(--bg-card)', borderRadius: 'var(--radius)',
      padding: 16, marginTop: 16, border: '1px solid var(--border)',
    }}>
      <div style={{
        fontSize: 13, color: 'var(--text-muted)', textTransform: 'uppercase',
        letterSpacing: '1px', marginBottom: 8,
      }}>
        Live Telemetry
      </div>
      <ResponsiveContainer width="100%" height={200}>
        <LineChart data={data} margin={{ top: 4, right: 8, left: 0, bottom: 0 }}>
          <CartesianGrid strokeDasharray="3 3" stroke="var(--border-light)" />
          <XAxis
            dataKey="t"
            tick={{ fill: 'var(--text-muted)', fontSize: 10 }}
            label={{ value: 'Time (s)', position: 'insideBottomRight', offset: -4, fill: 'var(--text-muted)', fontSize: 10 }}
          />
          <YAxis
            yAxisId="left"
            tick={{ fill: '#3b82f6', fontSize: 10 }}
            label={{ value: 'eRPM', angle: -90, position: 'insideLeft', fill: '#3b82f6', fontSize: 10 }}
          />
          <YAxis
            yAxisId="right" orientation="right" domain={[0, 100]}
            tick={{ fill: '#f472b6', fontSize: 10 }}
            label={{ value: 'Duty %', angle: 90, position: 'insideRight', fill: '#f472b6', fontSize: 10 }}
          />
          <Tooltip
            contentStyle={{
              background: 'var(--bg-secondary)', border: '1px solid var(--border)',
              borderRadius: 'var(--radius-sm)', fontSize: 11,
              fontFamily: 'var(--font-mono)',
            }}
          />
          <Legend wrapperStyle={{ fontSize: 11, paddingTop: 4 }} />
          <Line yAxisId="left" type="monotone" dataKey="eRPM" stroke="#3b82f6" strokeWidth={1.5} dot={false} isAnimationActive={false} />
          <Line yAxisId="right" type="monotone" dataKey="duty" stroke="#f472b6" strokeWidth={1.5} dot={false} isAnimationActive={false} />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
