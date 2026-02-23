import { useEffect, useState } from 'react';
import { useEscStore } from '../store/useEscStore';

function ArcGauge({ value, max, label, unit, color, stale }: {
  value: number; max: number; label: string; unit: string; color: string; stale?: boolean;
}) {
  const pct = Math.min(value / max, 1);
  const startAngle = 140;
  const sweep = 260;
  const angle = startAngle + pct * sweep;
  const r = 52;
  const cx = 65;
  const cy = 62;

  const toXY = (deg: number) => ({
    x: cx + r * Math.cos((deg * Math.PI) / 180),
    y: cy + r * Math.sin((deg * Math.PI) / 180),
  });

  const bgStart = toXY(startAngle);
  const bgEnd = toXY(startAngle + sweep);
  const valEnd = toXY(angle);

  const bgArc = `M ${bgStart.x} ${bgStart.y} A ${r} ${r} 0 1 1 ${bgEnd.x} ${bgEnd.y}`;
  const valSweep = pct * sweep;
  const largeArc = valSweep > 180 ? 1 : 0;
  const valArc = pct > 0.001
    ? `M ${bgStart.x} ${bgStart.y} A ${r} ${r} 0 ${largeArc} 1 ${valEnd.x} ${valEnd.y}`
    : '';

  const arcColor = stale ? 'var(--text-muted)' : color;

  return (
    <div style={{ textAlign: 'center', flex: 1, opacity: stale ? 0.4 : 1, transition: 'opacity 0.5s' }}>
      <svg width="100%" height="100" viewBox="0 0 130 100">
        <defs>
          <linearGradient id={`grad-${label}`} x1="0%" y1="0%" x2="100%" y2="0%">
            <stop offset="0%" style={{ stopColor: arcColor, stopOpacity: 0.4 }} />
            <stop offset="100%" style={{ stopColor: arcColor, stopOpacity: 1 }} />
          </linearGradient>
        </defs>
        <path d={bgArc} fill="none" stroke="var(--border)" strokeWidth={8} strokeLinecap="round" />
        {valArc && (
          <path d={valArc} fill="none" stroke={`url(#grad-${label})`} strokeWidth={8} strokeLinecap="round" />
        )}
        <text x={cx} y={cy - 2} textAnchor="middle" fill="var(--text-primary)" fontSize={20} fontWeight={700}
          fontFamily="var(--font-mono)" opacity={stale ? 0.4 : 1}>
          {value.toLocaleString()}
        </text>
        <text x={cx} y={cy + 14} textAnchor="middle" fill="var(--text-muted)" fontSize={10}>
          {unit}
        </text>
      </svg>
      <div style={{ fontSize: 11, color: 'var(--text-secondary)', marginTop: -4, fontWeight: 500 }}>{label}</div>
    </div>
  );
}

function StatCard({ label, value, unit, color, sub, stale }: {
  label: string; value: string; unit: string; color: string; sub?: string; stale?: boolean;
}) {
  return (
    <div style={{
      textAlign: 'center', flex: 1, padding: '12px 8px',
      borderRadius: 'var(--radius-sm)', background: 'rgba(255,255,255,0.02)',
      opacity: stale ? 0.4 : 1, transition: 'opacity 0.5s',
    }}>
      <div style={{ fontSize: 10, color: 'var(--text-muted)', marginBottom: 4, textTransform: 'uppercase', letterSpacing: '0.5px' }}>{label}</div>
      <div style={{ fontSize: 28, fontWeight: 700, color: stale ? 'var(--text-muted)' : color, fontFamily: 'var(--font-mono)', lineHeight: 1 }}>{value}</div>
      <div style={{ fontSize: 10, color: 'var(--text-muted)', marginTop: 2 }}>{unit}</div>
      {sub && <div style={{ fontSize: 10, color: 'var(--text-muted)', marginTop: 2 }}>{sub}</div>}
    </div>
  );
}

export function GaugePanel() {
  const snapshot = useEscStore(s => s.snapshot);
  const lastSnapshotMs = useEscStore(s => s.lastSnapshotMs);
  const telemActive = useEscStore(s => s.telemActive);
  const info = useEscStore(s => s.info);
  const [now, setNow] = useState(Date.now());

  // Tick every second to update staleness
  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(timer);
  }, []);

  const isStale = !telemActive || (lastSnapshotMs > 0 && (now - lastSnapshotMs) > 2000);

  if (!snapshot) {
    return (
      <div style={{
        background: 'var(--bg-card)', borderRadius: 'var(--radius)',
        padding: 24, border: '1px solid var(--border)',
        display: 'flex', alignItems: 'center', justifyContent: 'center',
        color: 'var(--text-muted)', fontSize: 13,
      }}>
        Connect to ESC and start telemetry to see live data
      </div>
    );
  }

  const eRPM = snapshot.stepPeriod > 0 ? Math.round(240000 / snapshot.stepPeriod) : 0;
  const vbus = (snapshot.vbusRaw * 3.3 / 4096 * 19.8);
  const ibus = ((snapshot.ibusRaw - 2048) / 93.0);
  const polePairs = info?.motorPolePairs ?? 1;
  const mechRPM = polePairs > 0 ? Math.round(eRPM / polePairs) : eRPM;

  return (
    <div style={{
      background: 'var(--bg-card)', borderRadius: 'var(--radius)',
      padding: '16px 12px', border: '1px solid var(--border)',
      position: 'relative',
    }}>
      {/* Stale badge */}
      {isStale && (
        <div style={{
          position: 'absolute', top: 8, right: 12,
          fontSize: 10, fontWeight: 600, textTransform: 'uppercase',
          letterSpacing: '0.5px', color: 'var(--accent-yellow)',
          background: 'rgba(234,179,8,0.1)', padding: '2px 8px',
          borderRadius: 10, border: '1px solid rgba(234,179,8,0.2)',
        }}>
          {telemActive ? 'NO DATA' : 'STALE'}
        </div>
      )}
      <div style={{ display: 'flex', alignItems: 'center', gap: 4 }}>
        <ArcGauge value={eRPM} max={info?.maxErpm ?? 30000} label="eRPM" unit="eRPM" color="#3b82f6" stale={isStale} />
        <StatCard label="Vbus" value={vbus.toFixed(1)} unit="V" color="var(--accent-green)"
          sub={`${snapshot.vbusRaw} ADC`} stale={isStale} />
        <StatCard label="Ibus" value={ibus.toFixed(2)} unit="A" color="var(--accent-yellow)"
          sub={`peak ${((snapshot.ibusMax - 2048) / 93.0).toFixed(1)} A`} stale={isStale} />
        <ArcGauge value={snapshot.dutyPct} max={100} label="Duty" unit="%" color="#f472b6" stale={isStale} />
        <StatCard label="mRPM" value={mechRPM.toLocaleString()} unit="RPM" color="var(--accent-cyan)"
          sub={`${polePairs}pp`} stale={isStale} />
      </div>
    </div>
  );
}
