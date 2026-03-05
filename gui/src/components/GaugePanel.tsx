import { useEffect, useState, useRef } from 'react';
import { useEscStore } from '../store/useEscStore';
import { isFocEnabled, FOC_SUB_STATES } from '../protocol/types';

/** EMA smoothing factor — 0 = frozen, 1 = no smoothing. 0.08 gives ~12-sample lag. */
const ALPHA = 0.08;
function ema(prev: number, cur: number): number {
  return prev + ALPHA * (cur - prev);
}

function ArcGauge({ value, max, label, unit, color, stale, format }: {
  value: number; max: number; label: string; unit: string; color: string; stale?: boolean;
  format?: (v: number) => string;
}) {
  const pct = Math.min(Math.abs(value) / max, 1);
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
  const displayVal = format ? format(value) : value.toLocaleString();

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
          <path d={valArc} fill="none" stroke={`url(#grad-${label})`} strokeWidth={8} strokeLinecap="round"
            style={{ transition: 'd 0.3s ease-out' }} />
        )}
        <text x={cx} y={cy - 2} textAnchor="middle" fill="var(--text-primary)" fontSize={18} fontWeight={700}
          fontFamily="var(--font-mono)" opacity={stale ? 0.4 : 1}>
          {displayVal}
        </text>
        <text x={cx} y={cy + 14} textAnchor="middle" fill="var(--text-muted)" fontSize={10}>
          {unit}
        </text>
      </svg>
      <div style={{ fontSize: 10, color: 'var(--text-secondary)', marginTop: -4, fontWeight: 500 }}>{label}</div>
    </div>
  );
}

function StatCard({ label, value, unit, color, sub, stale }: {
  label: string; value: string; unit: string; color: string; sub?: string; stale?: boolean;
}) {
  return (
    <div style={{
      textAlign: 'center', flex: 1, padding: '10px 6px',
      borderRadius: 'var(--radius-sm)', background: 'rgba(255,255,255,0.02)',
      opacity: stale ? 0.4 : 1, transition: 'opacity 0.5s', minWidth: 90,
    }}>
      <div style={{ fontSize: 9, color: 'var(--text-muted)', marginBottom: 3, textTransform: 'uppercase', letterSpacing: '0.5px' }}>{label}</div>
      <div style={{ fontSize: 22, fontWeight: 700, color: stale ? 'var(--text-muted)' : color, fontFamily: 'var(--font-mono)', lineHeight: 1 }}>{value}</div>
      <div style={{ fontSize: 9, color: 'var(--text-muted)', marginTop: 2 }}>{unit}</div>
      {sub && <div style={{ fontSize: 9, color: 'var(--text-muted)', marginTop: 1 }}>{sub}</div>}
    </div>
  );
}

export function GaugePanel() {
  const snapshot = useEscStore(s => s.snapshot);
  const lastSnapshotMs = useEscStore(s => s.lastSnapshotMs);
  const telemActive = useEscStore(s => s.telemActive);
  const info = useEscStore(s => s.info);
  const [now, setNow] = useState(Date.now());

  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(timer);
  }, []);

  const isStale = !telemActive || (lastSnapshotMs > 0 && (now - lastSnapshotMs) > 2000);
  const focMode = info ? isFocEnabled(info.featureFlags) : false;

  // useRef must be called unconditionally (before any early return)
  const smoothed = useRef({
    vbus: 0, ibus: 0, ibusPeak: 0,
    focRpm: 0, focSpeedRads: 0, focPowerW: 0,
    focIq: 0, focId: 0, focIa: 0, focIb: 0,
    focTheta: 0, focThetaObs: 0, focVbus: 0,
    eRPM: 0, mechRPM: 0, duty: 0,
  });

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

  const polePairs = info?.motorPolePairs ?? 1;

  // Raw values
  const rawVbus = snapshot.vbusRaw * 3.3 / 4096 * 19.8;
  const rawIbus = (snapshot.ibusRaw - 2048) / 93.0;
  const rawIbusPeak = (snapshot.ibusMax - 2048) / 93.0;
  const rawFocRpm = focMode && snapshot.focOmega !== 0
    ? Math.abs(snapshot.focOmega) * 60 / (2 * Math.PI * polePairs) : 0;
  const rawFocSpeedRads = focMode ? Math.abs(snapshot.focOmega) : 0;
  const rawFocPowerW = focMode ? 1.5 * snapshot.focVbus * snapshot.focIqMeas * (snapshot.focOmega > 0 ? 1 : -1) : 0;
  const rawERPM = snapshot.stepPeriod > 0 ? 240000 / snapshot.stepPeriod : 0;
  const rawMechRPM = polePairs > 0 ? rawERPM / polePairs : rawERPM;
  const s = smoothed.current;
  s.vbus = ema(s.vbus, rawVbus);
  s.ibus = ema(s.ibus, rawIbus);
  s.ibusPeak = ema(s.ibusPeak, rawIbusPeak);
  s.focRpm = ema(s.focRpm, rawFocRpm);
  s.focSpeedRads = ema(s.focSpeedRads, rawFocSpeedRads);
  s.focPowerW = ema(s.focPowerW, rawFocPowerW);
  s.focIq = ema(s.focIq, snapshot.focIqMeas);
  s.focId = ema(s.focId, snapshot.focIdMeas);
  s.focIa = ema(s.focIa, snapshot.focIa);
  s.focIb = ema(s.focIb, snapshot.focIb);
  s.focTheta = ema(s.focTheta, snapshot.focTheta);
  s.focThetaObs = ema(s.focThetaObs, snapshot.focThetaObs);
  s.focVbus = ema(s.focVbus, snapshot.focVbus);
  s.eRPM = ema(s.eRPM, rawERPM);
  s.mechRPM = ema(s.mechRPM, rawMechRPM);
  // In FOC mode, if firmware dutyPct is 0 but motor is running, estimate from throttle
  const rawDuty = (focMode && snapshot.dutyPct === 0 && snapshot.throttle > 0)
    ? (snapshot.throttle / 4095 * 100)
    : snapshot.dutyPct;
  s.duty = ema(s.duty, rawDuty);

  const vbus = s.vbus;
  const ibus = s.ibus;
  const ibusPeak = s.ibusPeak;
  const focRpm = Math.round(s.focRpm);
  const focSpeedRads = s.focSpeedRads;
  const focPowerW = s.focPowerW;
  const focSubStr = FOC_SUB_STATES[snapshot.focSubState] ?? '?';
  const eRPM = Math.round(s.eRPM);
  const mechRPM = Math.round(s.mechRPM);

  return (
    <div style={{
      background: 'var(--bg-card)', borderRadius: 'var(--radius)',
      padding: '12px 8px', border: '1px solid var(--border)',
      position: 'relative',
    }}>
      {/* Mode badge */}
      <div style={{
        position: 'absolute', top: 6, left: 12,
        fontSize: 9, fontWeight: 700, textTransform: 'uppercase',
        letterSpacing: '0.5px',
        color: focMode ? 'var(--accent-cyan)' : 'var(--accent-orange)',
        background: focMode ? 'rgba(34,211,238,0.1)' : 'rgba(249,115,22,0.1)',
        padding: '1px 6px', borderRadius: 8,
        border: `1px solid ${focMode ? 'rgba(34,211,238,0.2)' : 'rgba(249,115,22,0.2)'}`,
      }}>
        {focMode ? 'FOC' : '6-STEP'}
        {focMode && <span style={{ marginLeft: 4, opacity: 0.7 }}>{focSubStr}</span>}
      </div>

      {/* Stale badge */}
      {isStale && (
        <div style={{
          position: 'absolute', top: 6, right: 12,
          fontSize: 9, fontWeight: 600, textTransform: 'uppercase',
          letterSpacing: '0.5px', color: 'var(--accent-yellow)',
          background: 'rgba(234,179,8,0.1)', padding: '1px 6px',
          borderRadius: 8, border: '1px solid rgba(234,179,8,0.2)',
        }}>
          {telemActive ? 'NO DATA' : 'STALE'}
        </div>
      )}

      {/* Primary gauges row */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 2, marginTop: 12 }}>
        {focMode ? (
          <>
            <ArcGauge value={focRpm} max={info?.maxErpm ? Math.round(info.maxErpm / polePairs) : 20000}
              label="Speed" unit="RPM" color="#3b82f6" stale={isStale} />
            <StatCard label="Iq" value={s.focIq.toFixed(1)} unit="A" color="var(--accent-yellow)"
              sub={`torque current`} stale={isStale} />
            <StatCard label="Id" value={s.focId.toFixed(1)} unit="A"
              color={Math.abs(s.focId) > 1 ? 'var(--accent-red)' : 'var(--accent-green)'}
              sub={`field (target: 0)`} stale={isStale} />
            <StatCard label="Vbus" value={(focMode ? s.focVbus : vbus).toFixed(1)} unit="V" color="var(--accent-green)"
              sub={`${snapshot.vbusRaw} ADC`} stale={isStale} />
            <ArcGauge value={s.duty} max={100} label="Duty" unit="%" color="#f472b6" stale={isStale}
              format={v => Math.round(v).toString()} />
            <StatCard label="Power" value={Math.round(Math.abs(focPowerW)).toString()} unit="W" color="var(--accent-cyan)"
              sub={`${ibus.toFixed(1)}A bus`} stale={isStale} />
          </>
        ) : (
          <>
            <ArcGauge value={eRPM} max={info?.maxErpm ?? 30000} label="eRPM" unit="eRPM" color="#3b82f6" stale={isStale} />
            <StatCard label="Vbus" value={vbus.toFixed(1)} unit="V" color="var(--accent-green)"
              sub={`${snapshot.vbusRaw} ADC`} stale={isStale} />
            <StatCard label="Ibus" value={ibus.toFixed(1)} unit="A" color="var(--accent-yellow)"
              sub={`peak ${ibusPeak.toFixed(1)}A`} stale={isStale} />
            <ArcGauge value={s.duty} max={100} label="Duty" unit="%" color="#f472b6" stale={isStale}
              format={v => Math.round(v).toString()} />
            <StatCard label="mRPM" value={mechRPM.toLocaleString()} unit="RPM" color="var(--accent-cyan)"
              sub={`${polePairs}pp`} stale={isStale} />
          </>
        )}
      </div>

      {/* FOC secondary row */}
      {focMode && snapshot.focSubState >= 3 && (
        <div style={{
          display: 'flex', gap: 2, marginTop: 6,
          padding: '6px 4px', background: 'rgba(255,255,255,0.01)', borderRadius: 'var(--radius-sm)',
        }}>
          <StatCard label="Ia" value={s.focIa.toFixed(1)} unit="A" color="#a78bfa" stale={isStale} />
          <StatCard label="Ib" value={s.focIb.toFixed(1)} unit="A" color="#c084fc" stale={isStale} />
          <StatCard label="Theta" value={Math.round(s.focTheta * 180 / Math.PI).toString()} unit="deg" color="#22d3ee" stale={isStale} />
          <StatCard label="Obs Theta" value={Math.round(s.focThetaObs * 180 / Math.PI).toString()} unit="deg" color="#06b6d4" stale={isStale} />
          <StatCard label="Speed" value={Math.round(focSpeedRads).toString()} unit="rad/s" color="#60a5fa" stale={isStale} />
          <StatCard label="Ibus" value={ibus.toFixed(1)} unit="A" color="var(--accent-orange)"
            sub={`peak ${ibusPeak.toFixed(1)}A`} stale={isStale} />
        </div>
      )}
    </div>
  );
}
