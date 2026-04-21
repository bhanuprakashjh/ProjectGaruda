/**
 * CK Board Dashboard — real-time telemetry for dsPIC33CK + ATA6847 ESC.
 * Shows eRPM, phase currents, bus voltage, ZC diagnostics, ILIM status.
 * Values use EMA smoothing for readable display.
 */

import { useRef } from 'react';
import { useEscStore } from '../store/useEscStore';
import { CK_ESC_STATES, CK_FAULT_CODES, CK_CURRENT_SCALE, CK_VBUS_SCALE, isV4Firmware } from '../protocol/types';

/* EMA smoothing — factor 0.0-1.0, higher = more responsive.
 * Snaps to zero instantly when raw is 0 (motor stopped). */
function useSmoothed(raw: number, alpha = 0.15): number {
  const ref = useRef<number>(raw);
  if (raw === 0 || !Number.isFinite(raw)) {
    ref.current = 0;
  } else {
    ref.current += alpha * (raw - ref.current);
  }
  return ref.current;
}

function Gauge({ label, value, unit, color, warn, max, decimals = 0 }: {
  label: string; value: number | string; unit: string;
  color?: string; warn?: boolean; max?: number; decimals?: number;
}) {
  const pct = max && typeof value === 'number' ? Math.min(100, (value / max) * 100) : 0;
  const display = typeof value === 'number'
    ? (decimals > 0 ? value.toFixed(decimals) : value.toLocaleString())
    : value;
  return (
    <div style={{
      background: 'var(--bg-card)', borderRadius: 'var(--radius)',
      padding: '12px 16px', border: '1px solid var(--border)',
      minWidth: 140,
    }}>
      <div style={{ fontSize: 10, color: 'var(--text-muted)', letterSpacing: '0.5px', marginBottom: 4 }}>
        {label}
      </div>
      <div style={{
        fontSize: 24, fontWeight: 700, fontFamily: 'var(--font-mono)',
        color: warn ? 'var(--accent-red)' : (color || 'var(--text-primary)'),
      }}>
        {display}
        <span style={{ fontSize: 11, fontWeight: 400, marginLeft: 4, color: 'var(--text-muted)' }}>
          {unit}
        </span>
      </div>
      {max ? (
        <div style={{ marginTop: 6, height: 3, background: 'var(--border)', borderRadius: 2 }}>
          <div style={{
            height: '100%', borderRadius: 2, width: `${pct}%`,
            background: warn ? 'var(--accent-red)' : (color || 'var(--accent-blue)'),
            transition: 'width 0.3s',
          }} />
        </div>
      ) : null}
    </div>
  );
}

function StatusBadge({ label, active, color }: { label: string; active: boolean; color: string }) {
  return (
    <span style={{
      display: 'inline-block', padding: '2px 8px', borderRadius: 4,
      fontSize: 10, fontWeight: 600, letterSpacing: '0.5px',
      background: active ? color : 'var(--bg-secondary)',
      color: active ? '#fff' : 'var(--text-muted)',
      border: `1px solid ${active ? color : 'var(--border)'}`,
    }}>
      {label}
    </span>
  );
}

export function CkDashboard() {
  const snap = useEscStore(s => s.ckSnapshot);
  const info = useEscStore(s => s.info);
  const lastMs = useEscStore(s => s.lastSnapshotMs);

  const stale = Date.now() - lastMs > 2000;

  /* Smoothed values — EMA with alpha=0.15 for gauges.
   * Force zero when motor idle (state 0) so gauges reset on stop. */
  const idle = !snap || snap.state === 0;
  const smErpm = useSmoothed(idle ? 0 : (snap?.eRpm ?? 0), 0.15);
  const smVbus = useSmoothed(snap ? snap.vbusRaw / CK_VBUS_SCALE : 0, 0.15);
  const smDuty = useSmoothed(idle ? 0 : (snap?.dutyPct ?? 0), 0.2);
  /* Bus current is the meaningful value — DC average power draw.
   * Phase currents swing per commutation step; scope shows waveform detail. */
  const smIbus = useSmoothed(idle ? 0 : (snap ? snap.ibusRaw * CK_CURRENT_SCALE : 0), 0.1);

  if (!snap) {
    return (
      <div style={{ padding: 24, textAlign: 'center', color: 'var(--text-muted)' }}>
        Waiting for CK board telemetry...
      </div>
    );
  }

  const state = CK_ESC_STATES[snap.state] || `?${snap.state}`;
  const fault = CK_FAULT_CODES[snap.faultCode] || `?${snap.faultCode}`;
  const mechRpm = info ? Math.round(smErpm / info.motorPolePairs) : Math.round(smErpm);

  return (
    <div style={{ opacity: stale ? 0.5 : 1, transition: 'opacity 0.3s' }}>
      {/* Status bar */}
      <div style={{
        display: 'flex', alignItems: 'center', gap: 8, marginBottom: 16,
        padding: '8px 12px', background: 'var(--bg-card)', borderRadius: 'var(--radius)',
        border: '1px solid var(--border)', flexWrap: 'wrap',
      }}>
        <StatusBadge label={state} active={snap.state >= 3} color={
          snap.state === 6 ? '#ef4444' : snap.state >= 4 ? '#22c55e' : '#3b82f6'
        } />
        <StatusBadge label={snap.zcSynced ? 'ZC SYNCED' : 'NO SYNC'} active={snap.zcSynced} color="#22c55e" />
        <StatusBadge label={snap.ilimActive ? 'ILIM ACTIVE' : 'ILIM OK'} active={snap.ilimActive} color="#f59e0b" />
        {snap.faultCode > 0 && (
          <StatusBadge label={`FAULT: ${fault}`} active={true} color="#ef4444" />
        )}
        <div style={{ flex: 1 }} />
        <span style={{ fontSize: 10, color: 'var(--text-muted)', fontFamily: 'var(--font-mono)' }}>
          Step: {snap.currentStep} | FL: {snap.filterLevel} | Frc: {snap.forcedSteps} | Miss: {snap.missedSteps}
        </span>
        <span style={{ fontSize: 10, color: 'var(--text-muted)' }}>
          Uptime: {snap.uptimeSec}s
        </span>
      </div>

      {/* Main gauges */}
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(150px, 1fr))', gap: 12, marginBottom: 16 }}>
        <Gauge label="eRPM" value={Math.round(smErpm)} unit="" color="var(--accent-cyan)" max={info?.maxErpm || 100000} />
        <Gauge label="Mech RPM" value={mechRpm} unit="RPM" color="var(--accent-blue)" />
        <Gauge label="Duty" value={Math.round(smDuty)} unit="%" color="var(--accent-green)" max={100} />
        <Gauge label="Vbus" value={smVbus} unit="V" color="var(--accent-yellow)" decimals={1}
          warn={snap.vbusRaw < 14000} />
        <Gauge label="Pot" value={snap.potRaw} unit="" max={65535} />
      </div>

      {/* Current & Power */}
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(150px, 1fr))', gap: 12, marginBottom: 16 }}>
        <Gauge label="Bus Current" value={Math.round(smIbus)} unit="mA" color="#f97316"
          warn={snap.ilimActive} max={20000} />
        <Gauge label="Power" value={smVbus > 0 ? +(smVbus * smIbus / 1000).toFixed(1) : 0}
          unit="W" color="#eab308" decimals={1} />
      </div>

      {/* Diagnostics — V4 panel if V4 firmware, else legacy V3 ZC */}
      {info && isV4Firmware(info.featureFlags)
        ? <V4DiagnosticsPanel snap={snap} />
        : <V3ZcDiagnosticsPanel snap={snap} />}
    </div>
  );
}

/* ── V4 Sector PI diagnostics panel ─────────────────────────────
 * Surfaces the runtime tunable PI's actual behaviour:
 *   - Cap% : Commutate sectors that consumed a valid capture (= diagCaptures
 *     / sectorCount). 49% in baseline mode (rising-only feed).
 *   - R/F% : split of total Sets into rising vs falling sectors. Tells you
 *     which polarity is actually producing readable BEMF on this load.
 *   - PI delta : capValue − setValue. Hovers near 0 when PI is converged.
 *   - Bnk%/Mis%/Set% : ADC-fire distribution. Bnk = blanking-window
 *     rejects, Mis = past blanking but wrong GPIO state, Set = successful
 *     captures. Sum to 100% of (relevant) ADC fires.
 *
 * All counters are running totals since the last SectorPI_Start; ratios
 * are computed live from those uint32 totals. */
function V4DiagnosticsPanel({ snap }: { snap: import('../protocol/types').CkSnapshot }) {
  const fires = snap.adcBlankReject + snap.adcStateMismatch + snap.adcCaptureSet;
  const bnkPct = fires > 0 ? (100 * snap.adcBlankReject / fires) : 0;
  const misPct = fires > 0 ? (100 * snap.adcStateMismatch / fires) : 0;
  const setPct = fires > 0 ? (100 * snap.adcCaptureSet / fires) : 0;
  const setRising = snap.adcSetRising;
  const setFalling = Math.max(0, snap.adcCaptureSet - setRising);
  const rfRise = snap.adcCaptureSet > 0 ? (100 * setRising / snap.adcCaptureSet) : 0;
  const rfFall = snap.adcCaptureSet > 0 ? (100 * setFalling / snap.adcCaptureSet) : 0;
  const capPct = snap.goodZcCount > 0 ? Math.round(100 * snap.diagCaptures / snap.goodZcCount) : 0;
  /* Format big counters compactly: 1.23M / 12.3k */
  const fmt = (n: number) =>
    n >= 1_000_000 ? `${(n / 1_000_000).toFixed(2)}M`
    : n >= 10_000 ? `${(n / 1000).toFixed(1)}k`
    : `${n}`;
  /* SP mode bits: bit0 = active, bit1 = request */
  const spActive = (snap.v4SpBits & 1) !== 0;
  const spRequest = (snap.v4SpBits & 2) !== 0;
  /* Color the PI delta to highlight large values */
  const deltaAbs = Math.abs(snap.diagDelta);
  const deltaColor = deltaAbs > 100 ? 'var(--accent-red)'
                  : deltaAbs > 30 ? 'var(--accent-yellow)'
                  : 'var(--accent-green)';

  return (
    <div style={{
      padding: '12px 16px', background: 'var(--bg-card)', borderRadius: 'var(--radius)',
      border: '1px solid var(--border)',
    }}>
      <div style={{
        display: 'flex', alignItems: 'center', gap: 8, marginBottom: 8,
      }}>
        <span style={{
          fontSize: 11, color: 'var(--text-muted)', fontWeight: 600,
          letterSpacing: '0.5px',
        }}>
          V4 Sector-PI Diagnostics
        </span>
        {spActive && <StatusBadge label="SP ACTIVE" active color="#ec4899" />}
        {spRequest && !spActive && <StatusBadge label="SP REQ" active color="#f59e0b" />}
      </div>

      {/* Top row: PI loop state */}
      <div style={{
        display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(110px, 1fr))',
        gap: 8, marginBottom: 12,
      }}>
        <MiniStat label="PI Delta" value={snap.diagDelta} unit="HR" color={deltaColor} />
        <MiniStat label="Last Cap" value={snap.diagLastCapValue} unit="HR" />
        <MiniStat label="timerPeriod" value={snap.stepPeriodHR} unit="HR" />
        <MiniStat label="actual eRPM" value={snap.v4ErpmTp} unit="" />
      </div>

      {/* Cap rate + R/F split */}
      <div style={{
        display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 12, marginBottom: 12,
      }}>
        <PercentBar label={`Cap%  (${snap.diagCaptures} / ${snap.goodZcCount})`}
          value={capPct} color="#22c55e" />
        <PercentSplit label={`R/F%  Sets=${fmt(snap.adcCaptureSet)}`}
          rising={rfRise} falling={rfFall} />
      </div>

      {/* ADC fire distribution: Bnk / Mis / Set */}
      <div style={{ marginBottom: 6 }}>
        <div style={{ fontSize: 10, color: 'var(--text-muted)', marginBottom: 4 }}>
          ADC fire distribution&nbsp;
          <span style={{ fontFamily: 'var(--font-mono)' }}>(total {fmt(fires)})</span>
        </div>
        <div style={{
          display: 'flex', height: 10, borderRadius: 4, overflow: 'hidden',
          background: 'var(--bg-secondary)',
        }}>
          <div style={{ width: `${bnkPct}%`, background: '#64748b' }} title={`Blanked ${bnkPct.toFixed(1)}%`} />
          <div style={{ width: `${misPct}%`, background: '#ef4444' }} title={`State mismatch ${misPct.toFixed(1)}%`} />
          <div style={{ width: `${setPct}%`, background: '#22c55e' }} title={`Captured ${setPct.toFixed(1)}%`} />
        </div>
        <div style={{
          display: 'flex', justifyContent: 'space-between', marginTop: 4,
          fontSize: 10, color: 'var(--text-muted)', fontFamily: 'var(--font-mono)',
        }}>
          <span><span style={{ color: '#64748b' }}>■</span> Bnk {bnkPct.toFixed(0)}%</span>
          <span><span style={{ color: '#ef4444' }}>■</span> Mis {misPct.toFixed(0)}%</span>
          <span><span style={{ color: '#22c55e' }}>■</span> Set {setPct.toFixed(0)}%</span>
        </div>
      </div>
    </div>
  );
}

function MiniStat({ label, value, unit, color }: {
  label: string; value: number | string; unit: string; color?: string;
}) {
  return (
    <div style={{
      padding: '6px 10px', background: 'var(--bg-secondary)',
      borderRadius: 'var(--radius-sm)', border: '1px solid var(--border)',
    }}>
      <div style={{ fontSize: 9, color: 'var(--text-muted)', letterSpacing: '0.4px' }}>
        {label}
      </div>
      <div style={{
        fontSize: 16, fontWeight: 600, fontFamily: 'var(--font-mono)',
        color: color || 'var(--text-primary)', marginTop: 2,
      }}>
        {typeof value === 'number' ? value.toLocaleString() : value}
        {unit && <span style={{ fontSize: 10, color: 'var(--text-muted)', marginLeft: 3 }}>{unit}</span>}
      </div>
    </div>
  );
}

function PercentBar({ label, value, color }: { label: string; value: number; color: string }) {
  return (
    <div style={{
      padding: '8px 10px', background: 'var(--bg-secondary)',
      borderRadius: 'var(--radius-sm)', border: '1px solid var(--border)',
    }}>
      <div style={{
        display: 'flex', justifyContent: 'space-between', marginBottom: 4,
        fontSize: 10, color: 'var(--text-muted)',
      }}>
        <span>{label}</span>
        <strong style={{ color, fontFamily: 'var(--font-mono)' }}>{value}%</strong>
      </div>
      <div style={{ height: 6, background: 'var(--border)', borderRadius: 3 }}>
        <div style={{
          height: '100%', borderRadius: 3, width: `${Math.min(100, value)}%`,
          background: color, transition: 'width 0.3s',
        }} />
      </div>
    </div>
  );
}

function PercentSplit({ label, rising, falling }: {
  label: string; rising: number; falling: number;
}) {
  return (
    <div style={{
      padding: '8px 10px', background: 'var(--bg-secondary)',
      borderRadius: 'var(--radius-sm)', border: '1px solid var(--border)',
    }}>
      <div style={{
        display: 'flex', justifyContent: 'space-between', marginBottom: 4,
        fontSize: 10, color: 'var(--text-muted)',
      }}>
        <span>{label}</span>
        <strong style={{ fontFamily: 'var(--font-mono)' }}>
          <span style={{ color: '#3b82f6' }}>{rising.toFixed(0)}</span>
          <span style={{ color: 'var(--text-muted)' }}>/</span>
          <span style={{ color: '#ec4899' }}>{falling.toFixed(0)}</span>
        </strong>
      </div>
      <div style={{ display: 'flex', height: 6, borderRadius: 3, overflow: 'hidden', background: 'var(--border)' }}>
        <div style={{ width: `${rising}%`, background: '#3b82f6' }} title={`Rising ${rising.toFixed(1)}%`} />
        <div style={{ width: `${falling}%`, background: '#ec4899' }} title={`Falling ${falling.toFixed(1)}%`} />
      </div>
    </div>
  );
}

/* ── V3 legacy ZC diagnostics (preserved when V4 marker absent) ── */
function V3ZcDiagnosticsPanel({ snap }: { snap: import('../protocol/types').CkSnapshot }) {
  return (
    <div style={{
      padding: '12px 16px', background: 'var(--bg-card)', borderRadius: 'var(--radius)',
      border: '1px solid var(--border)',
    }}>
      <div style={{ fontSize: 11, color: 'var(--text-muted)', marginBottom: 8, fontWeight: 600 }}>
        Zero-Crossing Diagnostics
      </div>
      <div style={{
        display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(120px, 1fr))',
        gap: 8, fontSize: 12, fontFamily: 'var(--font-mono)',
      }}>
        <div>Tp: <strong>{snap.stepPeriod}</strong></div>
        <div>TpHR: <strong>{snap.stepPeriodHR}</strong></div>
        <div>ZcI: <strong>{snap.zcInterval}</strong> / PZcI: <strong>{snap.prevZcInterval}</strong></div>
        <div>Good ZC: <strong>{snap.goodZcCount}</strong></div>
        <div>IC Accepted: <strong>{snap.icAccepted}</strong></div>
        <div>IC False: <strong style={{ color: snap.icFalse > 100 ? '#ef4444' : 'inherit' }}>{snap.icFalse}</strong></div>
        <div>ATA Status: <strong>0x{snap.ataStatus.toString(16).padStart(2, '0')}</strong></div>
      </div>
    </div>
  );
}
