/**
 * CK Board Diagnostic Scope — stacked oscilloscope-style charts
 * for commutation analysis and motor debugging.
 *
 * 3 stacked panels:
 *   Top:    Speed & Control (eRPM, duty, step period)
 *   Middle: Currents & Voltage (Ia, Ib, Ibus, Vbus)
 *   Bottom: ZC Diagnostics (missed, forced, filter level, ZC alternation)
 *
 * Computed diagnostic channels:
 *   - ZC Alternation: |zcInterval - prevZcInterval| — key desync indicator
 *   - Comm Step: 0-5 sawtooth — shows smooth commutation
 *   - ZC Health: composite score (0=desync, 100=perfect)
 */

import { useState, useMemo } from 'react';
import { useEscStore } from '../store/useEscStore';
import { CK_CURRENT_SCALE, CK_VBUS_SCALE } from '../protocol/types';
import type { CkSnapshot } from '../protocol/types';
import {
  LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip,
  ResponsiveContainer, ReferenceLine,
} from 'recharts';

/* ── Channel definitions ─────────────────────────────────────────── */

interface CkChannel {
  key: string;
  label: string;
  unit: string;
  color: string;
  extract: (s: CkSnapshot, pp: number) => number;
  panel: 'speed' | 'current' | 'zc' | 'v4pi';
}

const CHANNELS: CkChannel[] = [
  /* Speed & Control */
  { key: 'erpm', label: 'eRPM', unit: '', color: '#3b82f6', panel: 'speed',
    extract: (s) => s.eRpm },
  { key: 'mechRpm', label: 'Mech RPM', unit: 'RPM', color: '#60a5fa', panel: 'speed',
    extract: (s, pp) => pp > 0 ? Math.round(s.eRpm / pp) : s.eRpm },
  { key: 'duty', label: 'Duty', unit: '%', color: '#f472b6', panel: 'speed',
    extract: (s) => s.dutyPct },
  { key: 'stepPeriod', label: 'Step Period', unit: 'T1 ticks', color: '#94a3b8', panel: 'speed',
    extract: (s) => s.stepPeriod },
  { key: 'stepPeriodHR', label: 'Step Period HR', unit: 'HR ticks', color: '#64748b', panel: 'speed',
    extract: (s) => s.stepPeriodHR },
  { key: 'commStep', label: 'Comm Step', unit: '', color: '#a78bfa', panel: 'speed',
    extract: (s) => s.currentStep },
  { key: 'pot', label: 'Pot (raw)', unit: '', color: '#c084fc', panel: 'speed',
    extract: (s) => s.potRaw },

  /* Currents & Voltage */
  { key: 'ia', label: 'Phase A (Ia)', unit: 'mA', color: '#60a5fa', panel: 'current',
    extract: (s) => Math.round(s.iaRaw * CK_CURRENT_SCALE) },
  { key: 'ib', label: 'Phase B (Ib)', unit: 'mA', color: '#34d399', panel: 'current',
    extract: (s) => Math.round(s.ibRaw * CK_CURRENT_SCALE) },
  { key: 'ibus', label: 'Bus Current', unit: 'mA', color: '#f97316', panel: 'current',
    extract: (s) => Math.round(s.ibusRaw * CK_CURRENT_SCALE) },
  { key: 'vbus', label: 'Vbus', unit: 'V', color: '#22c55e', panel: 'current',
    extract: (s) => +(s.vbusRaw / CK_VBUS_SCALE).toFixed(2) },

  /* ZC Diagnostics */
  { key: 'goodZc', label: 'Good ZC Count', unit: '', color: '#22d3ee', panel: 'zc',
    extract: (s) => s.goodZcCount },
  { key: 'zcInterval', label: 'ZC Interval', unit: '', color: '#2dd4bf', panel: 'zc',
    extract: (s) => s.zcInterval },
  { key: 'zcAlternation', label: 'ZC Alternation', unit: '', color: '#eab308', panel: 'zc',
    extract: (s) => Math.abs(s.zcInterval - s.prevZcInterval) },
  { key: 'filterLevel', label: 'Filter Level', unit: '', color: '#a78bfa', panel: 'zc',
    extract: (s) => s.filterLevel },
  { key: 'missed', label: 'Missed Steps', unit: '', color: '#ef4444', panel: 'zc',
    extract: (s) => s.missedSteps },
  { key: 'forced', label: 'Forced Steps', unit: '', color: '#f43f5e', panel: 'zc',
    extract: (s) => s.forcedSteps },
  { key: 'icAccepted', label: 'IC Accepted', unit: '', color: '#06b6d4', panel: 'zc',
    extract: (s) => s.icAccepted },
  { key: 'icFalse', label: 'IC False', unit: '', color: '#dc2626', panel: 'zc',
    extract: (s) => s.icFalse },
  { key: 'zcHealth', label: 'ZC Health', unit: '%', color: '#10b981', panel: 'zc',
    extract: (s) => {
      /* Composite: 100 = perfect sync, 0 = total desync */
      let score = 100;
      if (!s.zcSynced) score -= 40;
      score -= Math.min(30, s.missedSteps * 10);
      score -= Math.min(20, s.forcedSteps * 5);
      const alt = s.zcInterval > 0 ? Math.abs(s.zcInterval - s.prevZcInterval) / s.zcInterval : 0;
      score -= Math.min(10, alt * 50);
      return Math.max(0, Math.round(score));
    }},

  /* ── V4 Sector-PI channels ────────────────────────────────────
   * All sourced from V4-specific snapshot fields decoded in
   * decode.ts. Visible in the GUI even on V3 firmware (will read
   * 0) — but the V4-only preset and the V4 Sector PI panel make
   * them obvious where they belong. */
  { key: 'v4Delta', label: 'PI Delta', unit: 'HR', color: '#ef4444', panel: 'v4pi',
    extract: (s) => s.diagDelta },
  { key: 'v4LastCap', label: 'Last CapValue', unit: 'HR', color: '#a78bfa', panel: 'v4pi',
    extract: (s) => s.diagLastCapValue },
  { key: 'v4TimerPeriod', label: 'timerPeriod', unit: 'HR', color: '#22d3ee', panel: 'v4pi',
    extract: (s) => s.stepPeriodHR },
  { key: 'v4ErpmTp', label: 'eRPM (actual)', unit: '', color: '#3b82f6', panel: 'v4pi',
    extract: (s) => s.v4ErpmTp },
  { key: 'v4CapPct', label: 'Cap%', unit: '%', color: '#22c55e', panel: 'v4pi',
    extract: (s) => s.goodZcCount > 0 ? Math.round(100 * s.diagCaptures / s.goodZcCount) : 0 },
  { key: 'v4SetPct', label: 'Set% (of ADC fires)', unit: '%', color: '#84cc16', panel: 'v4pi',
    extract: (s) => {
      const total = s.adcBlankReject + s.adcStateMismatch + s.adcCaptureSet;
      return total > 0 ? +(100 * s.adcCaptureSet / total).toFixed(1) : 0;
    }},
  { key: 'v4MisPct', label: 'Mis% (of ADC fires)', unit: '%', color: '#f97316', panel: 'v4pi',
    extract: (s) => {
      const total = s.adcBlankReject + s.adcStateMismatch + s.adcCaptureSet;
      return total > 0 ? +(100 * s.adcStateMismatch / total).toFixed(1) : 0;
    }},
  { key: 'v4BnkPct', label: 'Bnk% (of ADC fires)', unit: '%', color: '#94a3b8', panel: 'v4pi',
    extract: (s) => {
      const total = s.adcBlankReject + s.adcStateMismatch + s.adcCaptureSet;
      return total > 0 ? +(100 * s.adcBlankReject / total).toFixed(1) : 0;
    }},
  { key: 'v4RisingPct', label: 'Rising % (of Sets)', unit: '%', color: '#3b82f6', panel: 'v4pi',
    extract: (s) => s.adcCaptureSet > 0 ? +(100 * s.adcSetRising / s.adcCaptureSet).toFixed(1) : 0 },
];

/* ── Presets ──────────────────────────────────────────────────────── */

interface Preset {
  label: string;
  channels: string[];
  description: string;
}

const PRESETS: Record<string, Preset> = {
  commHealth: {
    label: 'Commutation Health',
    channels: ['erpm', 'missed', 'forced', 'zcHealth'],
    description: 'Overall commutation quality — health score drops on desync',
  },
  startup: {
    label: 'Startup Debug',
    channels: ['erpm', 'duty', 'stepPeriod', 'goodZc'],
    description: 'OL ramp → CL handoff — watch stepPeriod decrease as eRPM rises',
  },
  zcQuality: {
    label: 'ZC Quality',
    channels: ['zcInterval', 'zcAlternation', 'filterLevel', 'icAccepted'],
    description: 'Zero-crossing detection — alternation should be near 0 when synced',
  },
  currents: {
    label: 'Current Analysis',
    channels: ['ia', 'ib', 'ibus', 'duty'],
    description: 'Phase currents vs duty — check ILIM behavior and current balance',
  },
  power: {
    label: 'Power & Speed',
    channels: ['erpm', 'duty', 'vbus', 'ibus'],
    description: 'Speed, duty, bus voltage and current — full power picture',
  },
  fullDiag: {
    label: 'Full Diagnostic',
    channels: ['erpm', 'duty', 'ia', 'ib', 'ibus', 'vbus', 'missed', 'forced', 'zcHealth'],
    description: 'Everything — all 3 panels active',
  },
  v4Pi: {
    label: 'V4 PI Loop',
    channels: ['v4Delta', 'v4LastCap', 'v4TimerPeriod', 'v4ErpmTp'],
    description: 'V4 sector PI internals — delta should hover near 0; cap/period track each other',
  },
  v4Capture: {
    label: 'V4 Capture Mix',
    channels: ['v4CapPct', 'v4SetPct', 'v4MisPct', 'v4BnkPct', 'v4RisingPct'],
    description: 'V4 ADC-fire distribution — Cap% from Commutate side, Set/Mis/Bnk from ADC side, Rising% shows which polarity dominates Sets',
  },
};

/* ── Styles ───────────────────────────────────────────────────────── */

const panelStyle: React.CSSProperties = {
  background: 'var(--bg-card)', borderRadius: 'var(--radius)',
  border: '1px solid var(--border)', padding: '8px 12px',
};

const chipStyle = (active: boolean, color: string): React.CSSProperties => ({
  padding: '2px 8px', borderRadius: 3, fontSize: 10, fontWeight: 600,
  cursor: 'pointer', transition: 'all 0.1s', letterSpacing: '0.3px',
  border: `1px solid ${active ? color : 'var(--border)'}`,
  background: active ? color + '22' : 'transparent',
  color: active ? color : 'var(--text-muted)',
});

/* ── Component ────────────────────────────────────────────────────── */

export function CkScopePanel() {
  const ckHistory = useEscStore(s => s.ckHistory);
  const info = useEscStore(s => s.info);
  const telemActive = useEscStore(s => s.telemActive);
  const pp = info?.motorPolePairs ?? 1;

  const [active, setActive] = useState<Set<string>>(
    new Set(PRESETS.commHealth.channels)
  );
  const [expanded, setExpanded] = useState(true);
  const [presetDesc, setPresetDesc] = useState(PRESETS.commHealth.description);

  const toggle = (key: string) => {
    setActive(prev => {
      const next = new Set(prev);
      if (next.has(key)) next.delete(key); else next.add(key);
      return next;
    });
    setPresetDesc('');
  };

  const applyPreset = (p: Preset) => {
    setActive(new Set(p.channels));
    setPresetDesc(p.description);
  };

  /* Build chart data — one entry per snapshot, grouped by panel */
  const activeChannels = useMemo(
    () => CHANNELS.filter(ch => active.has(ch.key)),
    [active]
  );

  const panels = ['speed', 'current', 'zc', 'v4pi'] as const;
  const panelLabels: Record<string, string> = {
    speed: 'Speed & Control',
    current: 'Currents & Voltage',
    zc: 'ZC Diagnostics',
    v4pi: 'V4 Sector PI',
  };

  const data = useMemo(() => {
    return ckHistory.map((s, i) => {
      const pt: Record<string, number> = { t: +(i * 0.02).toFixed(2) };
      for (const ch of activeChannels) {
        pt[ch.key] = ch.extract(s, pp);
      }
      return pt;
    });
  }, [ckHistory, activeChannels, pp]);

  const activePanels = panels.filter(p =>
    activeChannels.some(ch => ch.panel === p)
  );

  /* Group channels for selector */
  const panelGroups = new Map<string, CkChannel[]>();
  for (const ch of CHANNELS) {
    const list = panelGroups.get(ch.panel) || [];
    list.push(ch);
    panelGroups.set(ch.panel, list);
  }

  return (
    <div style={panelStyle}>
      {/* Header */}
      <div style={{
        display: 'flex', alignItems: 'center', justifyContent: 'space-between',
        marginBottom: expanded ? 8 : 0,
      }}>
        <button onClick={() => setExpanded(!expanded)} style={{
          display: 'flex', alignItems: 'center', gap: 8,
          background: 'none', border: 'none', color: 'var(--text-primary)',
          fontSize: 13, fontWeight: 600, padding: 0, cursor: 'pointer',
        }}>
          <svg width="14" height="14" viewBox="0 0 14 14" fill="none"
            style={{ transform: expanded ? 'rotate(90deg)' : 'rotate(0)', transition: 'transform 0.2s' }}>
            <path d="M5 3L9 7L5 11" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
          </svg>
          <span style={{ textTransform: 'uppercase', letterSpacing: '1px', color: 'var(--text-muted)' }}>
            Diagnostic Scope
          </span>
        </button>
        <div style={{ display: 'flex', alignItems: 'center', gap: 6, flexWrap: 'wrap' }}>
          {Object.entries(PRESETS).map(([key, p]) => (
            <button key={key} onClick={() => applyPreset(p)} style={{
              padding: '2px 8px', borderRadius: 3, fontSize: 9, fontWeight: 600,
              border: '1px solid var(--border)', background: 'transparent',
              color: 'var(--text-muted)', cursor: 'pointer', textTransform: 'uppercase',
              letterSpacing: '0.3px',
            }}>
              {p.label}
            </button>
          ))}
          {telemActive && (
            <span style={{ fontSize: 10, color: 'var(--accent-green)', fontWeight: 600, display: 'flex', alignItems: 'center', gap: 4 }}>
              <span style={{ width: 6, height: 6, borderRadius: '50%', background: 'var(--accent-green)', animation: 'pulse 1.5s infinite' }} />
              LIVE
            </span>
          )}
          <span style={{ fontSize: 11, color: 'var(--text-muted)', fontFamily: 'var(--font-mono)' }}>
            {activeChannels.length} ch / {ckHistory.length} pts
          </span>
        </div>
      </div>

      {expanded && (
        <>
          {/* Preset description */}
          {presetDesc && (
            <div style={{ fontSize: 11, color: 'var(--text-muted)', marginBottom: 8, fontStyle: 'italic' }}>
              {presetDesc}
            </div>
          )}

          {/* Channel selector — grouped by panel */}
          <div style={{
            display: 'flex', flexWrap: 'wrap', gap: 12, marginBottom: 12,
            padding: '8px 12px', background: 'var(--bg-secondary)',
            borderRadius: 'var(--radius-sm)', border: '1px solid var(--border-light)',
          }}>
            {[...panelGroups.entries()].map(([panel, channels]) => (
              <div key={panel} style={{ display: 'flex', flexDirection: 'column', gap: 3 }}>
                <span style={{ fontSize: 9, color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '0.5px' }}>
                  {panelLabels[panel] || panel}
                </span>
                <div style={{ display: 'flex', flexWrap: 'wrap', gap: 3 }}>
                  {channels.map(ch => (
                    <button key={ch.key} onClick={() => toggle(ch.key)}
                      style={chipStyle(active.has(ch.key), ch.color)}>
                      {ch.label}
                    </button>
                  ))}
                </div>
              </div>
            ))}
          </div>

          {/* Stacked charts — one per active panel */}
          {activePanels.length === 0 ? (
            <div style={{ textAlign: 'center', padding: 24, color: 'var(--text-muted)', fontSize: 12 }}>
              Select channels above to start plotting
            </div>
          ) : (
            <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
              {activePanels.map(panel => {
                const panelCh = activeChannels.filter(c => c.panel === panel);
                if (panelCh.length === 0) return null;
                const height = activePanels.length >= 3 ? 160 : activePanels.length === 2 ? 200 : 280;
                return (
                  <div key={panel} style={{
                    background: 'var(--bg-secondary)', borderRadius: 'var(--radius-sm)',
                    border: '1px solid var(--border-light)', padding: '4px 8px',
                  }}>
                    <div style={{ fontSize: 9, color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '0.5px', marginBottom: 2 }}>
                      {panelLabels[panel]}
                    </div>
                    <ResponsiveContainer width="100%" height={height}>
                      <LineChart data={data} margin={{ top: 4, right: 8, bottom: 4, left: 8 }}>
                        <CartesianGrid strokeDasharray="3 3" stroke="var(--border)" opacity={0.3} />
                        <XAxis dataKey="t" tick={{ fontSize: 9, fill: 'var(--text-muted)' }}
                          tickFormatter={(v: number) => `${v.toFixed(1)}s`} />
                        <YAxis tick={{ fontSize: 9, fill: 'var(--text-muted)' }} width={50} />
                        {(panel === 'zc' || panel === 'v4pi') && (
                          <ReferenceLine y={0} stroke="var(--border)" strokeDasharray="3 3" />
                        )}
                        <Tooltip
                          contentStyle={{
                            background: 'var(--bg-card)', border: '1px solid var(--border)',
                            borderRadius: 4, fontSize: 11, padding: '4px 8px',
                          }}
                          labelStyle={{ color: 'var(--text-muted)', fontSize: 10 }}
                          labelFormatter={(v: number) => `t = ${v}s`}
                        />
                        {panelCh.map(ch => (
                          <Line key={ch.key} type="monotone" dataKey={ch.key}
                            name={`${ch.label} (${ch.unit})`}
                            stroke={ch.color} strokeWidth={1.5} dot={false}
                            isAnimationActive={false} />
                        ))}
                      </LineChart>
                    </ResponsiveContainer>
                  </div>
                );
              })}
            </div>
          )}
        </>
      )}
    </div>
  );
}
