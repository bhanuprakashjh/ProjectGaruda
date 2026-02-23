import { useState, useMemo } from 'react';
import { useEscStore } from '../store/useEscStore';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Legend } from 'recharts';
import type { GspSnapshot } from '../protocol/types';

interface ScopeChannel {
  key: string;
  label: string;
  unit: string;
  color: string;
  extract: (s: GspSnapshot, info: { polePairs: number }) => number;
  format?: (v: number) => string;
  group: string;
}

const CHANNELS: ScopeChannel[] = [
  // Motor
  { key: 'eRPM', label: 'eRPM', unit: 'eRPM', color: '#3b82f6', group: 'Motor',
    extract: (s) => s.stepPeriod > 0 ? Math.round(240000 / s.stepPeriod) : 0 },
  { key: 'mechRPM', label: 'Mech RPM', unit: 'RPM', color: '#60a5fa', group: 'Motor',
    extract: (s, i) => s.stepPeriod > 0 ? Math.round(240000 / s.stepPeriod / i.polePairs) : 0 },
  { key: 'duty', label: 'Duty', unit: '%', color: '#f472b6', group: 'Motor',
    extract: (s) => s.dutyPct },
  { key: 'throttle', label: 'Throttle', unit: '', color: '#c084fc', group: 'Motor',
    extract: (s) => s.throttle },
  { key: 'step', label: 'Comm Step', unit: '', color: '#94a3b8', group: 'Motor',
    extract: (s) => s.currentStep },

  // Power
  { key: 'vbus', label: 'Vbus', unit: 'V', color: '#22c55e', group: 'Power',
    extract: (s) => +(s.vbusRaw * 3.3 / 4096 * 19.8).toFixed(2) },
  { key: 'ibus', label: 'Ibus', unit: 'A', color: '#eab308', group: 'Power',
    extract: (s) => +((s.ibusRaw - 2048) / 93.0).toFixed(3) },
  { key: 'ibusMax', label: 'Ibus Peak', unit: 'A', color: '#f97316', group: 'Power',
    extract: (s) => +((s.ibusMax - 2048) / 93.0).toFixed(3) },

  // BEMF & ZC
  { key: 'bemf', label: 'BEMF Raw', unit: 'ADC', color: '#a78bfa', group: 'BEMF & ZC',
    extract: (s) => s.bemfRaw },
  { key: 'zcThreshold', label: 'ZC Threshold', unit: 'ADC', color: '#fb923c', group: 'BEMF & ZC',
    extract: (s) => s.zcThreshold },
  { key: 'goodZc', label: 'Good ZC Count', unit: '', color: '#22d3ee', group: 'BEMF & ZC',
    extract: (s) => s.goodZcCount },
  { key: 'zcConfirmed', label: 'ZC Confirmed', unit: '', color: '#2dd4bf', group: 'BEMF & ZC',
    extract: (s) => s.zcConfirmedCount },
  { key: 'zcForced', label: 'ZC Forced Steps', unit: '', color: '#f43f5e', group: 'BEMF & ZC',
    extract: (s) => s.zcTimeoutForceCount },
  { key: 'stepPeriod', label: 'Step Period', unit: 'ticks', color: '#64748b', group: 'BEMF & ZC',
    extract: (s) => s.stepPeriod },

  // HWZC
  { key: 'hwzcZcCount', label: 'HWZC Total ZC', unit: '', color: '#06b6d4', group: 'HWZC',
    extract: (s) => s.hwzcTotalZcCount },
  { key: 'hwzcMissCount', label: 'HWZC Misses', unit: '', color: '#ef4444', group: 'HWZC',
    extract: (s) => s.hwzcTotalMissCount },
  { key: 'hwzcStepPeriod', label: 'HWZC Step Period', unit: 'ticks', color: '#8b5cf6', group: 'HWZC',
    extract: (s) => s.hwzcStepPeriodHR },

  // Morph
  { key: 'morphAlpha', label: 'Morph Alpha', unit: '', color: '#d946ef', group: 'Morph',
    extract: (s) => s.morphAlpha },
  { key: 'morphZcCount', label: 'Morph ZC Count', unit: '', color: '#e879f9', group: 'Morph',
    extract: (s) => s.morphZcCount },

  // Protection
  { key: 'clpciTrips', label: 'CLPCI Trips', unit: '', color: '#dc2626', group: 'Protection',
    extract: (s) => s.clpciTripCount },
  { key: 'fpciTrips', label: 'FPCI Trips', unit: '', color: '#b91c1c', group: 'Protection',
    extract: (s) => s.fpciTripCount },
];

const DEFAULT_ACTIVE = new Set(['eRPM', 'duty', 'bemf', 'zcThreshold']);

export function ScopePanel() {
  const history = useEscStore(s => s.history);
  const info = useEscStore(s => s.info);
  const telemActive = useEscStore(s => s.telemActive);
  const [activeChannels, setActiveChannels] = useState<Set<string>>(DEFAULT_ACTIVE);
  const [expanded, setExpanded] = useState(true);

  const polePairs = info?.motorPolePairs ?? 1;

  const toggle = (key: string) => {
    setActiveChannels(prev => {
      const next = new Set(prev);
      if (next.has(key)) next.delete(key);
      else next.add(key);
      return next;
    });
  };

  // Build chart data from history
  const data = useMemo(() => {
    return history.map((s, i) => {
      const point: Record<string, number> = { t: +(i * 0.02).toFixed(2) };
      for (const ch of CHANNELS) {
        if (activeChannels.has(ch.key)) {
          point[ch.key] = ch.extract(s, { polePairs });
        }
      }
      return point;
    });
  }, [history, activeChannels, polePairs]);

  // Determine Y-axis ranges for active channels
  const activeList = CHANNELS.filter(c => activeChannels.has(c.key));

  // Group channels by similar unit for shared axes
  const leftChannels = activeList.filter(c =>
    ['eRPM', 'mechRPM', 'stepPeriod', 'hwzcStepPeriod', 'bemf', 'zcThreshold'].includes(c.key)
  );
  const rightChannels = activeList.filter(c =>
    ['duty', 'throttle', 'morphAlpha'].includes(c.key)
  );
  const extraChannels = activeList.filter(c =>
    !leftChannels.includes(c) && !rightChannels.includes(c)
  );

  // Group the channel selector items
  const groups = new Map<string, ScopeChannel[]>();
  for (const ch of CHANNELS) {
    const list = groups.get(ch.group) || [];
    list.push(ch);
    groups.set(ch.group, list);
  }

  return (
    <div style={{
      background: 'var(--bg-card)', borderRadius: 'var(--radius)',
      padding: 16, marginTop: 16, border: '1px solid var(--border)',
    }}>
      {/* Header */}
      <div style={{
        display: 'flex', alignItems: 'center', justifyContent: 'space-between',
        marginBottom: expanded ? 12 : 0,
      }}>
        <button
          onClick={() => setExpanded(!expanded)}
          style={{
            display: 'flex', alignItems: 'center', gap: 8,
            background: 'none', border: 'none', color: 'var(--text-primary)',
            fontSize: 13, fontWeight: 600, padding: 0,
          }}
        >
          <svg width="14" height="14" viewBox="0 0 14 14" fill="none"
            style={{ transform: expanded ? 'rotate(90deg)' : 'rotate(0deg)', transition: 'transform 0.2s' }}>
            <path d="M5 3L9 7L5 11" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
          </svg>
          <span style={{ textTransform: 'uppercase', letterSpacing: '1px', color: 'var(--text-muted)' }}>
            Live Scope
          </span>
        </button>
        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          {telemActive && (
            <span style={{
              fontSize: 10, color: 'var(--accent-green)', fontWeight: 600,
              display: 'flex', alignItems: 'center', gap: 4,
            }}>
              <span style={{
                width: 6, height: 6, borderRadius: '50%',
                background: 'var(--accent-green)', animation: 'pulse 1.5s infinite',
              }} />
              LIVE
            </span>
          )}
          <span style={{ fontSize: 11, color: 'var(--text-muted)', fontFamily: 'var(--font-mono)' }}>
            {activeList.length} ch / {history.length} pts
          </span>
        </div>
      </div>

      {expanded && (
        <>
          {/* Channel selector */}
          <div style={{
            display: 'flex', flexWrap: 'wrap', gap: 12, marginBottom: 12,
            padding: '10px 12px', background: 'var(--bg-secondary)',
            borderRadius: 'var(--radius-sm)', border: '1px solid var(--border-light)',
          }}>
            {[...groups.entries()].map(([groupName, channels]) => (
              <div key={groupName} style={{ display: 'flex', flexDirection: 'column', gap: 3 }}>
                <span style={{ fontSize: 9, color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '0.5px' }}>
                  {groupName}
                </span>
                <div style={{ display: 'flex', flexWrap: 'wrap', gap: 3 }}>
                  {channels.map(ch => {
                    const active = activeChannels.has(ch.key);
                    return (
                      <button key={ch.key} onClick={() => toggle(ch.key)} style={{
                        padding: '2px 8px', borderRadius: 3, fontSize: 10, fontWeight: 500,
                        border: `1px solid ${active ? ch.color : 'var(--border)'}`,
                        background: active ? `${ch.color}22` : 'transparent',
                        color: active ? ch.color : 'var(--text-muted)',
                        cursor: 'pointer', whiteSpace: 'nowrap',
                        transition: 'all 0.15s',
                      }}>
                        {ch.label}
                      </button>
                    );
                  })}
                </div>
              </div>
            ))}
          </div>

          {/* Chart */}
          {activeList.length === 0 ? (
            <div style={{
              height: 200, display: 'flex', alignItems: 'center', justifyContent: 'center',
              color: 'var(--text-muted)', fontSize: 13,
            }}>
              Select channels above to display
            </div>
          ) : (
            <ResponsiveContainer width="100%" height={280}>
              <LineChart data={data} margin={{ top: 4, right: 8, left: 0, bottom: 0 }}>
                <CartesianGrid strokeDasharray="3 3" stroke="var(--border-light)" />
                <XAxis
                  dataKey="t"
                  tick={{ fill: 'var(--text-muted)', fontSize: 9 }}
                  label={{ value: 'Time (s)', position: 'insideBottomRight', offset: -4, fill: 'var(--text-muted)', fontSize: 9 }}
                />

                {/* Left axis for large-value channels */}
                {leftChannels.length > 0 && (
                  <YAxis
                    yAxisId="left"
                    tick={{ fill: leftChannels[0].color, fontSize: 9 }}
                    width={50}
                  />
                )}

                {/* Right axis for percentage channels */}
                {rightChannels.length > 0 && (
                  <YAxis
                    yAxisId="right" orientation="right"
                    domain={[0, rightChannels.some(c => c.key === 'throttle') ? 2000 : 100]}
                    tick={{ fill: rightChannels[0].color, fontSize: 9 }}
                    width={40}
                  />
                )}

                {/* Hidden axis for extra channels (auto-scale, no visible axis) */}
                {extraChannels.length > 0 && (
                  <YAxis yAxisId="extra" hide />
                )}

                <Tooltip
                  contentStyle={{
                    background: 'var(--bg-secondary)', border: '1px solid var(--border)',
                    borderRadius: 'var(--radius-sm)', fontSize: 10,
                    fontFamily: 'var(--font-mono)',
                  }}
                  formatter={(value: number, name: string) => {
                    const ch = CHANNELS.find(c => c.key === name);
                    return [`${typeof value === 'number' ? value.toFixed(1) : value} ${ch?.unit ?? ''}`, ch?.label ?? name];
                  }}
                />
                <Legend wrapperStyle={{ fontSize: 10, paddingTop: 4 }} />

                {activeList.map(ch => {
                  let yAxisId = 'extra';
                  if (leftChannels.includes(ch)) yAxisId = 'left';
                  else if (rightChannels.includes(ch)) yAxisId = 'right';

                  return (
                    <Line
                      key={ch.key}
                      yAxisId={yAxisId}
                      type="monotone"
                      dataKey={ch.key}
                      name={ch.key}
                      stroke={ch.color}
                      strokeWidth={1.5}
                      dot={false}
                      isAnimationActive={false}
                    />
                  );
                })}
              </LineChart>
            </ResponsiveContainer>
          )}

          {/* Current values readout */}
          {activeList.length > 0 && history.length > 0 && (
            <div style={{
              display: 'flex', flexWrap: 'wrap', gap: 12, marginTop: 8,
              padding: '8px 12px', background: 'var(--bg-secondary)',
              borderRadius: 'var(--radius-sm)',
            }}>
              {activeList.map(ch => {
                const lastSnap = history[history.length - 1];
                const val = ch.extract(lastSnap, { polePairs });
                return (
                  <div key={ch.key} style={{ display: 'flex', alignItems: 'baseline', gap: 4 }}>
                    <span style={{
                      width: 8, height: 8, borderRadius: 2,
                      background: ch.color, display: 'inline-block', flexShrink: 0,
                    }} />
                    <span style={{ fontSize: 10, color: 'var(--text-muted)' }}>{ch.label}:</span>
                    <span style={{ fontSize: 11, fontWeight: 600, fontFamily: 'var(--font-mono)', color: ch.color }}>
                      {typeof val === 'number' ? (Number.isInteger(val) ? val : val.toFixed(2)) : val}
                    </span>
                    <span style={{ fontSize: 9, color: 'var(--text-muted)' }}>{ch.unit}</span>
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
