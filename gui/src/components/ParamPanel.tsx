import { useState, useCallback } from 'react';
import { useEscStore } from '../store/useEscStore';
import { serial } from './ConnectionBar';
import { buildPacket, CMD } from '../protocol/gsp';
import { PARAM_NAMES, PARAM_UNITS, PARAM_TOOLTIPS, PARAM_GROUPS } from '../protocol/types';

const GROUP_ICONS: Record<number, string> = {
  0: '\uD83D\uDE80', // Startup & Ramp
  1: '\uD83D\uDD04', // Closed-Loop
  2: '\u26A1',       // Current Protection
  3: '\uD83C\uDFAF', // ZC Detection
  4: '\u2197',       // Duty Slew
  5: '\uD83D\uDD0B', // Voltage Protection
  6: '\u21BB',       // Recovery
  7: '\u2699',       // Motor Hardware
};

const GROUP_COLORS: Record<number, string> = {
  0: '#f97316',
  1: '#3b82f6',
  2: '#ef4444',
  3: '#a78bfa',
  4: '#22d3ee',
  5: '#22c55e',
  6: '#eab308',
  7: '#94a3b8',
};

export function ParamPanel() {
  const { connected, snapshot, params } = useEscStore();
  const [editValues, setEditValues] = useState<Record<number, string>>({});
  const [hoveredParam, setHoveredParam] = useState<number | null>(null);
  const isIdle = snapshot?.state === 0;
  const editable = connected && isIdle;

  const setParam = useCallback(async (id: number, value: number) => {
    const buf = new Uint8Array(6);
    const v = new DataView(buf.buffer);
    v.setUint16(0, id, true);
    v.setUint32(2, value, true);
    await serial.write(buildPacket(CMD.SET_PARAM, buf));
  }, []);

  const saveConfig = useCallback(async () => {
    await serial.write(buildPacket(CMD.SAVE_CONFIG));
  }, []);

  const loadDefaults = useCallback(async () => {
    await serial.write(buildPacket(CMD.LOAD_DEFAULTS));
  }, []);

  const groups = new Map<number, Array<[number, { descriptor: { id: number; min: number; max: number; group: number }; value: number }]>>();
  for (const [id, pv] of params) {
    const g = pv.descriptor.group;
    if (!groups.has(g)) groups.set(g, []);
    groups.get(g)!.push([id, pv]);
  }
  const sortedGroups = [...groups.entries()].sort((a, b) => a[0] - b[0]);

  const parseName = (id: number) => {
    const full = PARAM_NAMES[id] ?? `0x${id.toString(16)}`;
    const match = full.match(/^(.+?)\s*\[(.+)\]$/);
    if (match) return { display: match[1], varName: match[2] };
    return { display: full, varName: '' };
  };

  if (params.size === 0) {
    return (
      <div style={{
        background: 'var(--bg-card)', borderRadius: 'var(--radius)',
        padding: 24, marginTop: 16, border: '1px solid var(--border)',
        textAlign: 'center', color: 'var(--text-muted)', fontSize: 13,
      }}>
        Connect to ESC to view and edit parameters
      </div>
    );
  }

  return (
    <div style={{ marginTop: 16 }}>
      {/* Header bar */}
      <div style={{
        display: 'flex', justifyContent: 'space-between', alignItems: 'center',
        marginBottom: 12,
      }}>
        <div>
          <span style={{
            fontSize: 13, color: 'var(--text-muted)', textTransform: 'uppercase',
            letterSpacing: '1px',
          }}>
            Parameters
          </span>
          <span style={{
            fontSize: 11, color: 'var(--text-muted)', marginLeft: 8,
            fontFamily: 'var(--font-mono)',
          }}>
            ({params.size} total)
          </span>
        </div>
        <div style={{ display: 'flex', gap: 8 }}>
          <button onClick={loadDefaults} disabled={!editable}
            style={{
              padding: '6px 14px', borderRadius: 'var(--radius-sm)',
              border: '1px solid var(--border)',
              background: editable ? 'transparent' : 'var(--bg-input)',
              color: editable ? 'var(--accent-yellow)' : 'var(--text-muted)',
              fontWeight: 600, fontSize: 12,
            }}>
            Restore Defaults
          </button>
          <button onClick={saveConfig} disabled={!editable}
            style={{
              padding: '6px 14px', borderRadius: 'var(--radius-sm)', border: 'none',
              background: editable ? 'var(--accent-green)' : 'var(--bg-input)',
              color: editable ? '#000' : 'var(--text-muted)',
              fontWeight: 600, fontSize: 12,
            }}>
            Save to EEPROM
          </button>
        </div>
      </div>

      {!isIdle && connected && (
        <div style={{
          color: 'var(--accent-yellow)', marginBottom: 10, fontSize: 12,
          padding: '6px 12px', background: 'rgba(234,179,8,0.08)',
          border: '1px solid rgba(234,179,8,0.2)',
          borderRadius: 'var(--radius-sm)',
        }}>
          Parameters are read-only while motor is running
        </div>
      )}

      {/* Parameter grid */}
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: 12 }}>
        {sortedGroups.map(([group, items]) => {
          const groupColor = GROUP_COLORS[group] ?? 'var(--accent-blue)';
          return (
            <div key={group} style={{
              background: 'var(--bg-card)', border: '1px solid var(--border)',
              borderRadius: 'var(--radius)', overflow: 'hidden',
            }}>
              {/* Group header */}
              <div style={{
                padding: '10px 14px',
                borderBottom: '1px solid var(--border)',
                background: 'var(--bg-secondary)',
                display: 'flex', alignItems: 'center', gap: 8,
              }}>
                <span style={{ fontSize: 14 }}>{GROUP_ICONS[group] ?? '\u2022'}</span>
                <span style={{ fontSize: 13, fontWeight: 600, color: groupColor }}>
                  {PARAM_GROUPS[group] ?? `Group ${group}`}
                </span>
                <span style={{
                  fontSize: 10, color: 'var(--text-muted)', marginLeft: 'auto',
                  fontFamily: 'var(--font-mono)',
                }}>
                  {items.length}
                </span>
              </div>

              {/* Params */}
              <div style={{ padding: '8px 14px 14px' }}>
                {items.map(([id, pv]) => {
                  const editVal = editValues[id];
                  const displayVal = editVal !== undefined ? editVal : String(pv.value);
                  const { display, varName } = parseName(id);
                  const unit = PARAM_UNITS[id] ?? '';
                  const tooltip = PARAM_TOOLTIPS[id];
                  const isHovered = hoveredParam === id;

                  return (
                    <div key={id} style={{
                      padding: '8px 0', position: 'relative',
                      borderBottom: '1px solid var(--border-light)',
                    }}
                      onMouseEnter={() => setHoveredParam(id)}
                      onMouseLeave={() => setHoveredParam(null)}
                    >
                      <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
                        <div style={{ flex: 1, minWidth: 0 }}>
                          <div style={{
                            fontSize: 12, color: 'var(--text-primary)',
                            whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis',
                          }}>
                            {display}
                          </div>
                          {varName && (
                            <div style={{
                              fontSize: 10, color: 'var(--text-muted)',
                              fontFamily: 'var(--font-mono)',
                            }}>
                              {varName}
                            </div>
                          )}
                        </div>
                        <input type="number" value={displayVal}
                          min={pv.descriptor.min} max={pv.descriptor.max}
                          disabled={!editable}
                          style={{
                            width: 72, textAlign: 'right', padding: '3px 6px',
                            borderRadius: 'var(--radius-sm)',
                            border: `1px solid ${isHovered ? 'var(--accent-blue)' : 'var(--border)'}`,
                            background: editable ? 'var(--bg-input)' : 'var(--bg-secondary)',
                            color: 'var(--text-primary)', fontSize: 12,
                            fontFamily: 'var(--font-mono)',
                            transition: 'border-color 0.15s',
                          }}
                          onChange={(e) => setEditValues(prev => ({ ...prev, [id]: e.target.value }))}
                          onBlur={async () => {
                            const v = Number(displayVal);
                            if (!isNaN(v) && v >= pv.descriptor.min && v <= pv.descriptor.max) {
                              await setParam(id, v);
                            }
                            setEditValues(prev => { const n = { ...prev }; delete n[id]; return n; });
                          }}
                          onKeyDown={async (e) => {
                            if (e.key === 'Enter') (e.target as HTMLInputElement).blur();
                          }}
                        />
                        <span style={{
                          fontSize: 10, color: 'var(--text-muted)',
                          width: 36, textAlign: 'left',
                          fontFamily: 'var(--font-mono)',
                        }}>
                          {unit}
                        </span>
                      </div>
                      <div style={{
                        fontSize: 10, color: 'var(--text-muted)',
                        marginTop: 2, fontFamily: 'var(--font-mono)',
                      }}>
                        {pv.descriptor.min} \u2013 {pv.descriptor.max}
                      </div>

                      {/* Tooltip */}
                      {isHovered && tooltip && (
                        <div style={{
                          position: 'absolute', bottom: '100%', left: 0, right: 0,
                          marginBottom: 4, zIndex: 20,
                          background: 'var(--bg-secondary)',
                          border: '1px solid var(--border)',
                          borderRadius: 'var(--radius-sm)',
                          padding: '8px 10px', fontSize: 11,
                          color: 'var(--text-secondary)', lineHeight: 1.5,
                          boxShadow: 'var(--shadow-lg)',
                          animation: 'fadeIn 0.15s ease',
                        }}>
                          {tooltip}
                        </div>
                      )}
                    </div>
                  );
                })}
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
}
