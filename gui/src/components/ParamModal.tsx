import { useState, useCallback, useEffect } from 'react';
import { useEscStore } from '../store/useEscStore';
import { serial } from './ConnectionBar';
import { buildPacket, CMD } from '../protocol/gsp';
import { PARAM_NAMES, PARAM_UNITS, PARAM_TOOLTIPS, PARAM_GROUPS, PROFILE_NAMES } from '../protocol/types';

const GROUP_COLORS: Record<number, string> = {
  0: '#f97316', 1: '#3b82f6', 2: '#ef4444', 3: '#a78bfa',
  4: '#22d3ee', 5: '#22c55e', 6: '#eab308', 7: '#94a3b8',
};

export function ParamModal() {
  const { params, activeProfile, snapshot, paramModalOpen, setParamModalOpen } = useEscStore();
  const [editValues, setEditValues] = useState<Record<number, string>>({});
  const [hoveredParam, setHoveredParam] = useState<number | null>(null);
  const [pendingSaves, setPendingSaves] = useState<Set<number>>(new Set());
  const isIdle = snapshot?.state === 0;

  // Reset edits when modal opens
  useEffect(() => {
    if (paramModalOpen) {
      setEditValues({});
      setPendingSaves(new Set());
    }
  }, [paramModalOpen]);

  const setParam = useCallback(async (id: number, value: number) => {
    const buf = new Uint8Array(6);
    const v = new DataView(buf.buffer);
    v.setUint16(0, id, true);
    v.setUint32(2, value, true);
    await serial.write(buildPacket(CMD.SET_PARAM, buf));
    setPendingSaves(prev => { const n = new Set(prev); n.add(id); return n; });
  }, []);

  const saveConfig = useCallback(async () => {
    await serial.write(buildPacket(CMD.SAVE_CONFIG));
    setPendingSaves(new Set());
  }, []);

  const loadDefaults = useCallback(async () => {
    await serial.write(buildPacket(CMD.LOAD_DEFAULTS));
    setEditValues({});
    setPendingSaves(new Set());
  }, []);

  if (!paramModalOpen) return null;

  // Group params
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

  const profileName = PROFILE_NAMES[activeProfile] ?? `Profile ${activeProfile}`;
  const hasUnsaved = pendingSaves.size > 0;

  return (
    <>
      {/* Backdrop */}
      <div
        onClick={() => setParamModalOpen(false)}
        style={{
          position: 'fixed', inset: 0, background: 'rgba(0,0,0,0.6)',
          backdropFilter: 'blur(4px)', zIndex: 200,
        }}
      />

      {/* Modal */}
      <div style={{
        position: 'fixed', top: '3%', left: '5%', right: '5%', bottom: '3%',
        background: 'var(--bg-primary)', borderRadius: 'var(--radius-lg)',
        border: '1px solid var(--border)', boxShadow: '0 25px 60px rgba(0,0,0,0.5)',
        zIndex: 201, display: 'flex', flexDirection: 'column',
        animation: 'fadeInScale 0.2s ease',
        overflow: 'hidden',
      }}>
        {/* Modal header */}
        <div style={{
          padding: '16px 24px', borderBottom: '1px solid var(--border)',
          background: 'var(--bg-secondary)',
          display: 'flex', alignItems: 'center', gap: 12,
          flexShrink: 0,
        }}>
          <div style={{ flex: 1 }}>
            <h2 style={{ fontSize: 18, fontWeight: 700, margin: 0 }}>
              {profileName}
            </h2>
            <div style={{ fontSize: 12, color: 'var(--text-muted)', marginTop: 2 }}>
              {params.size} parameters across {groups.size} groups
              {hasUnsaved && (
                <span style={{ color: 'var(--accent-yellow)', marginLeft: 8 }}>
                  ({pendingSaves.size} unsaved to EEPROM)
                </span>
              )}
            </div>
          </div>

          {!isIdle && (
            <span style={{
              fontSize: 11, padding: '4px 10px', borderRadius: 4,
              background: 'rgba(234,179,8,0.1)', color: 'var(--accent-yellow)',
              border: '1px solid rgba(234,179,8,0.2)',
            }}>
              Read-only while motor runs
            </span>
          )}

          <button onClick={loadDefaults} disabled={!isIdle}
            style={{
              padding: '7px 14px', borderRadius: 'var(--radius-sm)',
              border: '1px solid var(--border)',
              background: 'transparent', color: isIdle ? 'var(--accent-yellow)' : 'var(--text-muted)',
              fontWeight: 600, fontSize: 12,
            }}>
            Defaults
          </button>
          <button onClick={saveConfig} disabled={!isIdle}
            style={{
              padding: '7px 14px', borderRadius: 'var(--radius-sm)', border: 'none',
              background: isIdle ? 'var(--accent-green)' : 'var(--bg-input)',
              color: isIdle ? '#000' : 'var(--text-muted)',
              fontWeight: 600, fontSize: 12,
            }}>
            Save to EEPROM
          </button>
          <button onClick={() => setParamModalOpen(false)}
            style={{
              width: 32, height: 32, borderRadius: 'var(--radius-sm)',
              border: '1px solid var(--border)', background: 'transparent',
              color: 'var(--text-muted)', fontSize: 18, display: 'flex',
              alignItems: 'center', justifyContent: 'center',
            }}>
            \u00d7
          </button>
        </div>

        {/* Modal body — scrollable */}
        <div style={{
          flex: 1, overflow: 'auto', padding: 24,
        }}>
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: 16 }}>
            {sortedGroups.map(([group, items]) => {
              const groupColor = GROUP_COLORS[group] ?? 'var(--accent-blue)';
              return (
                <div key={group} style={{
                  background: 'var(--bg-card)', border: '1px solid var(--border)',
                  borderRadius: 'var(--radius)', overflow: 'hidden',
                }}>
                  <div style={{
                    padding: '10px 14px', borderBottom: '1px solid var(--border)',
                    background: 'var(--bg-secondary)',
                    display: 'flex', alignItems: 'center', gap: 8,
                  }}>
                    <div style={{
                      width: 3, height: 16, borderRadius: 2, background: groupColor,
                    }} />
                    <span style={{ fontSize: 13, fontWeight: 600, color: groupColor }}>
                      {PARAM_GROUPS[group] ?? `Group ${group}`}
                    </span>
                  </div>

                  <div style={{ padding: '6px 14px 14px' }}>
                    {items.map(([id, pv]) => {
                      const editVal = editValues[id];
                      const displayVal = editVal !== undefined ? editVal : String(pv.value);
                      const { display, varName } = parseName(id);
                      const unit = PARAM_UNITS[id] ?? '';
                      const tooltip = PARAM_TOOLTIPS[id];
                      const isHovered = hoveredParam === id;
                      const wasSaved = pendingSaves.has(id);

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
                                display: 'flex', alignItems: 'center', gap: 4,
                              }}>
                                <span style={{ overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
                                  {display}
                                </span>
                                {wasSaved && (
                                  <span style={{
                                    width: 5, height: 5, borderRadius: '50%',
                                    background: 'var(--accent-yellow)', flexShrink: 0,
                                  }} />
                                )}
                              </div>
                              {varName && (
                                <div style={{ fontSize: 10, color: 'var(--text-muted)', fontFamily: 'var(--font-mono)' }}>
                                  {varName}
                                </div>
                              )}
                            </div>
                            <input type="number" value={displayVal}
                              min={pv.descriptor.min} max={pv.descriptor.max}
                              disabled={!isIdle}
                              style={{
                                width: 80, textAlign: 'right', padding: '4px 8px',
                                borderRadius: 'var(--radius-sm)',
                                border: `1px solid ${isHovered ? groupColor : 'var(--border)'}`,
                                background: isIdle ? 'var(--bg-input)' : 'var(--bg-secondary)',
                                color: 'var(--text-primary)', fontSize: 13,
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
                                if (e.key === 'Escape') {
                                  setEditValues(prev => { const n = { ...prev }; delete n[id]; return n; });
                                  (e.target as HTMLInputElement).blur();
                                }
                              }}
                            />
                            <span style={{
                              fontSize: 10, color: 'var(--text-muted)', width: 40,
                              fontFamily: 'var(--font-mono)',
                            }}>
                              {unit}
                            </span>
                          </div>
                          <div style={{
                            fontSize: 10, color: 'var(--text-muted)', marginTop: 2,
                            fontFamily: 'var(--font-mono)',
                          }}>
                            {pv.descriptor.min} \u2013 {pv.descriptor.max}
                          </div>

                          {isHovered && tooltip && (
                            <div style={{
                              position: 'absolute', bottom: '100%', left: 0, right: 0,
                              marginBottom: 4, zIndex: 20,
                              background: 'var(--bg-secondary)',
                              border: `1px solid ${groupColor}44`,
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
      </div>
    </>
  );
}
