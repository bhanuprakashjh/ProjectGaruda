import { useState, useCallback } from 'react';
import { useEscStore } from '../store/useEscStore';
import { serial } from './ConnectionBar';
import { buildPacket, CMD } from '../protocol/gsp';
import {
  CK_PARAM_GROUPS, CK_PARAM_NAMES, CK_PARAM_UNITS, CK_PARAM_TOOLTIPS,
  CK_PROFILE_NAMES, CK_PROFILE_COUNT, CK_VBUS_SCALE,
} from '../protocol/types';

const cardStyle: React.CSSProperties = {
  background: 'var(--bg-card)', borderRadius: 'var(--radius)',
  border: '1px solid var(--border)', padding: 16,
};

const sectionTitle: React.CSSProperties = {
  fontSize: 13, fontWeight: 700, marginBottom: 12, letterSpacing: '-0.2px',
};

const inputStyle: React.CSSProperties = {
  background: 'var(--bg-secondary)', border: '1px solid var(--border)',
  borderRadius: 'var(--radius-sm)', padding: '4px 8px', color: 'var(--text-primary)',
  fontSize: 12, fontFamily: 'var(--font-mono)', width: 80, textAlign: 'right' as const,
};

const btnStyle: React.CSSProperties = {
  padding: '6px 14px', borderRadius: 'var(--radius-sm)', border: 'none',
  fontSize: 11, fontWeight: 600, cursor: 'pointer', letterSpacing: '0.3px',
};

/* Helper: compute ILIM trip current from DAC value */
function ilimTripCurrent(dac: number, gain: number, rshunt: number): number {
  const vilim = 3.3 * dac / 128;
  return (vilim - 1.65) / (gain * rshunt);
}

/* Helper: SC threshold voltage from register value */
const SC_MV = [250, 500, 750, 1000, 1250, 1500, 1750, 2000];

/* Helper: CSA gain from register code */
const CSA_GAINS = [8, 16, 32, 64];

export function CkMotorSetup() {
  const { connected, info, params, addToast } = useEscStore();
  const [autoCalcKv, setAutoCalcKv] = useState(1400);
  const [autoCalcPp, setAutoCalcPp] = useState(7);
  const [autoCalcRs, setAutoCalcRs] = useState(65);
  const [autoCalcVbus, setAutoCalcVbus] = useState(12);

  const getParamValue = useCallback((id: number): number => {
    const p = params.get(id);
    return p ? p.value : 0;
  }, [params]);

  const setParam = useCallback(async (id: number, value: number) => {
    const buf = new Uint8Array(6);
    const dv = new DataView(buf.buffer);
    dv.setUint16(0, id, true);
    dv.setUint32(2, value, true);
    await serial.write(buildPacket(CMD.SET_PARAM, buf));
  }, []);

  const loadProfile = useCallback(async (profileId: number) => {
    await serial.write(buildPacket(CMD.LOAD_PROFILE, new Uint8Array([profileId])));
  }, []);

  const loadDefaults = useCallback(async () => {
    await serial.write(buildPacket(CMD.LOAD_DEFAULTS));
  }, []);

  const saveConfig = useCallback(async () => {
    await serial.write(buildPacket(CMD.SAVE_CONFIG));
  }, []);

  /* Auto-calculate derived params from motor constants */
  const autoCalculate = useCallback(() => {
    const kv = autoCalcKv;
    const pp = autoCalcPp;
    const rs = autoCalcRs; // mOhm
    const vbus = autoCalcVbus;
    const gain = CSA_GAINS[getParamValue(0xDC)] || 16;
    const rshunt = 0.003;

    // ILIM DAC: peak current = Vbus * 0.1 / (Rs/1000) * 2.5 safety factor
    const ipeak = vbus * 0.1 / (rs / 1000) * 2.5;
    const dacVal = Math.min(127, Math.round((1.65 + ipeak * gain * rshunt) * 128 / 3.3));

    // Vbus thresholds
    const vbusOv = Math.round(1.25 * vbus * CK_VBUS_SCALE);
    const vbusUv = Math.round(0.65 * vbus * CK_VBUS_SCALE);

    // Max eRPM: 80% of theoretical max
    const maxErpm = Math.round(kv * vbus * pp * 2 / 60 * 0.8);

    // Timing advance start: 10% of max eRPM
    const timAdvStart = Math.round(maxErpm * 0.1);

    // Align duty: target ~3A align current
    const alignCurrentA = 3.0;
    const alignDutyPctX10 = Math.min(200, Math.round(alignCurrentA * (rs / 1000) / vbus * 1000));

    addToast(`Auto-calc: DAC=${dacVal}, OV=${vbusOv}, UV=${vbusUv}, maxeRPM=${maxErpm}`, 'info');

    // Apply calculated values
    setParam(0xD4, dacVal);           // ILIM DAC
    setParam(0xE2, Math.min(65000, vbusOv)); // Vbus OV
    setParam(0xE3, Math.min(65000, vbusUv)); // Vbus UV
    setParam(0xCF, Math.min(200000, maxErpm)); // Max CL eRPM
    setParam(0xCE, Math.min(50000, timAdvStart)); // Timing advance start
    setParam(0xC5, alignDutyPctX10);  // Align duty
    setParam(0xC0, pp);               // Pole pairs
    setParam(0xC1, kv);               // KV
    setParam(0xC2, rs);               // Rs
  }, [autoCalcKv, autoCalcPp, autoCalcRs, autoCalcVbus, getParamValue, setParam, addToast]);

  if (!connected || !info) {
    return (
      <div style={cardStyle}>
        <p style={{ color: 'var(--text-muted)', fontSize: 13 }}>Connect to CK board to configure motor parameters.</p>
      </div>
    );
  }

  const noParams = params.size === 0;

  // Group params by group ID
  const groupedParams: Record<number, { id: number; value: number }[]> = {};
  for (const [id, pv] of params) {
    const group = pv.descriptor.group;
    if (!groupedParams[group]) groupedParams[group] = [];
    groupedParams[group].push({ id, value: pv.value });
  }

  // Current ILIM values for display
  const ilimDac = getParamValue(0xD4);
  const csaGainCode = getParamValue(0xDC);
  const csaGainVal = CSA_GAINS[csaGainCode] || 16;
  const tripA = ilimTripCurrent(ilimDac, csaGainVal, 0.003);
  const scThresh = getParamValue(0xD7);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 16 }}>
      {/* Section 1: Motor Configuration */}
      <div style={cardStyle}>
        <div style={sectionTitle}>Motor Configuration</div>

        <div style={{ display: 'flex', gap: 12, flexWrap: 'wrap', marginBottom: 12 }}>
          {CK_PROFILE_NAMES.slice(0, CK_PROFILE_COUNT).map((name, i) => (
            <button key={i} onClick={() => loadProfile(i)} style={{
              ...btnStyle,
              background: 'var(--accent-blue-dim)',
              color: 'var(--accent-blue)',
              border: '1px solid rgba(59,130,246,0.3)',
            }}>
              {name}
            </button>
          ))}
        </div>

        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(5, 1fr)', gap: 8, marginBottom: 12 }}>
          <label style={{ fontSize: 11, color: 'var(--text-muted)' }}>
            KV
            <input type="number" value={autoCalcKv} onChange={e => setAutoCalcKv(+e.target.value)}
              style={{ ...inputStyle, display: 'block', width: '100%', marginTop: 2 }} />
          </label>
          <label style={{ fontSize: 11, color: 'var(--text-muted)' }}>
            Pole Pairs
            <input type="number" value={autoCalcPp} onChange={e => setAutoCalcPp(+e.target.value)}
              style={{ ...inputStyle, display: 'block', width: '100%', marginTop: 2 }} />
          </label>
          <label style={{ fontSize: 11, color: 'var(--text-muted)' }}>
            Rs (m&Omega;)
            <input type="number" value={autoCalcRs} onChange={e => setAutoCalcRs(+e.target.value)}
              style={{ ...inputStyle, display: 'block', width: '100%', marginTop: 2 }} />
          </label>
          <label style={{ fontSize: 11, color: 'var(--text-muted)' }}>
            Vbus (V)
            <input type="number" value={autoCalcVbus} onChange={e => setAutoCalcVbus(+e.target.value)}
              style={{ ...inputStyle, display: 'block', width: '100%', marginTop: 2 }} />
          </label>
          <div style={{ display: 'flex', alignItems: 'flex-end' }}>
            <button onClick={autoCalculate} style={{
              ...btnStyle, background: 'var(--accent-green)', color: '#fff', width: '100%',
            }}>
              Auto-Calculate
            </button>
          </div>
        </div>
      </div>

      {/* Section 2: ATA6847 Settings */}
      <div style={cardStyle}>
        <div style={sectionTitle}>ATA6847 Settings</div>

        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 16 }}>
          {/* ILIM */}
          <div>
            <div style={{ fontSize: 11, fontWeight: 600, color: '#ef4444', marginBottom: 8 }}>Current Limit (ILIM)</div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
              <div style={{ display: 'flex', alignItems: 'center', gap: 8, fontSize: 11 }}>
                <span style={{ color: 'var(--text-muted)', width: 60 }}>DAC</span>
                <input type="range" min={0} max={127} value={ilimDac}
                  onChange={e => setParam(0xD4, +e.target.value)}
                  style={{ flex: 1 }} />
                <span style={{ fontFamily: 'var(--font-mono)', width: 30, textAlign: 'right' }}>{ilimDac}</span>
              </div>
              <div style={{ fontSize: 10, color: 'var(--text-muted)', paddingLeft: 68 }}>
                Trip: {tripA > 0 ? `${tripA.toFixed(1)}A` : 'N/A (below offset)'}
              </div>
              <ParamToggle id={0xD2} label="Enable" value={getParamValue(0xD2)} onChange={setParam} />
              <ParamToggle id={0xD3} label="Shutdown Mode" value={getParamValue(0xD3)} onChange={setParam} />
            </div>
          </div>

          {/* Short Circuit */}
          <div>
            <div style={{ fontSize: 11, fontWeight: 600, color: '#ef4444', marginBottom: 8 }}>Short Circuit (SC)</div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
              <div style={{ display: 'flex', alignItems: 'center', gap: 8, fontSize: 11 }}>
                <span style={{ color: 'var(--text-muted)', width: 60 }}>Threshold</span>
                <input type="range" min={0} max={7} value={scThresh}
                  onChange={e => setParam(0xD7, +e.target.value)}
                  style={{ flex: 1 }} />
                <span style={{ fontFamily: 'var(--font-mono)', width: 50, textAlign: 'right' }}>{SC_MV[scThresh]}mV</span>
              </div>
              <ParamToggle id={0xD6} label="Enable" value={getParamValue(0xD6)} onChange={setParam} />
            </div>
          </div>
        </div>
      </div>

      {/* Section 3: All Parameter Groups */}
      <div style={cardStyle}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 12 }}>
          <div style={sectionTitle}>All Parameters</div>
          <div style={{ display: 'flex', gap: 8 }}>
            <button onClick={loadDefaults} style={{
              ...btnStyle, background: 'var(--bg-secondary)', color: 'var(--text-muted)',
              border: '1px solid var(--border)',
            }}>
              Restore Defaults
            </button>
            <button onClick={saveConfig} style={{
              ...btnStyle, background: 'var(--accent-blue)', color: '#fff',
            }}>
              Save to EEPROM
            </button>
          </div>
        </div>

        {noParams ? (
          <p style={{ color: 'var(--text-muted)', fontSize: 12 }}>
            No parameters received. The board may not support runtime params yet.
          </p>
        ) : (
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: 12 }}>
            {Object.entries(CK_PARAM_GROUPS).map(([gid, group]) => {
              const groupId = Number(gid);
              const groupParams = groupedParams[groupId] || [];
              if (groupParams.length === 0) return null;

              return (
                <div key={groupId} style={{
                  background: 'var(--bg-secondary)', borderRadius: 'var(--radius-sm)',
                  padding: 10, borderLeft: `3px solid ${group.color}`,
                }}>
                  <div style={{ fontSize: 11, fontWeight: 600, color: group.color, marginBottom: 8 }}>
                    {group.name}
                  </div>
                  {groupParams.map(({ id, value }) => {
                    const meta = CK_PARAM_NAMES[id];
                    const unit = CK_PARAM_UNITS[id] || '';
                    const tooltip = CK_PARAM_TOOLTIPS[id];
                    const desc = params.get(id)?.descriptor;
                    if (!meta) return null;

                    return (
                      <ParamRow
                        key={id}
                        id={id}
                        name={meta.display}
                        value={value}
                        unit={unit}
                        tooltip={tooltip}
                        min={desc?.min ?? 0}
                        max={desc?.max ?? 65535}
                        onChange={setParam}
                      />
                    );
                  })}
                </div>
              );
            })}
          </div>
        )}
      </div>
    </div>
  );
}

/* Inline param row with number input */
function ParamRow({ id, name, value, unit, tooltip, min, max, onChange }: {
  id: number; name: string; value: number; unit: string; tooltip?: string;
  min: number; max: number; onChange: (id: number, value: number) => void;
}) {
  const [editing, setEditing] = useState(false);
  const [editVal, setEditVal] = useState(String(value));

  const commit = () => {
    const num = parseInt(editVal, 10);
    if (!isNaN(num) && num >= min && num <= max) {
      onChange(id, num);
    }
    setEditing(false);
  };

  return (
    <div style={{
      display: 'flex', alignItems: 'center', justifyContent: 'space-between',
      padding: '3px 0', fontSize: 11,
    }} title={tooltip}>
      <span style={{ color: 'var(--text-secondary)', flex: 1 }}>{name}</span>
      {editing ? (
        <input
          type="number" autoFocus
          value={editVal}
          onChange={e => setEditVal(e.target.value)}
          onBlur={commit}
          onKeyDown={e => { if (e.key === 'Enter') commit(); if (e.key === 'Escape') setEditing(false); }}
          style={{ ...inputStyle, width: 70 }}
          min={min} max={max}
        />
      ) : (
        <span
          onClick={() => { setEditVal(String(value)); setEditing(true); }}
          style={{
            fontFamily: 'var(--font-mono)', color: 'var(--text-primary)',
            cursor: 'pointer', padding: '2px 4px', borderRadius: 2,
            background: 'rgba(255,255,255,0.03)',
          }}
        >
          {value}
        </span>
      )}
      <span style={{ color: 'var(--text-muted)', marginLeft: 4, fontSize: 10, width: 50 }}>{unit}</span>
    </div>
  );
}

/* Toggle row for boolean params */
function ParamToggle({ id, label, value, onChange }: {
  id: number; label: string; value: number;
  onChange: (id: number, value: number) => void;
}) {
  return (
    <div style={{
      display: 'flex', alignItems: 'center', gap: 8, fontSize: 11,
    }}>
      <span style={{ color: 'var(--text-muted)', width: 60 }}>{label}</span>
      <button
        onClick={() => onChange(id, value ? 0 : 1)}
        style={{
          padding: '2px 10px', borderRadius: 'var(--radius-sm)',
          border: '1px solid var(--border)', fontSize: 10, fontWeight: 600,
          background: value ? 'var(--accent-green-dim)' : 'var(--bg-secondary)',
          color: value ? 'var(--accent-green)' : 'var(--text-muted)',
          cursor: 'pointer',
        }}
      >
        {value ? 'ON' : 'OFF'}
      </button>
    </div>
  );
}
