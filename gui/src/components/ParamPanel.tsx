import { useState, useCallback } from 'react';
import { useEscStore } from '../store/useEscStore';
import { serial } from './ConnectionBar';
import { buildPacket, CMD } from '../protocol/gsp';
import { PARAM_NAMES, PARAM_GROUPS } from '../protocol/types';

export function ParamPanel() {
  const { connected, snapshot, params } = useEscStore();
  const [editValues, setEditValues] = useState<Record<number, string>>({});
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
    // Re-fetch params
    const paramIds = [0x15, 0x16, 0x17, 0x20, 0x22, 0x30, 0x42, 0x41];
    for (const id of paramIds) {
      const buf = new Uint8Array(2);
      buf[0] = id & 0xFF;
      buf[1] = (id >> 8) & 0xFF;
      await serial.write(buildPacket(CMD.GET_PARAM, buf));
    }
  }, []);

  // Group params by group
  const groups = new Map<number, Array<[number, { descriptor: { id: number; min: number; max: number; group: number }; value: number }]>>();
  for (const [id, pv] of params) {
    const g = pv.descriptor.group;
    if (!groups.has(g)) groups.set(g, []);
    groups.get(g)!.push([id, pv]);
  }

  return (
    <div style={{ background: '#16213e', borderRadius: 8, padding: 16, marginTop: 16 }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 12 }}>
        <h3>Parameters</h3>
        <div style={{ display: 'flex', gap: 8 }}>
          <button onClick={saveConfig} disabled={!editable}>Save to EEPROM</button>
          <button onClick={loadDefaults} disabled={!editable}>Defaults</button>
        </div>
      </div>
      {!isIdle && connected && <div style={{ color: '#eab308', marginBottom: 8 }}>Parameters locked while motor is running</div>}
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 16 }}>
        {[...groups.entries()].map(([group, items]) => (
          <div key={group} style={{ border: '1px solid #333', borderRadius: 4, padding: 12 }}>
            <h4 style={{ marginBottom: 8 }}>{PARAM_GROUPS[group] ?? `Group ${group}`}</h4>
            {items.map(([id, pv]) => {
              const editVal = editValues[id];
              const displayVal = editVal !== undefined ? editVal : String(pv.value);
              return (
                <div key={id} style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 4 }}>
                  <span style={{ flex: 1, fontSize: 13 }}>{PARAM_NAMES[id] ?? `0x${id.toString(16)}`}</span>
                  <input type="number" value={displayVal}
                    min={pv.descriptor.min} max={pv.descriptor.max}
                    disabled={!editable}
                    style={{ width: 80, textAlign: 'right' }}
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
                </div>
              );
            })}
          </div>
        ))}
      </div>
    </div>
  );
}
