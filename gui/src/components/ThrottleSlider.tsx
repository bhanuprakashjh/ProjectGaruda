import { useState, useCallback } from 'react';
import { useEscStore } from '../store/useEscStore';
import { serial } from './ConnectionBar';
import { buildPacket, CMD } from '../protocol/gsp';

export function ThrottleSlider() {
  const { connected, throttleSource } = useEscStore();
  const [value, setValue] = useState(0);
  const enabled = connected && throttleSource === 'GSP';

  const onChange = useCallback(async (v: number) => {
    setValue(v);
    if (!enabled) return;
    const buf = new Uint8Array(2);
    buf[0] = v & 0xFF;
    buf[1] = (v >> 8) & 0xFF;
    try { await serial.write(buildPacket(CMD.SET_THROTTLE, buf)); } catch {}
  }, [enabled]);

  return (
    <div style={{ background: '#16213e', borderRadius: 8, padding: 16, marginTop: 16, opacity: enabled ? 1 : 0.4 }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
        <span>Throttle: {value}</span>
        <input type="range" min={0} max={2000} value={value}
          disabled={!enabled}
          onChange={(e) => onChange(Number(e.target.value))}
          style={{ flex: 1 }} />
      </div>
    </div>
  );
}
