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

  const pct = (value / 2000 * 100).toFixed(0);

  return (
    <div style={{
      background: 'var(--bg-card)', borderRadius: 'var(--radius)',
      padding: 16, border: '1px solid var(--border)',
      opacity: enabled ? 1 : 0.5,
      transition: 'opacity 0.2s',
    }}>
      <div style={{
        fontSize: 13, color: 'var(--text-muted)', textTransform: 'uppercase',
        letterSpacing: '1px', marginBottom: 10,
      }}>
        GSP Throttle
      </div>
      <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
        <input type="range" min={0} max={2000} value={value}
          disabled={!enabled}
          onChange={(e) => onChange(Number(e.target.value))}
          style={{ flex: 1 }} />
        <div style={{ textAlign: 'right', minWidth: 60 }}>
          <div style={{ fontSize: 18, fontWeight: 700, fontFamily: 'var(--font-mono)', color: 'var(--text-primary)' }}>{value}</div>
          <div style={{ fontSize: 10, color: 'var(--text-muted)' }}>{pct}%</div>
        </div>
      </div>
      {!enabled && (
        <div style={{ fontSize: 11, color: 'var(--text-muted)', marginTop: 6 }}>
          Switch throttle source to GSP to use this slider
        </div>
      )}
    </div>
  );
}
