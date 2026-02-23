import { useCallback } from 'react';
import { useEscStore } from '../store/useEscStore';
import { serial } from './ConnectionBar';
import { buildPacket, CMD } from '../protocol/gsp';
import { PROFILE_NAMES } from '../protocol/types';

const PROFILE_DESCRIPTIONS: Record<number, string> = {
  0: '10-pole, 24V, low-KV industrial motor',
  1: '14-pole, 12V, 1400KV drone motor',
  2: '28-pole, large multirotor motor',
  3: 'User-defined motor configuration',
};

const PROFILE_ICONS: Record<number, string> = {
  0: '\uD83C\uDFED', 1: '\uD83D\uDEE9', 2: '\uD83D\uDE81', 3: '\u270F',
};

export function ProfileSelector() {
  const { connected, snapshot, info, activeProfile, setParamModalOpen } = useEscStore();
  const isIdle = snapshot?.state === 0;
  const canLoad = connected && isIdle;

  const loadProfile = useCallback(async (profileId: number) => {
    if (!canLoad) return;
    await serial.write(buildPacket(CMD.LOAD_PROFILE, new Uint8Array([profileId])));
    // Open param modal after a short delay to let params re-fetch
    setTimeout(() => setParamModalOpen(true), 400);
  }, [canLoad, setParamModalOpen]);

  const clpciEnabled = info ? (info.featureFlags & (1 << 17)) !== 0 : false;
  const presyncEnabled = info ? (info.featureFlags & (1 << 18)) !== 0 : false;

  return (
    <div style={{
      background: 'var(--bg-card)', borderRadius: 'var(--radius)',
      padding: 16, marginTop: 16, border: '1px solid var(--border)',
    }}>
      <div style={{
        display: 'flex', alignItems: 'center', justifyContent: 'space-between',
        marginBottom: 12,
      }}>
        <div style={{
          fontSize: 13, color: 'var(--text-muted)', textTransform: 'uppercase',
          letterSpacing: '1px',
        }}>
          Motor Profiles
        </div>
        <div style={{ display: 'flex', gap: 12, fontSize: 11, color: 'var(--text-muted)' }}>
          <span>
            CLPCI <span style={{
              color: clpciEnabled ? 'var(--accent-green)' : 'var(--text-muted)',
              fontWeight: 600,
            }}>{clpciEnabled ? 'ON' : 'OFF'}</span>
          </span>
          <span>
            Presync <span style={{
              color: presyncEnabled ? 'var(--accent-green)' : 'var(--text-muted)',
              fontWeight: 600,
            }}>{presyncEnabled ? 'ON' : 'OFF'}</span>
          </span>
          <span style={{ fontStyle: 'italic' }}>compile-time</span>
        </div>
      </div>

      {/* Profile cards */}
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: 10 }}>
        {PROFILE_NAMES.map((name, i) => {
          const isActive = activeProfile === i;
          return (
            <button
              key={i}
              onClick={() => {
                if (isActive) {
                  setParamModalOpen(true);
                } else {
                  loadProfile(i);
                }
              }}
              disabled={!canLoad && !isActive}
              style={{
                padding: '12px 14px', borderRadius: 'var(--radius)',
                border: `1px solid ${isActive ? 'var(--accent-blue)' : 'var(--border)'}`,
                background: isActive ? 'var(--accent-blue-dim)' : 'var(--bg-secondary)',
                color: 'var(--text-primary)',
                textAlign: 'left', cursor: canLoad || isActive ? 'pointer' : 'default',
                transition: 'all 0.15s',
                position: 'relative',
                overflow: 'hidden',
              }}
            >
              {isActive && (
                <div style={{
                  position: 'absolute', top: 6, right: 8,
                  fontSize: 9, fontWeight: 700, textTransform: 'uppercase',
                  color: 'var(--accent-blue)', letterSpacing: '0.5px',
                }}>
                  ACTIVE
                </div>
              )}
              <div style={{ fontSize: 16, marginBottom: 4 }}>
                {PROFILE_ICONS[i] ?? '\u2699'}
              </div>
              <div style={{ fontSize: 13, fontWeight: 600, lineHeight: 1.2 }}>
                {name}
              </div>
              <div style={{ fontSize: 10, color: 'var(--text-muted)', marginTop: 4, lineHeight: 1.3 }}>
                {PROFILE_DESCRIPTIONS[i] ?? ''}
              </div>
              <div style={{
                fontSize: 10, marginTop: 8,
                color: isActive ? 'var(--accent-blue)' : 'var(--text-muted)',
                fontWeight: 500,
              }}>
                {isActive ? 'Click to edit params \u2192' : canLoad ? 'Click to load \u2192' : ''}
              </div>
            </button>
          );
        })}
      </div>
    </div>
  );
}
