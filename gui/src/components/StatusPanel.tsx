import { useEscStore } from '../store/useEscStore';
import { ESC_STATES, FAULT_CODES, PROFILE_NAMES } from '../protocol/types';

const STATE_COLORS: Record<string, string> = {
  IDLE: 'var(--text-muted)',
  ARMED: 'var(--accent-yellow)',
  ALIGN: 'var(--accent-orange)',
  OL_RAMP: 'var(--accent-orange)',
  MORPH: 'var(--accent-purple)',
  CLOSED_LOOP: 'var(--accent-green)',
  BRAKING: 'var(--accent-yellow)',
  RECOVERY: 'var(--accent-yellow)',
  FAULT: 'var(--accent-red)',
};

const RX_LINK_STATES = ['UNLOCKED', 'DETECTING', 'LOCKING', 'LOCKED', 'LOST'] as const;
const RX_PROTOCOLS = ['NONE', 'PWM', 'DSHOT'] as const;

export function StatusPanel() {
  const { snapshot, info, activeProfile, connected, rxStatus } = useEscStore();
  const flags = info?.featureFlags ?? 0;
  const hasRx = ((flags & (1 << 20)) | (flags & (1 << 21)) | (flags & (1 << 22))) !== 0;
  const state = snapshot ? (ESC_STATES[snapshot.state] ?? 'UNKNOWN') : '\u2014';
  const fault = snapshot ? (FAULT_CODES[snapshot.faultCode] ?? 'UNKNOWN') : '\u2014';
  const uptime = snapshot ? snapshot.uptimeSec : 0;
  const fw = info ? `${info.fwMajor}.${info.fwMinor}.${info.fwPatch}` : '\u2014';
  const stateColor = STATE_COLORS[state] ?? 'var(--text-secondary)';
  const isFault = snapshot?.faultCode !== 0 && snapshot?.faultCode !== undefined;

  const mins = Math.floor(uptime / 60);
  const secs = uptime % 60;
  const uptimeStr = mins > 0 ? `${mins}m ${secs}s` : `${secs}s`;

  const row: React.CSSProperties = {
    display: 'flex', justifyContent: 'space-between', alignItems: 'center',
    padding: '6px 0', borderBottom: '1px solid var(--border-light)',
  };

  return (
    <div style={{
      background: 'var(--bg-card)', borderRadius: 'var(--radius)',
      padding: 16, border: '1px solid var(--border)',
    }}>
      <h3 style={{ fontSize: 13, color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '1px', marginBottom: 12 }}>
        Status
      </h3>

      <div style={row}>
        <span style={{ fontSize: 12, color: 'var(--text-muted)' }}>State</span>
        <span style={{
          fontSize: 12, fontWeight: 600, color: stateColor,
          fontFamily: 'var(--font-mono)',
          ...(state === 'CLOSED_LOOP' || state === 'OL_RAMP' ? { animation: 'pulse 2s infinite' } : {}),
        }}>
          {state}
        </span>
      </div>

      <div style={row}>
        <span style={{ fontSize: 12, color: 'var(--text-muted)' }}>Fault</span>
        <span style={{
          fontSize: 12, fontWeight: isFault ? 700 : 400,
          color: isFault ? 'var(--accent-red)' : 'var(--text-muted)',
          fontFamily: 'var(--font-mono)',
        }}>
          {fault}
        </span>
      </div>

      <div style={row}>
        <span style={{ fontSize: 12, color: 'var(--text-muted)' }}>Uptime</span>
        <span style={{ fontSize: 12, color: 'var(--text-secondary)', fontFamily: 'var(--font-mono)' }}>
          {connected ? uptimeStr : '\u2014'}
        </span>
      </div>

      <div style={row}>
        <span style={{ fontSize: 12, color: 'var(--text-muted)' }}>Firmware</span>
        <span style={{ fontSize: 12, color: 'var(--text-secondary)', fontFamily: 'var(--font-mono)' }}>
          {fw}
        </span>
      </div>

      <div style={{ ...row, borderBottom: 'none' }}>
        <span style={{ fontSize: 12, color: 'var(--text-muted)' }}>Profile</span>
        <span style={{ fontSize: 12, color: 'var(--accent-blue)', fontWeight: 500 }}>
          {connected ? (PROFILE_NAMES[activeProfile] ?? `#${activeProfile}`) : '\u2014'}
        </span>
      </div>

      {/* ZC status indicators when running */}
      {snapshot && snapshot.state >= 3 && (
        <div style={{
          marginTop: 10, paddingTop: 10, borderTop: '1px solid var(--border)',
          display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 6,
        }}>
          <div style={{ fontSize: 10, color: 'var(--text-muted)' }}>
            ZC Sync
            <div style={{
              fontSize: 11, fontWeight: 600, fontFamily: 'var(--font-mono)',
              color: snapshot.zcSynced ? 'var(--accent-green)' : 'var(--accent-yellow)',
            }}>
              {snapshot.zcSynced ? 'LOCKED' : `${snapshot.goodZcCount}/sync`}
            </div>
          </div>
          <div style={{ fontSize: 10, color: 'var(--text-muted)' }}>
            HWZC
            <div style={{
              fontSize: 11, fontWeight: 600, fontFamily: 'var(--font-mono)',
              color: snapshot.hwzcEnabled ? 'var(--accent-cyan)' : 'var(--text-muted)',
            }}>
              {snapshot.hwzcEnabled ? 'ACTIVE' : 'SW'}
            </div>
          </div>
          <div style={{ fontSize: 10, color: 'var(--text-muted)' }}>
            ZC Count
            <div style={{ fontSize: 11, fontFamily: 'var(--font-mono)', color: 'var(--text-secondary)' }}>
              {snapshot.hwzcTotalZcCount}
            </div>
          </div>
          <div style={{ fontSize: 10, color: 'var(--text-muted)' }}>
            ZC Miss
            <div style={{
              fontSize: 11, fontFamily: 'var(--font-mono)',
              color: snapshot.hwzcTotalMissCount > 0 ? 'var(--accent-yellow)' : 'var(--text-secondary)',
            }}>
              {snapshot.hwzcTotalMissCount}
            </div>
          </div>
        </div>
      )}

      {/* RX status when RX features are enabled */}
      {hasRx && connected && rxStatus && (
        <div style={{
          marginTop: 10, paddingTop: 10, borderTop: '1px solid var(--border)',
          display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 6,
        }}>
          <div style={{ fontSize: 10, color: 'var(--text-muted)' }}>
            RX Link
            <div style={{
              fontSize: 11, fontWeight: 600, fontFamily: 'var(--font-mono)',
              color: rxStatus.linkState === 3 ? 'var(--accent-green)'
                : rxStatus.linkState === 4 ? 'var(--accent-red)'
                : 'var(--accent-yellow)',
            }}>
              {RX_LINK_STATES[rxStatus.linkState] ?? 'UNKNOWN'}
            </div>
          </div>
          <div style={{ fontSize: 10, color: 'var(--text-muted)' }}>
            Protocol
            <div style={{ fontSize: 11, fontWeight: 600, fontFamily: 'var(--font-mono)', color: 'var(--text-secondary)' }}>
              {RX_PROTOCOLS[rxStatus.protocol] ?? 'UNKNOWN'}
              {rxStatus.protocol === 2 && rxStatus.dshotRate > 0 ? ` ${rxStatus.dshotRate * 150}` : ''}
            </div>
          </div>
          <div style={{ fontSize: 10, color: 'var(--text-muted)' }}>
            CRC Errors
            <div style={{
              fontSize: 11, fontFamily: 'var(--font-mono)',
              color: rxStatus.crcErrors > 0 ? 'var(--accent-yellow)' : 'var(--text-secondary)',
            }}>
              {rxStatus.crcErrors}
            </div>
          </div>
          <div style={{ fontSize: 10, color: 'var(--text-muted)' }}>
            Dropped
            <div style={{
              fontSize: 11, fontFamily: 'var(--font-mono)',
              color: rxStatus.droppedFrames > 0 ? 'var(--accent-yellow)' : 'var(--text-secondary)',
            }}>
              {rxStatus.droppedFrames}
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
