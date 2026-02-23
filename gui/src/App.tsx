import { ConnectionBar } from './components/ConnectionBar';
import { StatusPanel } from './components/StatusPanel';
import { GaugePanel } from './components/GaugePanel';
import { ScopePanel } from './components/ScopePanel';
import { ControlPanel } from './components/ControlPanel';
import { ThrottleSlider } from './components/ThrottleSlider';
import { ProfileSelector } from './components/ProfileSelector';
import { ParamPanel } from './components/ParamPanel';
import { ParamModal } from './components/ParamModal';
import { HelpPanel, MicrochipIcon } from './components/HelpPanel';
import { useEscStore } from './store/useEscStore';

const toastColors = {
  success: { bg: 'var(--accent-green-dim)', border: 'var(--accent-green)' },
  error: { bg: 'var(--accent-red-dim)', border: 'var(--accent-red)' },
  info: { bg: 'var(--accent-blue-dim)', border: 'var(--accent-blue)' },
};

function GarudaLogo() {
  return (
    <svg width="32" height="32" viewBox="0 0 32 32" fill="none">
      <path d="M16 2L8 10L4 18L8 16L12 20L16 14L20 20L24 16L28 18L24 10L16 2Z"
        fill="var(--accent-blue)" opacity="0.9" />
      <path d="M16 14L12 20L10 28L16 22L22 28L20 20L16 14Z"
        fill="var(--accent-cyan)" opacity="0.7" />
      <circle cx="16" cy="9" r="2" fill="white" opacity="0.9" />
    </svg>
  );
}

export default function App() {
  const toasts = useEscStore(s => s.toasts);
  const removeToast = useEscStore(s => s.removeToast);
  const info = useEscStore(s => s.info);
  const connected = useEscStore(s => s.connected);

  return (
    <div style={{ minHeight: '100vh', display: 'flex', flexDirection: 'column' }}>
      {/* Header */}
      <header style={{
        background: 'var(--bg-secondary)',
        borderBottom: '1px solid var(--border)',
        padding: '0 24px',
        position: 'sticky', top: 0, zIndex: 100,
      }}>
        <div style={{
          maxWidth: 1280, margin: '0 auto',
          display: 'flex', alignItems: 'center', height: 56, gap: 12,
        }}>
          <GarudaLogo />
          <div>
            <div style={{ fontSize: 16, fontWeight: 700, letterSpacing: '-0.3px', lineHeight: 1.2 }}>
              Garuda ESC
            </div>
            <div style={{ fontSize: 10, color: 'var(--text-muted)', letterSpacing: '0.5px' }}>
              CONFIGURATOR
            </div>
          </div>

          {connected && info && (
            <div style={{
              marginLeft: 20, display: 'flex', alignItems: 'center', gap: 16,
              fontSize: 11, color: 'var(--text-muted)',
            }}>
              <span>FW <span style={{ color: 'var(--text-secondary)', fontFamily: 'var(--font-mono)' }}>
                v{info.fwMajor}.{info.fwMinor}.{info.fwPatch}
              </span></span>
              <span>Protocol <span style={{ color: 'var(--text-secondary)', fontFamily: 'var(--font-mono)' }}>
                v{info.protocolVersion}
              </span></span>
              <span>PWM <span style={{ color: 'var(--text-secondary)', fontFamily: 'var(--font-mono)' }}>
                {info.pwmFrequency >= 1000 ? `${info.pwmFrequency / 1000}k` : info.pwmFrequency} Hz
              </span></span>
            </div>
          )}

          <div style={{ flex: 1 }} />

          <div style={{ display: 'flex', alignItems: 'center', gap: 6, opacity: 0.6 }}>
            <MicrochipIcon size={16} />
            <span style={{ fontSize: 10, color: 'var(--text-muted)', letterSpacing: '0.5px' }}>
              MICROCHIP dsPIC33AK
            </span>
          </div>

          <div style={{ width: 1, height: 24, background: 'var(--border)', margin: '0 8px' }} />

          <ConnectionBar />
        </div>
      </header>

      {/* Main content */}
      <main style={{ flex: 1, maxWidth: 1280, margin: '0 auto', width: '100%', padding: '20px 24px' }}>
        {/* Top row: Status + Gauges */}
        <div style={{ display: 'grid', gridTemplateColumns: '240px 1fr', gap: 16 }}>
          <StatusPanel />
          <GaugePanel />
        </div>

        {/* Live Scope (replaces TimeChart) */}
        <ScopePanel />

        {/* Controls row */}
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 16, marginTop: 16 }}>
          <ControlPanel />
          <ThrottleSlider />
        </div>

        {/* Profile selector */}
        <ProfileSelector />

        {/* Inline parameters (quick view) */}
        <ParamPanel />

        {/* Help / Documentation */}
        <HelpPanel />
      </main>

      {/* Footer */}
      <footer style={{
        borderTop: '1px solid var(--border)',
        padding: '16px 24px',
        marginTop: 32,
      }}>
        <div style={{
          maxWidth: 1280, margin: '0 auto',
          display: 'flex', justifyContent: 'space-between', alignItems: 'center',
          fontSize: 11, color: 'var(--text-muted)',
        }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
            <MicrochipIcon size={14} />
            <span>Powered by Microchip dsPIC33AK128MC106</span>
          </div>
          <div>
            Garuda ESC Project &middot; WebSerial GUI &middot; GSP Protocol v2
          </div>
        </div>
      </footer>

      {/* Parameter editing modal */}
      <ParamModal />

      {/* Toast notifications */}
      <div style={{
        position: 'fixed', bottom: 20, right: 20,
        display: 'flex', flexDirection: 'column', gap: 8, zIndex: 1000,
      }}>
        {toasts.map(t => (
          <div key={t.id} onClick={() => removeToast(t.id)} style={{
            padding: '10px 16px', borderRadius: 'var(--radius)',
            cursor: 'pointer',
            background: toastColors[t.type].bg,
            borderLeft: `3px solid ${toastColors[t.type].border}`,
            color: 'var(--text-primary)', fontSize: 13, fontWeight: 500,
            boxShadow: 'var(--shadow-lg)',
            animation: 'fadeIn 0.2s ease-out',
            backdropFilter: 'blur(8px)',
          }}>
            {t.message}
          </div>
        ))}
      </div>
    </div>
  );
}
