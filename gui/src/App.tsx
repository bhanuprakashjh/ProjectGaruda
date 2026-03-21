import { Component, type ReactNode } from 'react';
import { ConnectionBar } from './components/ConnectionBar';
import { StatusPanel } from './components/StatusPanel';
import { GaugePanel } from './components/GaugePanel';
import { ScopePanel } from './components/ScopePanel';
import BurstScopePanel from './components/BurstScopePanel';
import { ControlPanel } from './components/ControlPanel';
import { ThrottleSlider } from './components/ThrottleSlider';
import { ProfileSelector } from './components/ProfileSelector';
import { ParamPanel } from './components/ParamPanel';
import { ParamModal } from './components/ParamModal';
import { HelpPanel, MicrochipIcon } from './components/HelpPanel';
import { MotorTuningPanel } from './components/MotorTuningPanel';
import { CkDashboard } from './components/CkDashboard';
import { CkMotorSetup } from './components/CkMotorSetup';
import { CkScopePanel } from './components/CkScopePanel';
import { CkMotorTest } from './components/CkMotorTest';
import { useEscStore, type TabId } from './store/useEscStore';
import { isCkBoard, BOARD_NAMES } from './protocol/types';

class ErrorBoundary extends Component<{ children: ReactNode }, { error: string | null }> {
  state = { error: null as string | null };
  static getDerivedStateFromError(e: Error) { return { error: e.message }; }
  render() {
    if (this.state.error) return (
      <div style={{ padding: 24, color: '#ef4444', background: '#1a1a2e', minHeight: '100vh' }}>
        <h2>Something went wrong</h2>
        <pre style={{ fontSize: 12, whiteSpace: 'pre-wrap' }}>{this.state.error}</pre>
        <button onClick={() => this.setState({ error: null })}
          style={{ marginTop: 12, padding: '8px 16px', background: '#3b82f6', color: '#fff', border: 'none', borderRadius: 4, cursor: 'pointer' }}>
          Try Again
        </button>
      </div>
    );
    return this.props.children;
  }
}

const toastColors = {
  success: { bg: 'var(--accent-green-dim)', border: 'var(--accent-green)' },
  error: { bg: 'var(--accent-red-dim)', border: 'var(--accent-red)' },
  info: { bg: 'var(--accent-blue-dim)', border: 'var(--accent-blue)' },
};

const TABS: { id: TabId; label: string; icon: string }[] = [
  { id: 'dashboard', label: 'Dashboard', icon: 'M3 3h7v7H3zM14 3h7v7h-7zM3 14h7v7H3zM14 14h7v7h-7z' },
  { id: 'scope', label: 'Scope', icon: 'M2 12h4l3-9 4 18 3-9h4' },
  { id: 'test', label: 'Motor Test', icon: 'M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 104 0M9 5a2 2 0 002 2h2a2 2 0 002-2' },
  { id: 'motor', label: 'Motor Setup', icon: 'M12 2L2 7l10 5 10-5-10-5zM2 17l10 5 10-5M2 12l10 5 10-5' },
  { id: 'params', label: 'Parameters', icon: 'M12 3v18M3 12h18M7.5 7.5l9 9M16.5 7.5l-9 9' },
  { id: 'help', label: 'Help', icon: 'M12 2a10 10 0 100 20 10 10 0 000-20zm0 14v-2m0-4a2 2 0 114 0c0 1.5-2 2-2 3' },
];

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

function TabBar() {
  const activeTab = useEscStore(s => s.activeTab);
  const setActiveTab = useEscStore(s => s.setActiveTab);

  return (
    <div style={{
      display: 'flex', gap: 2, padding: '0 24px',
      background: 'var(--bg-secondary)',
      borderBottom: '1px solid var(--border)',
    }}>
      <div style={{ maxWidth: 1280, margin: '0 auto', width: '100%', display: 'flex', gap: 2 }}>
        {TABS.map(tab => {
          const active = activeTab === tab.id;
          return (
            <button
              key={tab.id}
              onClick={() => setActiveTab(tab.id)}
              style={{
                padding: '10px 18px',
                background: active ? 'var(--bg-card)' : 'transparent',
                border: 'none',
                borderBottom: active ? '2px solid var(--accent-blue)' : '2px solid transparent',
                borderTopLeftRadius: 'var(--radius-sm)',
                borderTopRightRadius: 'var(--radius-sm)',
                color: active ? 'var(--accent-blue)' : 'var(--text-muted)',
                fontSize: 12,
                fontWeight: active ? 600 : 500,
                cursor: 'pointer',
                transition: 'all 0.15s',
                letterSpacing: '0.3px',
              }}
            >
              {tab.label}
            </button>
          );
        })}
      </div>
    </div>
  );
}

function DashboardTab() {
  const info = useEscStore(s => s.info);
  const isCk = info && isCkBoard(info.boardId);

  if (isCk) {
    return (
      <>
        <ControlPanel />
        <div style={{ marginTop: 16 }} />
        <CkDashboard />
      </>
    );
  }

  return (
    <>
      {/* Top row: Status + Gauges (AK board) */}
      <div style={{ display: 'grid', gridTemplateColumns: '240px 1fr', gap: 16 }}>
        <StatusPanel />
        <GaugePanel />
      </div>

      {/* Controls row */}
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 16, marginTop: 16 }}>
        <ControlPanel />
        <ThrottleSlider />
      </div>

      {/* Profile selector */}
      <ProfileSelector />
    </>
  );
}

function ScopeTab() {
  const info = useEscStore(s => s.info);
  if (info && isCkBoard(info.boardId)) {
    return <CkScopePanel />;
  }
  return (
    <>
      <ScopePanel />
      <BurstScopePanel />
    </>
  );
}

function MotorTab() {
  const info = useEscStore(s => s.info);
  if (info && isCkBoard(info.boardId)) {
    return <CkMotorSetup />;
  }
  return <MotorTuningPanel />;
}

function ParamsTab() {
  return <ParamPanel />;
}

function HelpTab() {
  return <HelpPanel alwaysExpanded />;
}

export default function App() {
  const toasts = useEscStore(s => s.toasts);
  const removeToast = useEscStore(s => s.removeToast);
  const info = useEscStore(s => s.info);
  const connected = useEscStore(s => s.connected);
  const activeTab = useEscStore(s => s.activeTab);

  const renderTab = () => {
    switch (activeTab) {
      case 'dashboard': return <DashboardTab />;
      case 'scope': return <ScopeTab />;
      case 'test': return <CkMotorTest />;
      case 'motor': return <MotorTab />;
      case 'params': return <ParamsTab />;
      case 'help': return <HelpTab />;
      default: return <DashboardTab />;
    }
  };

  return (
    <div style={{ minHeight: '100vh', display: 'flex', flexDirection: 'column' }}>
      {/* Header */}
      <header style={{
        background: 'var(--bg-secondary)',
        padding: '0 24px',
        position: 'sticky', top: 0, zIndex: 100,
      }}>
        <div style={{
          maxWidth: 1280, margin: '0 auto',
          display: 'flex', alignItems: 'center', height: 52, gap: 12,
        }}>
          <GarudaLogo />
          <div>
            <div style={{ fontSize: 15, fontWeight: 700, letterSpacing: '-0.3px', lineHeight: 1.2 }}>
              Garuda ESC
            </div>
            <div style={{ fontSize: 9, color: 'var(--text-muted)', letterSpacing: '0.5px' }}>
              CONFIGURATOR
            </div>
          </div>

          {connected && info && (
            <div style={{
              marginLeft: 16, display: 'flex', alignItems: 'center', gap: 12,
              fontSize: 10, color: 'var(--text-muted)',
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
            <MicrochipIcon size={14} />
            <span style={{ fontSize: 9, color: 'var(--text-muted)', letterSpacing: '0.5px' }}>
              {info ? (BOARD_NAMES[info.boardId] || `Board 0x${info.boardId.toString(16)}`) : 'dsPIC'}
            </span>
          </div>

          <div style={{ width: 1, height: 20, background: 'var(--border)', margin: '0 6px' }} />

          <ConnectionBar />
        </div>
      </header>

      {/* Tab navigation */}
      <TabBar />

      {/* Main content */}
      <main style={{ flex: 1, maxWidth: 1280, margin: '0 auto', width: '100%', padding: '16px 24px' }}>
        <ErrorBoundary>
          {renderTab()}
        </ErrorBoundary>
      </main>

      {/* Footer */}
      <footer style={{
        borderTop: '1px solid var(--border)',
        padding: '12px 24px',
        marginTop: 24,
      }}>
        <div style={{
          maxWidth: 1280, margin: '0 auto',
          display: 'flex', justifyContent: 'space-between', alignItems: 'center',
          fontSize: 10, color: 'var(--text-muted)',
        }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <MicrochipIcon size={12} />
            <span>Powered by Microchip {info && isCkBoard(info.boardId) ? 'dsPIC33CK64MP205 + ATA6847' : 'dsPIC33AK128MC106'}</span>
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
