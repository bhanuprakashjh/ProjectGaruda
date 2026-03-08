import { useState, useCallback, useRef, useEffect } from 'react';
import { useEscStore } from '../store/useEscStore';
import { serial } from './ConnectionBar';
import { isBurstScopeEnabled, SCOPE_TRIG_MODES, SCOPE_STATES, SCOPE_CHANNELS } from '../protocol/types';
import type { ScopeArmConfig } from '../protocol/types';
import { CMD, buildPacket, buildScopeArmPayload } from '../protocol/gsp';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip,
         ResponsiveContainer, Legend, ReferenceLine } from 'recharts';

const DT_US = 1e6 / 24000;  // 41.67 µs per sample

export default function BurstScopePanel() {
  const info = useEscStore(s => s.info);
  const scopeStatus = useEscStore(s => s.scopeStatus);
  const samples = useEscStore(s => s.scopeSamples);
  const scopeReading = useEscStore(s => s.scopeReading);
  const clearScopeSamples = useEscStore(s => s.clearScopeSamples);

  const [trigMode, setTrigMode] = useState(1); // fault
  const [prePct, setPrePct] = useState(50);
  const [trigCh, setTrigCh] = useState(3); // iq
  const [trigEdge, setTrigEdge] = useState(0); // rising
  const [threshold, setThreshold] = useState(1000);
  const [polling, setPolling] = useState(false);
  const pollRef = useRef<ReturnType<typeof setInterval> | null>(null);

  // Clean up polling on unmount
  useEffect(() => () => {
    if (pollRef.current) clearInterval(pollRef.current);
  }, []);

  // Stop polling when scope becomes READY
  useEffect(() => {
    if (scopeStatus?.state === 3 && pollRef.current) {
      clearInterval(pollRef.current);
      pollRef.current = null;
      setPolling(false);
    }
  }, [scopeStatus?.state]);

  if (!info || !isBurstScopeEnabled(info.featureFlags)) {
    return (
      <div style={{ padding: 16, color: 'var(--text-muted)', fontStyle: 'italic' }}>
        Burst scope not available (FEATURE_BURST_SCOPE not enabled in firmware)
      </div>
    );
  }

  const handleArm = useCallback(async () => {
    clearScopeSamples();
    const cfg: ScopeArmConfig = { trigMode, preTrigPct: prePct, trigChannel: trigCh, trigEdge, threshold };
    await serial.write(buildPacket(CMD.SCOPE_ARM, buildScopeArmPayload(cfg)));

    // Start polling status at 5Hz
    setPolling(true);
    if (pollRef.current) clearInterval(pollRef.current);
    pollRef.current = setInterval(() => {
      serial.write(buildPacket(CMD.SCOPE_STATUS));
    }, 200);

    // Timeout after 60s
    setTimeout(() => {
      if (pollRef.current) {
        clearInterval(pollRef.current);
        pollRef.current = null;
        setPolling(false);
      }
    }, 60000);
  }, [trigMode, prePct, trigCh, trigEdge, threshold, clearScopeSamples]);

  const stateLabel = scopeStatus ? (SCOPE_STATES[scopeStatus.state] || '?') : 'N/A';

  // Build chart data with time axis relative to trigger
  const chartData = samples.map((s, i) => ({
    t_us: +((i - Math.floor(samples.length * prePct / 100)) * DT_US).toFixed(1),
    ia: s.ia, ib: s.ib, id: s.id, iq: s.iq,
    vd: s.vd, vq: s.vq,
    theta: s.theta, omega: s.omega,
    modIndex: s.modIndex, obsX1: s.obsX1, obsX2: s.obsX2,
  }));

  return (
    <div style={{ padding: 16 }}>
      <h3 style={{ margin: '0 0 12px', fontSize: 14, fontWeight: 600 }}>
        Burst Scope (128 samples @ 24kHz = 5.33ms window)
      </h3>

      {/* Controls */}
      <div style={{
        display: 'flex', flexWrap: 'wrap', gap: 12, marginBottom: 16,
        padding: 12, background: 'var(--bg-secondary)', borderRadius: 'var(--radius-sm)',
      }}>
        <label style={{ fontSize: 11, display: 'flex', flexDirection: 'column', gap: 2 }}>
          Trigger
          <select value={trigMode} onChange={e => setTrigMode(+e.target.value)}
                  style={{ fontSize: 11, padding: '2px 4px' }}>
            {SCOPE_TRIG_MODES.map((name, i) => <option key={i} value={i}>{name}</option>)}
          </select>
        </label>

        <label style={{ fontSize: 11, display: 'flex', flexDirection: 'column', gap: 2 }}>
          Pre-trigger %
          <input type="number" min={0} max={100} value={prePct}
                 onChange={e => setPrePct(+e.target.value)}
                 style={{ width: 50, fontSize: 11, padding: '2px 4px' }} />
        </label>

        {trigMode === 3 && <>
          <label style={{ fontSize: 11, display: 'flex', flexDirection: 'column', gap: 2 }}>
            Channel
            <select value={trigCh} onChange={e => setTrigCh(+e.target.value)}
                    style={{ fontSize: 11, padding: '2px 4px' }}>
              {SCOPE_CHANNELS.map((name, i) => name ? <option key={i} value={i}>{name}</option> : null)}
            </select>
          </label>

          <label style={{ fontSize: 11, display: 'flex', flexDirection: 'column', gap: 2 }}>
            Edge
            <select value={trigEdge} onChange={e => setTrigEdge(+e.target.value)}
                    style={{ fontSize: 11, padding: '2px 4px' }}>
              <option value={0}>Rising</option>
              <option value={1}>Falling</option>
            </select>
          </label>

          <label style={{ fontSize: 11, display: 'flex', flexDirection: 'column', gap: 2 }}>
            Threshold (scaled)
            <input type="number" value={threshold}
                   onChange={e => setThreshold(+e.target.value)}
                   style={{ width: 70, fontSize: 11, padding: '2px 4px' }} />
          </label>
        </>}

        <button onClick={handleArm} disabled={polling || scopeReading}
                style={{
                  alignSelf: 'flex-end', padding: '4px 16px', fontSize: 11,
                  fontWeight: 600, cursor: polling ? 'wait' : 'pointer',
                  background: (polling || scopeReading) ? 'var(--bg-tertiary)' : 'var(--accent)',
                  color: 'white', border: 'none', borderRadius: 4,
                }}>
          {polling ? `Waiting... (${stateLabel})` : scopeReading ? 'Reading...' : 'Arm & Capture'}
        </button>

        <div style={{ alignSelf: 'flex-end', fontSize: 10, color: 'var(--text-muted)' }}>
          Status: {stateLabel}
          {scopeStatus && ` | ${scopeStatus.sampleCount} samples`}
          {samples.length > 0 && ` | ${samples.length} read`}
        </div>
      </div>

      {/* Charts */}
      {samples.length > 0 && (
        <>
          <div style={{ height: 200, marginBottom: 8 }}>
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={chartData} margin={{ top: 5, right: 30, left: 10, bottom: 5 }}>
                <CartesianGrid strokeDasharray="3 3" stroke="var(--border)" />
                <XAxis dataKey="t_us" fontSize={9} label={{ value: 'µs', position: 'insideBottomRight', offset: -5, fontSize: 9 }} />
                <YAxis yAxisId="current" fontSize={9} label={{ value: 'A', angle: -90, position: 'insideLeft', fontSize: 9 }} />
                <Tooltip contentStyle={{ fontSize: 10, background: 'var(--bg-primary)', border: '1px solid var(--border)' }} />
                <Legend iconSize={8} wrapperStyle={{ fontSize: 10 }} />
                <ReferenceLine x={0} yAxisId="current" stroke="red" strokeDasharray="3 3" label="trigger" />
                <Line yAxisId="current" type="monotone" dataKey="id" name="Id" stroke="#22c55e" strokeWidth={1.5} dot={false} isAnimationActive={false} />
                <Line yAxisId="current" type="monotone" dataKey="iq" name="Iq" stroke="#eab308" strokeWidth={1.5} dot={false} isAnimationActive={false} />
                <Line yAxisId="current" type="monotone" dataKey="ia" name="Ia" stroke="#a78bfa" strokeWidth={1} dot={false} isAnimationActive={false} />
                <Line yAxisId="current" type="monotone" dataKey="ib" name="Ib" stroke="#c084fc" strokeWidth={1} dot={false} isAnimationActive={false} />
              </LineChart>
            </ResponsiveContainer>
          </div>

          <div style={{ height: 160, marginBottom: 8 }}>
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={chartData} margin={{ top: 5, right: 30, left: 10, bottom: 5 }}>
                <CartesianGrid strokeDasharray="3 3" stroke="var(--border)" />
                <XAxis dataKey="t_us" fontSize={9} />
                <YAxis yAxisId="voltage" fontSize={9} label={{ value: 'V', angle: -90, position: 'insideLeft', fontSize: 9 }} />
                <YAxis yAxisId="mod" orientation="right" fontSize={9} domain={[0, 1]} />
                <Tooltip contentStyle={{ fontSize: 10, background: 'var(--bg-primary)', border: '1px solid var(--border)' }} />
                <Legend iconSize={8} wrapperStyle={{ fontSize: 10 }} />
                <ReferenceLine x={0} yAxisId="voltage" stroke="red" strokeDasharray="3 3" />
                <Line yAxisId="voltage" type="monotone" dataKey="vd" name="Vd" stroke="#f97316" strokeWidth={1.5} dot={false} isAnimationActive={false} />
                <Line yAxisId="voltage" type="monotone" dataKey="vq" name="Vq" stroke="#06b6d4" strokeWidth={1.5} dot={false} isAnimationActive={false} />
                <Line yAxisId="mod" type="monotone" dataKey="modIndex" name="ModIdx" stroke="#ef4444" strokeWidth={1.5} dot={false} isAnimationActive={false} />
              </LineChart>
            </ResponsiveContainer>
          </div>

          <div style={{ height: 140 }}>
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={chartData} margin={{ top: 5, right: 30, left: 10, bottom: 5 }}>
                <CartesianGrid strokeDasharray="3 3" stroke="var(--border)" />
                <XAxis dataKey="t_us" fontSize={9} label={{ value: 'µs', position: 'insideBottomRight', offset: -5, fontSize: 9 }} />
                <YAxis yAxisId="speed" fontSize={9} label={{ value: 'rad/s', angle: -90, position: 'insideLeft', fontSize: 9 }} />
                <YAxis yAxisId="flux" orientation="right" fontSize={9} />
                <Tooltip contentStyle={{ fontSize: 10, background: 'var(--bg-primary)', border: '1px solid var(--border)' }} />
                <Legend iconSize={8} wrapperStyle={{ fontSize: 10 }} />
                <ReferenceLine x={0} yAxisId="speed" stroke="red" strokeDasharray="3 3" />
                <Line yAxisId="speed" type="monotone" dataKey="omega" name="omega" stroke="#8b5cf6" strokeWidth={1.5} dot={false} isAnimationActive={false} />
                <Line yAxisId="flux" type="monotone" dataKey="obsX1" name="FluxA" stroke="#10b981" strokeWidth={1} dot={false} isAnimationActive={false} />
                <Line yAxisId="flux" type="monotone" dataKey="obsX2" name="FluxB" stroke="#f59e0b" strokeWidth={1} dot={false} isAnimationActive={false} />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </>
      )}
    </div>
  );
}
