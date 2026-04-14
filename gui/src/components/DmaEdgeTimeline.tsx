/**
 * DmaEdgeTimeline — visualize raw DMA comparator edges per commutation step.
 */

import { useState, useCallback, useRef, useEffect } from 'react';
import { serial } from './ConnectionBar';
import {
  buildPacket, CMD,
  decodeBurstStep, BURST_STEPS, BURST_STEP_SIZE,
  type BurstStep,
} from '../protocol/gsp';
import { useEscStore } from '../store/useEscStore';

const PWM_PERIOD_US = 25.0;
const STRIP_HEIGHT = 40;
const STRIP_MARGIN = 4;
const LEFT_LABEL_W = 90;
const RIGHT_PAD = 20;

/** Simple promise-based packet waiter using a queue */
type PendingWait = { cmd: number; resolve: (payload: Uint8Array) => void; timer: ReturnType<typeof setTimeout> };

export function DmaEdgeTimeline() {
  const [steps, setSteps] = useState<BurstStep[]>([]);
  const [status, setStatus] = useState('idle');
  const [capturing, setCapturing] = useState(false);
  const connected = useEscStore(s => s.connected);
  const waitQueue = useRef<PendingWait[]>([]);

  // Register packet handler that resolves waiting promises
  useEffect(() => {
    const handler = (cmd: number, payload: Uint8Array) => {
      const idx = waitQueue.current.findIndex(w => w.cmd === cmd);
      if (idx >= 0) {
        const w = waitQueue.current[idx];
        waitQueue.current.splice(idx, 1);
        clearTimeout(w.timer);
        w.resolve(new Uint8Array(payload)); // copy to avoid buffer reuse
      }
    };
    useEscStore.setState({ burstPacketHandler: handler });
    return () => {
      useEscStore.setState({ burstPacketHandler: undefined });
      // Clear any pending waits
      waitQueue.current.forEach(w => { clearTimeout(w.timer); w.resolve(new Uint8Array(0)); });
      waitQueue.current = [];
    };
  }, []);

  /** Send a command and wait for a response with matching cmd ID */
  const sendAndWait = useCallback(async (cmd: number, payload?: Uint8Array, timeout = 500): Promise<Uint8Array> => {
    return new Promise<Uint8Array>((resolve) => {
      const timer = setTimeout(() => {
        const idx = waitQueue.current.findIndex(w => w.cmd === cmd);
        if (idx >= 0) waitQueue.current.splice(idx, 1);
        resolve(new Uint8Array(0)); // empty = timeout
      }, timeout);
      // Register BEFORE sending to avoid race
      waitQueue.current.push({ cmd, resolve, timer });
      serial.write(buildPacket(cmd, payload)).catch(() => {
        const idx = waitQueue.current.findIndex(w => w.cmd === cmd);
        if (idx >= 0) waitQueue.current.splice(idx, 1);
        clearTimeout(timer);
        resolve(new Uint8Array(0));
      });
    });
  }, []);

  const capture = useCallback(async () => {
    if (!connected) return;
    setCapturing(true);
    setSteps([]);

    // ARM
    setStatus('arming...');
    const armResp = await sendAndWait(CMD.BURST_ARM, undefined, 1000);
    if (armResp.length === 0) { setStatus('arm timeout'); setCapturing(false); return; }
    setStatus('armed, waiting...');

    // Poll STATUS until FULL
    let full = false;
    for (let i = 0; i < 60; i++) {
      const resp = await sendAndWait(CMD.BURST_STATUS, undefined, 300);
      if (resp.length >= 2) {
        const state = resp[0];
        const count = resp[1];
        setStatus(`capturing ${count}/${BURST_STEPS}...`);
        if (state === 2) { full = true; break; }
      }
      await new Promise(r => setTimeout(r, 50));
    }
    if (!full) { setStatus('capture timeout — is motor running?'); setCapturing(false); return; }

    // Download all steps
    setStatus('downloading...');
    const downloaded: BurstStep[] = [];
    for (let i = 0; i < BURST_STEPS; i++) {
      const resp = await sendAndWait(CMD.BURST_GET_STEP, new Uint8Array([i]), 500);
      if (resp.length >= BURST_STEP_SIZE) {
        const step = decodeBurstStep(resp);
        if (step) downloaded.push(step);
        else setStatus(`step ${i}: decode failed (${resp.length} bytes)`);
      } else {
        setStatus(`step ${i}: short response (${resp.length} bytes)`);
      }
    }

    if (downloaded.length === 0) {
      setStatus('no steps decoded — check firmware');
    } else {
      setStatus(`${downloaded.length} steps, ${downloaded.reduce((s, st) => s + st.edgeCount, 0)} edges total`);
    }
    setSteps(downloaded);
    setCapturing(false);
  }, [connected, sendAndWait]);

  // Compute max time for uniform x-scaling
  const maxUs = steps.reduce((max, s) => {
    const stepMax = Math.max(...s.edgesUs, s.pollUs, s.predictedUs, 0);
    return Math.max(max, stepMax);
  }, 0) + 20;

  const chartWidth = 820;
  const plotWidth = chartWidth - LEFT_LABEL_W - RIGHT_PAD;
  const xScale = (us: number) => LEFT_LABEL_W + (us / (maxUs || 1)) * plotWidth;

  const pwmLines: number[] = [];
  for (let t = PWM_PERIOD_US; t < maxUs; t += PWM_PERIOD_US) pwmLines.push(t);

  return (
    <div style={{ padding: '12px' }}>
      <h3 style={{ margin: '0 0 8px', color: 'var(--text-primary)', fontSize: '14px' }}>
        DMA Edge Timeline
      </h3>

      <div style={{ display: 'flex', alignItems: 'center', gap: '12px', marginBottom: '8px' }}>
        <button
          onClick={capture}
          disabled={capturing || !connected}
          style={{
            padding: '6px 14px',
            background: capturing ? '#374151' : 'var(--accent-blue)',
            color: '#fff', border: 'none', borderRadius: '6px',
            cursor: capturing || !connected ? 'not-allowed' : 'pointer',
            fontFamily: 'var(--font-mono)', fontSize: '12px',
          }}
        >
          {capturing ? 'Capturing...' : 'Capture'}
        </button>
        <span style={{ color: 'var(--text-primary)', fontFamily: 'var(--font-mono)', fontSize: '12px' }}>
          {status}
        </span>
        <span style={{ color: '#6b7280', fontSize: '11px', marginLeft: 'auto' }}>
          PWM: {PWM_PERIOD_US} µs | {steps.length} steps
        </span>
      </div>

      {/* Legend */}
      <div style={{ display: 'flex', gap: '14px', marginBottom: '6px', fontSize: '11px', color: '#9ca3af' }}>
        <span><span style={{ color: '#22d3ee' }}>&#9679;</span> edge</span>
        <span><span style={{ color: '#ef4444' }}>|</span> POLL</span>
        <span><span style={{ color: '#3b82f6' }}>|</span> PRED</span>
        <span style={{ color: '#374151' }}>&#124; PWM</span>
      </div>

      {steps.length > 0 && (
        <svg
          width={chartWidth}
          height={steps.length * (STRIP_HEIGHT + STRIP_MARGIN) + 24}
          style={{ background: 'var(--bg-card)', borderRadius: '8px', border: '1px solid var(--border)' }}
        >
          {steps.map((step, si) => {
            const y = si * (STRIP_HEIGHT + STRIP_MARGIN) + 8;
            const midY = y + STRIP_HEIGHT / 2;
            return (
              <g key={si}>
                <rect x={LEFT_LABEL_W} y={y} width={plotWidth} height={STRIP_HEIGHT}
                  fill={step.wasTimeout ? '#1a0000' : '#0d1117'} rx={3} />

                {pwmLines.map((t, i) => (
                  <line key={`p${i}`} x1={xScale(t)} y1={y+2} x2={xScale(t)} y2={y+STRIP_HEIGHT-2}
                    stroke="#1e293b" strokeDasharray="2,4" />
                ))}

                {step.predictedUs > 0 && (
                  <line x1={xScale(step.predictedUs)} y1={y+2} x2={xScale(step.predictedUs)} y2={y+STRIP_HEIGHT-2}
                    stroke="#3b82f6" strokeWidth={2} strokeDasharray="4,2" />
                )}

                {step.pollUs > 0 && (
                  <line x1={xScale(step.pollUs)} y1={y+2} x2={xScale(step.pollUs)} y2={y+STRIP_HEIGHT-2}
                    stroke="#ef4444" strokeWidth={2} />
                )}

                {step.edgesUs.map((t, ei) => (
                  <circle key={`e${ei}`} cx={xScale(t)} cy={midY} r={3}
                    fill="#22d3ee" opacity={0.9} />
                ))}

                <text x={4} y={midY+4} fill="var(--text-primary)" fontSize={11} fontFamily="var(--font-mono)">
                  #{si} {step.phase} {step.polarity[0]} {step.edgeCount}e
                </text>

                {step.wasTimeout && (
                  <text x={LEFT_LABEL_W+4} y={midY+4} fill="#ef4444" fontSize={10}>TIMEOUT</text>
                )}
              </g>
            );
          })}

          {maxUs > 0 && Array.from({ length: Math.floor(maxUs / 50) + 1 }, (_, i) => i * 50).map(t => (
            <text key={`a${t}`} x={xScale(t)} y={steps.length * (STRIP_HEIGHT + STRIP_MARGIN) + 20}
              fill="#6b7280" fontSize={9} textAnchor="middle" fontFamily="var(--font-mono)">
              {t}µs
            </text>
          ))}
        </svg>
      )}

      {steps.length === 0 && !capturing && status !== 'idle' && (
        <div style={{ color: '#6b7280', fontFamily: 'var(--font-mono)', fontSize: '12px', padding: '20px', textAlign: 'center' }}>
          No edge data. Start the motor and click Capture.
        </div>
      )}
    </div>
  );
}
