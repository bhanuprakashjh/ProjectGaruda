/**
 * DmaEdgeTimeline — visualize raw DMA comparator edges per commutation step.
 *
 * Arms the firmware burst capture, downloads 12 steps, and renders each
 * as a horizontal strip showing:
 *   - Edge dots at their µs position (from commutation)
 *   - PWM period grid lines (every 25 µs at 40 kHz)
 *   - POLL marker (red vertical line)
 *   - PRED marker (blue vertical line)
 *   - Step metadata (phase, polarity, edge count)
 */

import { useState, useCallback, useEffect, useRef } from 'react';
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

export function DmaEdgeTimeline() {
  const [steps, setSteps] = useState<BurstStep[]>([]);
  const [status, setStatus] = useState('idle');
  const [capturing, setCapturing] = useState(false);
  const pendingRef = useRef<{ resolve: (v: any) => void; cmd: number } | null>(null);
  const connected = useEscStore(s => s.connected);

  // Listen for incoming packets via the store's packet handler
  useEffect(() => {
    const handler = (cmd: number, payload: Uint8Array) => {
      if (pendingRef.current && cmd === pendingRef.current.cmd) {
        const { resolve } = pendingRef.current;
        pendingRef.current = null;
        resolve(payload);
      }
    };
    useEscStore.setState({ burstPacketHandler: handler });
    return () => useEscStore.setState({ burstPacketHandler: undefined });
  }, []);

  const waitForResponse = useCallback((cmd: number, timeout = 500): Promise<Uint8Array | null> => {
    return new Promise(resolve => {
      pendingRef.current = { resolve, cmd };
      setTimeout(() => {
        if (pendingRef.current?.cmd === cmd) {
          pendingRef.current = null;
          resolve(null);
        }
      }, timeout);
    });
  }, []);

  const capture = useCallback(async () => {
    if (!connected) return;
    setCapturing(true);
    setSteps([]);
    setStatus('arming...');

    // ARM
    await serial.write(buildPacket(CMD.BURST_ARM));
    const armResp = await waitForResponse(CMD.BURST_ARM, 1000);
    if (!armResp) { setStatus('arm failed'); setCapturing(false); return; }

    // Poll STATUS until FULL
    setStatus('waiting for capture...');
    for (let i = 0; i < 60; i++) {
      await serial.write(buildPacket(CMD.BURST_STATUS));
      const resp = await waitForResponse(CMD.BURST_STATUS, 200);
      if (resp && resp.length >= 2) {
        const state = resp[0];
        const count = resp[1];
        setStatus(`capturing ${count}/${BURST_STEPS}...`);
        if (state === 2) break; // FULL
      }
      await new Promise(r => setTimeout(r, 30));
    }

    // Download all steps
    setStatus('downloading...');
    const downloaded: BurstStep[] = [];
    for (let i = 0; i < BURST_STEPS; i++) {
      await serial.write(buildPacket(CMD.BURST_GET_STEP, new Uint8Array([i])));
      const resp = await waitForResponse(CMD.BURST_GET_STEP, 500);
      if (resp && resp.length >= BURST_STEP_SIZE) {
        const step = decodeBurstStep(resp);
        if (step) downloaded.push(step);
      }
    }

    setSteps(downloaded);
    setStatus(`${downloaded.length} steps captured`);
    setCapturing(false);
  }, [connected, waitForResponse]);

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
          PWM: {PWM_PERIOD_US} µs (40 kHz)
        </span>
      </div>

      {/* Legend */}
      <div style={{ display: 'flex', gap: '14px', marginBottom: '6px', fontSize: '11px', color: '#9ca3af' }}>
        <span><span style={{ color: '#22d3ee' }}>&#9679;</span> edge</span>
        <span><span style={{ color: '#ef4444' }}>|</span> POLL</span>
        <span><span style={{ color: '#3b82f6' }}>|</span> PRED</span>
        <span style={{ color: '#374151' }}>| PWM</span>
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

                {/* PWM grid */}
                {pwmLines.map((t, i) => (
                  <line key={`p${i}`} x1={xScale(t)} y1={y+2} x2={xScale(t)} y2={y+STRIP_HEIGHT-2}
                    stroke="#1e293b" strokeDasharray="2,4" />
                ))}

                {/* PRED */}
                {step.predictedUs > 0 && (
                  <line x1={xScale(step.predictedUs)} y1={y+2} x2={xScale(step.predictedUs)} y2={y+STRIP_HEIGHT-2}
                    stroke="#3b82f6" strokeWidth={2} strokeDasharray="4,2" />
                )}

                {/* POLL */}
                {step.pollUs > 0 && (
                  <line x1={xScale(step.pollUs)} y1={y+2} x2={xScale(step.pollUs)} y2={y+STRIP_HEIGHT-2}
                    stroke="#ef4444" strokeWidth={2} />
                )}

                {/* Edges */}
                {step.edgesUs.map((t, ei) => (
                  <circle key={`e${ei}`} cx={xScale(t)} cy={midY} r={3}
                    fill="#22d3ee" opacity={0.9} />
                ))}

                {/* Label */}
                <text x={4} y={midY+4} fill="var(--text-primary)" fontSize={11} fontFamily="var(--font-mono)">
                  #{si} {step.phase} {step.polarity[0]} {step.edgeCount}e
                </text>

                {step.wasTimeout && (
                  <text x={LEFT_LABEL_W+4} y={midY+4} fill="#ef4444" fontSize={10}>TIMEOUT</text>
                )}
              </g>
            );
          })}

          {/* Time axis */}
          {maxUs > 0 && Array.from({ length: Math.floor(maxUs / 50) + 1 }, (_, i) => i * 50).map(t => (
            <text key={`a${t}`} x={xScale(t)} y={steps.length * (STRIP_HEIGHT + STRIP_MARGIN) + 20}
              fill="#6b7280" fontSize={9} textAnchor="middle" fontFamily="var(--font-mono)">
              {t}µs
            </text>
          ))}
        </svg>
      )}
    </div>
  );
}
