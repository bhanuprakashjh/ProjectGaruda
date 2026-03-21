/**
 * CK Motor Test Panel — automated sweep tests + manual pot recording.
 * Full telemetry CSV capture with all snapshot fields for debugging.
 */

import { useState, useRef, useCallback } from 'react';
import { useEscStore } from '../store/useEscStore';
import { serial } from './ConnectionBar';
import { buildPacket, CMD } from '../protocol/gsp';
import { CK_CURRENT_SCALE, CK_VBUS_SCALE, CK_ESC_STATES, CK_FAULT_CODES } from '../protocol/types';
import type { CkSnapshot } from '../protocol/types';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Legend } from 'recharts';

/* ── Full sample with all snapshot fields ─────────────────────────── */

interface FullSample {
  timeMs: number;
  throttlePct: number;
  /* Raw snapshot fields */
  state: number;
  faultCode: number;
  currentStep: number;
  ataStatus: number;
  potRaw: number;
  dutyPct: number;
  dutyRaw: number;
  zcSynced: number;
  vbusRaw: number;
  iaRaw: number;
  ibRaw: number;
  ibusRaw: number;
  stepPeriod: number;
  stepPeriodHR: number;
  eRpm: number;
  goodZcCount: number;
  zcInterval: number;
  prevZcInterval: number;
  icAccepted: number;
  icFalse: number;
  filterLevel: number;
  missedSteps: number;
  forcedSteps: number;
  ilimActive: number;
  systemTick: number;
  uptimeSec: number;
  /* Computed */
  iaMa: number;
  ibMa: number;
  ibusMa: number;
  vbusV: number;
  powerW: number;
  zcAlternation: number;
  mechRpm: number;
}

const CSV_HEADERS = [
  'time_ms','throttle_pct','state','fault','step','ata_status','pot_raw',
  'duty_pct','duty_raw','zc_synced','vbus_raw','ia_raw','ib_raw','ibus_raw',
  'step_period','step_period_hr','erpm','good_zc','zc_interval','prev_zc_interval',
  'ic_accepted','ic_false','filter_level','missed','forced','ilim_active',
  'system_tick','uptime_sec',
  'ia_mA','ib_mA','ibus_mA','vbus_V','power_W','zc_alternation','mech_rpm',
];

function snapshotToSample(s: CkSnapshot, timeMs: number, throttlePct: number, pp: number): FullSample {
  const iaMa = Math.round(s.iaRaw * CK_CURRENT_SCALE);
  const ibMa = Math.round(s.ibRaw * CK_CURRENT_SCALE);
  const ibusMa = Math.round(s.ibusRaw * CK_CURRENT_SCALE);
  const vbusV = +(s.vbusRaw / CK_VBUS_SCALE).toFixed(2);
  return {
    timeMs, throttlePct,
    state: s.state, faultCode: s.faultCode, currentStep: s.currentStep,
    ataStatus: s.ataStatus, potRaw: s.potRaw, dutyPct: s.dutyPct,
    dutyRaw: s.duty, zcSynced: s.zcSynced ? 1 : 0, vbusRaw: s.vbusRaw,
    iaRaw: s.iaRaw, ibRaw: s.ibRaw, ibusRaw: s.ibusRaw,
    stepPeriod: s.stepPeriod, stepPeriodHR: s.stepPeriodHR, eRpm: s.eRpm,
    goodZcCount: s.goodZcCount, zcInterval: s.zcInterval,
    prevZcInterval: s.prevZcInterval, icAccepted: s.icAccepted,
    icFalse: s.icFalse, filterLevel: s.filterLevel, missedSteps: s.missedSteps,
    forcedSteps: s.forcedSteps, ilimActive: s.ilimActive ? 1 : 0,
    systemTick: s.systemTick, uptimeSec: s.uptimeSec,
    iaMa, ibMa, ibusMa, vbusV,
    powerW: +(vbusV * ibusMa / 1000).toFixed(1),
    zcAlternation: Math.abs(s.zcInterval - s.prevZcInterval),
    mechRpm: pp > 0 ? Math.round(s.eRpm / pp) : s.eRpm,
  };
}

function sampleToCsvRow(s: FullSample): string {
  return [
    s.timeMs, s.throttlePct, s.state, s.faultCode, s.currentStep,
    s.ataStatus, s.potRaw, s.dutyPct, s.dutyRaw, s.zcSynced, s.vbusRaw,
    s.iaRaw, s.ibRaw, s.ibusRaw, s.stepPeriod, s.stepPeriodHR, s.eRpm,
    s.goodZcCount, s.zcInterval, s.prevZcInterval, s.icAccepted, s.icFalse,
    s.filterLevel, s.missedSteps, s.forcedSteps, s.ilimActive,
    s.systemTick, s.uptimeSec,
    s.iaMa, s.ibMa, s.ibusMa, s.vbusV, s.powerW, s.zcAlternation, s.mechRpm,
  ].join(',');
}

/* ── Test profiles ────────────────────────────────────────────────── */

interface Waypoint { throttlePct: number; durationMs: number; }
interface TestProfile { name: string; description: string; waypoints: Waypoint[]; }

const PROFILES: TestProfile[] = [
  { name: 'Slow Sweep', description: '0→100%→0% over 20s',
    waypoints: [{ throttlePct: 0, durationMs: 1000 }, { throttlePct: 100, durationMs: 10000 }, { throttlePct: 0, durationMs: 10000 }] },
  { name: 'Fast Sweep', description: '0→100%→0% over 6s',
    waypoints: [{ throttlePct: 0, durationMs: 500 }, { throttlePct: 100, durationMs: 3000 }, { throttlePct: 0, durationMs: 3000 }] },
  { name: 'Step 50%', description: 'Jump to 50%, hold 3s',
    waypoints: [{ throttlePct: 0, durationMs: 1000 }, { throttlePct: 50, durationMs: 100 }, { throttlePct: 50, durationMs: 3000 }, { throttlePct: 0, durationMs: 100 }, { throttlePct: 0, durationMs: 2000 }] },
  { name: 'Step 100%', description: 'Full throttle step 3s',
    waypoints: [{ throttlePct: 0, durationMs: 1000 }, { throttlePct: 100, durationMs: 100 }, { throttlePct: 100, durationMs: 3000 }, { throttlePct: 0, durationMs: 100 }, { throttlePct: 0, durationMs: 2000 }] },
  { name: 'Drone Sim', description: 'Rapid accel/decel cycles',
    waypoints: [{ throttlePct: 0, durationMs: 500 }, { throttlePct: 80, durationMs: 500 }, { throttlePct: 20, durationMs: 500 }, { throttlePct: 100, durationMs: 500 }, { throttlePct: 10, durationMs: 500 }, { throttlePct: 70, durationMs: 500 }, { throttlePct: 30, durationMs: 500 }, { throttlePct: 90, durationMs: 500 }, { throttlePct: 0, durationMs: 1000 }] },
  { name: 'Idle Endurance', description: '30s at 5% idle',
    waypoints: [{ throttlePct: 5, durationMs: 30000 }, { throttlePct: 0, durationMs: 1000 }] },
];

/* ── Helpers ──────────────────────────────────────────────────────── */

function pctToThrottle(pct: number): number {
  if (pct <= 0) return 0;
  return Math.round(2000 + (pct / 100) * 62000);
}

async function sendThrottle(pct: number) {
  const val = pctToThrottle(pct);
  const buf = new Uint8Array(3);
  buf[0] = val & 0xFF; buf[1] = (val >> 8) & 0xFF; buf[2] = pct > 0 ? 0x01 : 0x00;
  await serial.write(buildPacket(CMD.SET_THROTTLE, buf));
}

async function releaseThrottle() {
  const buf = new Uint8Array(3);
  buf[0] = 0; buf[1] = 0; buf[2] = 0x00;
  await serial.write(buildPacket(CMD.SET_THROTTLE, buf));
}

function downloadCsv(samples: FullSample[], name: string) {
  const csv = CSV_HEADERS.join(',') + '\n' + samples.map(sampleToCsvRow).join('\n');
  const blob = new Blob([csv], { type: 'text/csv' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `${name}_${Date.now()}.csv`;
  a.click();
  URL.revokeObjectURL(url);
}

/* ── Summary computation ──────────────────────────────────────────── */

function computeSummary(samples: FullSample[]) {
  if (samples.length === 0) return null;
  const maxErpm = Math.max(0, ...samples.map(s => s.eRpm));
  const maxIbus = Math.max(0, ...samples.map(s => Math.abs(s.ibusMa)));
  const maxIa = Math.max(0, ...samples.map(s => Math.abs(s.iaMa)));
  const maxIb = Math.max(0, ...samples.map(s => Math.abs(s.ibMa)));
  const maxPower = Math.max(0, ...samples.map(s => Math.abs(s.powerW)));
  const avgVbus = samples.reduce((s, x) => s + x.vbusV, 0) / samples.length;
  const minVbus = Math.min(...samples.map(s => s.vbusV));
  const totalFaults = samples.filter(s => s.faultCode > 0).length;
  const totalDesyncs = samples.filter(s => s.zcSynced === 0 && s.state >= 3).length;
  const maxForced = Math.max(0, ...samples.map(s => s.forcedSteps));
  const maxMissed = Math.max(0, ...samples.map(s => s.missedSteps));
  const avgZcAlt = samples.filter(s => s.eRpm > 0).reduce((s, x) => s + x.zcAlternation, 0) /
    Math.max(1, samples.filter(s => s.eRpm > 0).length);
  const hasFault = totalFaults > 0;
  return {
    maxErpm, maxIbus, maxIa, maxIb, maxPower, avgVbus: +avgVbus.toFixed(2),
    minVbus: +minVbus.toFixed(2), totalFaults, totalDesyncs, maxForced, maxMissed,
    avgZcAlt: +avgZcAlt.toFixed(1), peakDuty: Math.max(0, ...samples.map(s => s.dutyPct)),
    result: (hasFault ? 'FAIL' : 'PASS') as 'PASS' | 'FAIL',
    duration: ((samples[samples.length - 1].timeMs - samples[0].timeMs) / 1000).toFixed(1),
    count: samples.length,
  };
}

/* ── Component ────────────────────────────────────────────────────── */

export function CkMotorTest() {
  const connected = useEscStore(s => s.connected);
  const info = useEscStore(s => s.info);
  const pp = info?.motorPolePairs ?? 1;

  const [selectedProfile, setSelectedProfile] = useState(0);
  const [running, setRunning] = useState(false);
  const [recording, setRecording] = useState(false);
  const [progress, setProgress] = useState(0);
  const [currentThrottle, setCurrentThrottle] = useState(0);
  const [samples, setSamples] = useState<FullSample[]>([]);
  const [manualPct, setManualPct] = useState(0);

  const abortRef = useRef(false);
  const startTimeRef = useRef(0);
  const recordIntervalRef = useRef<number | null>(null);
  const recordSamplesRef = useRef<FullSample[]>([]);

  /* Capture one sample from current snapshot */
  const captureSample = useCallback((throttlePct: number): FullSample | null => {
    const s = useEscStore.getState().ckSnapshot;
    if (!s) return null;
    return snapshotToSample(s, Date.now() - startTimeRef.current, throttlePct, pp);
  }, [pp]);

  /* ── Manual pot recording mode ────────────────────────────── */

  const startRecording = useCallback(() => {
    startTimeRef.current = Date.now();
    recordSamplesRef.current = [];
    setRecording(true);
    setSamples([]);

    /* Sample at 50Hz from the live telemetry stream */
    recordIntervalRef.current = window.setInterval(() => {
      const s = useEscStore.getState().ckSnapshot;
      if (!s) return;
      const sample = snapshotToSample(s, Date.now() - startTimeRef.current, -1, pp);
      sample.throttlePct = -1; /* -1 = pot control (manual) */
      recordSamplesRef.current.push(sample);
    }, 20); /* 50Hz */
  }, [pp]);

  const stopRecording = useCallback(() => {
    if (recordIntervalRef.current !== null) {
      clearInterval(recordIntervalRef.current);
      recordIntervalRef.current = null;
    }
    setRecording(false);
    setSamples([...recordSamplesRef.current]);
  }, []);

  /* ── Automated sweep test ─────────────────────────────────── */

  const runTest = useCallback(async () => {
    if (!connected || running) return;
    const profile = PROFILES[selectedProfile];
    abortRef.current = false;
    const testSamples: FullSample[] = [];
    startTimeRef.current = Date.now();
    setRunning(true);
    setSamples([]);
    setProgress(0);

    await serial.write(buildPacket(CMD.START_MOTOR));
    await new Promise(r => setTimeout(r, 500));

    const totalMs = profile.waypoints.reduce((sum, w) => sum + w.durationMs, 0);
    let elapsed = 0;
    const stepMs = 50;

    for (let i = 0; i < profile.waypoints.length && !abortRef.current; i++) {
      const wp = profile.waypoints[i];
      const prevPct = i > 0 ? profile.waypoints[i - 1].throttlePct : 0;
      const steps = Math.max(1, Math.round(wp.durationMs / stepMs));

      for (let j = 0; j < steps && !abortRef.current; j++) {
        const pct = Math.round((prevPct + (wp.throttlePct - prevPct) * j / steps) * 10) / 10;
        await sendThrottle(pct);
        setCurrentThrottle(pct);

        const sample = captureSample(pct);
        if (sample) {
          testSamples.push(sample);
          if (sample.faultCode > 0) abortRef.current = true;
        }
        elapsed += stepMs;
        setProgress(Math.min(100, (elapsed / totalMs) * 100));
        await new Promise(r => setTimeout(r, stepMs));
      }
    }

    await releaseThrottle();
    setCurrentThrottle(0);
    await new Promise(r => setTimeout(r, 200));
    await serial.write(buildPacket(CMD.STOP_MOTOR));
    setRunning(false);
    setProgress(100);
    setSamples(testSamples);
  }, [connected, running, selectedProfile, captureSample]);

  const abortTest = useCallback(async () => {
    abortRef.current = true;
    await releaseThrottle();
    await serial.write(buildPacket(CMD.STOP_MOTOR));
    setRunning(false);
    setCurrentThrottle(0);
  }, []);

  /* Manual throttle */
  const sendManual = useCallback(async () => {
    if (!connected) return;
    await sendThrottle(manualPct);
    setCurrentThrottle(manualPct);
  }, [connected, manualPct]);

  const releaseManual = useCallback(async () => {
    await releaseThrottle();
    setCurrentThrottle(0);
    setManualPct(0);
  }, []);

  const summary = samples.length > 0 ? computeSummary(samples) : null;

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 16 }}>
      {/* Manual Pot Recording */}
      <div style={{ background: 'var(--bg-card)', borderRadius: 'var(--radius)', padding: 16, border: '1px solid var(--border)' }}>
        <div style={{ fontSize: 13, color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '1px', marginBottom: 12 }}>
          Manual Pot Recording
        </div>
        <div style={{ fontSize: 11, color: 'var(--text-muted)', marginBottom: 8 }}>
          Start the motor with SW1 or GUI, use the pot to control speed. Press Record to capture all telemetry at 50Hz. Download CSV for analysis.
        </div>
        <div style={{ display: 'flex', gap: 8, alignItems: 'center' }}>
          {!recording ? (
            <button onClick={startRecording} disabled={!connected || running}
              style={{ padding: '8px 20px', borderRadius: 'var(--radius-sm)', border: 'none', background: connected && !running ? '#ef4444' : 'var(--bg-input)', color: connected && !running ? '#fff' : 'var(--text-muted)', fontWeight: 600, fontSize: 13, cursor: connected && !running ? 'pointer' : 'default' }}>
              Record
            </button>
          ) : (
            <button onClick={stopRecording}
              style={{ padding: '8px 20px', borderRadius: 'var(--radius-sm)', border: 'none', background: '#ef4444', color: '#fff', fontWeight: 600, fontSize: 13, cursor: 'pointer', animation: 'pulse 1.5s infinite' }}>
              Stop ({recordSamplesRef.current.length} samples)
            </button>
          )}
          {recording && (
            <span style={{ fontSize: 12, color: '#ef4444', fontWeight: 600, display: 'flex', alignItems: 'center', gap: 4 }}>
              <span style={{ width: 8, height: 8, borderRadius: '50%', background: '#ef4444', animation: 'pulse 1s infinite' }} />
              RECORDING
            </span>
          )}
        </div>
      </div>

      {/* GSP Throttle Override */}
      <div style={{ background: 'var(--bg-card)', borderRadius: 'var(--radius)', padding: 16, border: '1px solid var(--border)' }}>
        <div style={{ fontSize: 13, color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '1px', marginBottom: 12 }}>
          GSP Throttle Override
        </div>
        <div style={{ display: 'flex', gap: 12, alignItems: 'center' }}>
          <input type="range" min="0" max="100" value={manualPct} onChange={e => setManualPct(+e.target.value)}
            disabled={running || !connected} style={{ flex: 1 }} />
          <span style={{ fontSize: 14, fontFamily: 'var(--font-mono)', minWidth: 45, textAlign: 'right' }}>{manualPct}%</span>
          <button onClick={sendManual} disabled={running || !connected}
            style={{ padding: '6px 14px', borderRadius: 'var(--radius-sm)', border: 'none', background: 'var(--accent-blue)', color: '#fff', fontWeight: 600, fontSize: 12, cursor: running || !connected ? 'default' : 'pointer', opacity: running || !connected ? 0.5 : 1 }}>
            Set
          </button>
          <button onClick={releaseManual} disabled={running || !connected}
            style={{ padding: '6px 14px', borderRadius: 'var(--radius-sm)', border: '1px solid var(--border)', background: 'var(--bg-input)', color: 'var(--text-primary)', fontWeight: 600, fontSize: 12, cursor: running || !connected ? 'default' : 'pointer', opacity: running || !connected ? 0.5 : 1 }}>
            Release
          </button>
        </div>
      </div>

      {/* Automated Test Profiles */}
      <div style={{ background: 'var(--bg-card)', borderRadius: 'var(--radius)', padding: 16, border: '1px solid var(--border)' }}>
        <div style={{ fontSize: 13, color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '1px', marginBottom: 12 }}>
          Automated Test Profiles
        </div>
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(180px, 1fr))', gap: 8 }}>
          {PROFILES.map((p, i) => (
            <button key={i} onClick={() => !running && setSelectedProfile(i)}
              style={{ padding: '8px 12px', borderRadius: 'var(--radius-sm)', cursor: running ? 'default' : 'pointer', border: `2px solid ${selectedProfile === i ? 'var(--accent-blue)' : 'var(--border)'}`, background: selectedProfile === i ? 'var(--accent-blue-dim)' : 'var(--bg-secondary)', textAlign: 'left' }}>
              <div style={{ fontSize: 12, fontWeight: 600, color: selectedProfile === i ? 'var(--accent-blue)' : 'var(--text-primary)' }}>{p.name}</div>
              <div style={{ fontSize: 10, color: 'var(--text-muted)' }}>{p.description}</div>
            </button>
          ))}
        </div>
        <div style={{ display: 'flex', gap: 8, marginTop: 12, alignItems: 'center' }}>
          {!running ? (
            <button onClick={runTest} disabled={!connected || recording}
              style={{ padding: '8px 24px', borderRadius: 'var(--radius-sm)', border: 'none', background: connected && !recording ? 'var(--accent-green)' : 'var(--bg-input)', color: connected && !recording ? '#000' : 'var(--text-muted)', fontWeight: 600, fontSize: 13, cursor: connected && !recording ? 'pointer' : 'default' }}>
              Run Test
            </button>
          ) : (
            <button onClick={abortTest}
              style={{ padding: '8px 24px', borderRadius: 'var(--radius-sm)', border: 'none', background: 'var(--accent-red)', color: '#fff', fontWeight: 600, fontSize: 13, cursor: 'pointer' }}>
              ABORT
            </button>
          )}
          {running && (
            <>
              <div style={{ flex: 1, height: 6, background: 'var(--border)', borderRadius: 3 }}>
                <div style={{ height: '100%', borderRadius: 3, width: `${progress}%`, background: 'var(--accent-blue)', transition: 'width 0.1s' }} />
              </div>
              <span style={{ fontSize: 12, fontFamily: 'var(--font-mono)', color: 'var(--accent-blue)' }}>
                {Math.round(progress)}% | {currentThrottle.toFixed(0)}%
              </span>
            </>
          )}
        </div>
      </div>

      {/* Results */}
      {summary && (
        <div style={{ background: 'var(--bg-card)', borderRadius: 'var(--radius)', padding: 16, border: '1px solid var(--border)' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 12 }}>
            <div style={{ fontSize: 13, color: 'var(--text-muted)', textTransform: 'uppercase', letterSpacing: '1px' }}>
              Test Results — {summary.count} samples, {summary.duration}s
            </div>
            <div style={{ display: 'flex', gap: 8 }}>
              <span style={{ padding: '2px 10px', borderRadius: 4, fontSize: 11, fontWeight: 700, background: summary.result === 'PASS' ? '#22c55e' : '#ef4444', color: '#fff' }}>
                {summary.result}
              </span>
              <button onClick={() => downloadCsv(samples, 'motor_test')}
                style={{ padding: '2px 10px', borderRadius: 4, fontSize: 11, fontWeight: 600, border: '1px solid var(--border)', background: 'transparent', color: 'var(--text-muted)', cursor: 'pointer' }}>
                Download CSV ({CSV_HEADERS.length} columns)
              </button>
            </div>
          </div>

          {/* Summary grid */}
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(110px, 1fr))', gap: 6, marginBottom: 16, fontSize: 11 }}>
            {([
              ['Max eRPM', summary.maxErpm.toLocaleString()],
              ['Max RPM', Math.round(summary.maxErpm / pp).toLocaleString()],
              ['Peak Duty', `${summary.peakDuty}%`],
              ['Max IBus', `${summary.maxIbus} mA`],
              ['Max |Ia|', `${summary.maxIa} mA`],
              ['Max |Ib|', `${summary.maxIb} mA`],
              ['Max Power', `${summary.maxPower} W`],
              ['Avg Vbus', `${summary.avgVbus} V`],
              ['Min Vbus', `${summary.minVbus} V`],
              ['Max Forced', summary.maxForced.toString(), summary.maxForced > 2],
              ['Max Missed', summary.maxMissed.toString(), summary.maxMissed > 0],
              ['Avg ZC Alt', summary.avgZcAlt.toString(), summary.avgZcAlt > 5],
              ['Desyncs', summary.totalDesyncs.toString(), summary.totalDesyncs > 0],
              ['Faults', summary.totalFaults.toString(), summary.totalFaults > 0],
            ] as [string, string, boolean?][]).map(([label, value, warn]) => (
              <div key={label} style={{ padding: '4px 8px', background: 'var(--bg-secondary)', borderRadius: 'var(--radius-sm)', border: `1px solid ${warn ? 'var(--accent-red)' : 'var(--border-light)'}` }}>
                <div style={{ fontSize: 9, color: 'var(--text-muted)', textTransform: 'uppercase' }}>{label}</div>
                <div style={{ fontSize: 13, fontWeight: 700, fontFamily: 'var(--font-mono)', color: warn ? 'var(--accent-red)' : 'var(--text-primary)' }}>{value}</div>
              </div>
            ))}
          </div>

          {/* Charts */}
          {samples.length > 2 && (
            <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
              {/* Speed & Duty */}
              <ChartPanel title="Speed & Duty" height={160} data={samples} lines={[
                { key: 'eRpm', name: 'eRPM', color: '#3b82f6', yAxisId: 'left' },
                { key: 'dutyPct', name: 'Duty %', color: '#f472b6', yAxisId: 'right' },
                { key: 'throttlePct', name: 'Throttle %', color: '#c084fc', yAxisId: 'right', dash: true },
              ]} />
              {/* Current & Voltage */}
              <ChartPanel title="Current & Voltage" height={140} data={samples} lines={[
                { key: 'ibusMa', name: 'IBus mA', color: '#f97316', yAxisId: 'left' },
                { key: 'iaMa', name: 'Ia mA', color: '#60a5fa', yAxisId: 'left' },
                { key: 'ibMa', name: 'Ib mA', color: '#34d399', yAxisId: 'left' },
                { key: 'vbusV', name: 'Vbus V', color: '#22c55e', yAxisId: 'right' },
              ]} />
              {/* ZC Health */}
              <ChartPanel title="ZC Diagnostics" height={120} data={samples} lines={[
                { key: 'forcedSteps', name: 'Forced', color: '#f43f5e', yAxisId: 'left' },
                { key: 'missedSteps', name: 'Missed', color: '#ef4444', yAxisId: 'left' },
                { key: 'zcAlternation', name: 'ZC Alt', color: '#eab308', yAxisId: 'left' },
                { key: 'filterLevel', name: 'FL', color: '#a78bfa', yAxisId: 'right' },
              ]} />
              {/* Step Period */}
              <ChartPanel title="Step Period & eRPM" height={120} data={samples} lines={[
                { key: 'stepPeriod', name: 'Tp (T1)', color: '#64748b', yAxisId: 'left' },
                { key: 'stepPeriodHR', name: 'Tp (HR)', color: '#06b6d4', yAxisId: 'right' },
              ]} />
            </div>
          )}
        </div>
      )}
    </div>
  );
}

/* ── Reusable chart panel ─────────────────────────────────────────── */

function ChartPanel({ title, height, data, lines }: {
  title: string; height: number;
  data: FullSample[];
  lines: { key: string; name: string; color: string; yAxisId: string; dash?: boolean }[];
}) {
  return (
    <div style={{ background: 'var(--bg-secondary)', borderRadius: 'var(--radius-sm)', padding: '4px 8px' }}>
      <div style={{ fontSize: 9, color: 'var(--text-muted)', textTransform: 'uppercase', marginBottom: 2 }}>{title}</div>
      <ResponsiveContainer width="100%" height={height}>
        <LineChart data={data}>
          <CartesianGrid strokeDasharray="3 3" stroke="var(--border)" opacity={0.3} />
          <XAxis dataKey="timeMs" tick={{ fontSize: 9, fill: 'var(--text-muted)' }}
            tickFormatter={(v: number) => `${(v / 1000).toFixed(1)}s`} />
          <YAxis yAxisId="left" tick={{ fontSize: 9, fill: 'var(--text-muted)' }} />
          <YAxis yAxisId="right" orientation="right" tick={{ fontSize: 9, fill: 'var(--text-muted)' }} />
          <Tooltip contentStyle={{ background: 'var(--bg-card)', border: '1px solid var(--border)', fontSize: 10, padding: '4px 8px' }}
            labelFormatter={(v: number) => `t = ${(v / 1000).toFixed(2)}s`} />
          <Legend wrapperStyle={{ fontSize: 10 }} />
          {lines.map(l => (
            <Line key={l.key} yAxisId={l.yAxisId} type="monotone" dataKey={l.key}
              name={l.name} stroke={l.color} strokeWidth={1.5} dot={false}
              strokeDasharray={l.dash ? '4 2' : undefined} isAnimationActive={false} />
          ))}
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
