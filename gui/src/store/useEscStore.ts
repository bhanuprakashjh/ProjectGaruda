import { create } from 'zustand';
import type { GspInfo, GspSnapshot, GspRxStatus, CkSnapshot, ParamDescriptor, ScopeSample, ScopeStatus } from '../protocol/types';

/* ── Running-average filter for noisy CK current/voltage readings ── */
const CK_AVG_WINDOW = 20; /* 20 samples @ 50Hz = 400ms window — smooths 6-step phase current swings */
const ckAvgBuf = {
  iaRaw:   [] as number[],
  ibRaw:   [] as number[],
  ibusRaw: [] as number[],
  vbusRaw: [] as number[],
  eRpm:    [] as number[],
};

function avgPush(buf: number[], val: number): number {
  buf.push(val);
  if (buf.length > CK_AVG_WINDOW) buf.shift();
  let sum = 0;
  for (let i = 0; i < buf.length; i++) sum += buf[i];
  return Math.round(sum / buf.length);
}

function smoothCkSnapshot(s: CkSnapshot): CkSnapshot {
  /* Reset all buffers when motor stops — don't carry stale averages */
  if (s.state === 0) {
    ckAvgBuf.iaRaw.length = 0;
    ckAvgBuf.ibRaw.length = 0;
    ckAvgBuf.ibusRaw.length = 0;
    ckAvgBuf.vbusRaw.length = 0;
    ckAvgBuf.eRpm.length = 0;
    return s;
  }
  return {
    ...s,
    iaRaw:   avgPush(ckAvgBuf.iaRaw, s.iaRaw),
    ibRaw:   avgPush(ckAvgBuf.ibRaw, s.ibRaw),
    ibusRaw: avgPush(ckAvgBuf.ibusRaw, s.ibusRaw),
    vbusRaw: avgPush(ckAvgBuf.vbusRaw, s.vbusRaw),
    eRpm:    avgPush(ckAvgBuf.eRpm, s.eRpm),
  };
}

export type TabId = 'dashboard' | 'scope' | 'test' | 'motor' | 'params' | 'help';

interface ParamValue {
  descriptor: ParamDescriptor;
  value: number;
}

interface Toast {
  message: string;
  type: 'success' | 'error' | 'info';
  id: number;
}

interface EscStore {
  connected: boolean;
  info: GspInfo | null;
  snapshot: GspSnapshot | null;
  ckSnapshot: CkSnapshot | null;
  lastSnapshotMs: number;
  history: GspSnapshot[];
  ckHistory: CkSnapshot[];
  params: Map<number, ParamValue>;
  activeProfile: number;
  rxStatus: GspRxStatus | null;
  throttleSource: 'ADC' | 'GSP';
  telemActive: boolean;
  toasts: Toast[];
  paramModalOpen: boolean;
  activeTab: TabId;
  scopeStatus: ScopeStatus | null;
  scopeSamples: ScopeSample[];
  scopeReading: boolean;

  setConnected: (v: boolean) => void;
  setInfo: (info: GspInfo) => void;
  pushSnapshot: (s: GspSnapshot) => void;
  pushCkSnapshot: (s: CkSnapshot) => void;
  setParams: (descriptors: ParamDescriptor[]) => void;
  mergeParams: (descriptors: ParamDescriptor[]) => void;
  setParamValue: (id: number, value: number) => void;
  setActiveProfile: (p: number) => void;
  setRxStatus: (rx: GspRxStatus) => void;
  setThrottleSource: (src: 'ADC' | 'GSP') => void;
  setTelemActive: (v: boolean) => void;
  setParamModalOpen: (v: boolean) => void;
  setActiveTab: (tab: TabId) => void;
  addToast: (message: string, type: Toast['type']) => void;
  removeToast: (id: number) => void;
  setScopeStatus: (st: ScopeStatus) => void;
  appendScopeSamples: (samples: ScopeSample[]) => void;
  clearScopeSamples: () => void;
  setScopeReading: (v: boolean) => void;
  reset: () => void;
}

const MAX_HISTORY = 500; // 10s at 50Hz

let toastId = 0;

export const useEscStore = create<EscStore>((set) => ({
  connected: false,
  info: null,
  snapshot: null,
  ckSnapshot: null,
  lastSnapshotMs: 0,
  history: [],
  ckHistory: [],
  params: new Map(),
  activeProfile: 0,
  rxStatus: null,
  throttleSource: 'ADC',
  telemActive: false,
  toasts: [],
  paramModalOpen: false,
  activeTab: 'dashboard',
  scopeStatus: null,
  scopeSamples: [],
  scopeReading: false,

  setConnected: (v) => set({ connected: v }),
  setInfo: (info) => set({ info, activeProfile: info.motorProfile }),
  pushSnapshot: (s) => set((state) => {
    const history = [...state.history, s];
    if (history.length > MAX_HISTORY) history.shift();
    return { snapshot: s, lastSnapshotMs: Date.now(), history };
  }),
  pushCkSnapshot: (raw) => set((state) => {
    const s = smoothCkSnapshot(raw);
    const ckHistory = [...state.ckHistory, s];
    if (ckHistory.length > MAX_HISTORY) ckHistory.shift();
    return { ckSnapshot: s, lastSnapshotMs: Date.now(), ckHistory };
  }),
  setParams: (descriptors) => set(() => {
    const params = new Map<number, ParamValue>();
    for (const d of descriptors) params.set(d.id, { descriptor: d, value: 0 });
    return { params };
  }),
  mergeParams: (descriptors) => set((state) => {
    const params = new Map(state.params);
    for (const d of descriptors) {
      const existing = params.get(d.id);
      params.set(d.id, { descriptor: d, value: existing?.value ?? 0 });
    }
    return { params };
  }),
  setParamValue: (id, value) => set((state) => {
    const params = new Map(state.params);
    const existing = params.get(id);
    if (existing) params.set(id, { ...existing, value });
    return { params };
  }),
  setActiveProfile: (p) => set({ activeProfile: p }),
  setRxStatus: (rx) => set({ rxStatus: rx }),
  setThrottleSource: (src) => set({ throttleSource: src }),
  setTelemActive: (v) => set({ telemActive: v }),
  setParamModalOpen: (v) => set({ paramModalOpen: v }),
  setActiveTab: (tab) => set({ activeTab: tab }),
  addToast: (message, type) => {
    const id = ++toastId;
    set((state) => ({ toasts: [...state.toasts, { message, type, id }] }));
    setTimeout(() => {
      set((state) => ({ toasts: state.toasts.filter(t => t.id !== id) }));
    }, 3000);
  },
  removeToast: (id) => set((state) => ({ toasts: state.toasts.filter(t => t.id !== id) })),
  setScopeStatus: (st) => set({ scopeStatus: st }),
  appendScopeSamples: (samples) => set((state) => ({ scopeSamples: [...state.scopeSamples, ...samples] })),
  clearScopeSamples: () => set({ scopeSamples: [], scopeReading: false }),
  setScopeReading: (v) => set({ scopeReading: v }),
  reset: () => {
    /* Clear running-average buffers */
    ckAvgBuf.iaRaw.length = 0;
    ckAvgBuf.ibRaw.length = 0;
    ckAvgBuf.ibusRaw.length = 0;
    ckAvgBuf.vbusRaw.length = 0;
    ckAvgBuf.eRpm.length = 0;
    return set({
    connected: false, info: null, snapshot: null, ckSnapshot: null, lastSnapshotMs: 0,
    history: [], ckHistory: [], params: new Map(), activeProfile: 0, rxStatus: null,
    throttleSource: 'ADC', telemActive: false, toasts: [],
    paramModalOpen: false, activeTab: 'dashboard',
    scopeStatus: null, scopeSamples: [], scopeReading: false,
  });},
}));
