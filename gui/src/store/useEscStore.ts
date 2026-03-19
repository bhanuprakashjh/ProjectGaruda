import { create } from 'zustand';
import type { GspInfo, GspSnapshot, GspRxStatus, CkSnapshot, ParamDescriptor, ScopeSample, ScopeStatus } from '../protocol/types';

export type TabId = 'dashboard' | 'scope' | 'motor' | 'params' | 'help';

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
  pushCkSnapshot: (s) => set((state) => {
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
  reset: () => set({
    connected: false, info: null, snapshot: null, ckSnapshot: null, lastSnapshotMs: 0,
    history: [], ckHistory: [], params: new Map(), activeProfile: 0, rxStatus: null,
    throttleSource: 'ADC', telemActive: false, toasts: [],
    paramModalOpen: false, activeTab: 'dashboard',
    scopeStatus: null, scopeSamples: [], scopeReading: false,
  }),
}));
