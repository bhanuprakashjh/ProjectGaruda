import { create } from 'zustand';
import type { GspInfo, GspSnapshot, ParamDescriptor } from '../protocol/types';

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
  lastSnapshotMs: number;
  history: GspSnapshot[];
  params: Map<number, ParamValue>;
  activeProfile: number;
  throttleSource: 'ADC' | 'GSP';
  telemActive: boolean;
  toasts: Toast[];
  paramModalOpen: boolean;

  setConnected: (v: boolean) => void;
  setInfo: (info: GspInfo) => void;
  pushSnapshot: (s: GspSnapshot) => void;
  setParams: (descriptors: ParamDescriptor[]) => void;
  mergeParams: (descriptors: ParamDescriptor[]) => void;
  setParamValue: (id: number, value: number) => void;
  setActiveProfile: (p: number) => void;
  setThrottleSource: (src: 'ADC' | 'GSP') => void;
  setTelemActive: (v: boolean) => void;
  setParamModalOpen: (v: boolean) => void;
  addToast: (message: string, type: Toast['type']) => void;
  removeToast: (id: number) => void;
  reset: () => void;
}

const MAX_HISTORY = 500; // 10s at 50Hz

let toastId = 0;

export const useEscStore = create<EscStore>((set) => ({
  connected: false,
  info: null,
  snapshot: null,
  lastSnapshotMs: 0,
  history: [],
  params: new Map(),
  activeProfile: 0,
  throttleSource: 'ADC',
  telemActive: false,
  toasts: [],
  paramModalOpen: false,

  setConnected: (v) => set({ connected: v }),
  setInfo: (info) => set({ info, activeProfile: info.motorProfile }),
  pushSnapshot: (s) => set((state) => {
    const history = [...state.history, s];
    if (history.length > MAX_HISTORY) history.shift();
    return { snapshot: s, lastSnapshotMs: Date.now(), history };
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
  setThrottleSource: (src) => set({ throttleSource: src }),
  setTelemActive: (v) => set({ telemActive: v }),
  setParamModalOpen: (v) => set({ paramModalOpen: v }),
  addToast: (message, type) => {
    const id = ++toastId;
    set((state) => ({ toasts: [...state.toasts, { message, type, id }] }));
    setTimeout(() => {
      set((state) => ({ toasts: state.toasts.filter(t => t.id !== id) }));
    }, 3000);
  },
  removeToast: (id) => set((state) => ({ toasts: state.toasts.filter(t => t.id !== id) })),
  reset: () => set({
    connected: false, info: null, snapshot: null, lastSnapshotMs: 0,
    history: [], params: new Map(), activeProfile: 0,
    throttleSource: 'ADC', telemActive: false, toasts: [],
    paramModalOpen: false,
  }),
}));
