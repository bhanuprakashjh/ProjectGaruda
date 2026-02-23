import { create } from 'zustand';
import type { GspInfo, GspSnapshot, ParamDescriptor } from '../protocol/types';

interface ParamValue {
  descriptor: ParamDescriptor;
  value: number;
}

interface EscStore {
  connected: boolean;
  info: GspInfo | null;
  snapshot: GspSnapshot | null;
  history: GspSnapshot[];
  params: Map<number, ParamValue>;
  throttleSource: 'ADC' | 'GSP';
  telemActive: boolean;

  setConnected: (v: boolean) => void;
  setInfo: (info: GspInfo) => void;
  pushSnapshot: (s: GspSnapshot) => void;
  setParams: (descriptors: ParamDescriptor[]) => void;
  setParamValue: (id: number, value: number) => void;
  setThrottleSource: (src: 'ADC' | 'GSP') => void;
  setTelemActive: (v: boolean) => void;
  reset: () => void;
}

const MAX_HISTORY = 300; // 30s at 10Hz

export const useEscStore = create<EscStore>((set) => ({
  connected: false,
  info: null,
  snapshot: null,
  history: [],
  params: new Map(),
  throttleSource: 'ADC',
  telemActive: false,

  setConnected: (v) => set({ connected: v }),
  setInfo: (info) => set({ info }),
  pushSnapshot: (s) => set((state) => {
    const history = [...state.history, s];
    if (history.length > MAX_HISTORY) history.shift();
    return { snapshot: s, history };
  }),
  setParams: (descriptors) => set(() => {
    const params = new Map<number, ParamValue>();
    for (const d of descriptors) params.set(d.id, { descriptor: d, value: 0 });
    return { params };
  }),
  setParamValue: (id, value) => set((state) => {
    const params = new Map(state.params);
    const existing = params.get(id);
    if (existing) params.set(id, { ...existing, value });
    return { params };
  }),
  setThrottleSource: (src) => set({ throttleSource: src }),
  setTelemActive: (v) => set({ telemActive: v }),
  reset: () => set({
    connected: false, info: null, snapshot: null,
    history: [], params: new Map(), throttleSource: 'ADC', telemActive: false,
  }),
}));
