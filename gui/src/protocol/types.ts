export interface GspInfo {
  protocolVersion: number;
  fwMajor: number;
  fwMinor: number;
  fwPatch: number;
  boardId: number;
  motorProfile: number;
  motorPolePairs: number;
  featureFlags: number;
  pwmFrequency: number;
  maxErpm: number;
}

export interface GspSnapshot {
  state: number;
  faultCode: number;
  currentStep: number;
  direction: number;
  throttle: number;
  dutyPct: number;
  vbusRaw: number;
  ibusRaw: number;
  ibusMax: number;
  bemfRaw: number;
  zcThreshold: number;
  stepPeriod: number;
  goodZcCount: number;
  risingZcWorks: boolean;
  fallingZcWorks: boolean;
  zcSynced: boolean;
  zcConfirmedCount: number;
  zcTimeoutForceCount: number;
  hwzcEnabled: boolean;
  hwzcPhase: number;
  hwzcTotalZcCount: number;
  hwzcTotalMissCount: number;
  hwzcStepPeriodHR: number;
  hwzcDbgLatchDisable: boolean;
  morphSubPhase: number;
  morphStep: number;
  morphZcCount: number;
  morphAlpha: number;
  clpciTripCount: number;
  fpciTripCount: number;
  systemTick: number;
  uptimeSec: number;
}

export interface ParamDescriptor {
  id: number;
  type: number;
  group: number;
  min: number;
  max: number;
}

export const ESC_STATES = ['IDLE', 'ARMED', 'ALIGN', 'OL_RAMP', 'MORPH', 'CLOSED_LOOP', 'BRAKING', 'RECOVERY', 'FAULT'] as const;
export const FAULT_CODES = ['NONE', 'OVERVOLTAGE', 'UNDERVOLTAGE', 'OVERCURRENT', 'BOARD_PCI', 'STALL', 'DESYNC', 'STARTUP_TIMEOUT', 'MORPH_TIMEOUT'] as const;

export const PARAM_NAMES: Record<number, string> = {
  0x15: 'Ramp Target (eRPM)',
  0x16: 'Ramp Accel (eRPM/s)',
  0x17: 'Ramp Duty (%)',
  0x20: 'CL Idle Duty (%)',
  0x22: 'Timing Advance Max (\u00b0)',
  0x30: 'HWZC Crossover (eRPM)',
  0x42: 'OC Soft Limit (mA)',
  0x41: 'OC Fault (mA)',
};

export const PARAM_GROUPS: Record<number, string> = {
  0: 'Startup',
  1: 'Closed-Loop',
  2: 'Overcurrent',
};
