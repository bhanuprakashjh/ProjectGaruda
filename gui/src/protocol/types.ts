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

export interface ParamListPage {
  totalCount: number;
  startIndex: number;
  entries: ParamDescriptor[];
}

export const ESC_STATES = ['IDLE', 'ARMED', 'ALIGN', 'OL_RAMP', 'MORPH', 'CLOSED_LOOP', 'BRAKING', 'RECOVERY', 'FAULT'] as const;
export const FAULT_CODES = ['NONE', 'OVERVOLTAGE', 'UNDERVOLTAGE', 'OVERCURRENT', 'BOARD_PCI', 'STALL', 'DESYNC', 'STARTUP_TIMEOUT', 'MORPH_TIMEOUT'] as const;

export const PROFILE_NAMES = ['Hurst DMB0224C10002', 'A2212 1400KV', '5010 750KV', 'Custom'] as const;

export const PARAM_NAMES: Record<number, string> = {
  // Startup & Ramp (group 0)
  0x15: 'Handoff Speed [rampTargetErpm]',
  0x16: 'Ramp Acceleration [rampAccelErpmPerS]',
  0x17: 'Ramp Max Duty [rampDutyPct]',
  0x51: 'Alignment Duty [alignDutyPct]',
  0x52: 'Initial Speed [initialErpm]',
  0x54: 'Sine Align Modulation [sineAlignModPct]',
  0x55: 'Sine Ramp Modulation [sineRampModPct]',
  // Closed-Loop Control (group 1)
  0x20: 'Idle Duty [clIdleDutyPct]',
  0x22: 'Timing Advance Limit [timingAdvMaxDeg]',
  0x30: 'HW Zero-Cross Enable Speed [hwzcCrossoverErpm]',
  0x53: 'Max Closed-Loop Speed [maxClosedLoopErpm]',
  0x56: 'Demag Duty Threshold [zcDemagDutyThresh]',
  0x57: 'Demag Extra Blanking [zcDemagBlankExtraPct]',
  // Current Protection (group 2)
  0x42: 'Current Soft Limit [ocSwLimitMa]',
  0x41: 'Current Hard Fault [ocFaultMa]',
  0x58: 'CMP3 Current Limit [ocLimitMa]',
  0x59: 'Startup Current Limit [ocStartupMa]',
  0x5A: 'Ramp Current Gate [rampCurrentGateMa]',
  // ZC Detection (group 3)
  0x64: 'ZC Blanking [zcBlankingPercent]',
  0x65: 'ZC ADC Deadband [zcAdcDeadband]',
  0x66: 'ZC Sync Threshold [zcSyncThreshold]',
  0x67: 'ZC Filter Threshold [zcFilterThreshold]',
  // Duty Slew (group 4)
  0x60: 'Duty Slew Up Rate [dutySlewUpPctPerMs]',
  0x61: 'Duty Slew Down Rate [dutySlewDownPctPerMs]',
  0x62: 'Post-Sync Settle Time [postSyncSettleMs]',
  0x63: 'Post-Sync Slew Divisor [postSyncSlewDivisor]',
  // Voltage Protection (group 5)
  0x68: 'Overvoltage Threshold [vbusOvAdc]',
  0x69: 'Undervoltage Threshold [vbusUvAdc]',
  // Recovery (group 6)
  0x6A: 'Desync Coast Time [desyncCoastMs]',
  0x6B: 'Max Restart Attempts [desyncMaxRestarts]',
  // Motor Hardware (group 7)
  0x50: 'Motor Pole Pairs [motorPolePairs]',
};

export const PARAM_UNITS: Record<number, string> = {
  0x15: 'eRPM', 0x16: 'eRPM/s', 0x17: '%',
  0x51: '%', 0x52: 'eRPM', 0x54: '%', 0x55: '%',
  0x20: '%', 0x22: '\u00b0', 0x30: 'eRPM',
  0x53: 'eRPM', 0x56: '%', 0x57: '%',
  0x42: 'mA', 0x41: 'mA', 0x58: 'mA', 0x59: 'mA', 0x5A: 'mA',
  0x64: '%', 0x65: 'counts', 0x66: 'count', 0x67: 'count',
  0x60: '%/ms', 0x61: '%/ms', 0x62: 'ms', 0x63: '\u00f7',
  0x68: 'ADC', 0x69: 'ADC',
  0x6A: 'ms', 0x6B: 'count',
  0x50: 'pairs',
};

export const PARAM_TOOLTIPS: Record<number, string> = {
  0x15: 'Speed at which open-loop ramp hands off to closed-loop BEMF tracking. Higher = more reliable lock, but slower startup.',
  0x16: 'How fast the motor accelerates during open-loop ramp. Lower = gentler start for heavy props.',
  0x17: 'Maximum duty cycle allowed during open-loop ramp. Limits startup current.',
  0x51: 'Duty cycle during rotor alignment phase. Higher = stronger alignment but more current.',
  0x52: 'Initial forced commutation speed at start of ramp. Lower = gentler start.',
  0x54: 'Sine drive modulation during alignment. Controls alignment current.',
  0x55: 'Sine drive modulation during V/f ramp. Controls ramp torque.',
  0x20: 'Minimum duty when motor is running at zero throttle. Keeps motor spinning at idle.',
  0x22: 'Maximum commutation timing advance at top speed. Improves efficiency but too much causes instability.',
  0x30: 'Speed above which hardware ADC comparators take over zero-crossing detection from software polling.',
  0x53: 'Maximum electrical RPM for closed-loop operation. Affects timing advance interpolation and step period limits.',
  0x56: 'Duty % above which extra demagnetization blanking is applied to ZC detection.',
  0x57: 'Extra blanking percentage added at 100% duty to reject demag noise.',
  0x42: 'Soft overcurrent threshold. Duty is reduced proportionally when bus current exceeds this.',
  0x41: 'Hard overcurrent threshold. Motor faults immediately if bus current exceeds this.',
  0x58: 'CMP3 hardware comparator threshold for cycle-by-cycle current chopping (CLPCI).',
  0x59: 'CMP3 threshold during startup/alignment. Set high to avoid false trips during switching transients.',
  0x5A: 'Hold ramp acceleration when bus current exceeds this. 0 = disabled.',
  0x64: 'Percentage of step period to blank after commutation before accepting ZC events.',
  0x65: 'ADC count deadband around ZC threshold. Reduces noise-triggered false crossings.',
  0x66: 'Consecutive confirmed zero-crossings needed to declare sync lock. Min 4 (morph exit requirement).',
  0x67: 'Consecutive matching ADC reads needed to confirm a single zero-crossing event.',
  0x60: 'Maximum duty increase rate. Limits acceleration torque impulse.',
  0x61: 'Maximum duty decrease rate. Limits regenerative braking current.',
  0x62: 'Time after ZC sync lock during which slew rate is reduced for smooth transition.',
  0x63: 'Slew-up rate divisor during post-sync settle. Higher = gentler closed-loop entry.',
  0x68: 'Bus voltage ADC reading above which an overvoltage fault triggers.',
  0x69: 'Bus voltage ADC reading below which an undervoltage fault triggers.',
  0x6A: 'Coast-down time after desync before attempting restart.',
  0x6B: 'Maximum restart attempts after desync before permanent fault. 0 = no restarts.',
  0x50: 'Number of magnetic pole pairs. Informational: used for eRPM to mechanical RPM conversion in GUI.',
};

export const PARAM_GROUPS: Record<number, string> = {
  0: 'Startup & Ramp',
  1: 'Closed-Loop Control',
  2: 'Current Protection',
  3: 'ZC Detection',
  4: 'Duty Slew',
  5: 'Voltage Protection',
  6: 'Recovery',
  7: 'Motor Hardware',
};

export const FEATURE_NAMES: Record<number, string> = {
  0: 'BEMF_CLOSED_LOOP', 1: 'VBUS_FAULT', 2: 'DESYNC_RECOVERY',
  3: 'DUTY_SLEW', 4: 'TIMING_ADVANCE', 5: 'DYNAMIC_BLANKING',
  6: 'VBUS_SAG_LIMIT', 7: 'BEMF_INTEGRATION', 8: 'SINE_STARTUP',
  9: 'ADC_CMP_ZC', 10: 'HW_OVERCURRENT', 11: 'LEARN_MODULES',
  12: 'ADAPTATION', 13: 'COMMISSION', 14: 'EEPROM_V2',
  15: 'X2CSCOPE', 16: 'GSP', 17: 'OC_CLPCI_ENABLE', 18: 'PRESYNC_RAMP',
};
