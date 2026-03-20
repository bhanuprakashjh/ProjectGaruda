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
  // FOC telemetry (zero when FOC not enabled)
  focIdMeas: number;
  focIqMeas: number;
  focTheta: number;
  focOmega: number;
  focVbus: number;
  focIa: number;
  focIb: number;
  focThetaObs: number;
  focVd: number;
  focVq: number;
  // Observer internals
  focFluxAlpha: number;
  focFluxBeta: number;
  focLambdaEst: number;
  focObsGain: number;
  // PI controller internals
  focPidDInteg: number;
  focPidQInteg: number;
  focPidSpdInteg: number;
  // Derived diagnostics
  focModIndex: number;
  focObsConfidence: number;
  focSubState: number;
  focOffsetIa: number;
  focOffsetIb: number;
}

/* ── Board IDs ─────────────────────────────────────────────── */
export const BOARD_ID_AK = 0x0001;  /* MCLV-48V-300W (dsPIC33AK) */
export const BOARD_ID_CK = 0x0002;  /* EV43F54A (dsPIC33CK + ATA6847) */

export const BOARD_NAMES: Record<number, string> = {
  [BOARD_ID_AK]: 'MCLV-48V-300W (AK)',
  [BOARD_ID_CK]: 'EV43F54A (CK)',
};

export function isCkBoard(boardId: number): boolean {
  return boardId === BOARD_ID_CK;
}

/* ── CK Board Snapshot (48 bytes) ────────────────────────────── */
export interface CkSnapshot {
  state: number;
  faultCode: number;
  currentStep: number;
  ataStatus: number;
  potRaw: number;
  dutyPct: number;
  zcSynced: boolean;
  vbusRaw: number;
  iaRaw: number;
  ibRaw: number;
  ibusRaw: number;
  duty: number;
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
  ilimActive: boolean;
  systemTick: number;
  uptimeSec: number;
}

export const CK_ESC_STATES = ['IDLE', 'ARMED', 'ALIGN', 'OL_RAMP', 'CLOSED_LOOP', 'RECOVERY', 'FAULT'] as const;
export const CK_FAULT_CODES = ['NONE', 'OVERVOLTAGE', 'UNDERVOLTAGE', 'STALL', 'DESYNC', 'STARTUP_TIMEOUT', 'ATA6847'] as const;

export const CK_PROFILE_NAMES = ['Hurst Long', 'A2212 1400KV', '2810 1350KV', 'Custom'] as const;
export const CK_PROFILE_COUNT = 3;

/* ── CK Board Parameter Groups ──────────────────────────────── */
export const CK_PARAM_GROUPS: Record<number, { name: string; color: string }> = {
  0: { name: 'Motor Identity', color: '#3b82f6' },
  1: { name: 'Startup & Ramp', color: '#f59e0b' },
  2: { name: 'Closed-Loop', color: '#10b981' },
  3: { name: 'ATA6847 Protection', color: '#ef4444' },
  4: { name: 'ATA6847 GDU', color: '#8b5cf6' },
  5: { name: 'ZC Detection', color: '#06b6d4' },
  6: { name: 'Voltage Protection', color: '#f97316' },
  7: { name: 'Recovery', color: '#64748b' },
};

export const CK_PARAM_NAMES: Record<number, { display: string; variable: string }> = {
  // Group 0: Motor Identity
  0xC0: { display: 'Pole Pairs', variable: 'polePairs' },
  0xC1: { display: 'Motor KV', variable: 'motorKv' },
  0xC2: { display: 'Phase Resistance', variable: 'motorRsMilliOhm' },
  0xC3: { display: 'Phase Inductance', variable: 'motorLsMicroH' },
  // Group 1: Startup & Ramp
  0xC4: { display: 'Align Time', variable: 'alignTimeMs' },
  0xC5: { display: 'Align Duty', variable: 'alignDutyPctX10' },
  0xC6: { display: 'Initial Step Period', variable: 'initialStepPeriod' },
  0xC7: { display: 'Min Step Period', variable: 'minStepPeriod' },
  0xC8: { display: 'Ramp Acceleration', variable: 'rampAccelErpmS' },
  0xC9: { display: 'Ramp Duty Cap', variable: 'rampDutyPctX10' },
  0xCA: { display: 'Ramp Target eRPM', variable: 'rampTargetErpm' },
  // Group 2: Closed-Loop
  0xCB: { display: 'CL Idle Duty', variable: 'clIdleDutyPct' },
  0xCC: { display: 'Timing Adv Min', variable: 'timingAdvMinDeg' },
  0xCD: { display: 'Timing Adv Max', variable: 'timingAdvMaxDeg' },
  0xCE: { display: 'Timing Adv Start', variable: 'timingAdvStartErpm' },
  0xCF: { display: 'Max CL eRPM', variable: 'maxClosedLoopErpm' },
  0xD0: { display: 'Duty Slew Up', variable: 'dutySlewUp' },
  0xD1: { display: 'Duty Slew Down', variable: 'dutySlewDown' },
  // Group 3: ATA6847 Protection
  0xD2: { display: 'ILIM Enable', variable: 'ilimEnable' },
  0xD3: { display: 'ILIM Shutdown', variable: 'ilimShutdownEnable' },
  0xD4: { display: 'ILIM DAC', variable: 'ilimDac' },
  0xD5: { display: 'ILIM Filter Time', variable: 'ilimFilterTime' },
  0xD6: { display: 'SC Enable', variable: 'scEnable' },
  0xD7: { display: 'SC Threshold', variable: 'scThreshold' },
  0xD8: { display: 'SC Filter Time', variable: 'scFilterTime' },
  // Group 4: ATA6847 GDU
  0xD9: { display: 'BEMF Enable', variable: 'bemfEnable' },
  0xDA: { display: 'Cross-Cond Time', variable: 'crossConductionTime' },
  0xDB: { display: 'Edge Blanking', variable: 'edgeBlankingTime' },
  0xDC: { display: 'CSA Gain', variable: 'csaGain' },
  // Group 5: ZC Detection
  0xDD: { display: 'ZC Deglitch Min', variable: 'zcDeglitchMin' },
  0xDE: { display: 'ZC Deglitch Max', variable: 'zcDeglitchMax' },
  0xDF: { display: 'ZC Timeout Mult', variable: 'zcTimeoutMult' },
  0xE0: { display: 'ZC Desync Thresh', variable: 'zcDesyncThresh' },
  0xE1: { display: 'ZC Miss Limit', variable: 'zcMissLimit' },
  // Group 6: Voltage Protection
  0xE2: { display: 'Vbus OV Threshold', variable: 'vbusOvThreshold' },
  0xE3: { display: 'Vbus UV Threshold', variable: 'vbusUvThreshold' },
  // Group 7: Recovery
  0xE4: { display: 'Max Restart Attempts', variable: 'desyncRestartMax' },
  0xE5: { display: 'Recovery Time', variable: 'recoveryTimeMs' },
};

export const CK_PARAM_UNITS: Record<number, string> = {
  0xC0: 'pairs', 0xC1: 'RPM/V', 0xC2: 'm\u03A9', 0xC3: '\u00B5H',
  0xC4: 'ms', 0xC5: '\u00D70.1%', 0xC6: 'ticks', 0xC7: 'ticks',
  0xC8: 'eRPM/s', 0xC9: '\u00D70.1%', 0xCA: 'eRPM',
  0xCB: '%', 0xCC: '\u00B0', 0xCD: '\u00B0', 0xCE: 'eRPM',
  0xCF: 'eRPM', 0xD0: 'cnt/tick', 0xD1: 'cnt/tick',
  0xD2: '', 0xD3: '', 0xD4: '0-127', 0xD5: '0-7',
  0xD6: '', 0xD7: '0-7', 0xD8: '0-7',
  0xD9: '', 0xDA: '0-7', 0xDB: '0-3', 0xDC: '8/16/32/64\u00D7',
  0xDD: 'reads', 0xDE: 'reads', 0xDF: '\u00D7', 0xE0: 'steps', 0xE1: 'steps',
  0xE2: 'ADC', 0xE3: 'ADC',
  0xE4: 'count', 0xE5: 'ms',
};

export const CK_PARAM_TOOLTIPS: Record<number, string> = {
  0xC0: 'Number of magnetic pole pairs.',
  0xC1: 'Motor velocity constant (RPM per volt).',
  0xC2: 'Phase resistance in milliohms.',
  0xC3: 'Phase inductance in microhenries.',
  0xC4: 'Rotor alignment dwell time.',
  0xC5: 'Alignment duty in 0.1% units (25 = 2.5%). Higher = more alignment current.',
  0xC6: 'Initial forced commutation period (Timer1 ticks). Lower = faster initial speed.',
  0xC7: 'Minimum step period for OL ramp termination. Determines OL\u2192CL handoff speed.',
  0xC8: 'Open-loop ramp acceleration in eRPM/s.',
  0xC9: 'Maximum duty during OL ramp in 0.1% units (170 = 17%). Limits startup current.',
  0xCA: 'Target eRPM for OL\u2192CL handoff (informational).',
  0xCB: 'Minimum duty when motor is running at zero throttle.',
  0xCC: 'Timing advance at low speed (degrees).',
  0xCD: 'Timing advance at max speed (degrees).',
  0xCE: 'eRPM where timing advance ramp begins.',
  0xCF: 'Timing advance curve endpoint. Also affects step period floor.',
  0xD0: 'Duty increase rate (PWM counts per Timer1 tick). Higher = faster throttle response.',
  0xD1: 'Duty decrease rate (PWM counts per Timer1 tick). Lower = gentler deceleration.',
  0xD2: 'Enable ATA6847 hardware current limit (cycle-by-cycle chopping).',
  0xD3: 'ILIM shutdown mode: 0=chop (motor keeps running), 1=shutdown (motor stops).',
  0xD4: 'ILIM DAC threshold (0-127). Trip current = (3.3\u00D7DAC/128 - 1.65) / (Gain\u00D7Rshunt).',
  0xD5: 'ILIM filter time (0-7). Higher = more noise rejection, slower response.',
  0xD6: 'Enable VDS short-circuit protection.',
  0xD7: 'SC voltage threshold (0-7). 7=2000mV max. Lower = more sensitive.',
  0xD8: 'SC filter time (0-7). Higher = more noise rejection.',
  0xD9: 'Enable BEMF comparators for ZC detection.',
  0xDA: 'Cross-conduction prevention time (0-7). Prevents shoot-through.',
  0xDB: 'Edge blanking time (0-3). Blanks gate driver after switching.',
  0xDC: 'Current sense amplifier gain: 0=8\u00D7, 1=16\u00D7, 2=32\u00D7, 3=64\u00D7.',
  0xDD: 'Minimum deglitch filter at high speed. Consecutive reads to confirm ZC.',
  0xDE: 'Maximum deglitch filter at low speed.',
  0xDF: 'Forced commutation timeout multiplier (e.g., 2 = timeout at 2\u00D7 step period).',
  0xE0: 'Consecutive missed ZCs to clear sync lock.',
  0xE1: 'Consecutive missed ZCs to declare desync fault.',
  0xE2: 'Overvoltage fault threshold (raw ADC, 16-bit scaled). ~1211 counts/V.',
  0xE3: 'Undervoltage fault threshold (raw ADC, 16-bit scaled). ~1211 counts/V.',
  0xE4: 'Maximum restart attempts after desync before permanent fault.',
  0xE5: 'Recovery dwell time between desync and restart.',
};

/* Current scaling: raw signed ADC (fractional 12-bit) to milliamps.
 * Phase (OA2/OA3 Gt=16, 3mΩ): mA = raw × 1.049
 * IBus: reconstructed per step, same scaling */
export const CK_CURRENT_SCALE = 1.049;

/* Vbus scaling: raw ADC to volts. EV43F54A divider ~1211 raw/V */
export const CK_VBUS_SCALE = 1211;

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

export const ESC_STATES = ['IDLE', 'ARMED', 'DETECT', 'ALIGN', 'OL_RAMP', 'MORPH', 'CLOSED_LOOP', 'BRAKING', 'RECOVERY', 'FAULT'] as const;
export const FAULT_CODES = ['NONE', 'OVERVOLTAGE', 'UNDERVOLTAGE', 'OVERCURRENT', 'BOARD_PCI', 'STALL', 'DESYNC', 'STARTUP_TIMEOUT', 'MORPH_TIMEOUT', 'RX_LOSS', 'FOC_INTERNAL', 'FOC_BUSLOSS', 'TRAP_BUS', 'TRAP_ILLEGAL', 'TRAP_ADDRESS', 'TRAP_STACK', 'TRAP_MATH', 'TRAP_GENERAL', 'TRAP_DEFAULT'] as const;
export const FOC_SUB_STATES = ['IDLE', 'ARMED', 'ALIGN', 'I/F RAMP', 'CLOSED_LOOP'] as const;
export const DETECT_PHASE_NAMES = ['Idle', 'Measuring Rs', 'Measuring Ls', 'Re-Aligning', 'Measuring Lambda', 'Auto-Tuning', 'Complete', 'Failed'] as const;

export const PROFILE_NAMES = ['Hurst Long (300W)', 'A2212 1400KV', '5010 750KV', '5055 580KV', 'Custom'] as const;
export const PROFILE_COUNT = 4; /* built-in profiles (excl. Custom) */

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
  // FOC Motor Model (group 8)
  0x70: 'Phase Resistance [focRsMilliOhm]',
  0x71: 'Phase Inductance [focLsMicroH]',
  0x72: 'Back-EMF Constant [focKeUvSRad]',
  0x73: 'Nominal Bus Voltage [focVbusNomCentiV]',
  0x74: 'Max Phase Current [focMaxCurrentCentiA]',
  0x75: 'Max Electrical Speed [focMaxElecRadS]',
  // FOC Tuning (group 9)
  0x76: 'Current Loop Kp [focKpDqMilli]',
  0x77: 'Current Loop Ki [focKiDq]',
  0x78: 'Observer LPF Alpha [focObsLpfAlphaMilli]',
  // FOC Startup (group 10)
  0x79: 'Align Current [focAlignIqCentiA]',
  0x7A: 'Ramp Current [focRampIqCentiA]',
  0x7B: 'Alignment Time [focAlignTimeMs]',
  0x7C: 'Iq Ramp Time [focIqRampTimeMs]',
  0x7D: 'Speed Ramp Rate [focRampRateRps2]',
  0x7E: 'CL Handoff Speed [focHandoffRadS]',
  0x7F: 'OC Fault Threshold [focFaultOcCentiA]',
  0x80: 'Stall Speed Threshold [focFaultStallDeciRadS]',
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
  // FOC Motor Model
  0x70: 'm\u03A9', 0x71: '\u00B5H', 0x72: '\u00B5V\u00B7s/rad',
  0x73: 'cV', 0x74: 'cA', 0x75: 'rad/s',
  // FOC Tuning
  0x76: '\u00D71000', 0x77: '1/s', 0x78: '\u00D71000',
  // FOC Startup
  0x79: 'cA', 0x7A: 'cA', 0x7B: 'ms', 0x7C: 'ms',
  0x7D: 'rad/s\u00B2', 0x7E: 'rad/s', 0x7F: 'cA', 0x80: 'drad/s',
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
  // FOC Motor Model
  0x70: 'Phase resistance in milliohms. Critical for FOC current control — wrong Rs causes observer drift and PI mismatch.',
  0x71: 'Phase inductance in microhenries. Affects current loop bandwidth and observer L\u00B7dI cancellation.',
  0x72: 'Back-EMF constant in \u00B5V\u00B7s/rad (electrical). Ke = \u03BB_pm. Formula: 60 / (\u221A3 \u00D7 2\u03C0 \u00D7 KV \u00D7 pole_pairs).',
  0x73: 'Nominal bus voltage in centivolts. Used for voltage clamp, dead-time comp, and speed limiting.',
  0x74: 'Maximum phase current in centiamps. Limits speed PI output and defines OC fault threshold.',
  0x75: 'Maximum electrical speed in rad/s. Caps speed reference and observer LPF tuning.',
  // FOC Tuning
  0x76: 'D/Q current loop proportional gain \u00D71000. Kp = \u03C9_bw \u00D7 Ls. Too high = current ringing.',
  0x77: 'D/Q current loop integral gain. Ki = \u03C9_bw \u00D7 Rs (parallel form). Too high = overshoot.',
  0x78: 'Observer low-pass filter coefficient \u00D71000. Higher = less filtering, better for high-speed motors.',
  // FOC Startup
  0x79: 'Alignment current in centiamps. Must overcome cogging torque. Too high = excess heating.',
  0x7A: 'Open-loop ramp current in centiamps. Must overcome prop drag during I/f acceleration.',
  0x7B: 'Alignment dwell time in ms. Heavy rotors need more time to settle.',
  0x7C: 'Duration to ramp Iq from alignment to running current. Smoother = less vibration.',
  0x7D: 'I/f speed ramp rate in rad/s\u00B2. Lower = gentler startup for heavy props.',
  0x7E: 'Speed threshold for closed-loop handoff. PLL must track within 20% for 10ms.',
  0x7F: 'Software overcurrent fault threshold in centiamps. Motor faults if |I| exceeds this.',
  0x80: 'Stall detection speed in decirad/s. Motor faults if speed stays below this while commanding motion.',
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
  8: 'FOC Motor Model',
  9: 'FOC Tuning',
  10: 'FOC Startup',
};

export const FEATURE_NAMES: Record<number, string> = {
  0: 'BEMF_CLOSED_LOOP', 1: 'VBUS_FAULT', 2: 'DESYNC_RECOVERY',
  3: 'DUTY_SLEW', 4: 'TIMING_ADVANCE', 5: 'DYNAMIC_BLANKING',
  6: 'VBUS_SAG_LIMIT', 7: 'BEMF_INTEGRATION', 8: 'SINE_STARTUP',
  9: 'ADC_CMP_ZC', 10: 'HW_OVERCURRENT', 11: 'LEARN_MODULES',
  12: 'ADAPTATION', 13: 'COMMISSION', 14: 'EEPROM_V2',
  15: 'X2CSCOPE', 16: 'GSP', 17: 'OC_CLPCI_ENABLE', 18: 'PRESYNC_RAMP',
  19: 'ADC_POT', 20: 'RX_PWM', 21: 'RX_DSHOT', 22: 'RX_AUTO',
  23: 'FOC',
  24: 'BURST_SCOPE',
};

export const FEATURE_BIT_FOC = 23;
export const FEATURE_BIT_BURST_SCOPE = 24;

export interface GspRxStatus {
  linkState: number;
  protocol: number;
  dshotRate: number;
  throttle: number;
  pulseUs: number;
  crcErrors: number;
  droppedFrames: number;
}

/** Helper: check if FOC feature is enabled in feature flags */
export function isFocEnabled(featureFlags: number): boolean {
  return (featureFlags & (1 << FEATURE_BIT_FOC)) !== 0;
}

/** Helper: check if burst scope is enabled */
export function isBurstScopeEnabled(featureFlags: number): boolean {
  return (featureFlags & (1 << FEATURE_BIT_BURST_SCOPE)) !== 0;
}

/* ── Burst Scope Types ─────────────────────────────────────────── */

export const SCOPE_TRIG_MODES = ['Manual', 'Fault', 'State Change', 'Threshold'] as const;
export const SCOPE_STATES = ['IDLE', 'ARMED', 'FILLING', 'READY'] as const;
export const SCOPE_CHANNELS = ['Ia', 'Ib', 'Id', 'Iq', 'Vd', 'Vq', 'Theta', '', '', 'Omega', 'ModIndex'] as const;
export const SCOPE_EDGES = ['Rising', 'Falling'] as const;

export const SCOPE_SAMPLE_SIZE = 26;
export const SCOPE_MAX_CHUNK = 9;
export const SCOPE_BUF_SIZE = 128;

export interface ScopeSample {
  ia: number;     /* A */
  ib: number;
  id: number;
  iq: number;
  vd: number;     /* V */
  vq: number;
  theta: number;  /* rad */
  obsX1: number;  /* V·s */
  obsX2: number;
  omega: number;  /* rad/s */
  modIndex: number; /* 0-1 */
  flags: number;
  state: number;
  tickLsb: number;
  cl: boolean;
  fault: boolean;
  mode: number;
}

export interface ScopeStatus {
  state: number;
  trigMode: number;
  preTrigPct: number;
  trigIdx: number;
  sampleCount: number;
  sampleSize: number;
}

export interface ScopeArmConfig {
  trigMode: number;
  preTrigPct: number;
  trigChannel: number;
  trigEdge: number;
  threshold: number;
}
