import type { GspInfo, GspSnapshot, GspRxStatus, CkSnapshot, ParamDescriptor, ParamListPage } from './types';

export function decodeInfo(data: Uint8Array): GspInfo {
  const v = new DataView(data.buffer, data.byteOffset, data.byteLength);
  return {
    protocolVersion: v.getUint8(0),
    fwMajor: v.getUint8(1),
    fwMinor: v.getUint8(2),
    fwPatch: v.getUint8(3),
    boardId: v.getUint16(4, true),
    motorProfile: v.getUint8(6),
    motorPolePairs: v.getUint8(7),
    featureFlags: v.getUint32(8, true),
    pwmFrequency: v.getUint32(12, true),
    maxErpm: v.getUint32(16, true),
  };
}

export function decodeSnapshot(data: Uint8Array): GspSnapshot {
  const v = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const hasDiag = data.byteLength >= 150;  // v3: observer/PI/diag fields
  const hasFocVdVq = data.byteLength >= 114;
  const hasFoc = data.byteLength >= 106;
  return {
    state: v.getUint8(0),
    faultCode: v.getUint8(1),
    currentStep: v.getUint8(2),
    direction: v.getUint8(3),
    throttle: v.getUint16(4, true),
    dutyPct: v.getUint8(6),
    vbusRaw: v.getUint16(8, true),
    ibusRaw: v.getUint16(10, true),
    ibusMax: v.getUint16(12, true),
    bemfRaw: v.getUint16(14, true),
    zcThreshold: v.getUint16(16, true),
    stepPeriod: v.getUint16(18, true),
    goodZcCount: v.getUint16(20, true),
    risingZcWorks: v.getUint8(22) !== 0,
    fallingZcWorks: v.getUint8(23) !== 0,
    zcSynced: v.getUint8(24) !== 0,
    zcConfirmedCount: v.getUint16(26, true),
    zcTimeoutForceCount: v.getUint16(28, true),
    hwzcEnabled: v.getUint8(30) !== 0,
    hwzcPhase: v.getUint8(31),
    hwzcTotalZcCount: v.getUint32(32, true),
    hwzcTotalMissCount: v.getUint32(36, true),
    hwzcStepPeriodHR: v.getUint32(40, true),
    hwzcDbgLatchDisable: v.getUint8(44) !== 0,
    morphSubPhase: v.getUint8(46),
    morphStep: v.getUint8(47),
    morphZcCount: v.getUint16(48, true),
    morphAlpha: v.getUint16(50, true),
    clpciTripCount: v.getUint32(52, true),
    fpciTripCount: v.getUint32(56, true),
    systemTick: v.getUint32(60, true),
    uptimeSec: v.getUint32(64, true),
    // FOC fields (backward-compatible: zero if snapshot is old format)
    focIdMeas: hasFoc ? v.getFloat32(68, true) : 0,
    focIqMeas: hasFoc ? v.getFloat32(72, true) : 0,
    focTheta: hasFoc ? v.getFloat32(76, true) : 0,
    focOmega: hasFoc ? v.getFloat32(80, true) : 0,
    focVbus: hasFoc ? v.getFloat32(84, true) : 0,
    focIa: hasFoc ? v.getFloat32(88, true) : 0,
    focIb: hasFoc ? v.getFloat32(92, true) : 0,
    focThetaObs: hasFoc ? v.getFloat32(96, true) : 0,
    focVd: hasFocVdVq ? v.getFloat32(100, true) : 0,
    focVq: hasFocVdVq ? v.getFloat32(104, true) : 0,
    // Observer internals (v3)
    focFluxAlpha: hasDiag ? v.getFloat32(108, true) : 0,
    focFluxBeta: hasDiag ? v.getFloat32(112, true) : 0,
    focLambdaEst: hasDiag ? v.getFloat32(116, true) : 0,
    focObsGain: hasDiag ? v.getFloat32(120, true) : 0,
    // PI controller internals (v3)
    focPidDInteg: hasDiag ? v.getFloat32(124, true) : 0,
    focPidQInteg: hasDiag ? v.getFloat32(128, true) : 0,
    focPidSpdInteg: hasDiag ? v.getFloat32(132, true) : 0,
    // Derived diagnostics (v3)
    focModIndex: hasDiag ? v.getFloat32(136, true) : 0,
    focObsConfidence: hasDiag ? v.getFloat32(140, true) : 0,
    // Sub-state and offsets — position depends on format version
    focSubState: hasDiag ? v.getUint8(144) : (hasFocVdVq ? v.getUint8(108) : (hasFoc ? v.getUint8(100) : 0)),
    focOffsetIa: hasDiag ? v.getUint16(146, true) : (hasFocVdVq ? v.getUint16(110, true) : (hasFoc ? v.getUint16(102, true) : 0)),
    focOffsetIb: hasDiag ? v.getUint16(148, true) : (hasFocVdVq ? v.getUint16(112, true) : (hasFoc ? v.getUint16(104, true) : 0)),
  };
}

/** CK board snapshot: 48-64 bytes.
 *  v2 (52B): zcLatencyPct, zcBlankPct, zcBypassCount.
 *  v3 (64B): zcMode, actualForcedComm, per-polarity counters. */
export function decodeCkSnapshot(data: Uint8Array): CkSnapshot {
  const v = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const hasDiag = data.byteLength >= 52;
  const hasV3 = data.byteLength >= 64;
  return {
    state: v.getUint8(0),
    faultCode: v.getUint8(1),
    currentStep: v.getUint8(2),
    ataStatus: v.getUint8(3),
    potRaw: v.getUint16(4, true),
    dutyPct: v.getUint8(6),
    zcSynced: v.getUint8(7) !== 0,
    vbusRaw: v.getUint16(8, true),
    iaRaw: v.getInt16(10, true),
    ibRaw: v.getInt16(12, true),
    ibusRaw: v.getInt16(14, true),
    duty: v.getUint16(16, true),
    stepPeriod: v.getUint16(18, true),
    stepPeriodHR: v.getUint16(20, true),
    eRpm: v.getUint32(22, true),
    goodZcCount: v.getUint16(26, true),
    zcInterval: v.getUint16(28, true),
    prevZcInterval: v.getUint16(30, true),
    icAccepted: v.getUint16(32, true),
    icFalse: v.getUint16(34, true),
    filterLevel: v.getUint8(36),
    missedSteps: v.getUint8(37),
    forcedSteps: v.getUint8(38),
    ilimActive: v.getUint8(39) !== 0,
    systemTick: v.getUint32(40, true),
    uptimeSec: v.getUint32(44, true),
    zcLatencyPct: hasDiag ? v.getUint8(48) : 0,
    zcBlankPct: hasDiag ? v.getUint8(49) : 0,
    zcBypassCount: hasDiag ? v.getUint16(50, true) : 0,
    zcMode: hasV3 ? v.getUint8(52) : 0,
    actualForcedComm: hasV3 ? v.getUint8(53) : 0,
    zcTimeoutCount: hasV3 ? v.getUint16(54, true) : 0,
    risingZcCount: hasV3 ? v.getUint16(56, true) : 0,
    fallingZcCount: hasV3 ? v.getUint16(58, true) : 0,
    risingTimeouts: hasV3 ? v.getUint16(60, true) : 0,
    fallingTimeouts: hasV3 ? v.getUint16(62, true) : 0,

    /* V4 sector-PI fields. The firmware writes the snapshot in V4
     * layout when FEATURE_V4_SECTOR_PI is on; we decode them here
     * regardless of the V3 bit interpretations above so the GUI can
     * pick which view to render based on the V4 marker bit. */
    diagLastCapValue: v.getUint16(28, true),
    diagDelta:        v.getInt16(30, true),     /* SIGNED PI error */
    diagCaptures:     v.getUint16(32, true),
    diagPiRuns:       v.getUint16(34, true),
    v4SpBits:         v.getUint8(37),
    v4ErpmTp:         v.getUint16(38, true),
    adcBlankReject:   hasV3 ? v.getUint32(48, true) : 0,
    adcStateMismatch: hasV3 ? v.getUint32(52, true) : 0,
    adcCaptureSet:    hasV3 ? v.getUint32(56, true) : 0,
    adcSetRising:     hasV3 ? v.getUint32(60, true) : 0,
  };
}

/** V2 paginated param list: 3-byte header + 12 bytes/entry (u32 min/max) */
export function decodeParamList(data: Uint8Array): ParamListPage {
  const v = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const totalCount = v.getUint8(0);
  const startIndex = v.getUint8(1);
  const entryCount = v.getUint8(2);
  const entries: ParamDescriptor[] = [];
  for (let i = 0; i < entryCount; i++) {
    const off = 3 + i * 12;
    entries.push({
      id: v.getUint16(off, true),
      type: v.getUint8(off + 2),
      group: v.getUint8(off + 3),
      min: v.getUint32(off + 4, true),
      max: v.getUint32(off + 8, true),
    });
  }
  return { totalCount, startIndex, entries };
}

export function decodeRxStatus(data: Uint8Array): GspRxStatus {
  const v = new DataView(data.buffer, data.byteOffset, data.byteLength);
  return {
    linkState: v.getUint8(0),
    protocol: v.getUint8(1),
    dshotRate: v.getUint8(2),
    // byte 3 = pad
    throttle: v.getUint16(4, true),
    pulseUs: v.getUint16(6, true),
    crcErrors: v.getUint16(8, true),
    droppedFrames: v.getUint16(10, true),
  };
}

export function decodeParamValue(data: Uint8Array): { id: number; value: number } {
  const v = new DataView(data.buffer, data.byteOffset, data.byteLength);
  return { id: v.getUint16(0, true), value: v.getUint32(2, true) };
}
