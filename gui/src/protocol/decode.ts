import type { GspInfo, GspSnapshot, ParamDescriptor } from './types';

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
    maxErpm: v.getUint16(16, true),
  };
}

export function decodeSnapshot(data: Uint8Array): GspSnapshot {
  const v = new DataView(data.buffer, data.byteOffset, data.byteLength);
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
  };
}

export function decodeParamList(data: Uint8Array): ParamDescriptor[] {
  const result: ParamDescriptor[] = [];
  const v = new DataView(data.buffer, data.byteOffset, data.byteLength);
  for (let i = 0; i + 8 <= data.length; i += 8) {
    result.push({
      id: v.getUint16(i, true),
      type: v.getUint8(i + 2),
      group: v.getUint8(i + 3),
      min: v.getUint16(i + 4, true),
      max: v.getUint16(i + 6, true),
    });
  }
  return result;
}

export function decodeParamValue(data: Uint8Array): { id: number; value: number } {
  const v = new DataView(data.buffer, data.byteOffset, data.byteLength);
  return { id: v.getUint16(0, true), value: v.getUint32(2, true) };
}
