const CRC_INIT = 0xFFFF;
const CRC_POLY = 0x1021;
const START_BYTE = 0x02;

function crc16Update(crc: number, byte: number): number {
  crc ^= (byte << 8) & 0xFFFF;
  for (let i = 0; i < 8; i++) {
    if (crc & 0x8000) crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF;
    else crc = (crc << 1) & 0xFFFF;
  }
  return crc;
}

export function buildPacket(cmdId: number, payload?: Uint8Array): Uint8Array {
  const payloadLen = payload ? payload.length : 0;
  const pktLen = 1 + payloadLen;
  let crc = crc16Update(CRC_INIT, pktLen);
  crc = crc16Update(crc, cmdId);
  if (payload) {
    for (const b of payload) crc = crc16Update(crc, b);
  }
  const frame = new Uint8Array(5 + payloadLen);
  frame[0] = START_BYTE;
  frame[1] = pktLen;
  frame[2] = cmdId;
  if (payload) frame.set(payload, 3);
  frame[3 + payloadLen] = (crc >> 8) & 0xFF;
  frame[4 + payloadLen] = crc & 0xFF;
  return frame;
}

export type PacketCallback = (cmdId: number, payload: Uint8Array) => void;

export class GspParser {
  private state: 'WAIT_START' | 'GOT_START' | 'COLLECTING' = 'WAIT_START';
  private pktBuf = new Uint8Array(254);
  private pktLen = 0;
  private pktIdx = 0;
  private crcBuf = [0, 0];
  private crcIdx = 0;
  private collectingCrc = false;
  private onPacket: PacketCallback;

  constructor(onPacket: PacketCallback) {
    this.onPacket = onPacket;
  }

  reset() {
    this.state = 'WAIT_START';
    this.pktIdx = 0;
    this.pktLen = 0;
    this.crcIdx = 0;
    this.collectingCrc = false;
  }

  feed(data: Uint8Array) {
    for (const b of data) {
      switch (this.state) {
        case 'WAIT_START':
          if (b === START_BYTE) this.state = 'GOT_START';
          break;
        case 'GOT_START':
          if (b >= 1 && b <= 249) {
            this.pktLen = b;
            this.pktIdx = 0;
            this.crcIdx = 0;
            this.collectingCrc = false;
            this.state = 'COLLECTING';
          } else {
            this.reset();
          }
          break;
        case 'COLLECTING':
          if (!this.collectingCrc) {
            this.pktBuf[this.pktIdx++] = b;
            if (this.pktIdx >= this.pktLen) {
              this.collectingCrc = true;
              this.crcIdx = 0;
            }
          } else {
            this.crcBuf[this.crcIdx++] = b;
            if (this.crcIdx >= 2) {
              let crc = crc16Update(CRC_INIT, this.pktLen);
              for (let i = 0; i < this.pktLen; i++)
                crc = crc16Update(crc, this.pktBuf[i]);
              const rxCrc = (this.crcBuf[0] << 8) | this.crcBuf[1];
              if (crc === rxCrc) {
                const cmdId = this.pktBuf[0];
                const payload = new Uint8Array(this.pktBuf.buffer, 1, this.pktLen - 1);
                this.onPacket(cmdId, new Uint8Array(payload));
              }
              this.reset();
            }
          }
          break;
      }
    }
  }
}

// GSP command IDs
export const CMD = {
  PING: 0x00,
  GET_INFO: 0x01,
  GET_SNAPSHOT: 0x02,
  START_MOTOR: 0x03,
  STOP_MOTOR: 0x04,
  CLEAR_FAULT: 0x05,
  SET_THROTTLE: 0x06,
  SET_THROTTLE_SRC: 0x07,
  HEARTBEAT: 0x08,
  GET_PARAM: 0x10,
  SET_PARAM: 0x11,
  SAVE_CONFIG: 0x12,
  LOAD_DEFAULTS: 0x13,
  TELEM_START: 0x14,
  TELEM_STOP: 0x15,
  GET_PARAM_LIST: 0x16,
  LOAD_PROFILE: 0x17,
  AUTO_DETECT: 0x20,
  GET_RX_STATUS: 0x26,
  SCOPE_ARM: 0x30,
  SCOPE_STATUS: 0x31,
  SCOPE_READ: 0x32,
  TELEM_FRAME: 0x80,
  ERROR: 0xFF,
} as const;

/* ── Burst Scope Helpers ─────────────────────────────────────── */

import type { ScopeSample, ScopeStatus, ScopeArmConfig } from './types';
import { SCOPE_SAMPLE_SIZE } from './types';

export function buildScopeArmPayload(cfg: ScopeArmConfig): Uint8Array {
  const buf = new Uint8Array(8);
  buf[0] = cfg.trigMode;
  buf[1] = cfg.preTrigPct;
  buf[2] = cfg.trigChannel;
  buf[3] = cfg.trigEdge;
  const dv = new DataView(buf.buffer);
  dv.setInt16(4, cfg.threshold, true);
  buf[6] = 0; // reserved
  buf[7] = 0;
  return buf;
}

export function buildScopeReadPayload(offset: number, count: number): Uint8Array {
  return new Uint8Array([offset, count]);
}

export function decodeScopeStatus(data: Uint8Array): ScopeStatus {
  return {
    state: data[0],
    trigMode: data[1],
    preTrigPct: data[2],
    trigIdx: data[3],
    sampleCount: data[4],
    sampleSize: data[5],
  };
}

export function decodeScopeSamples(data: Uint8Array): { offset: number; samples: ScopeSample[] } {
  const offset = data[0];
  const count = data[1];
  const samples: ScopeSample[] = [];
  const v = new DataView(data.buffer, data.byteOffset, data.byteLength);

  for (let i = 0; i < count; i++) {
    const base = 2 + i * SCOPE_SAMPLE_SIZE;
    if (base + SCOPE_SAMPLE_SIZE > data.byteLength) break;

    const flags = data[base + 22];
    samples.push({
      ia:       v.getInt16(base + 0, true) / 1000,
      ib:       v.getInt16(base + 2, true) / 1000,
      id:       v.getInt16(base + 4, true) / 1000,
      iq:       v.getInt16(base + 6, true) / 1000,
      vd:       v.getInt16(base + 8, true) / 100,
      vq:       v.getInt16(base + 10, true) / 100,
      theta:    v.getInt16(base + 12, true) / 10000,
      obsX1:    v.getInt16(base + 14, true) / 100000,
      obsX2:    v.getInt16(base + 16, true) / 100000,
      omega:    v.getInt16(base + 18, true) / 10,
      modIndex: v.getInt16(base + 20, true) / 10000,
      flags,
      state:    data[base + 23],
      tickLsb:  v.getUint16(base + 24, true),
      cl:       (flags & 0x01) !== 0,
      fault:    (flags & 0x02) !== 0,
      mode:     (flags >> 2) & 0x07,
    });
  }
  return { offset, samples };
}
