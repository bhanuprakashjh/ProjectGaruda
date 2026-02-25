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
  GET_RX_STATUS: 0x26,
  TELEM_FRAME: 0x80,
  ERROR: 0xFF,
} as const;
