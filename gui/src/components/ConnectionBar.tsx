import { useRef, useCallback } from 'react';
import { useEscStore } from '../store/useEscStore';
import { SerialManager } from '../protocol/serial';
import { GspParser, buildPacket, CMD } from '../protocol/gsp';
import { decodeInfo, decodeSnapshot, decodeParamList, decodeParamValue } from '../protocol/decode';

const serial = new SerialManager();
let parser: GspParser;
let heartbeatTimer: ReturnType<typeof setInterval> | null = null;

export function ConnectionBar() {
  const { connected, setConnected, setInfo, pushSnapshot, setParams, setParamValue, setTelemActive, reset } = useEscStore();
  const connectingRef = useRef(false);

  const handlePacket = useCallback((cmdId: number, payload: Uint8Array) => {
    switch (cmdId) {
      case CMD.PING: break;
      case CMD.GET_INFO: setInfo(decodeInfo(payload)); break;
      case CMD.GET_SNAPSHOT: pushSnapshot(decodeSnapshot(payload)); break;
      case CMD.TELEM_FRAME: {
        // seq(2) + snapshot(68)
        const snapData = payload.slice(2);
        pushSnapshot(decodeSnapshot(snapData));
        break;
      }
      case CMD.GET_PARAM_LIST: setParams(decodeParamList(payload)); break;
      case CMD.GET_PARAM:
      case CMD.SET_PARAM: {
        const { id, value } = decodeParamValue(payload);
        setParamValue(id, value);
        break;
      }
    }
  }, [setInfo, pushSnapshot, setParams, setParamValue]);

  const connect = useCallback(async () => {
    if (connectingRef.current) return;
    connectingRef.current = true;
    try {
      if (!('serial' in navigator)) {
        alert('WebSerial not supported. Use Chrome or Edge.');
        return;
      }
      parser = new GspParser(handlePacket);
      await serial.connect();
      setConnected(true);

      // Auto-discover
      await serial.write(buildPacket(CMD.PING));
      await serial.write(buildPacket(CMD.GET_INFO));
      await serial.write(buildPacket(CMD.GET_PARAM_LIST));

      // Read loop
      serial.startReading((data) => parser.feed(data)).catch(() => {
        disconnect();
      });

      // Fetch initial param values after a short delay
      setTimeout(async () => {
        const paramIds = [0x15, 0x16, 0x17, 0x20, 0x22, 0x30, 0x42, 0x41];
        for (const id of paramIds) {
          const buf = new Uint8Array(2);
          buf[0] = id & 0xFF;
          buf[1] = (id >> 8) & 0xFF;
          await serial.write(buildPacket(CMD.GET_PARAM, buf));
        }
      }, 200);
    } catch (e) {
      console.error('Connect failed:', e);
    } finally {
      connectingRef.current = false;
    }
  }, [handlePacket, setConnected]);

  const disconnect = useCallback(async () => {
    if (heartbeatTimer) { clearInterval(heartbeatTimer); heartbeatTimer = null; }
    setTelemActive(false);
    await serial.disconnect();
    reset();
  }, [reset, setTelemActive]);

  const startTelem = useCallback(async () => {
    await serial.write(buildPacket(CMD.TELEM_START, new Uint8Array([50])));
    setTelemActive(true);
    // Start heartbeat
    if (heartbeatTimer) clearInterval(heartbeatTimer);
    heartbeatTimer = setInterval(async () => {
      try { await serial.write(buildPacket(CMD.HEARTBEAT)); } catch {}
    }, 200);
  }, [setTelemActive]);

  const stopTelem = useCallback(async () => {
    await serial.write(buildPacket(CMD.TELEM_STOP));
    setTelemActive(false);
    if (heartbeatTimer) { clearInterval(heartbeatTimer); heartbeatTimer = null; }
  }, [setTelemActive]);

  const telemActive = useEscStore(s => s.telemActive);

  const barStyle: React.CSSProperties = {
    display: 'flex', alignItems: 'center', gap: 12,
    padding: '12px 16px', background: '#16213e', borderRadius: 8,
  };

  return (
    <div style={barStyle}>
      <span style={{ fontWeight: 700, fontSize: 18 }}>Garuda ESC</span>
      <div style={{ flex: 1 }} />
      {!connected ? (
        <button onClick={connect}>Connect</button>
      ) : (
        <>
          <span style={{ color: '#4ade80' }}>● Connected</span>
          {!telemActive ? (
            <button onClick={startTelem}>Start Telemetry</button>
          ) : (
            <button onClick={stopTelem}>Stop Telemetry</button>
          )}
          <button onClick={disconnect}>Disconnect</button>
        </>
      )}
    </div>
  );
}

export { serial };
