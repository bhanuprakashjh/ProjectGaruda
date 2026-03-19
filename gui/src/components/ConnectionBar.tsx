import { useRef, useCallback } from 'react';
import { useEscStore } from '../store/useEscStore';
import { SerialManager } from '../protocol/serial';
import { GspParser, buildPacket, CMD, decodeScopeStatus, decodeScopeSamples } from '../protocol/gsp';
import { decodeInfo, decodeSnapshot, decodeCkSnapshot, decodeParamList, decodeParamValue } from '../protocol/decode';
import type { ParamDescriptor, ParamListPage } from '../protocol/types';
import { isCkBoard, BOARD_ID_CK } from '../protocol/types';

const serial = new SerialManager();
let parser: GspParser;
let heartbeatTimer: ReturnType<typeof setInterval> | null = null;

export function ConnectionBar() {
  const { connected, setConnected, setInfo, pushSnapshot, pushCkSnapshot, setParams, setParamValue, setActiveProfile, setTelemActive, addToast, reset, setScopeStatus, appendScopeSamples, clearScopeSamples, setScopeReading } = useEscStore();
  const connectingRef = useRef(false);
  const pendingParamPages = useRef<ParamDescriptor[]>([]);

  const ERR_NAMES: Record<number, string> = {
    0x01: 'Unknown command',
    0x02: 'Bad length',
    0x03: 'Busy',
    0x04: 'Wrong state (motor must be idle)',
    0x05: 'Value out of range',
    0x06: 'Unknown parameter',
    0x07: 'Cross-validation failed',
    0x08: 'EEPROM cooldown',
  };

  /** Accumulate param list pages, commit to store only when all received */
  const handleParamListPage = useCallback(async (page: ParamListPage) => {
    if (page.startIndex === 0) {
      pendingParamPages.current = [...page.entries];
    } else {
      pendingParamPages.current = [...pendingParamPages.current, ...page.entries];
    }

    const fetched = page.startIndex + page.entries.length;
    if (fetched < page.totalCount) {
      await serial.write(buildPacket(CMD.GET_PARAM_LIST, new Uint8Array([fetched])));
    } else {
      setParams(pendingParamPages.current);
      pendingParamPages.current = [];

      const allParams = useEscStore.getState().params;
      for (const [id] of allParams) {
        const buf = new Uint8Array(2);
        buf[0] = id & 0xFF;
        buf[1] = (id >> 8) & 0xFF;
        await serial.write(buildPacket(CMD.GET_PARAM, buf));
      }
    }
  }, [setParams]);

  /** Re-fetch all param values (after profile load or defaults restore) */
  const refetchAllParams = useCallback(async () => {
    const allParams = useEscStore.getState().params;
    for (const [id] of allParams) {
      const buf = new Uint8Array(2);
      buf[0] = id & 0xFF;
      buf[1] = (id >> 8) & 0xFF;
      await serial.write(buildPacket(CMD.GET_PARAM, buf));
    }
  }, []);

  const handlePacket = useCallback((cmdId: number, payload: Uint8Array) => {
    switch (cmdId) {
      case CMD.PING: break;
      case CMD.GET_INFO: {
        const info = decodeInfo(payload);
        setInfo(info);
        break;
      }
      case CMD.GET_SNAPSHOT: {
        const info = useEscStore.getState().info;
        if (info && isCkBoard(info.boardId)) {
          try { pushCkSnapshot(decodeCkSnapshot(payload)); } catch { /* ignore malformed */ }
        } else {
          pushSnapshot(decodeSnapshot(payload));
        }
        break;
      }
      case CMD.TELEM_FRAME: {
        const snapData = payload.slice(2);  // skip 2-byte seq header
        const info = useEscStore.getState().info;
        if (info && isCkBoard(info.boardId)) {
          if (snapData.length >= 48) {
            try { pushCkSnapshot(decodeCkSnapshot(snapData)); } catch { /* ignore malformed */ }
          }
        } else {
          if (snapData.length >= 68) {
            try { pushSnapshot(decodeSnapshot(snapData)); } catch { /* ignore malformed */ }
          }
        }
        break;
      }
      case CMD.GET_PARAM_LIST: {
        const page = decodeParamList(payload);
        handleParamListPage(page);
        break;
      }
      case CMD.GET_PARAM:
      case CMD.SET_PARAM: {
        const { id, value } = decodeParamValue(payload);
        setParamValue(id, value);
        if (cmdId === CMD.SET_PARAM) addToast(`Parameter updated`, 'success');
        break;
      }
      case CMD.SAVE_CONFIG:
        addToast('Parameters saved to EEPROM', 'success');
        break;
      case CMD.LOAD_DEFAULTS:
        addToast('Defaults restored', 'info');
        refetchAllParams();
        break;
      case CMD.LOAD_PROFILE: {
        const profileId = payload.length > 0 ? payload[0] : 0;
        setActiveProfile(profileId);
        addToast(`Profile ${profileId} loaded`, 'success');
        refetchAllParams();
        break;
      }
      case CMD.START_MOTOR:
        addToast('Motor starting', 'info');
        break;
      case CMD.STOP_MOTOR:
        addToast('Motor stopped', 'info');
        break;
      case CMD.CLEAR_FAULT:
        addToast('Fault cleared', 'info');
        break;
      case CMD.AUTO_DETECT:
        addToast('Auto-detect started', 'info');
        break;
      case CMD.SCOPE_ARM:
        addToast('Scope armed', 'info');
        break;
      case CMD.SCOPE_STATUS: {
        if (payload.length >= 6) {
          const st = decodeScopeStatus(payload);
          setScopeStatus(st);
          // Auto-read when READY (only if not already reading)
          if (st.state === 3 && st.sampleCount > 0 && !useEscStore.getState().scopeReading) {
            // Clear old samples and start chunked readout
            appendScopeSamples([]); // no-op, just ensure fresh
            useEscStore.setState({ scopeSamples: [], scopeReading: true });
            const count = Math.min(9, st.sampleCount);
            serial.write(buildPacket(CMD.SCOPE_READ, new Uint8Array([0, count])));
          }
        }
        break;
      }
      case CMD.SCOPE_READ: {
        if (payload.length >= 2) {
          const { offset, samples } = decodeScopeSamples(payload);
          appendScopeSamples(samples);
          // Continue reading if more
          const scopeStatus = useEscStore.getState().scopeStatus;
          if (scopeStatus) {
            const nextOffset = offset + samples.length;
            if (nextOffset < scopeStatus.sampleCount && samples.length > 0) {
              const count = Math.min(9, scopeStatus.sampleCount - nextOffset);
              serial.write(buildPacket(CMD.SCOPE_READ, new Uint8Array([nextOffset, count])));
            } else {
              setScopeReading(false);
              addToast(`Scope: ${useEscStore.getState().scopeSamples.length} samples read`, 'success');
            }
          }
        }
        break;
      }
      case CMD.ERROR: {
        const errCode = payload.length > 0 ? payload[0] : 0xFF;
        let msg = ERR_NAMES[errCode] ?? `Error 0x${errCode.toString(16).toUpperCase()}`;
        if (errCode === 0x08 && payload.length > 1) {
          msg += ` (${payload[1]}s remaining)`;
        }
        addToast(msg, 'error');
        break;
      }
    }
  }, [setInfo, pushSnapshot, pushCkSnapshot, setParams, setParamValue, setActiveProfile, addToast, handleParamListPage, refetchAllParams, setScopeStatus, appendScopeSamples, clearScopeSamples, setScopeReading]);

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

      serial.startReading((data) => parser.feed(data)).catch(() => {
        disconnect();
      });

      await serial.write(buildPacket(CMD.TELEM_STOP));
      await new Promise(r => setTimeout(r, 150));
      parser.reset();

      await serial.write(buildPacket(CMD.PING));
      await serial.write(buildPacket(CMD.GET_INFO));
      await new Promise(r => setTimeout(r, 50));
      await serial.write(buildPacket(CMD.GET_SNAPSHOT));
      await new Promise(r => setTimeout(r, 50));
      await serial.write(buildPacket(CMD.GET_PARAM_LIST));
    } catch (e: any) {
      console.error('Connect failed:', e);
      alert('Connect failed: ' + (e?.message || e));
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

  const btnBase: React.CSSProperties = {
    padding: '6px 14px', borderRadius: 'var(--radius-sm)', border: 'none',
    fontSize: 12, fontWeight: 600, letterSpacing: '0.3px',
  };

  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
      {!connected ? (
        <button onClick={connect} style={{
          ...btnBase, background: 'var(--accent-blue)', color: '#fff',
        }}>
          Connect
        </button>
      ) : (
        <>
          <div style={{
            width: 7, height: 7, borderRadius: '50%',
            background: 'var(--accent-green)',
            boxShadow: '0 0 6px var(--accent-green)',
          }} />
          {!telemActive ? (
            <button onClick={startTelem} style={{
              ...btnBase, background: 'var(--accent-green-dim)',
              color: 'var(--accent-green)', border: '1px solid rgba(34,197,94,0.3)',
            }}>
              Telemetry
            </button>
          ) : (
            <button onClick={stopTelem} style={{
              ...btnBase, background: 'var(--accent-red-dim)',
              color: 'var(--accent-red)', border: '1px solid rgba(239,68,68,0.3)',
            }}>
              Stop Telem
            </button>
          )}
          <button onClick={disconnect} style={{
            ...btnBase, background: 'transparent',
            color: 'var(--text-muted)', border: '1px solid var(--border)',
          }}>
            Disconnect
          </button>
        </>
      )}
    </div>
  );
}

export { serial };
