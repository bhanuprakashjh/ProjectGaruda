export class SerialManager {
  private port: SerialPort | null = null;
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
  private writer: WritableStreamDefaultWriter<Uint8Array> | null = null;
  private reading = false;

  async connect(baudRate = 115200): Promise<void> {
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate });
    this.writer = this.port.writable!.getWriter();
  }

  async disconnect(): Promise<void> {
    this.reading = false;
    if (this.reader) {
      try { await this.reader.cancel(); } catch {}
      this.reader.releaseLock();
      this.reader = null;
    }
    if (this.writer) {
      this.writer.releaseLock();
      this.writer = null;
    }
    if (this.port) {
      try { await this.port.close(); } catch {}
      this.port = null;
    }
  }

  async write(data: Uint8Array): Promise<void> {
    if (!this.writer) throw new Error('Not connected');
    await this.writer.write(data);
  }

  async startReading(onData: (data: Uint8Array) => void): Promise<void> {
    if (!this.port?.readable) throw new Error('Not connected');
    this.reading = true;
    this.reader = this.port.readable.getReader();
    try {
      while (this.reading) {
        const { value, done } = await this.reader.read();
        if (done || !this.reading) break;
        if (value) onData(value);
      }
    } finally {
      if (this.reader) {
        this.reader.releaseLock();
        this.reader = null;
      }
    }
  }

  get isConnected(): boolean {
    return this.port !== null;
  }
}
