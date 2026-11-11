/**
 * ArduPilot DataFlash (.bin) writer. The format is self-describing: every message type is
 * announced by an FMT record (type, length, name, format string, labels) and each record is
 * `0xA3 0x95 <type> <payload>`. Files open in Mission Planner, MAVExplorer, UAV Log Viewer,
 * PlotJuggler, pymavlink …
 */
import type { Stream, Telemetry } from './sensors';

const HEAD1 = 0xa3, HEAD2 = 0x95, FMT_TYPE = 128, FMT_LEN = 89;

/** Size in bytes of each DataFlash format character. */
export const FORMAT_SIZE: Record<string, number> = {
  b: 1, B: 1, M: 1, h: 2, H: 2, c: 2, C: 2, i: 4, I: 4, f: 4, e: 4, E: 4, L: 4, d: 8, q: 8, Q: 8, n: 4, N: 16, Z: 64,
};

export function messageLength(format: string): number {
  let n = 3;
  for (const ch of format) n += FORMAT_SIZE[ch] ?? 0;
  return n;
}

function putString(view: DataView, off: number, s: string, len: number) {
  for (let i = 0; i < len; i++) view.setUint8(off + i, i < s.length ? s.charCodeAt(i) & 0x7f : 0);
}

function writeField(view: DataView, off: number, ch: string, v: number | string): number {
  const x = typeof v === 'number' && Number.isFinite(v) ? v : 0;
  const clampI = (val: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, Math.round(val)));
  switch (ch) {
    case 'b': view.setInt8(off, clampI(x, -128, 127)); return 1;
    case 'B': case 'M': view.setUint8(off, clampI(x, 0, 255)); return 1;
    case 'h': view.setInt16(off, clampI(x, -32768, 32767), true); return 2;
    case 'H': view.setUint16(off, clampI(x, 0, 65535), true); return 2;
    case 'c': view.setInt16(off, clampI(x * 100, -32768, 32767), true); return 2;
    case 'C': view.setUint16(off, clampI(x * 100, 0, 65535), true); return 2;
    case 'i': view.setInt32(off, clampI(x, -2147483648, 2147483647), true); return 4;
    case 'I': view.setUint32(off, clampI(x, 0, 4294967295), true); return 4;
    case 'e': view.setInt32(off, clampI(x * 100, -2147483648, 2147483647), true); return 4;
    case 'E': view.setUint32(off, clampI(x * 100, 0, 4294967295), true); return 4;
    case 'L': view.setInt32(off, clampI(x * 1e7, -2147483648, 2147483647), true); return 4;
    case 'f': view.setFloat32(off, typeof v === 'number' ? v : 0, true); return 4;
    case 'd': view.setFloat64(off, typeof v === 'number' ? v : 0, true); return 8;
    case 'q': view.setBigInt64(off, BigInt(Math.round(x)), true); return 8;
    case 'Q': view.setBigUint64(off, BigInt(Math.max(0, Math.round(x))), true); return 8;
    case 'n': putString(view, off, String(v), 4); return 4;
    case 'N': putString(view, off, String(v), 16); return 16;
    case 'Z': putString(view, off, String(v), 64); return 64;
    default: throw new Error(`DataFlash: bilinmeyen format karakteri "${ch}"`);
  }
}

/** Encode all streams, time-ordered, with FMT headers first. */
export function encodeDataFlash(tel: Telemetry): Uint8Array {
  const streams = tel.streams.filter((s) => s.rows.length > 0);
  for (const s of streams) {
    if (s.name.length > 4) throw new Error(`DataFlash: mesaj adı 4 karakteri aşamaz (${s.name})`);
    if (s.format.length !== s.labels.length) throw new Error(`DataFlash: ${s.name} format / etiket sayısı uyuşmuyor`);
    if (s.labels.join(',').length > 64) throw new Error(`DataFlash: ${s.name} etiketleri 64 karakteri aşıyor`);
  }
  const ids = new Map<Stream, number>();
  streams.forEach((s, i) => ids.set(s, 129 + i));
  const lens = new Map<Stream, number>();
  let total = FMT_LEN * (streams.length + 1);
  for (const s of streams) {
    const l = messageLength(s.format);
    lens.set(s, l);
    total += l * s.rows.length;
  }
  const buf = new ArrayBuffer(total);
  const view = new DataView(buf);
  let off = 0;
  const fmt = (type: number, length: number, name: string, format: string, labels: string) => {
    view.setUint8(off, HEAD1); view.setUint8(off + 1, HEAD2); view.setUint8(off + 2, FMT_TYPE);
    view.setUint8(off + 3, type); view.setUint8(off + 4, length);
    putString(view, off + 5, name, 4); putString(view, off + 9, format, 16); putString(view, off + 25, labels, 64);
    off += FMT_LEN;
  };
  fmt(FMT_TYPE, FMT_LEN, 'FMT', 'BBnNZ', 'Type,Length,Name,Format,Columns');
  for (const s of streams) fmt(ids.get(s)!, lens.get(s)!, s.name, s.format, s.labels.join(','));
  // k-way merge by TimeUS (rows inside a stream are already time-ordered)
  const cursor = streams.map(() => 0);
  for (;;) {
    let best = -1, bestT = Infinity;
    for (let k = 0; k < streams.length; k++) {
      const r = streams[k].rows[cursor[k]];
      if (r && (r[0] as number) < bestT) { bestT = r[0] as number; best = k; }
    }
    if (best < 0) break;
    const s = streams[best];
    const row = s.rows[cursor[best]++];
    view.setUint8(off, HEAD1); view.setUint8(off + 1, HEAD2); view.setUint8(off + 2, ids.get(s)!);
    let p = off + 3;
    for (let j = 0; j < s.format.length; j++) p += writeField(view, p, s.format[j], row[j]);
    off += lens.get(s)!;
  }
  return new Uint8Array(buf, 0, off);
}
