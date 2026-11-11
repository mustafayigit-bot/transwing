/**
 * MAVLink telemetry log (.tlog) writer: each record is an 8-byte big-endian UNIX timestamp (µs)
 * followed by a MAVLink 1 frame. Opens in Mission Planner / QGroundControl replay, MAVExplorer.
 */
import type { MissionTrace } from '../physics/fdm/mission';
import { AP_MODE, ascii, type Telemetry } from './sensors';

/** X.25 CRC used by MAVLink. */
export function x25(bytes: Uint8Array, start: number, end: number, seed = 0xffff): number {
  let crc = seed;
  for (let i = start; i < end; i++) {
    let tmp = bytes[i] ^ (crc & 0xff);
    tmp = (tmp ^ (tmp << 4)) & 0xff;
    crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
  }
  return crc;
}

interface Msg {
  id: number;
  crcExtra: number;
  len: number;
}
const M = {
  HEARTBEAT: { id: 0, crcExtra: 50, len: 9 },
  SYS_STATUS: { id: 1, crcExtra: 124, len: 31 },
  GPS_RAW_INT: { id: 24, crcExtra: 24, len: 30 },
  ATTITUDE: { id: 30, crcExtra: 39, len: 28 },
  GLOBAL_POSITION_INT: { id: 33, crcExtra: 104, len: 28 },
  SERVO_OUTPUT_RAW: { id: 36, crcExtra: 222, len: 21 },
  NAV_CONTROLLER_OUTPUT: { id: 62, crcExtra: 183, len: 26 },
  VFR_HUD: { id: 74, crcExtra: 20, len: 20 },
  BATTERY_STATUS: { id: 147, crcExtra: 154, len: 36 },
  WIND: { id: 168, crcExtra: 1, len: 12 },
  VIBRATION: { id: 241, crcExtra: 90, len: 32 },
  STATUSTEXT: { id: 253, crcExtra: 83, len: 51 },
} satisfies Record<string, Msg>;

const DEG = Math.PI / 180;
const i16 = (x: number) => Math.max(-32768, Math.min(32767, Math.round(x)));
const u16 = (x: number) => Math.max(0, Math.min(65535, Math.round(x)));

export function encodeTlog(trace: MissionTrace, tel: Telemetry): Uint8Array {
  const c = trace.ch;
  const n = c.t.length;
  const chunks: Uint8Array[] = [];
  let seq = 0;
  const t0us = BigInt(tel.options.startEpochMs) * 1000n;
  const emit = (tS: number, m: Msg, fill: (v: DataView) => void) => {
    const out = new Uint8Array(8 + 6 + m.len + 2);
    const v = new DataView(out.buffer);
    v.setBigUint64(0, t0us + BigInt(Math.round(tS * 1e6)), false);
    out[8] = 0xfe; out[9] = m.len; out[10] = seq++ & 0xff; out[11] = 1; out[12] = 1; out[13] = m.id;
    fill(new DataView(out.buffer, 14, m.len));
    const tmp = new Uint8Array(5 + m.len + 1);
    tmp.set(out.subarray(9, 14 + m.len), 0);
    tmp[5 + m.len] = m.crcExtra;
    const crc = x25(tmp, 0, tmp.length);
    out[14 + m.len] = crc & 0xff; out[15 + m.len] = crc >> 8;
    chunks.push(out);
  };
  const opt = tel.options;
  const R = 6378137, cosLat = Math.cos(opt.homeLatDeg * DEG);
  const lat = (i: number) => Math.round((opt.homeLatDeg + (c.pn[i] / R) / DEG) * 1e7);
  const lon = (i: number) => Math.round((opt.homeLonDeg + (c.pe[i] / (R * cosLat)) / DEG) * 1e7);
  const dt = n > 1 ? c.t[1] - c.t[0] : 0.1;
  const every = (hz: number) => Math.max(1, Math.round(1 / (hz * dt)));
  const e4 = every(4), e2 = every(2), e1 = every(1);
  const events = [...trace.events];
  let evi = 0;
  const maxFold = Math.max(1, ...c.fold);
  for (let i = 0; i < n; i++) {
    const t = c.t[i];
    while (evi < events.length && events[evi].t <= t) {
      const e = events[evi++];
      const txt = ascii(e.text).slice(0, 50);
      emit(t, M.STATUSTEXT, (v) => {
        v.setUint8(0, e.level === 'alarm' ? 2 : e.level === 'warn' ? 4 : 6);
        for (let k = 0; k < txt.length; k++) v.setUint8(1 + k, txt.charCodeAt(k));
      });
    }
    if (i % e1 === 0) {
      emit(t, M.HEARTBEAT, (v) => {
        v.setUint32(0, AP_MODE.AUTO, true); // custom_mode
        v.setUint8(4, 21); // MAV_TYPE_VTOL_TILTROTOR
        v.setUint8(5, 3); // MAV_AUTOPILOT_ARDUPILOTMEGA
        v.setUint8(6, 0x80 | 0x40 | 0x10 | 0x08 | 0x01); // armed, manual input, stabilise, guided, custom mode
        v.setUint8(7, c.phase[i] >= 9 ? 3 : 4); // standby / active
        v.setUint8(8, 3);
      });
      emit(t, M.SYS_STATUS, (v) => {
        v.setUint32(0, 0x0fffffff, true); v.setUint32(4, 0x0fffffff, true); v.setUint32(8, 0x0fffffff, true);
        v.setUint16(12, 250, true);
        v.setUint16(14, u16(c.battV[i] * 1000), true);
        v.setInt16(16, i16(c.battI[i] * 100), true);
        v.setInt8(30, Math.max(0, Math.min(100, Math.round(c.soc[i]))));
      });
      emit(t, M.BATTERY_STATUS, (v) => {
        v.setInt32(0, Math.round(c.usedAh[i] * 1000), true);
        v.setInt32(4, Math.round(c.energyWh[i] * 36), true); // hJ
        v.setInt16(8, i16(c.cellT[i] * 100), true);
        v.setUint16(10, u16(c.battV[i] * 1000), true); // pack voltage in the first slot (> 10 cells)
        for (let k = 1; k < 10; k++) v.setUint16(10 + 2 * k, 65535, true);
        v.setInt16(30, i16(c.battI[i] * 100), true);
        v.setUint8(32, 0); v.setUint8(33, 1); v.setUint8(34, 3); // id, function = all, type = Li-ion
        v.setInt8(35, Math.max(0, Math.min(100, Math.round(c.soc[i]))));
      });
      emit(t, M.VIBRATION, (v) => {
        v.setBigUint64(0, BigInt(Math.round(t * 1e6)), true);
        v.setFloat32(8, tel.ch.vibeX[i], true); v.setFloat32(12, tel.ch.vibeY[i], true); v.setFloat32(16, tel.ch.vibeZ[i], true);
      });
      emit(t, M.WIND, (v) => {
        const sp = Math.hypot(c.windN[i], c.windE[i]);
        v.setFloat32(0, ((Math.atan2(-c.windE[i], -c.windN[i]) / DEG) + 360) % 360, true); // direction the wind comes from
        v.setFloat32(4, sp, true); v.setFloat32(8, c.windU[i], true);
      });
    }
    if (i % e4 === 0) {
      emit(t, M.ATTITUDE, (v) => {
        v.setUint32(0, Math.round(t * 1000), true);
        v.setFloat32(4, c.roll[i] * DEG, true); v.setFloat32(8, c.pitch[i] * DEG, true);
        v.setFloat32(12, Math.atan2(Math.sin(c.yaw[i] * DEG), Math.cos(c.yaw[i] * DEG)), true);
        v.setFloat32(16, c.p[i] * DEG, true); v.setFloat32(20, c.q[i] * DEG, true); v.setFloat32(24, c.r[i] * DEG, true);
      });
      emit(t, M.GLOBAL_POSITION_INT, (v) => {
        v.setUint32(0, Math.round(t * 1000), true);
        v.setInt32(4, lat(i), true); v.setInt32(8, lon(i), true);
        v.setInt32(12, Math.round((opt.homeAltMslM + c.alt[i]) * 1000), true); v.setInt32(16, Math.round(c.alt[i] * 1000), true);
        v.setInt16(20, i16(c.vn[i] * 100), true); v.setInt16(22, i16(c.ve[i] * 100), true); v.setInt16(24, i16(c.vd[i] * 100), true);
        v.setUint16(26, u16(c.yaw[i] * 100), true);
      });
      emit(t, M.VFR_HUD, (v) => {
        let thr = 0;
        for (let k = 0; k < trace.nRotors; k++) thr += c[`thr${k + 1}`][i] / trace.nRotors;
        v.setFloat32(0, tel.ch.arspd[i], true); v.setFloat32(4, c.gs[i], true); v.setFloat32(8, opt.homeAltMslM + c.alt[i], true); v.setFloat32(12, -c.vd[i], true);
        v.setInt16(16, i16(c.yaw[i]), true); v.setUint16(18, u16(thr), true);
      });
    }
    if (i % e2 === 0) {
      emit(t, M.GPS_RAW_INT, (v) => {
        v.setBigUint64(0, BigInt(Math.round(t * 1e6)), true);
        v.setInt32(8, lat(i), true); v.setInt32(12, lon(i), true); v.setInt32(16, Math.round((opt.homeAltMslM + tel.ch.gpsAlt[i]) * 1000), true);
        v.setUint16(20, u16(tel.ch.hdop[i] * 100), true); v.setUint16(22, u16(tel.ch.hdop[i] * 150), true);
        v.setUint16(24, u16(c.gs[i] * 100), true); v.setUint16(26, u16((((Math.atan2(c.ve[i], c.vn[i]) / DEG) + 360) % 360) * 100), true);
        v.setUint8(28, 3); v.setUint8(29, tel.ch.gpsSats[i]);
      });
      emit(t, M.SERVO_OUTPUT_RAW, (v) => {
        v.setUint32(0, Math.round(t * 1e6) >>> 0, true);
        const srv = (deg: number) => u16(1500 + 400 * Math.max(-1, Math.min(1, deg / 25)));
        for (let k = 0; k < 4; k++) v.setUint16(4 + 2 * k, u16(1000 + 10 * (c[`thr${k + 1}`]?.[i] ?? 0)), true);
        v.setUint16(12, srv(c.ail[i]), true); v.setUint16(14, srv(-c.elev[i] - c.rud[i]), true); v.setUint16(16, srv(-c.elev[i] + c.rud[i]), true);
        v.setUint16(18, u16(1000 + 1000 * (1 - c.fold[i] / maxFold)), true);
        v.setUint8(20, 0);
      });
      emit(t, M.NAV_CONTROLLER_OUTPUT, (v) => {
        v.setFloat32(0, c.rollCmd[i], true); v.setFloat32(4, c.pitchCmd[i], true); v.setFloat32(8, c.altCmd[i] - c.alt[i], true);
        v.setFloat32(12, c.spdCmd[i] - c.ias[i], true); v.setFloat32(16, c.xtrack[i], true);
        v.setInt16(20, i16(c.yaw[i]), true); v.setInt16(22, i16(c.yaw[i]), true); v.setUint16(24, u16(c.wpDist[i]), true);
      });
    }
  }
  let total = 0;
  for (const ch of chunks) total += ch.length;
  const out = new Uint8Array(total);
  let off = 0;
  for (const ch of chunks) { out.set(ch, off); off += ch.length; }
  return out;
}
