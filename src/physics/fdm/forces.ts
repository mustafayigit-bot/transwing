/** Force / moment evaluation of the 6-DOF model: rotors + motors, strip aerodynamics, fuselage. */
import { DEG, GeomState, interp, type FdmModel } from './model';

export interface ForceInput {
  /** velocity of the CG relative to the air mass, body axes */
  vb: number[];
  /** body angular rate */
  w: number[];
  /** rotor speeds rad/s */
  omega: number[];
  /** ESC duty 0..1 per rotor */
  duty: number[];
  /** controls [aileron, elevator, rudder] rad */
  surf: number[];
  rho: number;
  vBatt: number;
}

export interface ForceOutput {
  F: number[]; // total non-gravity force, body
  M: number[]; // total moment about CG, body
  Faero: number[];
  Maero: number[];
  thrust: number[];
  qAero: number[]; // aerodynamic torque on each rotor
  qMotor: number[];
  current: number[]; // motor phase current
  battCurrent: number;
  vi: number[];
  axialSpeed: number[];
  omegaDot: number[];
  /** moment about the hinge axis from aero + thrust on each panel [L, R] (+ = folding direction) */
  hinge: number[];
  /** angular momentum of the rotors, body */
  hRot: number[];
  wingCl: number;
  stallFraction: number;
  alpha: number;
  beta: number;
}

export function newForceOutput(nRotors: number): ForceOutput {
  const z = () => new Array<number>(nRotors).fill(0);
  return {
    F: [0, 0, 0], M: [0, 0, 0], Faero: [0, 0, 0], Maero: [0, 0, 0], thrust: z(), qAero: z(), qMotor: z(), current: z(), battCurrent: 0,
    vi: z(), axialSpeed: z(), omegaDot: z(), hinge: [0, 0], hRot: [0, 0, 0], wingCl: 0, stallFraction: 0, alpha: 0, beta: 0,
  };
}

/** Propeller thrust and shaft power from speed and axial inflow. */
export function propPoint(m: FdmModel, omega: number, axial: number, rho: number, D: number): { T: number; P: number; J: number } {
  const n = omega / (2 * Math.PI);
  if (n < 0.5) return { T: 0, P: 0, J: 0 };
  const J = Math.max(0, axial) / (n * D);
  const pr = m.spec.prop;
  const ct = interp(pr.j, pr.ct, J), cp = interp(pr.j, pr.cp, J);
  const n2 = n * n, D4 = D * D * D * D;
  return { T: ct * rho * n2 * D4, P: cp * rho * n2 * n * D4 * D, J };
}

/** Rotor speed (rad/s) needed for a thrust at the given axial inflow (bisection, monotonic). */
export function omegaForThrust(m: FdmModel, thrust: number, axial: number, rho: number, D: number): number {
  if (thrust <= 0) {
    // zero-thrust speed: windmilling advance ratio (keeps the prop unloaded but spinning)
    const j0 = zeroThrustJ(m);
    return axial > 1 ? (2 * Math.PI * axial) / (j0 * D) : 0;
  }
  let lo = 0, hi = 1500;
  for (let i = 0; i < 30; i++) {
    const mid = (lo + hi) / 2;
    if (propPoint(m, mid, axial, rho, D).T < thrust) lo = mid;
    else hi = mid;
  }
  return (lo + hi) / 2;
}

const j0Cache = new WeakMap<object, number>();
export function zeroThrustJ(m: FdmModel): number {
  const pr = m.spec.prop;
  let v = j0Cache.get(pr);
  if (v === undefined) {
    v = pr.j[pr.j.length - 1];
    for (let i = 1; i < pr.j.length; i++) {
      if (pr.ct[i] <= 0) {
        const t = pr.ct[i - 1] / (pr.ct[i - 1] - pr.ct[i]);
        v = pr.j[i - 1] + t * (pr.j[i] - pr.j[i - 1]);
        break;
      }
    }
    j0Cache.set(pr, v);
  }
  return v;
}

/** Duty cycle that holds a rotor speed in steady state (thrust linearisation feed-forward). */
export function dutyForOmega(m: FdmModel, omega: number, axial: number, rho: number, D: number, vBatt: number): number {
  if (omega <= 1) return 0;
  const { P } = propPoint(m, omega, axial, rho, D);
  const q = P / omega;
  const i = Math.max(0, q / m.kt) + m.spec.motor.noLoadCurrentA;
  return Math.min(1, Math.max(0, (omega / m.kvRad + i * m.rTot) / Math.max(1, vBatt)));
}

const STALL_SHARPNESS = 40;

/**
 * Evaluate all forces. `geom` must already be updated for the current fold angle.
 * `prevWingCl` provides the wing lift coefficient for the tail downwash (one-evaluation lag).
 */
export function evalForces(m: FdmModel, geom: GeomState, inp: ForceInput, out: ForceOutput, prevWingCl: number): void {
  const { vb, w, rho } = inp;
  const nR = m.rotors.length;
  let Fx = 0, Fy = 0, Fz = 0, Mx = 0, My = 0, Mz = 0;
  out.hinge[0] = 0; out.hinge[1] = 0;
  out.hRot[0] = 0; out.hRot[1] = 0; out.hRot[2] = 0;
  let battI = 0;
  const motor = m.spec.motor;
  const Jr = m.spec.prop.rotorInertiaKgM2;

  // ---- rotors
  for (let k = 0; k < nR; k++) {
    const r = m.rotors[k];
    const hx = geom.hub[3 * k], hy = geom.hub[3 * k + 1], hz = geom.hub[3 * k + 2];
    const tx = geom.taxis[3 * k], ty = geom.taxis[3 * k + 1], tz = geom.taxis[3 * k + 2];
    // hub velocity through the air
    const ux = vb[0] + (w[1] * hz - w[2] * hy), uy = vb[1] + (w[2] * hx - w[0] * hz), uz = vb[2] + (w[0] * hy - w[1] * hx);
    const va = ux * tx + uy * ty + uz * tz;
    const om = inp.omega[k];
    const pp = propPoint(m, om, va, rho, r.diameter);
    const T = pp.T;
    const qA = om > 1 ? pp.P / om : 0;
    // motor: phase current from applied voltage and back-EMF
    let i = (inp.duty[k] * inp.vBatt - om / m.kvRad) / m.rTot;
    if (i > motor.maxCurrentA) i = motor.maxCurrentA;
    if (i < 0) i = 0; // no regenerative braking
    const qM = Math.max(0, i - motor.noLoadCurrentA) * m.kt;
    out.thrust[k] = T; out.qAero[k] = qA; out.qMotor[k] = qM; out.current[k] = i; out.axialSpeed[k] = va;
    out.omegaDot[k] = (qM - qA - 2e-5 * om) / Jr;
    battI += i * Math.min(1, Math.max(0, (om / m.kvRad + i * m.rTot) / Math.max(1, inp.vBatt)));
    // induced velocity at the disc (momentum theory, axial flight)
    const vaa = Math.max(0, va);
    out.vi[k] = T > 0 ? -vaa / 2 + Math.sqrt((vaa / 2) ** 2 + T / (2 * rho * r.area)) : 0;
    // thrust force + reaction torque on the airframe
    const fx = T * tx, fy = T * ty, fz = T * tz;
    const mqx = -r.spin * qM * tx, mqy = -r.spin * qM * ty, mqz = -r.spin * qM * tz;
    const mx = hy * fz - hz * fy + mqx, my = hz * fx - hx * fz + mqy, mz = hx * fy - hy * fx + mqz;
    Fx += fx; Fy += fy; Fz += fz; Mx += mx; My += my; Mz += mz;
    out.hRot[0] += r.spin * Jr * om * tx; out.hRot[1] += r.spin * Jr * om * ty; out.hRot[2] += r.spin * Jr * om * tz;
    const b = r.body - 1;
    const pv = geom.pivotRel[b], ax = m.axis[b];
    const rx = hx - pv[0], ry = hy - pv[1], rz = hz - pv[2];
    out.hinge[b] += (ry * fz - rz * fy + mqx) * ax[0] + (rz * fx - rx * fz + mqy) * ax[1] + (rx * fy - ry * fx + mqz) * ax[2];
  }
  out.battCurrent = battI;
  const Tx = Fx, Ty = Fy, Tz = Fz, TMx = Mx, TMy = My, TMz = Mz;

  // ---- strips
  const V = Math.hypot(vb[0], vb[1], vb[2]);
  const eps = (2 * prevWingCl) / (Math.PI * m.AR); // downwash angle at the tail
  let clSum = 0, stallArea = 0, wingArea = 0;
  let ax_ = 0, ay_ = 0, az_ = 0, amx = 0, amy = 0, amz = 0;
  const surf = inp.surf;
  const nS = m.strips.length;
  for (let k = 0; k < nS; k++) {
    const st = m.strips[k];
    const px = geom.sp[3 * k], py = geom.sp[3 * k + 1], pz = geom.sp[3 * k + 2];
    // strip velocity through the air
    let ux = vb[0] + (w[1] * pz - w[2] * py), uy = vb[1] + (w[2] * px - w[0] * pz), uz = vb[2] + (w[0] * py - w[1] * px);
    // propeller slipstream (air moves along −t̂ behind each disc)
    for (let j = 0; j < nR; j++) {
      const vi = out.vi[j];
      if (vi < 0.05) continue;
      const tx = geom.taxis[3 * j], ty = geom.taxis[3 * j + 1], tz = geom.taxis[3 * j + 2];
      const dx = px - geom.hub[3 * j], dy = py - geom.hub[3 * j + 1], dz = pz - geom.hub[3 * j + 2];
      const sDown = -(dx * tx + dy * ty + dz * tz);
      if (sDown <= 0) continue;
      const rx = dx + sDown * tx, ry = dy + sDown * ty, rz = dz + sDown * tz;
      const R = m.rotors[j].radius;
      const rad2 = rx * rx + ry * ry + rz * rz;
      if (rad2 > R * R) continue;
      const vs = vi * (1 + sDown / Math.sqrt(sDown * sDown + R * R));
      // the strip "moves" against the slipstream: add +t̂·vs to its air-relative velocity
      ux += tx * vs; uy += ty * vs; uz += tz * vs;
    }
    if (st.kind === 1) uy += eps * Math.max(0, vb[0]); // tail sits in the wing downwash
    const cx = geom.sc[3 * k], cy = geom.sc[3 * k + 1], cz = geom.sc[3 * k + 2];
    const nx = geom.sn[3 * k], ny = geom.sn[3 * k + 1], nz = geom.sn[3 * k + 2];
    const wc = ux * cx + uy * cy + uz * cz;
    const wn = ux * nx + uy * ny + uz * nz;
    const v2 = wc * wc + wn * wn;
    if (v2 < 1e-6) continue;
    const vmag = Math.sqrt(v2);
    const alpha = Math.atan2(-wn, wc);
    const ctrl = st.gAil * surf[0] + st.gElev * surf[1] + st.gRud * surf[2];
    const ae = alpha + ctrl;
    // attached-flow / flat-plate blend
    const e1 = Math.exp(-STALL_SHARPNESS * (ae - (st.aStallPos + 2 * DEG)));
    const e2 = Math.exp(STALL_SHARPNESS * (ae - (st.aStallNeg - 2 * DEG)));
    const sig = (1 + e1 + e2) / ((1 + e1) * (1 + e2));
    const clLin = st.a * (ae - st.alpha0);
    const sa = Math.sin(ae), ca = Math.cos(ae);
    const cn = m.cd90 * sa;
    const cl = (1 - sig) * clLin + sig * cn * ca;
    const cd = (1 - sig) * (st.cd0 + st.kInd * clLin * clLin + 0.02 * ctrl * ctrl) + sig * (Math.abs(cn * sa) + st.cd0);
    const cm = (1 - sig) * st.cm0 + sig * (-0.25 * cn * Math.abs(sa)); // centre of pressure moves to mid-chord
    const qS = 0.5 * rho * v2 * st.area;
    // unit vectors of the relative wind in the (c, n) plane: a = (−wc, −wn)/|w|
    const acx = -wc / vmag, anx = -wn / vmag;
    const L = qS * cl, Dg = qS * cd;
    // drag along a, lift along (a_n, −a_c)
    const fc = Dg * acx + L * anx;
    const fn = Dg * anx - L * acx;
    const fx = fc * cx + fn * nx, fy = fc * cy + fn * ny, fz = fc * cz + fn * nz;
    // section pitching moment about c × n
    const mm = qS * st.chord * cm;
    const mvx = mm * (cy * nz - cz * ny), mvy = mm * (cz * nx - cx * nz), mvz = mm * (cx * ny - cy * nx);
    ax_ += fx; ay_ += fy; az_ += fz;
    amx += py * fz - pz * fy + mvx; amy += pz * fx - px * fz + mvy; amz += px * fy - py * fx + mvz;
    if (st.body !== 0) {
      const b = st.body - 1;
      const pv = geom.pivotRel[b], ax = m.axis[b];
      const rx = px - pv[0], ry = py - pv[1], rz = pz - pv[2];
      out.hinge[b] += (ry * fz - rz * fy + mvx) * ax[0] + (rz * fx - rx * fz + mvy) * ax[1] + (rx * fy - ry * fx + mvz) * ax[2];
    }
    if (st.kind === 0) {
      wingArea += st.area;
      clSum += (1 - sig) * clLin * st.area;
      stallArea += sig * st.area;
    }
  }
  // ---- fuselage + nacelles: bluff-body drag and Munk moments
  {
    const bp = m.bodyPoint;
    const px = bp[0] - geom.cg[0], py = bp[1] - geom.cg[1], pz = bp[2] - geom.cg[2];
    const ux = vb[0] + (w[1] * pz - w[2] * py), uy = vb[1] + (w[2] * px - w[0] * pz), uz = vb[2] + (w[0] * py - w[1] * px);
    const um = Math.hypot(ux, uy, uz);
    const k = -0.5 * rho * um;
    const fx = k * m.cdaFront * ux, fy = k * m.cdaTop * uy, fz = k * m.cdaSide * uz;
    ax_ += fx; ay_ += fy; az_ += fz;
    amx += py * fz - pz * fy; amy += pz * fx - px * fz; amz += px * fy - py * fx;
    // Munk (apparent-mass) moments of a slender body: destabilising in pitch and yaw
    const km = 0.85 * rho * m.fuselageVolume * Math.max(0, ux);
    amz += km * -uy;
    amy += km * uz;
  }
  Fx += ax_; Fy += ay_; Fz += az_; Mx += amx; My += amy; Mz += amz;
  out.F[0] = Fx; out.F[1] = Fy; out.F[2] = Fz; out.M[0] = Mx; out.M[1] = My; out.M[2] = Mz;
  out.Faero[0] = Fx - Tx; out.Faero[1] = Fy - Ty; out.Faero[2] = Fz - Tz;
  out.Maero[0] = Mx - TMx; out.Maero[1] = My - TMy; out.Maero[2] = Mz - TMz;
  out.wingCl = wingArea > 0 ? clSum / wingArea : 0;
  out.stallFraction = wingArea > 0 ? stallArea / wingArea : 0;
  out.alpha = V > 0.5 ? Math.atan2(-vb[1], vb[0]) : 0;
  out.beta = V > 0.5 ? Math.asin(Math.max(-1, Math.min(1, vb[2] / V))) : 0;
}
