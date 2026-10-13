/** Procedural geometry builders (pure three.js, no React). */
import { BufferGeometry, Float32BufferAttribute, LatheGeometry, Vector2, Vector3 } from 'three';

/** NACA 4-digit closed profile, chord-normalised. Returns loop: TE → upper → LE → lower → TE. */
export function nacaProfile(code: string, n = 28): [number, number][] {
  const m = Number(code[0]) / 100;
  const p = Number(code[1]) / 10;
  const t = Number(code.slice(2)) / 100;
  const pts = (x: number) => {
    const yt = 5 * t * (0.2969 * Math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4);
    let yc = 0, dyc = 0;
    if (m > 0 && p > 0) {
      if (x < p) {
        yc = (m / (p * p)) * (2 * p * x - x * x);
        dyc = ((2 * m) / (p * p)) * (p - x);
      } else {
        yc = (m / ((1 - p) ** 2)) * (1 - 2 * p + 2 * p * x - x * x);
        dyc = ((2 * m) / ((1 - p) ** 2)) * (p - x);
      }
    }
    const th = Math.atan(dyc);
    return { xu: x - yt * Math.sin(th), yu: yc + yt * Math.cos(th), xl: x + yt * Math.sin(th), yl: yc - yt * Math.cos(th) };
  };
  const xs = Array.from({ length: n }, (_, i) => (1 - Math.cos((Math.PI * i) / (n - 1))) / 2); // cosine spacing 0..1
  const upper: [number, number][] = [];
  const lower: [number, number][] = [];
  for (const x of xs) {
    const q = pts(x);
    upper.push([q.xu, q.yu]);
    lower.push([q.xl, q.yl]);
  }
  // TE(upper) -> LE -> TE(lower), skip duplicate LE
  return [...upper.reverse(), ...lower.slice(1)];
}

export interface LoftSection {
  /** leading-edge point */
  le: Vector3;
  chord: number;
}

/**
 * Loft an airfoil between sections placed along z (wing) or y (fin when `vertical`).
 * Chord direction is -x (LE forward). Thickness direction is +y (or +z when vertical).
 */
export function loftAirfoil(sections: LoftSection[], airfoil: string, opts: { vertical?: boolean; flip?: boolean } = {}): BufferGeometry {
  const prof = nacaProfile(airfoil);
  const K = prof.length;
  const pos: number[] = [];
  for (const s of sections) {
    for (const [xc, yc] of prof) {
      if (opts.vertical) pos.push(s.le.x - xc * s.chord, s.le.y, s.le.z + yc * s.chord);
      else pos.push(s.le.x - xc * s.chord, s.le.y + yc * s.chord, s.le.z);
    }
  }
  const idx: number[] = [];
  const quad = (a: number, b: number, c: number, d: number) => {
    if (opts.flip) idx.push(a, c, b, a, d, c);
    else idx.push(a, b, c, a, c, d);
  };
  for (let s = 0; s < sections.length - 1; s++) {
    for (let k = 0; k < K - 1; k++) {
      const a = s * K + k, b = s * K + k + 1, c = (s + 1) * K + k + 1, d = (s + 1) * K + k;
      quad(a, b, c, d);
    }
  }
  // end caps (fan around the section centroid)
  for (const [si, dir] of [[0, 1], [sections.length - 1, -1]] as const) {
    const s = sections[si];
    const cx = s.le.x - 0.4 * s.chord;
    const ci = pos.length / 3;
    if (opts.vertical) pos.push(cx, s.le.y, s.le.z);
    else pos.push(cx, s.le.y, s.le.z);
    for (let k = 0; k < K - 1; k++) {
      const a = si * K + k, b = si * K + k + 1;
      const forward = (dir === 1) !== !!opts.flip;
      if (forward) idx.push(ci, b, a);
      else idx.push(ci, a, b);
    }
  }
  const g = new BufferGeometry();
  g.setAttribute('position', new Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

/** Fuselage body (without nose cone) as a lathe along x. */
export function fuselageGeometry(lengthM: number, diameterM: number, noseLengthM: number): BufferGeometry {
  const r = diameterM / 2;
  const x0 = -lengthM / 2;
  const x1 = lengthM / 2 - noseLengthM;
  const tailCone = 0.28 * lengthM;
  const pts: Vector2[] = [];
  const N = 40;
  for (let i = 0; i <= N; i++) {
    const x = x0 + ((x1 - x0) * i) / N;
    let rad = r;
    if (x < x0 + tailCone) {
      const u = (x - x0) / tailCone;
      rad = r * (0.32 + 0.68 * Math.sin((u * Math.PI) / 2));
    }
    pts.push(new Vector2(rad, x));
  }
  pts.unshift(new Vector2(0, x0));
  pts.push(new Vector2(0, x1));
  const g = new LatheGeometry(pts, 36);
  g.rotateZ(-Math.PI / 2); // lathe axis y -> x
  g.rotateX(Math.PI / 2);
  return g;
}

/** Ogive nose cone from its base (at x = baseX) forward. */
export function noseGeometry(baseX: number, lengthM: number, diameterM: number): BufferGeometry {
  const r = diameterM / 2;
  const pts: Vector2[] = [new Vector2(0, baseX)];
  const N = 24;
  for (let i = 0; i <= N; i++) {
    const u = i / N;
    pts.push(new Vector2(r * Math.sqrt(Math.max(0, 1 - u * u)), baseX + u * lengthM));
  }
  const g = new LatheGeometry(pts, 36);
  g.rotateZ(-Math.PI / 2);
  g.rotateX(Math.PI / 2);
  return g;
}

/** Tapered blade along +y (root at origin), chord along x, small thickness. */
export function bladeGeometry(radiusM: number): BufferGeometry {
  const c0 = radiusM * 0.16, c1 = radiusM * 0.07, t = radiusM * 0.025;
  const r0 = radiusM * 0.12;
  const v = [
    [-c0 / 2, r0, -t], [c0 / 2, r0, -t], [c1 / 2, radiusM, -t / 2], [-c1 / 2, radiusM, -t / 2],
    [-c0 / 2, r0, t], [c0 / 2, r0, t], [c1 / 2, radiusM, t / 2], [-c1 / 2, radiusM, t / 2],
  ];
  // blade lies in the y-z plane (prop disc), thin along x
  const pos = v.flatMap(([a, b, c]) => [c, b, a]);
  const idx = [0, 1, 2, 0, 2, 3, 4, 6, 5, 4, 7, 6, 0, 4, 5, 0, 5, 1, 1, 5, 6, 1, 6, 2, 2, 6, 7, 2, 7, 3, 3, 7, 4, 3, 4, 0];
  const g = new BufferGeometry();
  g.setAttribute('position', new Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}
