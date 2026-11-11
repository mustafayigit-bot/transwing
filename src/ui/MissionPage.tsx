import { useEffect, useMemo, useRef, useState } from 'react';
import { CartesianGrid, Legend, Line, LineChart, ReferenceLine, ResponsiveContainer, Tooltip, XAxis, YAxis } from 'recharts';
import { download } from '../lib/exporters';
import { runAnalysis, runMission } from '../lib/simClient';
import { PHASES, type MissionTrace } from '../physics/fdm/mission';
import type { AnalysisReport, TrimPoint } from '../physics/fdm/trim';
import { MissionViewer } from '../scene/MissionViewer';
import { useApp } from '../store/app';
import { useCurrentProduct, usePayload } from '../store/results';
import { synthesizeTelemetry } from '../telemetry/sensors';
import type { MissionRequest } from '../workers/sim.worker';
import { AssumptionList, Btn, EstimateBadge, Section, Select, Slider, STATUS_STYLES, fmt } from './common';
import { TelemetryPanel } from './TelemetryPanel';

const COLORS = ['#38bdf8', '#f472b6', '#a3e635', '#fb923c', '#c084fc', '#facc15', '#2dd4bf', '#f87171'];
const PHASE_COLORS = ['#64748b', '#f59e0b', '#ec4899', '#22c55e', '#0ea5e9', '#14b8a6', '#a855f7', '#f59e0b', '#64748b', '#334155'];
const axisStyle = { fontSize: 10, fill: '#94a3b8' };

interface Preset {
  id: string;
  label: string;
  left: string[];
  right?: string[];
  leftUnit: string;
  rightUnit?: string;
}
const rot = (k: string, n: number) => Array.from({ length: n }, (_, i) => `${k}${i + 1}`);
const presets = (n: number): Preset[] => [
  { id: 'profile', label: 'İrtifa ve hız', left: ['alt', 'altCmd'], right: ['ias', 'gs'], leftUnit: 'm', rightUnit: 'm/s' },
  { id: 'att', label: 'Tutum (roll / pitch)', left: ['roll', 'rollCmd', 'pitch', 'pitchCmd'], right: ['aoa'], leftUnit: '°', rightUnit: '°' },
  { id: 'trans', label: 'Geçiş: katlanma, hız, itki', left: ['fold', 'foldCmd', 'ias'], right: ['thrustTot'], leftUnit: '° · m/s', rightUnit: 'N' },
  { id: 'thrust', label: 'Rotor itkileri', left: rot('thrust', n), leftUnit: 'N' },
  { id: 'rpm', label: 'Rotor devirleri', left: rot('rpm', n), right: rot('thr', n), leftUnit: 'rpm', rightUnit: '% gaz' },
  { id: 'cur', label: 'Motor akımları', left: rot('cur', n), leftUnit: 'A' },
  { id: 'batt', label: 'Batarya: gerilim, akım, SoC', left: ['battV'], right: ['battI', 'soc'], leftUnit: 'V', rightUnit: 'A · %' },
  { id: 'power', label: 'Güç ve enerji', left: ['power'], right: ['energyWh'], leftUnit: 'W', rightUnit: 'Wh' },
  { id: 'hinge', label: 'Menteşe momenti', left: ['hingeL', 'hingeR'], right: ['fold'], leftUnit: 'N·m', rightUnit: '°' },
  { id: 'surf', label: 'Kontrol yüzeyleri', left: ['ail', 'elev', 'rud'], leftUnit: '°' },
  { id: 'temp', label: 'Sıcaklıklar', left: [...rot('tmot', n), ...rot('tesc', n), 'cellT'], leftUnit: '°C' },
  { id: 'wind', label: 'Rüzgâr (K / D / yukarı)', left: ['windN', 'windE', 'windU'], leftUnit: 'm/s' },
  { id: 'nav', label: 'Güdüm: rota sapması', left: ['xtrack'], right: ['wpDist'], leftUnit: 'm', rightUnit: 'm' },
  { id: 'load', label: 'Yük faktörü ve CL', left: ['nz'], right: ['cl', 'stall'], leftUnit: 'g', rightUnit: '–' },
  { id: 'bend', label: 'Kanat kökü eğilme momenti', left: ['bendL', 'bendR'], right: ['nz'], leftUnit: 'N·m', rightUnit: 'g' },
  { id: 'vibe', label: 'Titreşim (VIBE)', left: ['vibeX', 'vibeY', 'vibeZ'], leftUnit: 'm/s²' },
  { id: 'imuA', label: 'IMU ivmeölçer (FRD)', left: ['accX', 'accY', 'accZ'], leftUnit: 'm/s²' },
  { id: 'imuG', label: 'IMU jiroskop', left: ['gyrX', 'gyrY', 'gyrZ'], leftUnit: 'rad/s' },
  { id: 'altS', label: 'İrtifa sensörleri: gerçek / GPS / baro', left: ['alt', 'gpsAlt', 'baroAlt'], right: ['gpsErrH'], leftUnit: 'm', rightUnit: 'm' },
  { id: 'asS', label: 'Hava hızı: gerçek / pitot', left: ['ias', 'arspd'], leftUnit: 'm/s' },
];

function MissionChart({ trace, preset, window, cursor }: { trace: MissionTrace; preset: Preset; window: [number, number]; cursor: number }) {
  const rows = useMemo(() => {
    const c = trace.ch;
    const n = c.t.length;
    let i0 = 0, i1 = n - 1;
    while (i0 < n - 1 && c.t[i0] < window[0]) i0++;
    while (i1 > i0 && c.t[i1] > window[1]) i1--;
    const step = Math.max(1, Math.floor((i1 - i0) / 1200));
    const keys = [...preset.left, ...(preset.right ?? [])];
    const out: Record<string, number>[] = [];
    for (let i = i0; i <= i1; i += step) {
      const r: Record<string, number> = { t: c.t[i] };
      for (const k of keys) r[k] = c[k]?.[i] ?? NaN;
      out.push(r);
    }
    return out;
  }, [trace, preset, window]);
  return (
    <ResponsiveContainer width="100%" height="100%">
      <LineChart data={rows} margin={{ top: 4, right: 4, bottom: 0, left: -6 }}>
        <CartesianGrid stroke="#1e293b" />
        <XAxis dataKey="t" type="number" domain={['dataMin', 'dataMax']} tick={axisStyle} tickFormatter={(v) => (Number(v) >= 600 ? `${(Number(v) / 60).toFixed(0)}dk` : `${Number(v).toFixed(0)}s`)} />
        <YAxis yAxisId="l" tick={axisStyle} width={52} label={{ value: preset.leftUnit, angle: -90, position: 'insideLeft', fontSize: 9, fill: '#64748b', offset: 12 }} />
        {preset.right && <YAxis yAxisId="r" orientation="right" tick={axisStyle} width={46} label={{ value: preset.rightUnit, angle: 90, position: 'insideRight', fontSize: 9, fill: '#64748b', offset: 12 }} />}
        <Tooltip contentStyle={{ background: '#0f172a', border: '1px solid #334155', fontSize: 11 }} labelFormatter={(v) => `t = ${Number(v).toFixed(1)} s`} formatter={(v: number) => (Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(2))} />
        <Legend wrapperStyle={{ fontSize: 10 }} />
        {preset.left.map((k, i) => <Line key={k} yAxisId="l" dataKey={k} stroke={COLORS[i % COLORS.length]} dot={false} strokeWidth={1.4} isAnimationActive={false} strokeDasharray={k.endsWith('Cmd') ? '4 3' : undefined} />)}
        {preset.right?.map((k, i) => <Line key={k} yAxisId="r" dataKey={k} stroke={COLORS[(i + preset.left.length) % COLORS.length]} dot={false} strokeWidth={1.4} isAnimationActive={false} />)}
        <ReferenceLine yAxisId="l" x={cursor} stroke="#e2e8f0" strokeDasharray="3 3" />
      </LineChart>
    </ResponsiveContainer>
  );
}

function MiniMap({ trace, index }: { trace: MissionTrace; index: number }) {
  const c = trace.ch;
  const { pts, box } = useMemo(() => {
    const step = Math.max(1, Math.floor(c.t.length / 1500));
    let minN = 0, maxN = 0, minE = 0, maxE = 0;
    const p: [number, number][] = [];
    for (let i = 0; i < c.t.length; i += step) {
      p.push([c.pe[i], c.pn[i]]);
      minN = Math.min(minN, c.pn[i]); maxN = Math.max(maxN, c.pn[i]); minE = Math.min(minE, c.pe[i]); maxE = Math.max(maxE, c.pe[i]);
    }
    for (const w of trace.config.waypointsM) { minN = Math.min(minN, w[0]); maxN = Math.max(maxN, w[0]); minE = Math.min(minE, w[1]); maxE = Math.max(maxE, w[1]); }
    const size = Math.max(maxN - minN, maxE - minE, 200) * 1.15;
    return { pts: p, box: { cx: (minE + maxE) / 2, cy: (minN + maxN) / 2, size } };
  }, [c, trace]);
  const X = (e: number) => 100 + ((e - box.cx) / box.size) * 200;
  const Y = (n: number) => 100 - ((n - box.cy) / box.size) * 200;
  return (
    <svg viewBox="0 0 200 200" className="h-full w-full rounded border border-slate-800 bg-slate-950">
      <polyline points={pts.map(([e, n]) => `${X(e)},${Y(n)}`).join(' ')} fill="none" stroke="#f472b6" strokeWidth={1} />
      {trace.config.waypointsM.map((w, i) => <circle key={i} cx={X(w[1])} cy={Y(w[0])} r={3} fill="none" stroke="#22d3ee" />)}
      <circle cx={X(0)} cy={Y(0)} r={3} fill="#facc15" />
      <circle cx={X(c.pe[index] ?? 0)} cy={Y(c.pn[index] ?? 0)} r={3.5} fill="#fff" />
      <text x={6} y={12} fontSize={8} fill="#64748b">K ↑ · {(box.size / 1000).toFixed(box.size > 5000 ? 0 : 1)} km</text>
    </svg>
  );
}

function trimRow(label: string, t: TrimPoint | null) {
  if (!t) return null;
  return (
    <tr className="border-t border-slate-800/70">
      <td className="py-0.5 pr-2 text-slate-400">{label}</td>
      <td className="text-right font-mono">{fmt(t.elecPowerW, 0)} W</td>
      <td className="text-right font-mono">{fmt(t.rpm, 0)}</td>
      <td className="text-right font-mono">{fmt(t.propEfficiency * 100, 0)} %</td>
      <td className="text-right font-mono">{fmt(t.liftToDrag, 1)}</td>
    </tr>
  );
}

function AnalysisPanel({ a }: { a: AnalysisReport }) {
  const d = a.derivatives;
  const curve = a.trim4.filter((t) => t.converged).map((t) => {
    const t2 = a.trim2.find((x) => x.speedMs === t.speedMs && x.converged);
    return { v: t.speedMs, p4: t.elecPowerW, p2: t2?.elecPowerW, ld: t.liftToDrag };
  });
  const der: [string, number][] = [
    ['CLα', d.CLalpha], ['Cmα', d.Cmalpha], ['Cmq', d.Cmq], ['CYβ', d.CYbeta], ['Clβ', d.Clbeta], ['Cnβ', d.Cnbeta], ['Clp', d.Clp], ['Cnr', d.Cnr],
    ['Clr', d.Clr], ['Cnp', d.Cnp], ['Clδa', d.CldeltaA], ['Cmδe', d.CmdeltaE], ['Cnδr', d.CndeltaR],
  ];
  const h = a.hover;
  return (
    <div className="space-y-3 text-[11px]">
      <div className="grid grid-cols-2 gap-x-3 gap-y-0.5 font-mono text-slate-300">
        <span className="text-slate-500">Kütle</span><span>{fmt(a.massKg)} kg</span>
        <span className="text-slate-500">VLM CLα (2π kesit)</span><span>{fmt(a.vlm.clAlphaPerRad, 2)} /rad</span>
        <span className="text-slate-500">Stall hızı / CLmax</span><span>{fmt(a.stallSpeedMs)} m/s / {fmt(a.clMax, 2)}</span>
        <span className="text-slate-500">(L/D)max</span><span>{fmt(a.calibration.maxLD, 2)} @ {a.calibration.speedMs} m/s</span>
        <span className="text-slate-500">Gövde+nasel CD·S</span><span>{fmt(a.calibration.cdaFrontM2, 4)} m²</span>
        <span className="text-slate-500">Statik marj</span><span className={d.staticMargin < 0.03 ? 'text-red-400' : ''}>{fmt(d.staticMargin * 100)} % MAC</span>
        <span className="text-slate-500">Nötr nokta / CG x</span><span>{fmt(d.neutralPointXM, 3)} / {fmt(d.cgXM, 3)} m</span>
        <span className="text-slate-500">Atalet Ixx/Iyy/Izz</span><span>{a.inertia.map((x) => x.toFixed(1)).join(' / ')}</span>
        <span className="text-slate-500">CG hover x</span><span>{fmt(a.cgHover[0], 3)} m</span>
      </div>
      <div>
        <div className="mb-1 font-semibold text-slate-200">Seyir trim ({fmt(a.cruise4.speedMs)} m/s): 4 motor ↔ 2 motor</div>
        <table className="w-full">
          <thead className="text-slate-500"><tr><th /><th className="text-right font-normal">Elektrik</th><th className="text-right font-normal">rpm</th><th className="text-right font-normal">η perv.</th><th className="text-right font-normal">L/D</th></tr></thead>
          <tbody>{trimRow('4 motor', a.cruise4)}{trimRow('2 motor', a.cruise2)}</tbody>
        </table>
        <div className="mt-1 text-slate-400">α = {fmt(a.cruise4.alphaDeg)}°, elevator = {fmt(a.cruise4.elevatorDeg)}°, itki = {fmt(a.cruise4.thrustN)} N, CL = {fmt(a.cruise4.cl, 2)}</div>
      </div>
      <div className="h-36">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={curve} margin={{ top: 4, right: 4, bottom: 0, left: -10 }}>
            <CartesianGrid stroke="#1e293b" />
            <XAxis dataKey="v" type="number" domain={['dataMin', 'dataMax']} tick={axisStyle} />
            <YAxis tick={axisStyle} width={50} />
            <Tooltip contentStyle={{ background: '#0f172a', border: '1px solid #334155', fontSize: 11 }} labelFormatter={(v) => `${v} m/s`} formatter={(v: number) => `${v.toFixed(0)} W`} />
            <Legend wrapperStyle={{ fontSize: 10 }} />
            <Line dataKey="p4" name="Elektrik gücü — 4 motor [W]" stroke="#f472b6" dot={false} isAnimationActive={false} />
            <Line dataKey="p2" name="2 motor [W]" stroke="#38bdf8" dot={false} isAnimationActive={false} connectNulls />
            <ReferenceLine x={a.stallSpeedMs} stroke="#ef4444" label={{ value: 'Vs', fontSize: 9, fill: '#ef4444' }} />
          </LineChart>
        </ResponsiveContainer>
      </div>
      <div>
        <div className="mb-1 font-semibold text-slate-200">Hover trim {h.converged ? '' : <span className="text-red-400">(trim sağlanamadı)</span>}</div>
        <div className="grid grid-cols-2 gap-x-3 gap-y-0.5 font-mono text-slate-300">
          <span className="text-slate-500">Rotor itkileri</span><span>{h.thrustN.map((t) => t.toFixed(0)).join(' / ')} N</span>
          <span className="text-slate-500">Devirler</span><span>{h.rpm.map((t) => t.toFixed(0)).join(' / ')}</span>
          <span className="text-slate-500">Motor akımları</span><span>{h.currentA.map((t) => t.toFixed(0)).join(' / ')} A</span>
          <span className="text-slate-500">Elektrik gücü</span><span>{fmt(h.elecPowerW / 1000, 2)} kW</span>
          <span className="text-slate-500">Download (gövde/kanat)</span><span>{fmt(h.downloadN, 0)} N</span>
          <span className="text-slate-500">Maks. rotor itkisi / marj</span><span className={h.thrustMargin < 0.2 ? 'text-amber-300' : ''}>{fmt(h.maxThrustN, 0)} N / {fmt(h.thrustMargin * 100, 0)} %</span>
          <span className="text-slate-500">Menteşe momenti</span><span>{fmt(h.hingeMomentNm[1])} N·m</span>
        </div>
      </div>
      <div>
        <div className="mb-1 font-semibold text-slate-200">Kararlılık ve kontrol türevleri (1/rad)</div>
        <div className="grid grid-cols-4 gap-x-2 gap-y-0.5 font-mono text-slate-300">
          {der.map(([k, v]) => <span key={k}><span className="text-slate-500">{k}</span> {v.toFixed(3)}</span>)}
        </div>
      </div>
      <div>
        <div className="mb-1 font-semibold text-slate-200">Dinamik modlar</div>
        <table className="w-full">
          <thead className="text-slate-500"><tr><th className="text-left font-normal">Mod</th><th className="text-right font-normal">f [Hz]</th><th className="text-right font-normal">ζ</th><th className="text-right font-normal">τ / T₂ [s]</th></tr></thead>
          <tbody>
            {a.modes.map((m) => (
              <tr key={m.name} className={`border-t border-slate-800/70 ${m.stable || Math.abs(m.real) < 2e-3 ? '' : 'text-amber-300'}`}>
                <td className="py-0.5">{m.name}{Math.abs(m.real) < 2e-3 ? ' (nötr)' : m.stable ? '' : ' (kararsız)'}</td>
                <td className="text-right font-mono">{m.imag > 1e-6 ? m.freqHz.toFixed(2) : '—'}</td>
                <td className="text-right font-mono">{m.imag > 1e-6 ? m.damping.toFixed(2) : '—'}</td>
                <td className="text-right font-mono">{Number.isFinite(m.timeConstS) ? (m.stable ? m.timeConstS : m.timeConstS * Math.LN2).toFixed(1) : '∞'}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      <AssumptionList items={a.assumptions} />
    </div>
  );
}

export function MissionPage() {
  const product = useCurrentProduct();
  const s = useApp();
  const payload = usePayload(product, s.payloadId);
  const def = product?.fdm?.missionDefault;
  const [cfg, setCfg] = useState(() => ({ hoverHeightM: 20, cruiseAltitudeM: 200, cruiseSpeedMs: 24, takeoffHoverS: 180, landingHoverS: 180 }));
  const [wps, setWps] = useState<[number, number][]>([[38, 0]]);
  useEffect(() => {
    if (!def) return;
    setCfg({ hoverHeightM: def.hoverHeightM, cruiseAltitudeM: def.cruiseAltitudeM, cruiseSpeedMs: def.cruiseSpeedMs, takeoffHoverS: def.takeoffHoverS, landingHoverS: def.landingHoverS });
    setWps(def.waypointsM.map(([n, e]) => [n / 1000, e / 1000]));
  }, [product?.id]); // eslint-disable-line react-hooks/exhaustive-deps

  const trace = s.mission && s.mission.productId === product?.id ? s.mission : null;
  const [t, setT] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(10);
  const [presetId, setPresetId] = useState('profile');
  const [presetId2, setPresetId2] = useState('batt');
  const [win, setWin] = useState<[number, number] | null>(null);
  const tEnd = trace ? trace.ch.t[trace.ch.t.length - 1] : 0;
  const raf = useRef(0);
  useEffect(() => {
    if (!playing || !trace) return;
    let last = performance.now();
    const tick = (now: number) => {
      const dt = (now - last) / 1000;
      last = now;
      setT((v) => {
        const nv = v + dt * speed;
        if (nv >= tEnd) { setPlaying(false); return tEnd; }
        return nv;
      });
      raf.current = requestAnimationFrame(tick);
    };
    raf.current = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf.current);
  }, [playing, speed, trace, tEnd]);

  const tel = useMemo(() => (trace && product ? synthesizeTelemetry(trace, product) : null), [trace, product]);
  const traceX = useMemo(() => (trace && tel ? { ...trace, ch: { ...trace.ch, ...tel.ch } } : trace), [trace, tel]);

  if (!product) return null;
  if (!product.fdm) return <div className="p-6 text-sm text-slate-300">Bu ürün için 6-DOF veri bloğu (<code>fdm</code>) tanımlı değil.</div>;
  const hz = trace ? 1 / Math.max(1e-6, trace.ch.t[1] - trace.ch.t[0]) : 10;
  const index = trace ? Math.min(trace.ch.t.length - 1, Math.round(t * hz)) : 0;
  const ps = presets(product.geometry.rotors.length);
  const window_: [number, number] = win ?? [0, tEnd];

  const buildRequest = (): MissionRequest => ({
    product, payload,
    config: {
      ...cfg, waypointsM: wps.map(([n, e]) => [n * 1000, e * 1000] as [number, number]), cruiseMotors: s.cruiseMotors, env: s.env, seed: s.sim.seed,
      gustStartS: s.sim.gustStartS, gustDurationS: s.sim.gustDurationS, initialSoc: s.batterySoc,
      cdaFront: trace?.cdaFront ?? (s.analysis?.productId === product.id ? s.analysis.calibration.cdaFrontM2 : undefined),
    },
  });
  const run = async () => {
    s.set({ missionRunning: true, missionProgress: 0, missionError: null });
    setPlaying(false);
    try {
      const res = await runMission(buildRequest(), (f, ph) => useApp.setState({ missionProgress: f, missionPhase: ph }));
      s.set({ mission: res, missionRunning: false });
      setT(0);
      setWin(null);
    } catch (e) {
      s.set({ missionRunning: false, missionError: e instanceof Error ? e.message : String(e) });
    }
  };
  const analyse = async () => {
    s.set({ analysisRunning: true });
    try {
      const a = await runAnalysis({ product, payload, altitudeM: s.env.altitudeM, isaDeltaK: s.env.isaDeltaK, cruiseSpeedMs: cfg.cruiseSpeedMs });
      s.set({ analysis: a, analysisRunning: false });
    } catch (e) {
      s.set({ analysisRunning: false, missionError: e instanceof Error ? e.message : String(e) });
    }
  };
  const exportCsv = () => {
    if (!trace) return;
    const keys = Object.keys(trace.ch);
    const lines = [keys.join(',')];
    for (let i = 0; i < trace.ch.t.length; i++) lines.push(keys.map((k) => Number(trace.ch[k][i]).toFixed(k === 't' ? 2 : 3)).join(','));
    download(`specter_${product.id}_gorev_telemetri.csv`, lines.join('\n') + '\n', 'text/csv');
  };
  const exportJson = () => {
    if (!trace) return;
    download(`specter_${product.id}_gorev_ozet.json`, JSON.stringify({ productId: trace.productId, config: trace.config, summary: trace.summary, phases: trace.phases, events: trace.events, assumptions: trace.assumptions }, null, 2), 'application/json');
  };
  const sm = trace?.summary;
  const bat = product.fdm.battery;
  const cards: { label: string; value: string; sub?: string; status: 'green' | 'yellow' | 'red' | 'neutral' }[] = sm
    ? [
        { label: 'Sonuç', value: sm.completed ? 'Tamamlandı' : 'Başarısız', sub: sm.outcome, status: sm.completed ? 'green' : 'red' },
        { label: 'Süre / mesafe', value: `${(sm.durationS / 60).toFixed(1)} dk`, sub: `${(sm.distanceM / 1000).toFixed(1)} km`, status: 'neutral' },
        { label: 'Enerji', value: `${sm.energyWh.toFixed(0)} Wh`, sub: `Seyir ort. ${sm.cruiseMeanPowerW.toFixed(0)} W · hover ort. ${(sm.hoverMeanPowerW / 1000).toFixed(2)} kW`, status: 'neutral' },
        { label: 'Son SoC', value: `${(sm.finalSoc * 100).toFixed(0)} %`, sub: `Rezerv sınırı %${(bat.reserveSoc * 100).toFixed(0)}`, status: sm.finalSoc >= bat.reserveSoc + 0.05 ? 'green' : sm.finalSoc >= bat.reserveSoc ? 'yellow' : 'red' },
        { label: 'Min. hücre gerilimi', value: `${sm.minCellV.toFixed(2)} V`, sub: `Uyarı ${bat.lowCellV} / kritik ${bat.criticalCellV} V`, status: sm.minCellV > bat.lowCellV ? 'green' : sm.minCellV > bat.criticalCellV ? 'yellow' : 'red' },
        { label: 'Maks. batarya akımı', value: `${sm.maxCRate.toFixed(2)} C`, sub: `${sm.maxBatteryCurrentA.toFixed(0)} A · sürekli ${bat.maxContinuousC}C üstü ${sm.timeAboveContinuousCS.toFixed(0)} s`, status: sm.maxCRate <= bat.maxContinuousC ? 'green' : sm.maxCRate <= bat.peakC ? 'yellow' : 'red' },
        { label: 'İleri geçiş', value: `${sm.transitionDurationS.toFixed(1)} s`, sub: `İrtifa kaybı ${sm.transitionAltLossM.toFixed(1)} m · geri geçiş ${sm.backTransitionDurationS.toFixed(1)} s`, status: sm.transitionAltLossM < 5 ? 'green' : sm.transitionAltLossM < 15 ? 'yellow' : 'red' },
        { label: 'Maks. menteşe momenti', value: `${sm.maxHingeMomentNm.toFixed(0)} N·m`, sub: `Eyleyici limiti ${product.fdm.hingeActuator.maxTorqueNm} N·m`, status: sm.maxHingeMomentNm <= 0.8 * product.fdm.hingeActuator.maxTorqueNm ? 'green' : sm.maxHingeMomentNm <= product.fdm.hingeActuator.maxTorqueNm ? 'yellow' : 'red' },
        { label: 'Motor / ESC sıcaklığı', value: `${sm.maxMotorTempC.toFixed(0)} / ${sm.maxEscTempC.toFixed(0)} °C`, sub: `Doyma ${sm.saturationTimeS.toFixed(1)} s · stall ${sm.stallTimeS.toFixed(1)} s`, status: sm.maxMotorTempC < product.fdm.motor.maxTempC ? 'green' : 'red' },
        { label: 'Seyir takibi', value: `±${sm.maxAltErrorM.toFixed(1)} m`, sub: `Rota sapması ${sm.maxCrossTrackM.toFixed(0)} m · maks. yatış ${sm.maxBankDeg.toFixed(0)}° · n ${sm.maxLoadFactor.toFixed(2)}`, status: 'neutral' },
      ]
    : [];

  return (
    <div className="flex h-full min-h-0">
      {/* ---------------- left: mission set-up ---------------- */}
      <aside className="flex w-80 shrink-0 flex-col overflow-y-auto border-r border-slate-800 bg-slate-950/90">
        <div className="px-3 py-2">
          <div className="text-sm font-semibold text-slate-100">Görev simülasyonu — 6-DOF</div>
          <div className="text-[11px] text-slate-400">{product.name} · {payload ? `${payload.name} (${payload.massKg} kg)` : 'payload yok'} · {s.cruiseMotors} motor seyir</div>
        </div>
        <Section title="Görev profili">
          <Slider label="Hover yüksekliği" value={cfg.hoverHeightM} min={5} max={60} step={1} unit="m" onChange={(v) => setCfg({ ...cfg, hoverHeightM: v })} />
          <Slider label="Seyir irtifası (yerden)" value={cfg.cruiseAltitudeM} min={60} max={1000} step={10} unit="m" onChange={(v) => setCfg({ ...cfg, cruiseAltitudeM: v })} />
          <Slider label="Seyir hızı" value={cfg.cruiseSpeedMs} min={Math.ceil(product.fdm.transition.fbwMinSpeedMs)} max={Math.round(product.fdm.transition.fbwMinSpeedMs * 1.8)} step={0.1} unit="m/s" format={(v) => v.toFixed(1)} onChange={(v) => setCfg({ ...cfg, cruiseSpeedMs: v })} />
          <Slider label="Kalkış hover süresi (tırmanma dahil)" value={cfg.takeoffHoverS} min={15} max={300} step={5} unit="s" onChange={(v) => setCfg({ ...cfg, takeoffHoverS: v })} />
          <Slider label="İniş hover süresi (alçalma dahil)" value={cfg.landingHoverS} min={20} max={300} step={5} unit="s" onChange={(v) => setCfg({ ...cfg, landingHoverS: v })} />
          <div className="text-xs text-slate-300">Waypoint’ler (kalkış noktasına göre, km) — sonunda kalkış noktasına dönülür</div>
          {wps.map((w, i) => (
            <div key={i} className="flex items-center gap-1 text-xs">
              <span className="w-8 text-slate-500">WP{i + 1}</span>
              <label className="flex items-center gap-1">K<input type="number" step={0.5} className="w-20 rounded border border-slate-700 bg-slate-800 px-1 py-0.5 font-mono" value={w[0]} onChange={(e) => setWps(wps.map((x, k) => (k === i ? [Number(e.target.value) || 0, x[1]] : x)))} /></label>
              <label className="flex items-center gap-1">D<input type="number" step={0.5} className="w-20 rounded border border-slate-700 bg-slate-800 px-1 py-0.5 font-mono" value={w[1]} onChange={(e) => setWps(wps.map((x, k) => (k === i ? [x[0], Number(e.target.value) || 0] : x)))} /></label>
              {wps.length > 1 && <button className="text-slate-400 hover:text-white" onClick={() => setWps(wps.filter((_, k) => k !== i))}>✕</button>}
            </div>
          ))}
          <Btn onClick={() => setWps([...wps, [wps[wps.length - 1][0], wps[wps.length - 1][1] + 2]])}>+ Waypoint</Btn>
          <div className="flex gap-1 text-xs">
            <span className="self-center text-slate-300">Seyir motoru</span>
            <Btn active={s.cruiseMotors === 4} onClick={() => s.set({ cruiseMotors: 4 })}>4 motor</Btn>
            <Btn active={s.cruiseMotors === 2} onClick={() => s.set({ cruiseMotors: 2 })}>2 motor</Btn>
          </div>
          <p className="text-[10px] text-slate-500">
            Ortam (rüzgâr {s.env.windSpeedMs} m/s @ {s.env.windFromDeg}° pusula, türbülans: {s.env.turbulence}, irtifa {s.env.altitudeM} m, ISA{s.env.isaDeltaK >= 0 ? '+' : ''}{s.env.isaDeltaK}), batarya doluluğu ve seed Konfigüratör sekmesindeki senaryo panelinden alınır.
          </p>
          <Btn className="w-full py-1.5 text-sm" active disabled={s.missionRunning} onClick={run}>
            {s.missionRunning ? `Simüle ediliyor… %${(s.missionProgress * 100).toFixed(0)} · ${PHASES[s.missionPhase]}` : '▶ Görevi simüle et'}
          </Btn>
          {s.missionRunning && <div className="h-1 rounded bg-slate-800"><div className="h-1 rounded bg-sky-500" style={{ width: `${s.missionProgress * 100}%` }} /></div>}
          {s.missionError && <p className="text-xs text-red-400">{s.missionError}</p>}
        </Section>
        {trace && (
          <>
            <Section title="Görev sonucu">
              <div className="grid grid-cols-2 gap-2">
                {cards.map((c) => (
                  <div key={c.label} className={`rounded-lg border p-2 ${STATUS_STYLES[c.status]}`}>
                    <div className="text-[10px] uppercase tracking-wide text-slate-400">{c.label}</div>
                    <div className="whitespace-nowrap font-mono text-sm font-semibold text-slate-50">{c.value}</div>
                    {c.sub && <div className="text-[10px] leading-tight text-slate-400">{c.sub}</div>}
                  </div>
                ))}
              </div>
              <div className="flex gap-1">
                <Btn onClick={exportCsv}>Telemetri CSV</Btn>
                <Btn onClick={exportJson}>Özet JSON</Btn>
              </div>
              <div className="text-[10px] text-slate-500">Hesap süresi {(trace.wallTimeMs / 1000).toFixed(1)} s · {trace.ch.t.length} örnek × {Object.keys(trace.ch).length} kanal<EstimateBadge /></div>
            </Section>
            <Section title="Faz bazında enerji">
              <table className="w-full text-[11px]">
                <thead className="text-slate-500"><tr><th className="text-left font-normal">Faz</th><th className="text-right font-normal">Süre</th><th className="text-right font-normal">Ort. güç</th><th className="text-right font-normal">Enerji</th></tr></thead>
                <tbody>
                  {trace.phases.filter((p) => p.tEnd > p.tStart).map((p, i) => (
                    <tr key={i} className="cursor-pointer border-t border-slate-800/70 hover:bg-slate-800/60" onClick={() => { setWin([Math.max(0, p.tStart - 3), p.tEnd + 3]); setT(p.tStart); }}>
                      <td className="py-0.5"><span className="mr-1 inline-block h-2 w-2 rounded-sm" style={{ background: PHASE_COLORS[p.phase] }} />{p.name}</td>
                      <td className="text-right font-mono">{(p.tEnd - p.tStart).toFixed(0)} s</td>
                      <td className="text-right font-mono">{p.meanPowerW.toFixed(0)} W</td>
                      <td className="text-right font-mono">{p.energyWh.toFixed(0)} Wh</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </Section>
            <Section title={`Olaylar (${trace.events.length})`}>
              <ul className="max-h-56 space-y-0.5 overflow-y-auto text-[11px]">
                {trace.events.map((e, i) => (
                  <li key={i} className={`cursor-pointer hover:underline ${e.level === 'alarm' ? 'text-red-400' : e.level === 'warn' ? 'text-amber-300' : 'text-slate-400'}`} onClick={() => setT(e.t)}>
                    <span className="font-mono">{e.t.toFixed(1)} s</span> — {e.text}
                  </li>
                ))}
              </ul>
              <AssumptionList items={trace.assumptions} />
            </Section>
          </>
        )}
      </aside>

      {/* ---------------- centre: replay + charts ---------------- */}
      <div className="flex min-w-0 flex-1 flex-col">
        {trace ? (
          <>
            <div className="relative min-h-0 flex-1">
              <MissionViewer product={product} payload={payload} trace={trace} index={index} />
              <div className="pointer-events-none absolute left-3 top-3 rounded-lg bg-slate-900/80 px-3 py-2 font-mono text-[11px] text-slate-200">
                <div className="mb-1 font-sans text-xs font-semibold" style={{ color: PHASE_COLORS[trace.ch.phase[index]] }}>{PHASES[trace.ch.phase[index]]}</div>
                <div>İrtifa {trace.ch.alt[index].toFixed(1)} m · IAS {trace.ch.ias[index].toFixed(1)} · GS {trace.ch.gs[index].toFixed(1)} m/s</div>
                <div>Roll {trace.ch.roll[index].toFixed(1)}° · Pitch {trace.ch.pitch[index].toFixed(1)}° · Baş {trace.ch.yaw[index].toFixed(0)}°</div>
                <div>Katlanma {trace.ch.fold[index].toFixed(0)}° · İtki {trace.ch.thrustTot[index].toFixed(0)} N</div>
                <div>{trace.ch.battV[index].toFixed(1)} V · {trace.ch.battI[index].toFixed(0)} A · SoC {trace.ch.soc[index].toFixed(0)} % · {(trace.ch.power[index] / 1000).toFixed(2)} kW</div>
              </div>
              <div className="absolute bottom-3 right-3 h-40 w-40"><MiniMap trace={trace} index={index} /></div>
            </div>
            <div className="shrink-0 border-t border-slate-800 bg-slate-950/95">
              <div className="flex items-center gap-2 px-3 py-1.5">
                <Btn onClick={() => { if (t >= tEnd) setT(0); setPlaying(!playing); }}>{playing ? '❚❚' : '▶'}</Btn>
                <input type="range" className="flex-1 accent-pink-400" min={0} max={tEnd} step={0.1} value={t} onChange={(e) => { setT(Number(e.target.value)); setPlaying(false); }} />
                <span className="w-32 text-right font-mono text-[11px] text-slate-300">{t.toFixed(1)} / {tEnd.toFixed(0)} s</span>
                <Select<string> className="!w-20" value={String(speed)} onChange={(v) => setSpeed(Number(v))} options={['1', '5', '10', '30', '120'].map((v) => ({ value: v, label: `${v}×` }))} />
              </div>
              {/* phase timeline */}
              <div className="mx-3 flex h-3 overflow-hidden rounded">
                {trace.phases.filter((p) => p.tEnd > p.tStart).map((p, i) => (
                  <div key={i} title={`${p.name}: ${(p.tEnd - p.tStart).toFixed(0)} s`} className="cursor-pointer" style={{ width: `${((p.tEnd - p.tStart) / tEnd) * 100}%`, background: PHASE_COLORS[p.phase], minWidth: 2 }} onClick={() => { setWin([Math.max(0, p.tStart - 3), p.tEnd + 3]); setT(p.tStart); }} />
                ))}
              </div>
              <div className="flex items-center gap-2 px-3 py-1 text-[11px] text-slate-400">
                Grafik penceresi: {win ? `${win[0].toFixed(0)}–${win[1].toFixed(0)} s` : 'tüm görev'}
                {win && <Btn onClick={() => setWin(null)}>Tümü</Btn>}
                <span className="ml-auto">Bir faza tıklayarak yakınlaşın</span>
              </div>
              <div className="grid h-56 grid-cols-2 gap-2 px-2 pb-2">
                {[[presetId, setPresetId] as const, [presetId2, setPresetId2] as const].map(([id, set], k) => (
                  <div key={k} className="flex min-h-0 flex-col">
                    <Select<string> value={id} onChange={set} options={ps.map((p) => ({ value: p.id, label: p.label }))} />
                    <div className="min-h-0 flex-1"><MissionChart trace={traceX ?? trace} preset={ps.find((p) => p.id === id) ?? ps[0]} window={window_} cursor={t} /></div>
                  </div>
                ))}
              </div>
            </div>
          </>
        ) : (
          <div className="flex h-full flex-col items-center justify-center gap-3 p-8 text-center text-sm text-slate-400">
            <p className="max-w-xl">
              Kalkış → hover → ileri geçiş → tırmanış → waypoint seyri → alçalma → geri geçiş → hover → iniş görevini 6 serbestlik dereceli
              uçuş dinamiği modeliyle baştan sona simüle eder. 76 km’lik görev tarayıcıda yaklaşık 15–30 saniyede hesaplanır.
            </p>
            <Btn active className="px-4 py-2 text-sm" disabled={s.missionRunning} onClick={run}>{s.missionRunning ? `Simüle ediliyor… %${(s.missionProgress * 100).toFixed(0)}` : '▶ Görevi simüle et'}</Btn>
          </div>
        )}
      </div>

      {/* ---------------- right: trim & stability ---------------- */}
      <aside className="flex w-80 shrink-0 flex-col overflow-y-auto border-l border-slate-800 bg-slate-950/90">
        {trace && tel && <TelemetryPanel product={product} trace={trace} tel={tel} buildRequest={buildRequest} />}
        <Section title="Trim, performans ve kararlılık" defaultOpen={!trace}>
          <Btn className="w-full" active disabled={s.analysisRunning} onClick={analyse}>{s.analysisRunning ? 'Analiz ediliyor…' : 'Analizi çalıştır (VLM + trim + türevler)'}</Btn>
          {s.analysis && s.analysis.productId === product.id ? <AnalysisPanel a={s.analysis} /> : <p className="text-[11px] text-slate-500">Seyir trim noktası, 4 ↔ 2 motor karşılaştırması, hover rotor yük dağılımı, kararlılık türevleri ve dinamik modlar.</p>}
        </Section>
      </aside>
    </div>
  );
}
