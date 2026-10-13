import { useMemo, useRef, useState } from 'react';
import { CartesianGrid, Legend, Line, LineChart, ReferenceArea, ReferenceLine, ResponsiveContainer, Tooltip, XAxis, YAxis } from 'recharts';
import { parseCsvTrace } from '../lib/csvImport';
import { cruisePerformance } from '../physics/cruise';
import { hoverWindCheck } from '../physics/hover';
import { atmosphere } from '../physics/atmosphere';
import { diskAreaM2, hoverShaftPowerW } from '../physics/rotor';
import type { SimTrace } from '../physics/sim';
import { activeTrace, frameIndex, useApp } from '../store/app';
import { useCurrentProduct, useEvaluation, usePayload } from '../store/results';
import { Btn, Select, Toggle } from './common';

type ChartTab = 'timeseries' | 'powerSpeed' | 'marginWind';
type Metric = 'posErr' | 'alt' | 'tilt' | 'thrust' | 'power' | 'wind' | 'fold';

const METRICS: { value: Metric; label: string }[] = [
  { value: 'posErr', label: 'Konum sapması [m]' },
  { value: 'alt', label: 'İrtifa sapması [m]' },
  { value: 'tilt', label: 'Tutum (roll / pitch) [°]' },
  { value: 'thrust', label: 'Rotor itkileri [N]' },
  { value: 'power', label: 'Anlık güç [kW]' },
  { value: 'wind', label: 'Rüzgâr [m/s]' },
  { value: 'fold', label: 'Katlanma açısı [°]' },
];
const COLORS = ['#38bdf8', '#f472b6', '#a3e635', '#fb923c', '#c084fc', '#facc15', '#2dd4bf', '#f87171'];
const axisStyle = { fontSize: 10, fill: '#94a3b8' };

function seriesOf(tr: SimTrace, metric: Metric): { keys: string[]; rows: Record<string, number>[] } {
  const step = Math.max(1, Math.floor(tr.t.length / 700));
  const rows: Record<string, number>[] = [];
  const r = tr.refPos;
  const fwd = tr.mode !== 'hover' && tr.source === 'internal';
  let keys: string[] = [];
  for (let i = 0; i < tr.t.length; i += step) {
    const p = tr.pos[i];
    const row: Record<string, number> = { t: tr.t[i] };
    switch (metric) {
      case 'posErr':
        row.sapma = fwd ? Math.abs(p[1] - r[1]) : Math.hypot(p[0] - r[0], p[1] - r[1], p[2] - r[2]);
        keys = ['sapma'];
        break;
      case 'alt':
        row.irtifa = p[1] - r[1];
        keys = ['irtifa'];
        break;
      case 'tilt':
        row.roll = tr.att[i][0];
        row.pitch = tr.att[i][1];
        keys = ['roll', 'pitch'];
        break;
      case 'thrust':
        tr.thrustN[i].forEach((v, k) => (row[`T${k + 1}`] = v));
        keys = tr.thrustN[0]?.map((_, k) => `T${k + 1}`) ?? [];
        break;
      case 'power':
        row.guc = tr.powerW[i] / 1000;
        keys = ['guc'];
        break;
      case 'wind':
        row.wx = tr.windMs[i][0];
        row.wy = tr.windMs[i][1];
        row.wz = tr.windMs[i][2];
        keys = ['wx', 'wy', 'wz'];
        break;
      case 'fold':
        row.katlanma = tr.foldDeg[i];
        keys = ['katlanma'];
        break;
    }
    rows.push(row);
  }
  return { keys, rows };
}

function TimeSeries() {
  const s = useApp();
  const [metric, setMetric] = useState<Metric>('posErr');
  const internal = s.trace;
  const external = s.externalTrace;
  const a = useMemo(() => (internal ? seriesOf(internal, metric) : null), [internal, metric]);
  const b = useMemo(() => (external ? seriesOf(external, metric) : null), [external, metric]);
  const saturation = useMemo(() => {
    if (!internal) return [];
    const spans: [number, number][] = [];
    let start: number | null = null;
    internal.saturated.forEach((sat, i) => {
      const on = sat.some(Boolean);
      if (on && start === null) start = internal.t[i];
      if (!on && start !== null) { spans.push([start, internal.t[i]]); start = null; }
    });
    if (start !== null) spans.push([start, internal.t[internal.t.length - 1]]);
    return spans;
  }, [internal]);
  if (!a && !b) return <Empty text="Grafik için simülasyon çalıştırın veya harici CSV yükleyin." />;
  return (
    <div className="flex h-full flex-col">
      <div className="mb-1 flex items-center gap-2">
        <div className="w-56"><Select value={metric} onChange={setMetric} options={METRICS} /></div>
        {saturation.length > 0 && <span className="text-[10px] text-red-400">Kırmızı bant = rotor doyması</span>}
      </div>
      <div className="min-h-0 flex-1">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart margin={{ top: 4, right: 12, bottom: 0, left: -10 }}>
            <CartesianGrid stroke="#1e293b" />
            <XAxis dataKey="t" type="number" domain={['dataMin', 'dataMax']} tick={axisStyle} tickFormatter={(v) => `${Number(v).toFixed(0)}s`} allowDuplicatedCategory={false} />
            <YAxis tick={axisStyle} width={50} />
            <Tooltip contentStyle={{ background: '#0f172a', border: '1px solid #334155', fontSize: 11 }} labelFormatter={(v) => `t = ${Number(v).toFixed(2)} s`} formatter={(v: number) => v.toFixed(2)} />
            <Legend wrapperStyle={{ fontSize: 10 }} />
            {saturation.map(([x1, x2], k) => <ReferenceArea key={k} x1={x1} x2={x2} fill="#ef4444" fillOpacity={0.15} />)}
            {a?.keys.map((k, i) => <Line key={`a${k}`} data={a.rows} dataKey={k} name={`${k} (dahili)`} stroke={COLORS[i % COLORS.length]} dot={false} strokeWidth={1.5} isAnimationActive={false} />)}
            {b?.keys.map((k, i) => <Line key={`b${k}`} data={b.rows} dataKey={k} name={`${k} (harici)`} stroke={COLORS[(i + 3) % COLORS.length]} strokeDasharray="5 3" dot={false} strokeWidth={1.5} isAnimationActive={false} />)}
            <ReferenceLine x={s.replayT} stroke="#e2e8f0" strokeDasharray="3 3" />
          </LineChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}

function PowerSpeed() {
  const product = useCurrentProduct();
  const s = useApp();
  const payload = usePayload(product, s.payloadId);
  const data = useMemo(() => {
    if (!product) return null;
    const base = { product, payload, altitudeM: s.env.altitudeM, isaDeltaK: s.env.isaDeltaK };
    const c2 = cruisePerformance({ ...base, motors: 2 }).value;
    const c4 = cruisePerformance({ ...base, motors: 4 }).value;
    const map = new Map<number, Record<string, number>>();
    for (const [k, c] of [['m2', c2], ['m4', c4]] as const)
      for (const p of c.curve) {
        const key = Math.round(p.speedMs * 4) / 4;
        map.set(key, { ...(map.get(key) ?? { v: key }), [k]: p.electricPowerW / 1000 });
      }
    const rows = [...map.values()].sort((x, y) => x.v - y.v);
    return { rows, c2, c4 };
  }, [product, payload, s.env.altitudeM, s.env.isaDeltaK]);
  if (!data || !product) return null;
  const c = s.cruiseMotors === 2 ? data.c2 : data.c4;
  const avail2 = (2 * product.propulsion.motorMaxPowerW) / product.propulsion.motorEscEfficiency / 1000;
  const avail4 = (4 * product.propulsion.motorMaxPowerW) / product.propulsion.motorEscEfficiency / 1000;
  return (
    <ResponsiveContainer width="100%" height="100%">
      <LineChart data={data.rows} margin={{ top: 4, right: 12, bottom: 0, left: -10 }}>
        <CartesianGrid stroke="#1e293b" />
        <XAxis dataKey="v" type="number" domain={['dataMin', 'dataMax']} tick={axisStyle} tickFormatter={(v) => `${Number(v).toFixed(0)}`} label={{ value: 'Hava hızı [m/s]', position: 'insideBottomRight', offset: -2, fontSize: 10, fill: '#64748b' }} />
        <YAxis tick={axisStyle} width={50} domain={[0, (dataMax: number) => Math.min(dataMax, avail4 * 1.1)]} allowDataOverflow />
        <Tooltip contentStyle={{ background: '#0f172a', border: '1px solid #334155', fontSize: 11 }} labelFormatter={(v) => `V = ${Number(v).toFixed(1)} m/s`} formatter={(v: number) => `${v.toFixed(2)} kW`} />
        <Legend wrapperStyle={{ fontSize: 10 }} />
        <Line dataKey="m2" name="Gerekli elektrik gücü — 2 motor" stroke="#38bdf8" dot={false} isAnimationActive={false} connectNulls />
        <Line dataKey="m4" name="Gerekli elektrik gücü — 4 motor" stroke="#f472b6" dot={false} isAnimationActive={false} connectNulls />
        <ReferenceLine y={avail2} stroke="#38bdf8" strokeDasharray="4 4" label={{ value: 'Mevcut (2)', fontSize: 9, fill: '#38bdf8', position: 'insideTopLeft' }} />
        <ReferenceLine y={avail4} stroke="#f472b6" strokeDasharray="4 4" label={{ value: 'Mevcut (4)', fontSize: 9, fill: '#f472b6', position: 'insideTopLeft' }} />
        <ReferenceLine x={c.stallSpeedMs} stroke="#ef4444" label={{ value: 'Vs', fontSize: 10, fill: '#ef4444', position: 'top' }} />
        <ReferenceLine x={c.minPowerSpeedMs} stroke="#a3e635" strokeDasharray="2 2" label={{ value: 'Vmp', fontSize: 10, fill: '#a3e635', position: 'insideTop' }} />
        <ReferenceLine x={c.bestRangeSpeedMs} stroke="#facc15" strokeDasharray="2 2" label={{ value: 'Vbr', fontSize: 10, fill: '#facc15', position: 'insideTopRight' }} />
      </LineChart>
    </ResponsiveContainer>
  );
}

function MarginVsWind() {
  const product = useCurrentProduct();
  const s = useApp();
  const evalRes = useEvaluation();
  const payload = usePayload(product, s.payloadId);
  const rows = useMemo(() => {
    if (!product || !s.specs) return [];
    const inp = { product, payload, altitudeM: s.env.altitudeM, isaDeltaK: s.env.isaDeltaK, controlReserveFraction: s.specs.mission.controlReserveFraction };
    const rho = atmosphere(s.env.altitudeM, s.env.isaDeltaK).value.densityKgM3;
    const out: Record<string, number>[] = [];
    for (let v = 0; v <= 30; v += 0.5) {
      const row: Record<string, number> = { v };
      for (const [key, dir] of [['scen', s.env.windFromDeg], ['side', 90], ['head', 0]] as const) {
        const c = hoverWindCheck(inp, v, dir);
        const worst = Math.max(...c.rotorThrustN.map((t, i) => hoverShaftPowerW(Math.max(0, t), rho, diskAreaM2(product.geometry.rotors[i].diameterM), product.propulsion.figureOfMerit) / product.propulsion.motorMaxPowerW));
        row[key] = (1 - worst) * 100;
      }
      out.push(row);
    }
    return out;
  }, [product, payload, s.env.altitudeM, s.env.isaDeltaK, s.env.windFromDeg, s.specs]);
  return (
    <ResponsiveContainer width="100%" height="100%">
      <LineChart data={rows} margin={{ top: 4, right: 12, bottom: 0, left: -10 }}>
        <CartesianGrid stroke="#1e293b" />
        <XAxis dataKey="v" type="number" tick={axisStyle} domain={[0, 30]} label={{ value: 'Rüzgâr [m/s]', position: 'insideBottomRight', offset: -2, fontSize: 10, fill: '#64748b' }} />
        <YAxis tick={axisStyle} width={50} unit="%" domain={[-50, 100]} allowDataOverflow />
        <Tooltip contentStyle={{ background: '#0f172a', border: '1px solid #334155', fontSize: 11 }} labelFormatter={(v) => `${v} m/s`} formatter={(v: number) => `${v.toFixed(1)} %`} />
        <Legend wrapperStyle={{ fontSize: 10 }} />
        <ReferenceArea y1={-50} y2={0} fill="#ef4444" fillOpacity={0.12} />
        <Line dataKey="scen" name={`Senaryo yönü (${s.env.windFromDeg}°)`} stroke="#38bdf8" dot={false} isAnimationActive={false} />
        <Line dataKey="side" name="Yan (90°)" stroke="#f472b6" dot={false} strokeDasharray="4 2" isAnimationActive={false} />
        <Line dataKey="head" name="Karşıdan (0°)" stroke="#a3e635" dot={false} strokeDasharray="4 2" isAnimationActive={false} />
        {evalRes && <ReferenceLine x={evalRes.ev.effWind} stroke="#e2e8f0" label={{ value: 'mevcut', fontSize: 9, fill: '#e2e8f0', position: 'top' }} />}
        {evalRes && <ReferenceLine x={evalRes.ev.windLimit.value.maxWindMs} stroke="#ef4444" strokeDasharray="3 3" label={{ value: 'limit', fontSize: 9, fill: '#ef4444', position: 'insideTopRight' }} />}
      </LineChart>
    </ResponsiveContainer>
  );
}

function Empty({ text }: { text: string }) {
  return <div className="flex h-full items-center justify-center text-xs text-slate-500">{text}</div>;
}

function ReplayBar() {
  const s = useApp();
  const tr = activeTrace(s);
  const fileRef = useRef<HTMLInputElement>(null);
  const [ned, setNed] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const end = tr?.t[tr.t.length - 1] ?? 0;
  const i = tr ? frameIndex(tr, s.replayT) : 0;
  const onFile = async (f: File) => {
    try {
      const text = await f.text();
      const ext = parseCsvTrace(text, { nedFrame: ned, label: f.name });
      s.set({ externalTrace: ext, replaySource: 'external', replayT: 0, replayPlaying: true });
      setErr(null);
    } catch (e) {
      setErr(e instanceof Error ? e.message : String(e));
    }
  };
  return (
    <div className="flex flex-wrap items-center gap-2 border-b border-slate-800 px-3 py-1.5">
      <Btn disabled={!tr} onClick={() => s.set({ replayPlaying: !s.replayPlaying, replayT: s.replayT >= end ? 0 : s.replayT })}>
        {s.replayPlaying ? '❚❚ Duraklat' : '▶ Oynat'}
      </Btn>
      <Btn disabled={!tr} onClick={() => s.set({ replayT: 0 })}>⏮</Btn>
      <input type="range" className="min-w-[160px] flex-1 accent-pink-400" min={0} max={end || 1} step={0.02} value={s.replayT} disabled={!tr} onChange={(e) => s.set({ replayT: Number(e.target.value), replayPlaying: false })} />
      <span className="w-24 font-mono text-[11px] text-slate-300">{s.replayT.toFixed(2)} / {end.toFixed(1)} s</span>
      <Select<string> value={String(s.replaySpeed)} onChange={(v) => s.set({ replaySpeed: Number(v) })} className="!w-20" options={['0.25', '0.5', '1', '2', '4'].map((v) => ({ value: v, label: `${v}×` }))} />
      {s.trace && s.externalTrace && (
        <Select value={s.replaySource} onChange={(v) => s.set({ replaySource: v })} className="!w-36" options={[{ value: 'internal', label: 'Replay: dahili' }, { value: 'external', label: 'Replay: harici' }]} />
      )}
      {tr && tr.saturated[i]?.some(Boolean) && <span className="rounded bg-red-500/30 px-1.5 text-[10px] font-semibold text-red-200">ROTOR DOYMASI</span>}
      <div className="ml-auto flex items-center gap-2">
        <Toggle label="NED (z aşağı)" checked={ned} onChange={setNed} />
        <Btn onClick={() => fileRef.current?.click()} title="JSBSim / PX4 SITL / CFD: time,x,y,z,roll,pitch,yaw,thrust_1..n">⇪ Harici CSV</Btn>
        {s.externalTrace && <Btn onClick={() => s.set({ externalTrace: null, replaySource: 'internal' })}>✕ {s.externalTrace.label}</Btn>}
        <input ref={fileRef} type="file" accept=".csv,.txt" className="hidden" onChange={(e) => { const f = e.target.files?.[0]; if (f) onFile(f); e.target.value = ''; }} />
      </div>
      {err && <div className="w-full text-[11px] text-red-400">{err}</div>}
    </div>
  );
}

export function BottomPanel() {
  const [tab, setTab] = useState<ChartTab>('timeseries');
  const [open, setOpen] = useState(true);
  return (
    <div className={`shrink-0 border-t border-slate-800 bg-slate-950/95 ${open ? 'h-72' : ''}`}>
      <ReplayBar />
      <div className="flex items-center gap-1 px-3 py-1">
        {([['timeseries', 'Zaman serileri'], ['powerSpeed', 'Güç–hız eğrisi'], ['marginWind', 'Rüzgâra karşı güç marjı']] as const).map(([id, label]) => (
          <Btn key={id} active={tab === id && open} onClick={() => { setTab(id); setOpen(true); }}>{label}</Btn>
        ))}
        <button className="ml-auto text-xs text-slate-400 hover:text-white" onClick={() => setOpen(!open)}>{open ? '▾ Gizle' : '▴ Grafikler'}</button>
      </div>
      {open && (
        <div className="h-[calc(100%-4.5rem)] px-2 pb-2">
          {tab === 'timeseries' && <TimeSeries />}
          {tab === 'powerSpeed' && <PowerSpeed />}
          {tab === 'marginWind' && <MarginVsWind />}
        </div>
      )}
    </div>
  );
}
