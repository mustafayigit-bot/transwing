import { useMemo, useRef, useState } from 'react';
import { Vector3 } from 'three';
import { payloadById } from '../physics/massProps';
import type { SimTrace } from '../physics/sim';
import { MiniViewer } from '../scene/Viewer';
import { currentSimInput, runSimulation } from '../lib/simClient';
import { useApp, type CameraPreset } from '../store/app';
import { buildCards, evaluate } from '../store/results';
import { Btn, EstimateBadge, Select, STATUS_DOT, fmt } from './common';

interface Row {
  label: string;
  unit: string;
  a: number;
  b: number;
  digits?: number;
  /** true when a larger value is better (for colouring Δ) */
  higherBetter?: boolean;
}

export function ComparePage() {
  const s = useApp();
  const specs = s.specs!;
  const bId = s.compareB.productId || specs.products.find((p) => p.id !== s.productId)?.id || s.productId;
  const pa = specs.products.find((p) => p.id === s.productId)!;
  const pb = specs.products.find((p) => p.id === bId)!;
  const bPayloadId = pb.payloadOptions.some((o) => o.id === s.compareB.payloadId) ? s.compareB.payloadId : pb.payloadOptions[0]?.id ?? 'none';
  const payA = payloadById(pa, s.payloadId);
  const payB = payloadById(pb, bPayloadId);
  const sync = useRef({ pos: new Vector3(), target: new Vector3(), valid: false });
  const [cam, setCam] = useState<{ preset: CameraPreset; n: number } | null>(null);
  const [sims, setSims] = useState<{ a: SimTrace; b: SimTrace } | null>(null);
  const [running, setRunning] = useState(false);

  const evA = useMemo(() => evaluate({ product: pa, payload: payA, env: s.env, mission: specs.mission, cruiseMotors: s.cruiseMotors, batterySoc: s.batterySoc }), [pa, payA, s.env, specs, s.cruiseMotors, s.batterySoc]);
  const evB = useMemo(() => evaluate({ product: pb, payload: payB, env: s.env, mission: specs.mission, cruiseMotors: s.cruiseMotors, batterySoc: s.batterySoc }), [pb, payB, s.env, specs, s.cruiseMotors, s.batterySoc]);
  const cardsA = buildCards(evA), cardsB = buildCards(evB);
  const foldFrac = Math.min(1, Math.max(0, s.foldDeg / pa.geometry.hinge.maxFoldDeg));

  const rows: Row[] = [
    { label: 'Kanat açıklığı', unit: 'm', a: pa.geometry.spanM, b: pb.geometry.spanM, digits: 2 },
    { label: 'Kalkış kütlesi', unit: 'kg', a: evA.hover.value.massKg, b: evB.hover.value.massKg },
    { label: 'Payload', unit: 'kg', a: payA?.massKg ?? 0, b: payB?.massKg ?? 0, higherBetter: true },
    { label: 'Hover elektrik gücü', unit: 'kW', a: evA.hover.value.totalElectricPowerW / 1000, b: evB.hover.value.totalElectricPowerW / 1000, digits: 2 },
    { label: 'T/W', unit: '', a: evA.hover.value.thrustToWeight, b: evB.hover.value.thrustToWeight, digits: 2, higherBetter: true },
    { label: 'Hover güç marjı', unit: '%', a: evA.hover.value.powerMargin * 100, b: evB.hover.value.powerMargin * 100, higherBetter: true },
    { label: 'Maks. hover rüzgârı', unit: 'm/s', a: evA.windLimit.value.maxWindMs, b: evB.windLimit.value.maxWindMs, higherBetter: true },
    { label: 'Gereken eğim', unit: '°', a: evA.windLimit.value.atScenario.tiltDeg, b: evB.windLimit.value.atScenario.tiltDeg },
    { label: 'Dayanım', unit: 'dk', a: evA.mission.value.enduranceS / 60, b: evB.mission.value.enduranceS / 60, higherBetter: true },
    { label: 'Menzil', unit: 'km', a: evA.mission.value.rangeM / 1000, b: evB.mission.value.rangeM / 1000, higherBetter: true },
    { label: 'Stall hızı', unit: 'm/s', a: evA.cruise.value.stallSpeedMs, b: evB.cruise.value.stallSpeedMs },
    { label: 'En iyi menzil hızı', unit: 'm/s', a: evA.cruise.value.bestRangeSpeedMs, b: evB.cruise.value.bestRangeSpeedMs },
    { label: 'Maks. hız', unit: 'm/s', a: evA.cruise.value.maxSpeedMs, b: evB.cruise.value.maxSpeedMs, higherBetter: true },
    { label: 'Batarya enerjisi', unit: 'Wh', a: pa.energy.batteryWh, b: pb.energy.batteryWh, digits: 0, higherBetter: true },
  ];
  if (sims) {
    rows.push(
      { label: 'Sim: maks. konum sapması', unit: 'm', a: sims.a.summary.maxPosErrorM, b: sims.b.summary.maxPosErrorM, digits: 2 },
      { label: 'Sim: doyma süresi', unit: 's', a: sims.a.summary.saturationTimeS, b: sims.b.summary.saturationTimeS, digits: 2 },
      { label: 'Sim: enerji', unit: 'Wh', a: sims.a.summary.energyWh, b: sims.b.summary.energyWh },
    );
  }
  const simulateBoth = async () => {
    const ia = currentSimInput(pa.id, s.payloadId), ib = currentSimInput(pb.id, bPayloadId);
    if (!ia || !ib) return;
    setRunning(true);
    try {
      const [a, b] = await Promise.all([runSimulation(ia), runSimulation(ib)]);
      setSims({ a, b });
    } finally {
      setRunning(false);
    }
  };

  const side = (label: string, product: typeof pa, payload: typeof payA, role: 'master' | 'slave', cards: typeof cardsA, picker: React.ReactNode, trace?: SimTrace) => (
    <div className="flex min-w-0 flex-1 flex-col border-r border-slate-800">
      <div className="flex items-center gap-2 border-b border-slate-800 px-3 py-2">
        <span className="rounded bg-slate-800 px-1.5 text-xs font-bold text-sky-300">{label}</span>
        {picker}
      </div>
      <div className="relative min-h-0 flex-1">
        <MiniViewer product={product} payload={payload} foldDeg={product.geometry.hinge.maxFoldDeg * foldFrac} sync={{ role, state: sync.current }} cameraRequest={role === 'master' ? cam : null} />
        <div className="pointer-events-none absolute bottom-2 left-2 flex flex-wrap gap-1">
          {cards.map((c) => (
            <span key={c.id} className="flex items-center gap-1 rounded bg-slate-900/80 px-1.5 py-0.5 text-[10px] text-slate-200">
              <span className={`h-1.5 w-1.5 rounded-full ${STATUS_DOT[c.status]}`} />{c.label}: <b className="font-mono">{c.value}</b>
            </span>
          ))}
          {trace && (
            <span className={`rounded px-1.5 py-0.5 text-[10px] ${trace.summary.positionLost ? 'bg-red-600/80' : 'bg-emerald-700/80'} text-white`}>
              Sim: {trace.summary.positionLost ? 'pozisyon kaybı' : 'pozisyon tutuldu'}
            </span>
          )}
        </div>
      </div>
    </div>
  );

  return (
    <div className="flex h-full flex-col">
      <div className="flex items-center gap-2 border-b border-slate-800 px-3 py-1.5 text-xs text-slate-400">
        Aynı senaryo: {s.scenarioId ? specs.scenarios.find((x) => x.id === s.scenarioId)?.name : 'özel'} · rüzgâr {s.env.windSpeedMs} m/s @ {s.env.windFromDeg}° · {s.env.altitudeM} m, ISA{s.env.isaDeltaK >= 0 ? '+' : ''}{s.env.isaDeltaK}
        <span className="ml-2">Senkron kamera (A yönetir):</span>
        {(['front', 'side', 'top', 'iso'] as const).map((p) => (
          <Btn key={p} onClick={() => setCam({ preset: p, n: (cam?.n ?? 0) + 1 })}>{{ front: 'Ön', side: 'Yan', top: 'Üst', iso: 'İzo' }[p]}</Btn>
        ))}
        <Btn className="ml-auto" onClick={simulateBoth} disabled={running} active>{running ? 'Simüle ediliyor…' : `▶ İkisini simüle et (${s.flightMode})`}</Btn>
      </div>
      <div className="flex min-h-0 flex-1">
        {side('A', pa, payA, 'master', cardsA,
          <>
            <div className="w-28"><Select value={pa.id} onChange={(v) => useApp.getState().selectProduct(v)} options={specs.products.map((p) => ({ value: p.id, label: p.id }))} /></div>
            <div className="w-44"><Select value={s.payloadId} onChange={(v) => s.set({ payloadId: v })} options={[{ value: 'none', label: 'Payload yok' }, ...pa.payloadOptions.map((o) => ({ value: o.id, label: o.name }))]} /></div>
          </>, sims?.a)}
        {side('B', pb, payB, 'slave', cardsB,
          <>
            <div className="w-28"><Select value={pb.id} onChange={(v) => { s.set({ compareB: { productId: v, payloadId: bPayloadId } }); setSims(null); }} options={specs.products.map((p) => ({ value: p.id, label: p.id }))} /></div>
            <div className="w-44"><Select value={bPayloadId} onChange={(v) => { s.set({ compareB: { productId: pb.id, payloadId: v } }); setSims(null); }} options={[{ value: 'none', label: 'Payload yok' }, ...pb.payloadOptions.map((o) => ({ value: o.id, label: o.name }))]} /></div>
          </>, sims?.b)}
      </div>
      <div className="max-h-[40%] shrink-0 overflow-y-auto border-t border-slate-800">
        <table className="w-full text-xs">
          <thead className="sticky top-0 bg-slate-900 text-slate-400">
            <tr>
              <th className="px-3 py-1 text-left font-medium">Metrik</th>
              <th className="px-3 py-1 text-right font-medium">A: {pa.id}</th>
              <th className="px-3 py-1 text-right font-medium">B: {pb.id}</th>
              <th className="px-3 py-1 text-right font-medium">Δ (B − A)</th>
              <th className="px-3 py-1 text-right font-medium">% fark</th>
            </tr>
          </thead>
          <tbody>
            {rows.map((r) => {
              const d = r.b - r.a;
              const p = r.a !== 0 ? (d / Math.abs(r.a)) * 100 : NaN;
              const good = r.higherBetter === undefined ? null : r.higherBetter ? d > 0 : d < 0;
              const color = Math.abs(d) < 1e-9 || good === null ? 'text-slate-300' : good ? 'text-emerald-400' : 'text-red-400';
              return (
                <tr key={r.label} className="border-t border-slate-800/70">
                  <td className="px-3 py-1 text-slate-300">{r.label}<EstimateBadge /></td>
                  <td className="px-3 py-1 text-right font-mono">{fmt(r.a, r.digits ?? 1)} {r.unit}</td>
                  <td className="px-3 py-1 text-right font-mono">{fmt(r.b, r.digits ?? 1)} {r.unit}</td>
                  <td className={`px-3 py-1 text-right font-mono ${color}`}>{d >= 0 ? '+' : ''}{fmt(d, r.digits ?? 1)}</td>
                  <td className={`px-3 py-1 text-right font-mono ${color}`}>{Number.isFinite(p) ? `${p >= 0 ? '+' : ''}${p.toFixed(1)} %` : '—'}</td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </div>
  );
}
