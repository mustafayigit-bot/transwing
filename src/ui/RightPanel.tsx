import { runCurrentSimulation } from '../lib/simClient';
import { foldForMode, useApp } from '../store/app';
import { useCurrentProduct, useEvaluation } from '../store/results';
import type { TurbulenceLevel } from '../physics/types';
import { AssumptionList, Btn, EstimateBadge, Section, Select, Slider, STATUS_DOT, STATUS_STYLES, Toggle, fmt } from './common';

const TURB: { value: TurbulenceLevel; label: string }[] = [
  { value: 'none', label: 'Yok' },
  { value: 'light', label: 'Hafif' },
  { value: 'moderate', label: 'Orta' },
  { value: 'severe', label: 'Şiddetli' },
];

export function RightPanel() {
  const product = useCurrentProduct();
  const s = useApp();
  const evalRes = useEvaluation();
  if (!product || !s.specs) return null;
  const maxFold = product.geometry.hinge.maxFoldDeg;
  const payloadOpts = [{ value: 'none', label: 'Payload yok' }, ...product.payloadOptions.map((o) => ({ value: o.id, label: `${o.name} (${o.massKg} kg)` }))];
  const tr = s.trace;

  return (
    <aside className="flex h-full w-80 shrink-0 flex-col overflow-y-auto border-l border-slate-800 bg-slate-950/90">
      <Section title="Hazır senaryolar">
        <Select
          value={s.scenarioId ?? ''}
          onChange={(id) => { const sc = s.specs!.scenarios.find((x) => x.id === id); if (sc) s.applyScenario(sc); }}
          options={[{ value: '', label: '— Özel senaryo —' }, ...s.specs.scenarios.map((x) => ({ value: x.id, label: x.name }))]}
        />
        {s.scenarioId && <p className="text-[11px] text-slate-400">{s.specs.scenarios.find((x) => x.id === s.scenarioId)?.description}</p>}
      </Section>

      <Section title="Ortam">
        <Slider label="İrtifa (MSL)" value={s.env.altitudeM} min={0} max={4500} step={50} unit="m" onChange={(v) => s.setEnv({ altitudeM: v })} />
        <Slider label="ISA sapması" value={s.env.isaDeltaK} min={-30} max={40} step={1} unit="K" onChange={(v) => s.setEnv({ isaDeltaK: v })} />
        <Slider label="Rüzgâr hızı (10 m ref.)" value={s.env.windSpeedMs} min={0} max={30} step={0.5} unit="m/s" onChange={(v) => s.setEnv({ windSpeedMs: v })} />
        <Slider label="Rüzgâr yönü (geldiği, buruna göre)" value={s.env.windFromDeg} min={0} max={359} step={1} unit="°" onChange={(v) => s.setEnv({ windFromDeg: v })} />
        <div className="grid grid-cols-2 items-center gap-2">
          <span className="text-xs text-slate-300">Türbülans (Dryden)</span>
          <Select value={s.env.turbulence} onChange={(v) => s.setEnv({ turbulence: v })} options={TURB} />
        </div>
        <Slider label="Gust genliği (1-cosine)" value={s.env.gustAmplitudeMs} min={0} max={15} step={0.5} unit="m/s" onChange={(v) => s.setEnv({ gustAmplitudeMs: v })} />
        <Toggle label="Logaritmik rüzgâr profili (z₀ = 0,03 m)" checked={s.env.logProfile} onChange={(v) => s.setEnv({ logProfile: v })} />
      </Section>

      <Section title="Konfigürasyon">
        <div className="grid grid-cols-[90px_1fr] items-center gap-2 text-xs text-slate-300">
          <span>Payload</span>
          <Select value={s.payloadId} onChange={(v) => s.set({ payloadId: v })} options={payloadOpts} />
          <span>Uçuş modu</span>
          <Select value={s.flightMode} onChange={(v) => s.set({ flightMode: v, foldDeg: foldForMode(v, maxFold), trace: null })} options={[{ value: 'hover', label: 'Hover' }, { value: 'transition', label: 'Transition' }, { value: 'cruise', label: 'Cruise' }]} />
          <span>Cruise motoru</span>
          <div className="flex gap-1">
            <Btn active={s.cruiseMotors === 2} onClick={() => s.set({ cruiseMotors: 2 })}>2 motor</Btn>
            <Btn active={s.cruiseMotors === 4} onClick={() => s.set({ cruiseMotors: 4 })}>4 motor</Btn>
          </div>
        </div>
        <Slider label="Batarya doluluğu" value={s.batterySoc} min={0.1} max={1} step={0.01} format={(v) => `${(v * 100).toFixed(0)} %`} onChange={(v) => s.set({ batterySoc: v })} />
      </Section>

      <Section title="Sonuçlar">
        {evalRes && (
          <div className="grid grid-cols-2 gap-2">
            {evalRes.cards.map((c) => (
              <div key={c.id} className={`rounded-lg border p-2 ${STATUS_STYLES[c.status]}`}>
                <div className="flex items-center gap-1.5 text-[10px] uppercase tracking-wide text-slate-400">
                  <span className={`h-2 w-2 rounded-full ${STATUS_DOT[c.status]}`} />
                  {c.label}
                </div>
                <div className="mt-0.5 whitespace-nowrap font-mono text-base font-semibold text-slate-50">{c.value}<EstimateBadge /></div>
                {c.sub && <div className="text-[10px] leading-tight text-slate-400">{c.sub}</div>}
                <AssumptionList items={c.assumptions} />
              </div>
            ))}
          </div>
        )}
      </Section>

      <Section title="Zaman alanı simülasyonu">
        <div className="grid grid-cols-2 gap-2">
          <Slider label="Süre" value={s.sim.durationS} min={5} max={120} step={5} unit="s" onChange={(v) => s.setSim({ durationS: v })} />
          <label className="block text-xs text-slate-300">
            Seed
            <input type="number" className="mt-1 w-full rounded border border-slate-700 bg-slate-800 px-2 py-0.5 font-mono text-xs" value={s.sim.seed} onChange={(e) => s.setSim({ seed: Math.round(Number(e.target.value) || 0) })} />
          </label>
          <Slider label="Gust başlangıcı" value={s.sim.gustStartS} min={0} max={Math.max(1, s.sim.durationS - 1)} step={0.5} unit="s" onChange={(v) => s.setSim({ gustStartS: v })} />
          <Slider label="Gust süresi" value={s.sim.gustDurationS} min={0.5} max={10} step={0.5} unit="s" onChange={(v) => s.setSim({ gustDurationS: v })} />
        </div>
        <Btn className="w-full py-1.5 text-sm" onClick={() => runCurrentSimulation()} disabled={s.simRunning} active>
          {s.simRunning ? 'Simülasyon çalışıyor (Web Worker)…' : `▶ Simüle et (${s.flightMode}, RK4 dt = 0,01 s)`}
        </Btn>
        {s.simError && <p className="text-xs text-red-400">{s.simError}</p>}
        {tr && (
          <div className={`rounded-lg border p-2 text-[11px] ${STATUS_STYLES[tr.summary.positionLost ? 'red' : tr.summary.saturationEvents ? 'yellow' : 'green']}`}>
            <div className="mb-1 font-semibold text-slate-100">{tr.summary.positionLost ? (tr.summary.crashed ? '✖ Yere çarpma' : '✖ Pozisyon kaybı') : '✓ Pozisyon tutuldu'}</div>
            <div className="grid grid-cols-2 gap-x-2 font-mono text-slate-300">
              <span className="text-slate-500">Maks. sapma</span><span>{fmt(tr.summary.maxPosErrorM, 2)} m</span>
              <span className="text-slate-500">RMS sapma</span><span>{fmt(tr.summary.rmsPosErrorM, 2)} m</span>
              <span className="text-slate-500">Maks. eğim</span><span>{fmt(tr.summary.maxTiltDeg, 1)}°</span>
              <span className="text-slate-500">Doyma</span><span>{tr.summary.saturationEvents} olay / {fmt(tr.summary.saturationTimeS, 2)} s</span>
              <span className="text-slate-500">Tepe güç</span><span>{fmt(tr.summary.peakPowerW / 1000, 1)} kW</span>
              <span className="text-slate-500">Enerji</span><span>{fmt(tr.summary.energyWh, 1)} Wh</span>
            </div>
            <AssumptionList items={tr.assumptions} />
          </div>
        )}
      </Section>
    </aside>
  );
}
