import { useMemo, useState } from 'react';
import type { PayloadOption, Product } from '../data/schema';
import { MiniViewer } from '../scene/Viewer';
import { useApp } from '../store/app';
import { evaluate } from '../store/results';
import { Btn, EstimateBadge } from './common';

/** Synthetic "max payload" option: heaviest allowed mass with the largest drag area of the options. */
function maxPayload(p: Product): PayloadOption {
  return {
    id: '_max', name: 'Maks. payload', massKg: p.mass.payloadMaxKg,
    dragAreaM2: Math.max(0, ...p.payloadOptions.map((o) => o.dragAreaM2)), variant: 'both', shape: 'box', estimate: true,
  };
}

function MatrixCard({ product, hover }: { product: Product; hover: boolean }) {
  const s = useApp();
  const ev = useMemo(
    () => evaluate({ product, payload: maxPayload(product), env: s.env, mission: s.specs!.mission, cruiseMotors: s.cruiseMotors, batterySoc: 1 }),
    [product, s.env, s.specs, s.cruiseMotors],
  );
  const mtow = product.mass.emptyKg + product.mass.batteryKg + product.mass.payloadMaxKg;
  const metrics: [string, string][] = [
    ['MTOW', `${mtow.toFixed(0)} kg`],
    ['Dayanım', `${(ev.mission.value.enduranceS / 60).toFixed(0)} dk`],
    ['Menzil', `${(ev.mission.value.rangeM / 1000).toFixed(0)} km`],
    ['Maks. rüzgâr', `${ev.windLimit.value.maxWindMs.toFixed(1)} m/s`],
  ];
  const active = s.productId === product.id;
  return (
    <div className={`flex flex-col overflow-hidden rounded-xl border ${active ? 'border-sky-500' : 'border-slate-800'} bg-slate-900/60`}>
      <div className="h-44">
        <MiniViewer product={product} payload={null} foldDeg={hover ? product.geometry.hinge.maxFoldDeg : 0} frameloop="demand" />
      </div>
      <div className="p-3">
        <div className="flex items-baseline justify-between">
          <div className="text-sm font-semibold text-slate-100">{product.name}</div>
          <div className="font-mono text-xs text-slate-400">{product.id}</div>
        </div>
        <div className="mt-2 grid grid-cols-2 gap-2">
          {metrics.map(([k, v]) => (
            <div key={k} className="rounded bg-slate-800/60 p-1.5">
              <div className="text-[10px] uppercase tracking-wide text-slate-500">{k}</div>
              <div className="font-mono text-sm text-slate-100">{v}<EstimateBadge /></div>
            </div>
          ))}
        </div>
        <Btn className="mt-2 w-full" active={active} onClick={() => { s.selectProduct(product.id); s.set({ tab: 'config' }); }}>Konfigüratörde aç →</Btn>
      </div>
    </div>
  );
}

export function MatrixPage() {
  const specs = useApp((s) => s.specs)!;
  const [hover, setHover] = useState(false);
  const sizes = [...new Set(specs.products.map((p) => p.sizeClass))];
  return (
    <div className="h-full overflow-auto p-4">
      <div className="mb-3 flex items-center gap-3">
        <h2 className="text-base font-semibold text-slate-100">Ürün matrisi</h2>
        <span className="text-xs text-slate-400">Metrikler maksimum payload ve mevcut ortam koşulları ile hesaplanır.</span>
        <div className="ml-auto flex gap-1">
          <Btn active={!hover} onClick={() => setHover(false)}>Cruise görünümü</Btn>
          <Btn active={hover} onClick={() => setHover(true)}>Hover görünümü</Btn>
        </div>
      </div>
      <div className="grid grid-cols-[auto_1fr_1fr] gap-4">
        <div />
        <div className="text-center text-xs font-semibold uppercase tracking-wider text-slate-400">Sivil</div>
        <div className="text-center text-xs font-semibold uppercase tracking-wider text-slate-400">Askeri</div>
        {sizes.map((size) => (
          <div key={size} className="contents">
            <div className="flex items-center text-sm font-bold text-slate-300 [writing-mode:vertical-rl] rotate-180">{size}</div>
            {(['civil', 'military'] as const).map((v) => {
              const p = specs.products.find((x) => x.sizeClass === size && x.variant === v);
              return p ? <MatrixCard key={p.id} product={p} hover={hover} /> : <div key={v} />;
            })}
          </div>
        ))}
      </div>
    </div>
  );
}
