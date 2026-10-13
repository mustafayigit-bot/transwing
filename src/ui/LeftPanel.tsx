import { useMemo } from 'react';
import { PART_GROUPS, type PartGroup } from '../data/schema';
import { aspectRatio, wingAreaM2 } from '../physics/airframe';
import { massProperties, partMass } from '../physics/massProps';
import { rotorHub } from '../kinematics/hinge';
import { foldForMode, useApp } from '../store/app';
import { useCurrentProduct, usePayload } from '../store/results';
import { GROUP_COLORS, GROUP_LABELS } from '../scene/materials';
import { captureScreenshot } from '../scene/Viewer';
import { downloadDataUrl } from '../lib/exporters';
import { Btn, EstimateBadge, Section, Slider, Toggle, fmt } from './common';

export function LeftPanel() {
  const product = useCurrentProduct();
  const s = useApp();
  const payload = usePayload(product, s.payloadId);
  const mp = useMemo(() => (product ? massProperties(product, payload, s.foldDeg).value : null), [product, payload, s.foldDeg]);
  const hoverInfo = useMemo(() => {
    if (!product) return null;
    const fold = product.geometry.hinge.maxFoldDeg;
    const m = massProperties(product, payload, fold).value;
    const hubs = product.geometry.rotors.map((r) => rotorHub(product, r, fold));
    const cx = hubs.reduce((a, h) => a + h.x, 0) / hubs.length;
    const c = massProperties(product, payload, 0).value;
    const xac = product.geometry.hinge.positionM[0];
    return { cgHover: m.cgM[0], thrustCentroid: cx, cgCruise: c.cgM[0], wingQc: xac };
  }, [product, payload]);
  if (!product || !mp || !hoverInfo) return null;
  const g = product.geometry;
  const maxFold = g.hinge.maxFoldDeg;
  const setMode = (m: 'hover' | 'transition' | 'cruise') => s.set({ flightMode: m, foldDeg: foldForMode(m, maxFold), transitionPlaying: false, trace: null });

  return (
    <aside className="flex h-full w-72 shrink-0 flex-col overflow-y-auto border-r border-slate-800 bg-slate-950/90">
      <div className="px-3 py-2">
        <div className="text-sm font-semibold text-slate-100">{product.name}</div>
        <div className="text-[11px] text-slate-400">
          {product.sizeClass} · {product.variant === 'civil' ? 'Sivil' : 'Askeri'}
          {product.derivedFrom && <> · {product.derivedFrom} referansından ölçeklendi (s = {product.scaleFactor})</>}
        </div>
      </div>
      <Section title="Uçuş modu ve katlanma">
        <div className="grid grid-cols-3 gap-1">
          <Btn active={s.flightMode === 'cruise'} onClick={() => setMode('cruise')}>Cruise</Btn>
          <Btn active={s.flightMode === 'transition'} onClick={() => setMode('transition')}>Transition</Btn>
          <Btn active={s.flightMode === 'hover'} onClick={() => setMode('hover')}>Hover</Btn>
        </div>
        <Slider label="Kanat katlanma açısı" value={Number(s.foldDeg.toFixed(1))} min={0} max={maxFold} step={0.5} unit="°" onChange={(v) => s.set({ foldDeg: v, transitionPlaying: false })} />
        <div className="flex items-center gap-2">
          <Btn active={s.transitionPlaying} onClick={() => s.set({ transitionPlaying: !s.transitionPlaying, trace: null })}>
            {s.transitionPlaying ? '■ Durdur' : '▶ Geçişi oynat'}
          </Btn>
          <div className="flex-1">
            <Slider label="Süre (yön başına)" value={s.transitionDurationS} min={1} max={20} step={0.5} unit="s" onChange={(v) => s.set({ transitionDurationS: v })} />
          </div>
        </div>
        <div className="text-[10px] text-slate-500">
          Menteşe ekseni: az {g.hinge.axisAzimuthDeg}° / el {g.hinge.axisElevationDeg}° · maks {maxFold}°<EstimateBadge show={g.hinge.estimate} />
        </div>
      </Section>

      <Section title="İnceleme araçları">
        <div className="grid grid-cols-4 gap-1">
          {(['front', 'side', 'top', 'iso'] as const).map((p) => (
            <Btn key={p} onClick={() => s.requestCamera(p)}>{{ front: 'Ön', side: 'Yan', top: 'Üst', iso: 'İzo' }[p]}</Btn>
          ))}
        </div>
        <Slider label="Exploded view" value={s.explode} min={0} max={1} step={0.01} format={(v) => v.toFixed(2)} onChange={(v) => s.set({ explode: v })} />
        <div className="grid grid-cols-2 gap-1">
          <Toggle label="Bağlantı çizgileri" checked={s.showConnectors} onChange={(v) => s.set({ showConnectors: v })} />
          <Toggle label="Wireframe" checked={s.wireframe} onChange={(v) => s.set({ wireframe: v })} />
          <Toggle label="CG işareti" checked={s.showCG} onChange={(v) => s.set({ showCG: v })} />
          <Toggle label="Kuvvet okları (√ ölçek)" checked={s.showForces} onChange={(v) => s.set({ showForces: v })} />
          <Toggle label="Rüzgâr alanı" checked={s.showWind} onChange={(v) => s.set({ showWind: v })} />
          <Toggle label="İnsan (1,80 m)" checked={s.showHuman} onChange={(v) => s.set({ showHuman: v })} />
          <Toggle label="Kesit düzlemi" checked={s.sectionEnabled} onChange={(v) => s.set({ sectionEnabled: v })} />
        </div>
        {s.sectionEnabled && (
          <Slider label="Kesit konumu (x)" value={s.sectionOffset} min={-g.spanM / 2} max={g.spanM / 2} step={0.01} unit="m" format={(v) => v.toFixed(2)} onChange={(v) => s.set({ sectionOffset: v })} />
        )}
        <Btn onClick={() => { const u = captureScreenshot(); if (u) downloadDataUrl(`specter_${product.id}.png`, u); }}>📷 PNG ekran görüntüsü</Btn>
      </Section>

      <Section title="Kütle ve ağırlık merkezi">
        <div className="grid grid-cols-2 gap-x-2 gap-y-0.5 font-mono text-[11px] text-slate-300">
          <span className="text-slate-500">Toplam kütle</span><span>{fmt(mp.totalKg)} kg</span>
          <span className="text-slate-500">CG (x, y, z)</span><span>{mp.cgM.map((v) => v.toFixed(3)).join(', ')} m</span>
          <span className="text-slate-500">Hover CG x</span><span>{fmt(hoverInfo.cgHover, 3)} m</span>
          <span className="text-slate-500">Rotor itki merkezi x</span><span>{fmt(hoverInfo.thrustCentroid, 3)} m</span>
          <span className="text-slate-500">Cruise CG − menteşe x</span><span>{fmt(hoverInfo.cgCruise - hoverInfo.wingQc, 3)} m</span>
          <span className="text-slate-500">Kanat alanı / AR</span><span>{fmt(wingAreaM2(product), 2)} m² / {fmt(aspectRatio(product), 1)}</span>
          <span className="text-slate-500">Boş / batarya</span><span>{product.mass.emptyKg} / {product.mass.batteryKg} kg<EstimateBadge show={product.mass.estimate} /></span>
        </div>
        <p className="text-[10px] text-slate-500">Katlanma CG’yi kaydırır: kanat grubu menteşe etrafında geriye döner. Hover’da CG itki merkezinin önündeyse ön rotorlar daha yüklüdür.</p>
      </Section>

      <Section title="Parça ağacı">
        <PartsTree />
      </Section>
    </aside>
  );
}

function PartsTree() {
  const product = useCurrentProduct()!;
  const s = useApp();
  const payload = usePayload(product, s.payloadId);
  const groups = useMemo(() => {
    const m = new Map<PartGroup, typeof product.parts>();
    for (const g of PART_GROUPS) m.set(g, []);
    for (const p of product.parts) m.get(p.group)!.push(p);
    return m;
  }, [product]);
  const toggleGroup = (g: PartGroup) => {
    const ids = groups.get(g)!.map((p) => p.id);
    const hidden = new Set(s.hiddenParts);
    const allHidden = ids.every((id) => hidden.has(id));
    ids.forEach((id) => (allHidden ? hidden.delete(id) : hidden.add(id)));
    s.set({ hiddenParts: [...hidden] });
  };
  return (
    <div className="space-y-1">
      {s.isolatedPart && <Btn onClick={() => s.set({ isolatedPart: null })} className="w-full">İzolasyonu kaldır</Btn>}
      {[...groups.entries()].filter(([, parts]) => parts.length).map(([g, parts]) => {
        const groupMass = parts.reduce((a, p) => a + partMass(p, payload), 0);
        return (
          <details key={g} open className="rounded border border-slate-800">
            <summary className="flex cursor-pointer items-center gap-2 px-2 py-1 text-xs text-slate-200">
              <span className="h-2.5 w-2.5 rounded-sm" style={{ background: GROUP_COLORS[g] }} />
              <span className="flex-1">{GROUP_LABELS[g]} <span className="text-slate-500">({g})</span></span>
              <span className="font-mono text-[10px] text-slate-400">{groupMass.toFixed(1)} kg</span>
              <button className="text-[10px] text-slate-400 hover:text-white" onClick={(e) => { e.preventDefault(); toggleGroup(g); }} title="Grubu göster/gizle">👁</button>
            </summary>
            <ul className="pb-1">
              {parts.map((p) => {
                const hidden = s.hiddenParts.includes(p.id);
                return (
                  <li
                    key={p.id}
                    onMouseEnter={() => s.set({ hoveredPart: p.id })}
                    onMouseLeave={() => s.set({ hoveredPart: null })}
                    className={`flex items-center gap-1 px-2 py-0.5 text-[11px] ${s.selectedPart === p.id ? 'bg-sky-500/20 text-sky-100' : 'text-slate-300 hover:bg-slate-800'}`}
                  >
                    <button className="flex-1 truncate text-left" onClick={() => s.set({ selectedPart: p.id })}>
                      <span className={hidden ? 'text-slate-600 line-through' : ''}>{p.name}</span>
                    </button>
                    <span className="font-mono text-[10px] text-slate-500">{partMass(p, payload).toFixed(1)}</span>
                    <button title="Göster/gizle" className="px-0.5 text-slate-400 hover:text-white" onClick={() => s.togglePart(p.id)}>{hidden ? '◌' : '●'}</button>
                    <button title="İzole et" className={`px-0.5 ${s.isolatedPart === p.id ? 'text-sky-300' : 'text-slate-400 hover:text-white'}`} onClick={() => s.set({ isolatedPart: s.isolatedPart === p.id ? null : p.id, selectedPart: p.id })}>◎</button>
                  </li>
                );
              })}
            </ul>
          </details>
        );
      })}
    </div>
  );
}
