import { useEffect, useRef, useState } from 'react';
import { validateSpecs } from './data/schema';
import { runCurrentSimulation } from './lib/simClient';
import { decodeState, startUrlSync } from './lib/urlState';
import { ConfigViewer } from './scene/Viewer';
import { foldForMode, useApp } from './store/app';
import { useEvaluation } from './store/results';
import { AssumptionsModal } from './ui/AssumptionsModal';
import { BottomPanel } from './ui/BottomPanel';
import { ComparePage } from './ui/ComparePage';
import { EnvelopePage } from './ui/EnvelopePage';
import { LeftPanel } from './ui/LeftPanel';
import { MatrixPage } from './ui/MatrixPage';
import { PartCard } from './ui/PartCard';
import { RightPanel } from './ui/RightPanel';
import { TopBar } from './ui/TopBar';
import { STATUS_DOT } from './ui/common';

const DISCLAIMER = 'Tahmini mühendislik modeli; sertifikasyon hesabı değildir.';

async function loadSpecs() {
  try {
    const res = await fetch(`${import.meta.env.BASE_URL}specs.json`, { cache: 'no-cache' });
    if (!res.ok) throw new Error(`specs.json yüklenemedi (HTTP ${res.status})`);
    const { specs, issues } = validateSpecs(await res.json());
    if (!specs) {
      useApp.setState({ issues, loadError: 'specs.json şema doğrulamasından geçemedi.' });
      return;
    }
    const st = useApp.getState();
    const ref = specs.products.find((p) => p.id === specs.referenceProductId) ?? specs.products[0];
    useApp.setState({ specs, issues, productId: ref.id, payloadId: ref.payloadOptions[0]?.id ?? 'none', foldDeg: foldForMode(st.flightMode, ref.geometry.hinge.maxFoldDeg) });
    if (window.location.hash.length > 1) {
      const patch = decodeState(window.location.hash, useApp.getState());
      useApp.setState(patch);
      const product = specs.products.find((p) => p.id === useApp.getState().productId)!;
      if (!product.payloadOptions.some((o) => o.id === useApp.getState().payloadId)) useApp.setState({ payloadId: 'none' });
    }
  } catch (e) {
    useApp.setState({ loadError: e instanceof Error ? e.message : String(e) });
  }
}

function PresentationOverlay() {
  const evalRes = useEvaluation();
  const specs = useApp((s) => s.specs);
  const scenarioId = useApp((s) => s.scenarioId);
  const trace = useApp((s) => s.trace);
  const started = useRef(false);
  useEffect(() => {
    // loop the selected scenario: run it once, replay loops; otherwise loop the transition
    if (started.current) return;
    started.current = true;
    const s = useApp.getState();
    if (s.scenarioId) runCurrentSimulation();
    else useApp.setState({ transitionPlaying: true, trace: null });
    const onKey = (e: KeyboardEvent) => e.key === 'Escape' && useApp.setState({ presentation: false, transitionPlaying: false });
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, []);
  if (!evalRes) return null;
  return (
    <div className="pointer-events-none absolute inset-0 z-20 flex flex-col justify-between p-6">
      <div className="flex items-start justify-between">
        <div>
          <div className="text-3xl font-black tracking-[0.3em] text-sky-300 drop-shadow">SPECTER</div>
          <div className="text-lg text-slate-200">{evalRes.product.name}</div>
          {scenarioId && <div className="text-sm text-slate-400">{specs?.scenarios.find((x) => x.id === scenarioId)?.name}</div>}
        </div>
        <button className="pointer-events-auto rounded border border-slate-600 bg-slate-900/70 px-3 py-1 text-xs text-slate-200" onClick={() => useApp.setState({ presentation: false, transitionPlaying: false })}>
          Sunumdan çık (Esc)
        </button>
      </div>
      <div>
        <div className="flex flex-wrap gap-3">
          {evalRes.cards.map((c) => (
            <div key={c.id} className="rounded-lg bg-slate-900/70 px-3 py-2 backdrop-blur">
              <div className="flex items-center gap-1.5 text-[10px] uppercase tracking-wider text-slate-400"><span className={`h-2 w-2 rounded-full ${STATUS_DOT[c.status]}`} />{c.label}</div>
              <div className="font-mono text-xl text-white">{c.value}</div>
            </div>
          ))}
          {trace && (
            <div className={`rounded-lg px-3 py-2 ${trace.summary.positionLost ? 'bg-red-600/70' : 'bg-emerald-700/60'}`}>
              <div className="text-[10px] uppercase tracking-wider text-slate-200">Simülasyon</div>
              <div className="font-mono text-xl text-white">{trace.summary.positionLost ? 'Pozisyon kaybı' : `±${trace.summary.maxPosErrorM.toFixed(2)} m`}</div>
            </div>
          )}
        </div>
        <div className="mt-2 text-[11px] text-amber-300/80">{DISCLAIMER}</div>
      </div>
    </div>
  );
}

export default function App() {
  const specs = useApp((s) => s.specs);
  const loadError = useApp((s) => s.loadError);
  const issues = useApp((s) => s.issues);
  const tab = useApp((s) => s.tab);
  const presentation = useApp((s) => s.presentation);
  const [showAssumptions, setShowAssumptions] = useState(false);

  useEffect(() => {
    loadSpecs();
    return startUrlSync();
  }, []);

  if (loadError) {
    return (
      <div className="p-6 text-sm">
        <h1 className="mb-2 text-lg font-semibold text-red-400">Veri dosyası hatası</h1>
        <p className="mb-2 text-slate-300">{loadError}</p>
        <ul className="list-disc pl-5 text-xs text-red-300">{issues.map((i, k) => <li key={k}>{i.message}</li>)}</ul>
      </div>
    );
  }
  if (!specs) return <div className="flex h-screen items-center justify-center text-slate-400">specs.json yükleniyor…</div>;

  if (presentation) {
    return (
      <div className="relative h-screen w-screen">
        <ConfigViewer />
        <PresentationOverlay />
      </div>
    );
  }

  const errors = issues.filter((i) => i.level === 'error').length;
  const warnings = issues.length - errors;
  return (
    <div className="flex h-screen flex-col">
      <TopBar onAssumptions={() => setShowAssumptions(true)} />
      <div className="flex items-center gap-3 border-b border-amber-500/30 bg-amber-500/10 px-3 py-1 text-[11px] text-amber-200">
        <span>⚠ {DISCLAIMER}</span>
        <span className="text-amber-300/70">Tüm ürün verileri yer tutucudur (“tahmini” rozeti).</span>
        {issues.length > 0 && (
          <button className="ml-auto underline" onClick={() => setShowAssumptions(true)}>
            Veri doğrulama: {errors} hata, {warnings} uyarı
          </button>
        )}
      </div>
      <main className="flex min-h-0 flex-1">
        {tab === 'config' && (
          <>
            <LeftPanel />
            <div className="flex min-w-0 flex-1 flex-col">
              <div className="relative min-h-0 flex-1">
                <ConfigViewer />
                <PartCard />
              </div>
              <BottomPanel />
            </div>
            <RightPanel />
          </>
        )}
        {tab !== 'config' && (
          <div className="min-w-0 flex-1">
            {tab === 'compare' && <ComparePage />}
            {tab === 'envelope' && <EnvelopePage />}
            {tab === 'matrix' && <MatrixPage />}
          </div>
        )}
      </main>
      {showAssumptions && <AssumptionsModal onClose={() => setShowAssumptions(false)} />}
    </div>
  );
}
