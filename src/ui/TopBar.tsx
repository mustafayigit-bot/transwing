import { useState } from 'react';
import { cardsToCsv, download, downloadDataUrl, resultsJson, traceToCsv } from '../lib/exporters';
import { shareUrl } from '../lib/urlState';
import { activeTrace, useApp, type Tab } from '../store/app';
import { useEvaluation } from '../store/results';
import { captureScreenshot } from '../scene/Viewer';
import { Btn } from './common';

const TABS: { id: Tab; label: string }[] = [
  { id: 'config', label: 'Konfigüratör' },
  { id: 'compare', label: 'Karşılaştırma' },
  { id: 'envelope', label: 'Operasyonel zarf' },
  { id: 'matrix', label: 'Ürün matrisi' },
];

export function TopBar({ onAssumptions }: { onAssumptions: () => void }) {
  const specs = useApp((s) => s.specs);
  const productId = useApp((s) => s.productId);
  const tab = useApp((s) => s.tab);
  const selectProduct = useApp((s) => s.selectProduct);
  const evalRes = useEvaluation();
  const [copied, setCopied] = useState(false);
  const [menu, setMenu] = useState(false);
  if (!specs) return null;
  const sizes = [...new Set(specs.products.map((p) => p.sizeClass))];

  const copy = async () => {
    const url = shareUrl();
    try {
      await navigator.clipboard.writeText(url);
    } catch {
      window.prompt('Bağlantıyı kopyalayın:', url);
    }
    setCopied(true);
    setTimeout(() => setCopied(false), 1500);
  };
  const exportJson = () => {
    if (!evalRes) return;
    const s = useApp.getState();
    download(
      `specter_${s.productId}_sonuclar.json`,
      resultsJson({ productId: s.productId, payloadId: s.payloadId, scenarioId: s.scenarioId, env: s.env, flightMode: s.flightMode, cruiseMotors: s.cruiseMotors, batterySoc: s.batterySoc, cards: evalRes.cards.map(({ assumptions: _a, ...c }) => c) }, evalRes.ev, s.trace),
      'application/json',
    );
  };
  const exportCsv = () => {
    const s = useApp.getState();
    const tr = activeTrace(s);
    if (tr) download(`specter_${s.productId}_simulasyon.csv`, traceToCsv(tr), 'text/csv');
    if (evalRes) download(`specter_${s.productId}_metrikler.csv`, cardsToCsv(evalRes.cards), 'text/csv');
  };
  const exportPng = () => {
    const url = captureScreenshot() ?? (document.querySelector('canvas') as HTMLCanvasElement | null)?.toDataURL('image/png');
    if (url) downloadDataUrl(`specter_${useApp.getState().productId}.png`, url);
  };

  return (
    <header className="flex flex-wrap items-center gap-3 border-b border-slate-800 bg-slate-950/95 px-3 py-2">
      <div className="flex items-baseline gap-2">
        <span className="text-sm font-black tracking-[0.25em] text-sky-300">SPECTER</span>
        <span className="text-xs text-slate-400">Configurator</span>
      </div>
      <div className="flex items-center gap-1">
        {sizes.map((size) => (
          <div key={size} className="flex overflow-hidden rounded-md border border-slate-700">
            {specs.products.filter((p) => p.sizeClass === size).map((p) => (
              <button
                key={p.id}
                onClick={() => selectProduct(p.id)}
                title={p.name}
                className={`px-2 py-1 text-xs font-medium ${p.id === productId ? 'bg-sky-500 text-white' : 'bg-slate-800 text-slate-300 hover:bg-slate-700'}`}
              >
                {p.id}
              </button>
            ))}
          </div>
        ))}
      </div>
      <nav className="flex gap-1">
        {TABS.map((t) => (
          <Btn key={t.id} active={tab === t.id} onClick={() => useApp.setState({ tab: t.id })}>{t.label}</Btn>
        ))}
      </nav>
      <div className="ml-auto flex items-center gap-1">
        <Btn onClick={onAssumptions} title="Tüm model varsayımları ve veri doğrulama">Varsayımlar</Btn>
        <Btn onClick={copy} title="Ürün, payload ve senaryo durumunu içeren bağlantı">{copied ? 'Kopyalandı ✓' : 'Paylaş'}</Btn>
        <div className="relative">
          <Btn onClick={() => setMenu(!menu)}>Dışa aktar ▾</Btn>
          {menu && (
            <div className="absolute right-0 z-30 mt-1 w-44 rounded-md border border-slate-700 bg-slate-900 p-1 shadow-xl" onMouseLeave={() => setMenu(false)}>
              {[
                ['Sonuçlar (JSON)', exportJson],
                ['Sonuçlar + sim. (CSV)', exportCsv],
                ['Görünüm (PNG)', exportPng],
              ].map(([l, f]) => (
                <button key={l as string} className="block w-full rounded px-2 py-1 text-left text-xs text-slate-200 hover:bg-slate-800" onClick={() => { (f as () => void)(); setMenu(false); }}>
                  {l as string}
                </button>
              ))}
            </div>
          )}
        </div>
        <Btn onClick={() => useApp.setState({ presentation: true, tab: 'config' })} title="Paneller gizlenir, kamera döner, senaryo döngüde oynar">Sunum modu</Btn>
      </div>
    </header>
  );
}
