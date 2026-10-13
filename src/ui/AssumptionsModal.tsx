import { useMemo } from 'react';
import { useApp } from '../store/app';
import { useEvaluation } from '../store/results';

/** Global assumptions made while building the app (data placeholders, modelling choices). */
export const GLOBAL_ASSUMPTIONS: string[] = [
  'Tüm ürün değerleri yer tutucudur (estimate: true). 5 m ve 10 m ürünler 7 m referanstan türetildi: uzunluk ∝ s, kütle ∝ s^2,5, alan ∝ s², motor gücü ∝ s^2,75.',
  'Askeri varyantlarda aviyonik ×2,2, veri linki ×2,5, gövde ×1,1, kablo demeti ×1,3 kütle çarpanı uygulandı.',
  'Gövde ekseni: x ileri (burun), y yukarı, z sağ kanat. Sol kanat sağ kanadın x-y düzlemine göre aynasıdır.',
  'Menteşe ekseni varsayılanı: cruise’daki kanat açıklık yönünü 65° geriye süpürülmüş yöne, pervane eksenini (+x) dikeye (+y) taşıyan tek dönüş (designHingeForSweep(65)).',
  'Açıklık oranı (spanFraction) menteşeden uca, açıkta kalan yarı açıklık üzerinden ölçülür.',
  'Rotor göbeği, kanat çeyrek veter çizgisinin hubOffsetM kadar önündedir; hover’da bu ofset dikey yüksekliğe dönüşür.',
  'Varsayılan batarya konumu, yarım payload ile hover CG’sini rotor itki merkezine yaklaştıracak şekilde üretici tarafından seçildi (gövde sınırında kırpılır).',
  'Cruise’da 2 motor modunda kapanan motorlar foldableInCruise = true olan (dış) motorlardır.',
  'Senaryo rüzgâr hızı 10 m referans yüksekliğinde tanımlıdır; hover değerlendirmesi görev hover yüksekliğinde (mission.hoverHeightM) yapılır.',
  'Rüzgâr yönü, rüzgârın geldiği yön olarak burna göre verilir (0° = karşıdan, 90° = sağdan).',
  'Harici CSV ekseni: x ileri, y sağ, z yukarı (NED seçeneği ile z aşağı). Açılar derece; |açı| ≤ 2π ise radyan kabul edilir.',
  'Prosedürel model görselleştirme amaçlıdır; parça kütleleri ve konumları veri dosyasından gelir, mesh geometrisi kütleyi etkilemez.',
  'Görselde pervane dönüş hızı √(itki / nominal hover itkisi) ile orantılıdır (gerçek devir değildir).',
];

export function AssumptionsModal({ onClose }: { onClose: () => void }) {
  const issues = useApp((s) => s.issues);
  const ev = useEvaluation();
  const trace = useApp((s) => s.trace);
  const modelAssumptions = useMemo(() => {
    if (!ev) return [];
    const all = [...ev.ev.hover.assumptions, ...ev.ev.cruise.assumptions, ...ev.ev.mission.assumptions, ...ev.ev.windLimit.assumptions, ...ev.ev.massCruise.assumptions, ...(trace?.assumptions ?? [])];
    return [...new Set(all)];
  }, [ev, trace]);
  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/60 p-6" onClick={onClose}>
      <div className="max-h-full w-full max-w-3xl overflow-y-auto rounded-xl border border-slate-700 bg-slate-900 p-5 shadow-2xl" onClick={(e) => e.stopPropagation()}>
        <div className="mb-3 flex items-center justify-between">
          <h2 className="text-lg font-semibold text-slate-100">Varsayımlar ve veri doğrulama</h2>
          <button className="text-slate-400 hover:text-white" onClick={onClose}>✕</button>
        </div>
        <p className="mb-3 rounded border border-amber-500/40 bg-amber-500/10 p-2 text-xs text-amber-200">Tahmini mühendislik modeli; sertifikasyon hesabı değildir.</p>
        <h3 className="mb-1 text-sm font-semibold text-slate-200">Veri doğrulama (specs.json)</h3>
        {issues.length === 0 ? (
          <p className="mb-3 text-xs text-emerald-400">Şema ve tutarlılık kontrolleri temiz.</p>
        ) : (
          <ul className="mb-3 space-y-0.5 text-xs">
            {issues.map((i, k) => (
              <li key={k} className={i.level === 'error' ? 'text-red-400' : 'text-amber-300'}>[{i.level}] {i.productId ? `${i.productId}: ` : ''}{i.message}</li>
            ))}
          </ul>
        )}
        <h3 className="mb-1 text-sm font-semibold text-slate-200">Uygulama varsayımları</h3>
        <ul className="mb-3 list-disc space-y-0.5 pl-5 text-xs text-slate-300">{GLOBAL_ASSUMPTIONS.map((a) => <li key={a}>{a}</li>)}</ul>
        <h3 className="mb-1 text-sm font-semibold text-slate-200">Fizik modeli varsayımları (mevcut konfigürasyon)</h3>
        <ul className="list-disc space-y-0.5 pl-5 text-xs text-slate-300">{modelAssumptions.map((a) => <li key={a}>{a}</li>)}</ul>
      </div>
    </div>
  );
}
