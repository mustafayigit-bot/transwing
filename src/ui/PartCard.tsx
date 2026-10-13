import { useApp } from '../store/app';
import { useCurrentProduct, usePayload } from '../store/results';
import { partMass } from '../physics/massProps';
import { GROUP_COLORS, GROUP_LABELS } from '../scene/materials';
import { Btn, EstimateBadge } from './common';

export function PartCard() {
  const product = useCurrentProduct();
  const selected = useApp((s) => s.selectedPart);
  const isolated = useApp((s) => s.isolatedPart);
  const payloadId = useApp((s) => s.payloadId);
  const payload = usePayload(product, payloadId);
  const part = product?.parts.find((p) => p.id === selected);
  if (!product || !part) return null;
  const m = partMass(part, payload);
  const rows: [string, React.ReactNode][] = [
    ['ID', <span className="font-mono">{part.id}</span>],
    ['Grup', <span className="flex items-center gap-1"><span className="h-2 w-2 rounded-sm" style={{ background: GROUP_COLORS[part.group] }} />{GROUP_LABELS[part.group]}</span>],
    ['Kütle', <span className="font-mono">{m.toFixed(2)} kg{part.group === 'Payload' && payload ? ` (${payload.name})` : ''}</span>],
    ['Konum (cruise)', <span className="font-mono">{part.positionM.map((v) => v.toFixed(3)).join(', ')} m</span>],
    ['Bağlantı', part.attach === 'body' ? 'Gövde' : part.attach === 'wingL' ? 'Sol kanat (menteşe ile döner)' : 'Sağ kanat (menteşe ile döner)'],
    ['Malzeme', part.material],
    ['BOM ref.', <span className="font-mono">{part.bomRef}</span>],
  ];
  return (
    <div className="pointer-events-auto absolute left-3 top-3 z-20 w-72 rounded-lg border border-slate-700 bg-slate-900/95 p-3 shadow-2xl backdrop-blur">
      <div className="mb-2 flex items-start justify-between gap-2">
        <div>
          <div className="text-sm font-semibold text-slate-100">{part.name}<EstimateBadge show={part.estimate} /></div>
          <div className="text-[10px] uppercase tracking-wider text-slate-500">{part.kind}</div>
        </div>
        <button className="text-slate-400 hover:text-white" onClick={() => useApp.setState({ selectedPart: null })}>✕</button>
      </div>
      <dl className="grid grid-cols-[90px_1fr] gap-y-1 text-[11px]">
        {rows.map(([k, v]) => (
          <div key={k} className="contents">
            <dt className="text-slate-500">{k}</dt>
            <dd className="text-slate-200">{v}</dd>
          </div>
        ))}
      </dl>
      <div className="mt-2 flex gap-1">
        <Btn active={isolated === part.id} onClick={() => useApp.setState({ isolatedPart: isolated === part.id ? null : part.id })}>◎ İzole et</Btn>
        <Btn onClick={() => useApp.getState().togglePart(part.id)}>Göster / gizle</Btn>
      </div>
    </div>
  );
}
