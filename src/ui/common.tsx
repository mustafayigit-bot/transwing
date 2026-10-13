import { useState, type ReactNode } from 'react';
import type { Status } from '../store/results';

export function Section({ title, children, right, defaultOpen = true }: { title: string; children: ReactNode; right?: ReactNode; defaultOpen?: boolean }) {
  const [open, setOpen] = useState(defaultOpen);
  return (
    <section className="border-b border-slate-800/80">
      <header className="flex cursor-pointer select-none items-center justify-between px-3 py-2" onClick={() => setOpen(!open)}>
        <h3 className="text-[11px] font-semibold uppercase tracking-wider text-slate-400">
          <span className="mr-1 inline-block w-3 text-slate-500">{open ? '▾' : '▸'}</span>
          {title}
        </h3>
        <div onClick={(e) => e.stopPropagation()}>{right}</div>
      </header>
      {open && <div className="space-y-2 px-3 pb-3">{children}</div>}
    </section>
  );
}

export function Slider({ label, value, min, max, step, onChange, unit, format }: {
  label: string; value: number; min: number; max: number; step: number; onChange: (v: number) => void; unit?: string; format?: (v: number) => string;
}) {
  return (
    <label className="block">
      <div className="flex justify-between text-xs text-slate-300">
        <span>{label}</span>
        <span className="font-mono text-slate-200">{format ? format(value) : value}{unit ? ` ${unit}` : ''}</span>
      </div>
      <input type="range" className="w-full accent-sky-500" min={min} max={max} step={step} value={value} onChange={(e) => onChange(Number(e.target.value))} />
    </label>
  );
}

export function Toggle({ label, checked, onChange }: { label: string; checked: boolean; onChange: (v: boolean) => void }) {
  return (
    <label className="flex cursor-pointer items-center gap-2 text-xs text-slate-300">
      <input type="checkbox" className="accent-sky-500" checked={checked} onChange={(e) => onChange(e.target.checked)} />
      {label}
    </label>
  );
}

export function Btn({ children, onClick, active, title, className = '', disabled }: { children: ReactNode; onClick?: () => void; active?: boolean; title?: string; className?: string; disabled?: boolean }) {
  return (
    <button
      title={title}
      disabled={disabled}
      onClick={onClick}
      className={`rounded-md border px-2 py-1 text-xs transition-colors disabled:opacity-40 ${
        active ? 'border-sky-500 bg-sky-500/20 text-sky-200' : 'border-slate-700 bg-slate-800/70 text-slate-200 hover:border-slate-500 hover:bg-slate-700/70'
      } ${className}`}
    >
      {children}
    </button>
  );
}

export function Select<T extends string>({ value, onChange, options, className = '' }: { value: T; onChange: (v: T) => void; options: { value: T; label: string }[]; className?: string }) {
  return (
    <select
      value={value}
      onChange={(e) => onChange(e.target.value as T)}
      className={`w-full rounded-md border border-slate-700 bg-slate-800 px-2 py-1 text-xs text-slate-100 ${className}`}
    >
      {options.map((o) => (
        <option key={o.value} value={o.value}>{o.label}</option>
      ))}
    </select>
  );
}

export function EstimateBadge({ show = true }: { show?: boolean }) {
  if (!show) return null;
  return (
    <span title="Tahmini / yer tutucu değer — gerçek veriyle değiştirilmeli" className="ml-1 rounded border border-amber-500/50 bg-amber-500/15 px-1 text-[9px] font-semibold uppercase tracking-wide text-amber-300">
      tahmini
    </span>
  );
}

export const STATUS_STYLES: Record<Status, string> = {
  green: 'border-emerald-500/60 bg-emerald-500/10',
  yellow: 'border-amber-400/60 bg-amber-400/10',
  red: 'border-red-500/70 bg-red-500/15',
  neutral: 'border-slate-700 bg-slate-800/40',
};
export const STATUS_DOT: Record<Status, string> = {
  green: 'bg-emerald-400',
  yellow: 'bg-amber-400',
  red: 'bg-red-500 animate-pulse',
  neutral: 'bg-slate-500',
};

export function AssumptionList({ items, summary = 'Varsayımlar' }: { items: string[]; summary?: string }) {
  return (
    <details className="group mt-1 text-[11px] text-slate-400">
      <summary className="cursor-pointer select-none text-slate-500 hover:text-slate-300">{summary} ({items.length})</summary>
      <ul className="mt-1 list-disc space-y-0.5 pl-4">
        {items.map((a, i) => <li key={i}>{a}</li>)}
      </ul>
    </details>
  );
}

export function fmt(x: number, d = 1) {
  return Number.isFinite(x) ? x.toFixed(d) : '—';
}
