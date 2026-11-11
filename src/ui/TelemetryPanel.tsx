import { useMemo, useState } from 'react';
import { Area, CartesianGrid, ComposedChart, Line, ResponsiveContainer, Tooltip, XAxis, YAxis } from 'recharts';
import type { Product } from '../data/schema';
import { download } from '../lib/exporters';
import { runMonteCarlo } from '../lib/simClient';
import type { MissionTrace } from '../physics/fdm/mission';
import { useApp } from '../store/app';
import { encodeDataFlash } from '../telemetry/dataflash';
import { encodeTlog } from '../telemetry/mavlink';
import { DEFAULT_TELEMETRY, synthesizeTelemetry, type Telemetry } from '../telemetry/sensors';
import { evaluateAlarms, MC_BAND_KEYS, phaseStats } from '../telemetry/stats';
import type { MissionRequest } from '../workers/sim.worker';
import { AssumptionList, Btn, EstimateBadge, Section, Select, STATUS_DOT, fmt } from './common';

const axisStyle = { fontSize: 10, fill: '#94a3b8' };

const STAT_CHANNELS: { value: string; label: string; unit: string; digits: number }[] = [
  { value: 'power', label: 'Elektrik gücü', unit: 'W', digits: 0 },
  { value: 'battI', label: 'Batarya akımı', unit: 'A', digits: 0 },
  { value: 'battV', label: 'Batarya gerilimi', unit: 'V', digits: 1 },
  { value: 'cellV', label: 'Hücre gerilimi', unit: 'V', digits: 2 },
  { value: 'cRate', label: 'C oranı', unit: 'C', digits: 2 },
  { value: 'ias', label: 'Hava hızı', unit: 'm/s', digits: 1 },
  { value: 'alt', label: 'İrtifa', unit: 'm', digits: 1 },
  { value: 'roll', label: 'Roll', unit: '°', digits: 1 },
  { value: 'pitch', label: 'Pitch', unit: '°', digits: 1 },
  { value: 'aoa', label: 'Hücum açısı', unit: '°', digits: 1 },
  { value: 'nz', label: 'Yük faktörü', unit: 'g', digits: 2 },
  { value: 'thrustTot', label: 'Toplam itki', unit: 'N', digits: 0 },
  { value: 'rpm1', label: 'Rotor 1 devri (sol iç)', unit: 'rpm', digits: 0 },
  { value: 'rpm2', label: 'Rotor 2 devri (sol dış)', unit: 'rpm', digits: 0 },
  { value: 'cur1', label: 'Motor 1 faz akımı', unit: 'A', digits: 0 },
  { value: 'cur2', label: 'Motor 2 faz akımı', unit: 'A', digits: 0 },
  { value: 'tmot1', label: 'Motor 1 sıcaklığı', unit: '°C', digits: 0 },
  { value: 'tesc1', label: 'ESC 1 sıcaklığı', unit: '°C', digits: 0 },
  { value: 'hingeR', label: 'Menteşe momenti (sağ)', unit: 'N·m', digits: 1 },
  { value: 'bendR', label: 'Kanat kökü eğilme momenti (sağ)', unit: 'N·m', digits: 1 },
  { value: 'vibeX', label: 'Titreşim X', unit: 'm/s²', digits: 1 },
  { value: 'vibeY', label: 'Titreşim Y', unit: 'm/s²', digits: 1 },
  { value: 'vibeZ', label: 'Titreşim Z', unit: 'm/s²', digits: 1 },
  { value: 'accZ', label: 'IMU AccZ', unit: 'm/s²', digits: 1 },
  { value: 'elev', label: 'Elevator', unit: '°', digits: 1 },
  { value: 'ail', label: 'Aileron', unit: '°', digits: 1 },
];

const BAND_LABEL: Record<string, string> = { alt: 'İrtifa [m]', ias: 'Hava hızı [m/s]', power: 'Güç [W]', battV: 'Batarya gerilimi [V]', soc: 'SoC [%]', roll: 'Roll [°]', pitch: 'Pitch [°]' };

export function TelemetryPanel({ product, trace, tel, buildRequest }: { product: Product; trace: MissionTrace; tel: Telemetry; buildRequest: () => MissionRequest }) {
  const s = useApp();
  const [lat, setLat] = useState(DEFAULT_TELEMETRY.homeLatDeg);
  const [lon, setLon] = useState(DEFAULT_TELEMETRY.homeLonDeg);
  const [statKey, setStatKey] = useState('power');
  const [runs, setRuns] = useState(10);
  const [bandKey, setBandKey] = useState<string>('alt');
  const [busy, setBusy] = useState(false);
  const all = useMemo(() => ({ ...trace.ch, ...tel.ch }), [trace, tel]);
  const alarms = useMemo(() => evaluateAlarms(trace, tel.ch, product), [trace, tel, product]);
  const stat = STAT_CHANNELS.find((c) => c.value === statKey) ?? STAT_CHANNELS[0];
  const stats = useMemo(() => (all[statKey] ? phaseStats(trace.ch.phase, trace.ch.t, all[statKey]) : []), [all, statKey, trace]);

  const exportLog = (kind: 'bin' | 'tlog') => {
    setBusy(true);
    // re-synthesise with the chosen home position; yield to the UI first
    setTimeout(() => {
      try {
        const t = synthesizeTelemetry(trace, product, { homeLatDeg: lat, homeLonDeg: lon });
        const bytes = kind === 'bin' ? encodeDataFlash(t) : encodeTlog(trace, t);
        download(`specter_${product.id}_sim.${kind}`, new Blob([bytes], { type: 'application/octet-stream' }));
      } finally {
        setBusy(false);
      }
    }, 20);
  };
  const monteCarlo = async () => {
    s.set({ monteCarloRunning: true, monteCarloProgress: 0 });
    try {
      const res = await runMonteCarlo({ ...buildRequest(), runs }, (f) => useApp.setState({ monteCarloProgress: f }));
      s.set({ monteCarlo: res, monteCarloRunning: false });
    } catch (e) {
      s.set({ monteCarloRunning: false, missionError: e instanceof Error ? e.message : String(e) });
    }
  };
  const mc = s.monteCarlo;
  const bandRows = useMemo(() => {
    if (!mc) return [];
    const b = mc.bands.ch[bandKey];
    return mc.bands.t.map((t, i) => ({ t, p50: b.p50[i], band: [b.p05[i], b.p95[i]] as [number, number] }));
  }, [mc, bandKey]);

  return (
    <>
      <Section title="Tahmini telemetri — ArduPilot log">
        <p className="text-[11px] text-slate-400">
          Görev, sensör modelleriyle (IMU + rotor titreşimi, GPS, baro, pitot, ESC, batarya) ArduPilot mesajlarına dönüştürülür:
          {' '}<span className="font-mono text-slate-300">{tel.streams.filter((x) => x.rows.length).map((x) => x.name).join(' ')}</span>
        </p>
        <div className="grid grid-cols-2 gap-2 text-xs text-slate-300">
          <label>Kalkış enlemi<input type="number" step={0.0001} className="mt-0.5 w-full rounded border border-slate-700 bg-slate-800 px-1 py-0.5 font-mono" value={lat} onChange={(e) => setLat(Number(e.target.value) || 0)} /></label>
          <label>Kalkış boylamı<input type="number" step={0.0001} className="mt-0.5 w-full rounded border border-slate-700 bg-slate-800 px-1 py-0.5 font-mono" value={lon} onChange={(e) => setLon(Number(e.target.value) || 0)} /></label>
        </div>
        <div className="flex flex-wrap gap-1">
          <Btn active disabled={busy} onClick={() => exportLog('bin')} title="Mission Planner, MAVExplorer, UAV Log Viewer, PlotJuggler">DataFlash .bin</Btn>
          <Btn disabled={busy} onClick={() => exportLog('tlog')} title="Mission Planner / QGroundControl telemetri kaydı">MAVLink .tlog</Btn>
        </div>
        <p className="text-[10px] text-slate-500">Loglar “simüle edilmiş tahmin” olarak işaretlidir (MSG, SIM_SPECTER). Özel mesajlar: HNG (menteşe), LOAD (yük / eğilme momenti), WND, PHS.</p>
        <AssumptionList items={tel.assumptions} />
      </Section>

      <Section title="Alarm eşikleri">
        <table className="w-full text-[11px]">
          <thead className="text-slate-500"><tr><th className="text-left font-normal">Kanal</th><th className="text-right font-normal">En kötü</th><th className="text-right font-normal">Uyarı / kritik</th></tr></thead>
          <tbody>
            {alarms.map((a) => (
              <tr key={a.id} className="border-t border-slate-800/70" title={`${a.where} · uyarı ötesinde ${a.timeBeyondWarnS.toFixed(0)} s`}>
                <td className="py-0.5"><span className={`mr-1 inline-block h-2 w-2 rounded-full ${STATUS_DOT[a.status]}`} />{a.label}</td>
                <td className="whitespace-nowrap text-right font-mono">{fmt(a.worst, Math.abs(a.worst) < 10 ? 2 : 0)} {a.unit}</td>
                <td className="whitespace-nowrap text-right font-mono text-slate-400">{a.highIsBad ? '>' : '<'} {fmt(a.warn, Math.abs(a.warn) < 10 ? 2 : 0)} / {fmt(a.critical, Math.abs(a.critical) < 10 ? 2 : 0)}</td>
              </tr>
            ))}
          </tbody>
        </table>
        <p className="text-[10px] text-slate-500">Satırın üzerine gelince en kötü değerin fazı ve zamanı görünür.<EstimateBadge /></p>
      </Section>

      <Section title="Faz bazında telemetri aralıkları">
        <Select<string> value={statKey} onChange={setStatKey} options={STAT_CHANNELS.map((c) => ({ value: c.value, label: `${c.label} [${c.unit}]` }))} />
        <table className="w-full text-[11px]">
          <thead className="text-slate-500"><tr><th className="text-left font-normal">Faz</th><th className="text-right font-normal">min</th><th className="text-right font-normal">ort.</th><th className="text-right font-normal">%95</th><th className="text-right font-normal">maks</th></tr></thead>
          <tbody>
            {stats.map((p) => (
              <tr key={p.phase} className={`border-t border-slate-800/70 ${p.phase < 0 ? 'font-semibold text-slate-100' : ''}`}>
                <td className="py-0.5">{p.name}</td>
                {[p.range.min, p.range.mean, p.range.p95, p.range.max].map((v, i) => <td key={i} className="text-right font-mono">{fmt(v, stat.digits)}</td>)}
              </tr>
            ))}
          </tbody>
        </table>
      </Section>

      <Section title="Monte Carlo — beklenen telemetri bantları" defaultOpen>
        <p className="text-[11px] text-slate-400">Aynı görev farklı rüzgâr / türbülans / sensör seed’leriyle tekrarlanır (seed, seed+1, …). Türbülans “yok” ise koşular aynı çıkar.</p>
        <div className="flex items-center gap-2">
          <div className="w-28"><Select<string> value={String(runs)} onChange={(v) => setRuns(Number(v))} options={['5', '10', '20', '40'].map((v) => ({ value: v, label: `${v} koşu` }))} /></div>
          <Btn active disabled={s.monteCarloRunning || s.missionRunning} onClick={monteCarlo}>{s.monteCarloRunning ? `Çalışıyor… %${(s.monteCarloProgress * 100).toFixed(0)}` : '▶ Monte Carlo'}</Btn>
        </div>
        {s.monteCarloRunning && <div className="h-1 rounded bg-slate-800"><div className="h-1 rounded bg-sky-500" style={{ width: `${s.monteCarloProgress * 100}%` }} /></div>}
        {mc && (
          <>
            <div className={`rounded border p-2 text-xs ${mc.successRate >= 0.99 ? 'border-emerald-500/60 bg-emerald-500/10' : mc.successRate >= 0.8 ? 'border-amber-400/60 bg-amber-400/10' : 'border-red-500/70 bg-red-500/15'}`}>
              <div className="font-semibold text-slate-100">Başarı oranı: {(mc.successRate * 100).toFixed(0)} % ({mc.runs.length} koşu)</div>
              {Object.entries(mc.outcomes).map(([k, v]) => <div key={k} className="text-slate-300">{v} × {k}</div>)}
            </div>
            <table className="w-full text-[11px]">
              <thead className="text-slate-500"><tr><th className="text-left font-normal">Metrik</th><th className="text-right font-normal">min</th><th className="text-right font-normal">ort.</th><th className="text-right font-normal">%95</th><th className="text-right font-normal">maks</th></tr></thead>
              <tbody>
                {mc.metrics.map((m) => (
                  <tr key={m.id} className="border-t border-slate-800/70">
                    <td className="py-0.5">{m.label} <span className="text-slate-500">[{m.unit}]</span></td>
                    {[m.range.min, m.range.mean, m.range.p95, m.range.max].map((v, i) => <td key={i} className="text-right font-mono">{fmt(v, Math.abs(m.range.max) < 10 ? 2 : Math.abs(m.range.max) < 100 ? 1 : 0)}</td>)}
                  </tr>
                ))}
              </tbody>
            </table>
            <Select<string> value={bandKey} onChange={setBandKey} options={MC_BAND_KEYS.map((k) => ({ value: k, label: `Bant: ${BAND_LABEL[k]}` }))} />
            <div className="h-40">
              <ResponsiveContainer width="100%" height="100%">
                <ComposedChart data={bandRows} margin={{ top: 4, right: 4, bottom: 0, left: -8 }}>
                  <CartesianGrid stroke="#1e293b" />
                  <XAxis dataKey="t" type="number" domain={['dataMin', 'dataMax']} tick={axisStyle} tickFormatter={(v) => `${(Number(v) / 60).toFixed(0)}dk`} />
                  <YAxis tick={axisStyle} width={50} domain={['auto', 'auto']} />
                  <Tooltip contentStyle={{ background: '#0f172a', border: '1px solid #334155', fontSize: 11 }} labelFormatter={(v) => `t = ${Number(v).toFixed(0)} s`} formatter={(v: number | number[]) => (Array.isArray(v) ? `${v[0].toFixed(1)} – ${v[1].toFixed(1)}` : v.toFixed(1))} />
                  <Area dataKey="band" name="%5 – %95" stroke="none" fill="#38bdf8" fillOpacity={0.3} isAnimationActive={false} />
                  <Line dataKey="p50" name="medyan" stroke="#f472b6" dot={false} strokeWidth={1.4} isAnimationActive={false} />
                </ComposedChart>
              </ResponsiveContainer>
            </div>
          </>
        )}
      </Section>
    </>
  );
}
