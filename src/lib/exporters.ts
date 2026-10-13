/** JSON / CSV / PNG export helpers. */
import type { SimTrace } from '../physics/sim';
import type { Evaluation } from '../store/results';

export function download(filename: string, data: string | Blob, mime = 'text/plain') {
  const blob = typeof data === 'string' ? new Blob([data], { type: mime }) : data;
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

export function downloadDataUrl(filename: string, dataUrl: string) {
  const a = document.createElement('a');
  a.href = dataUrl;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
}

/** Strip large arrays (power curve) only where needed; everything else goes to JSON verbatim. */
export function resultsJson(meta: Record<string, unknown>, ev: Evaluation, trace: SimTrace | null) {
  return JSON.stringify(
    {
      generatedAt: new Date().toISOString(),
      disclaimer: 'Tahmini mühendislik modeli; sertifikasyon hesabı değildir.',
      ...meta,
      hover: ev.hover,
      cruise: ev.cruise,
      mission: ev.mission,
      hoverWindLimit: ev.windLimit,
      massPropertiesCruise: ev.massCruise,
      massPropertiesHover: ev.massHover,
      simulation: trace ? { summary: trace.summary, assumptions: trace.assumptions, label: trace.label, mode: trace.mode } : null,
    },
    null,
    2,
  );
}

export function traceToCsv(trace: SimTrace): string {
  const n = trace.thrustN[0]?.length ?? 0;
  const head = ['time', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', ...Array.from({ length: n }, (_, i) => `thrust_${i + 1}`), 'power_w', 'wind_x', 'wind_y', 'wind_z', 'fold_deg', 'airspeed_ms', 'saturated'];
  const lines = [head.join(',')];
  for (let i = 0; i < trace.t.length; i++) {
    // CSV uses x fwd, y right, z up (scene frame is x fwd, y up, z right)
    const [px, py, pz] = trace.pos[i];
    const [wx, wy, wz] = trace.windMs[i];
    lines.push(
      [
        trace.t[i].toFixed(3), px.toFixed(4), pz.toFixed(4), py.toFixed(4),
        ...trace.att[i].map((a) => a.toFixed(3)),
        ...trace.thrustN[i].map((t) => t.toFixed(2)),
        trace.powerW[i].toFixed(1), wx.toFixed(3), wz.toFixed(3), wy.toFixed(3),
        trace.foldDeg[i].toFixed(2), trace.airspeedMs[i].toFixed(3),
        trace.saturated[i].some(Boolean) ? 1 : 0,
      ].join(','),
    );
  }
  return lines.join('\n') + '\n';
}

export function cardsToCsv(rows: { label: string; value: string; sub?: string; status: string }[]): string {
  const esc = (s: string) => `"${s.replace(/"/g, '""')}"`;
  return ['metric,value,detail,status', ...rows.map((r) => [esc(r.label), esc(r.value), esc(r.sub ?? ''), r.status].join(','))].join('\n') + '\n';
}
