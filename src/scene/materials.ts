import type { PartGroup } from '../data/schema';

export const GROUP_COLORS: Record<PartGroup, string> = {
  Airframe: '#cfd8e3',
  Wing: '#eef2f7',
  Propulsion: '#4b5563',
  Energy: '#f59e0b',
  Avionics: '#22c55e',
  Payload: '#38bdf8',
  Landing: '#64748b',
};

export const KIND_COLORS: Partial<Record<string, string>> = {
  hinge: '#a78bfa',
  rotor: '#1f2937',
  nose: '#b6c2d1',
};

export const GROUP_LABELS: Record<PartGroup, string> = {
  Airframe: 'Gövde yapısı',
  Wing: 'Kanat',
  Propulsion: 'İtki',
  Energy: 'Enerji',
  Avionics: 'Aviyonik',
  Payload: 'Faydalı yük',
  Landing: 'İniş takımı',
};
