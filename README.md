# Specter Configurator

Specter Transwing (katlanır / dönen kanatlı) elektrikli VTOL İHA ailesi için tarayıcıda çalışan
interaktif **ürün konfigüratörü ve performans simülatörü**.

> ⚠ **Tahmini mühendislik modeli; sertifikasyon hesabı değildir.**
> `public/specs.json` içindeki tüm sayılar **yer tutucudur** (`estimate: true`), UI'da “tahmini” rozetiyle gösterilir.

## Çalıştırma

```bash
npm install
npm run dev          # http://localhost:5173
npm test             # birim testleri (vitest): fizik, kinematik, şema, CSV, URL
npm run build        # tip kontrolü + üretim derlemesi (dist/)
npm run preview      # derlenmiş sürümü sun
npm run gen:specs    # public/specs.json'ı yer tutucu verilerle YENİDEN üretir (üzerine yazar!)
```

Yığın: React 18 + TypeScript + Vite, Three.js (@react-three/fiber, @react-three/drei), zustand,
Tailwind, recharts, zod. Fizik `src/physics/` altında React'ten bağımsız saf TypeScript'tir;
zaman alanı simülasyonu ve zarf taramaları Web Worker'da (`src/workers/sim.worker.ts`) çalışır.

## Özellikler (kısa)

- **Parametrik 3B model**: NACA 4 haneli profil loft'lu taper kanat, gövde + burun konisi, kuyruk,
  naseller, katlanır pervaneler, menteşeler, batarya, aviyonik, payload (seçime göre şekil), kızak.
  Kanatlar `hinge_left` / `hinge_right` pivot node'ları altında; tek bir eğik eksenli dönüş hem
  katlanmayı hem pervane yönelimini sağlar. Ölçek: 1,80 m insan silüeti ve 1 m ızgara.
- **İnceleme**: kamera ön ayarları, exploded view (bağlantı çizgili), parça ağacı (göster/gizle/izole),
  tıklanınca parça kartı, hover outline, CG işareti (katlanma ve payload ile güncellenir), kesit düzlemi,
  wireframe, PNG.
- **Uçuş modu**: katlanma slider'ı, Cruise / Transition / Hover, ayarlanabilir süreli geçiş animasyonu,
  itkiyle orantılı pervane dönüşü, 2 motorlu cruise'da pervanelerin nasele katlanması.
- **Fizik**: ISA, momentum teorisi hover, sürüklenme poları, görev enerji bütçesi, rüzgâr modelleri
  (sabit, logaritmik profil, Dryden, 1-cosine), hover rüzgâr limiti (eğim / itki / moment otoritesi),
  RK4 zaman alanı simülasyonu (hover 3-DOF + PID, geçiş/cruise noktasal kütle).
- **Senaryo paneli + sonuç kartları** (yeşil/sarı/kırmızı, her kartta varsayımlar), hazır senaryolar.
- **Görselleştirme**: InstancedMesh rüzgâr alanı, ölçekli kuvvet okları, replay (oynat/duraklat/zaman
  çizelgesi, iz çizgisi, doymuş rotor kırmızı yanıp söner), grafikler (zaman serileri, güç–hız, rüzgâra
  karşı güç marjı).
- **Karşılaştırma** (senkron kamera, Δ ve % tablo), **operasyonel zarf** (rüzgâr × irtifa / payload,
  6 ürün sınırları üst üste), **ürün matrisi** (3×2), **paylaşılabilir URL** (hash), **JSON/CSV/PNG
  dışa aktarma**, **sunum modu** (Esc ile çıkış), **harici CSV içe aktarma** (JSBSim / PX4 SITL / CFD).

## Veri dosyası: `public/specs.json`

Uygulama açılışta `specs.json`'ı yükler ve zod şemasıyla (`src/data/schema.ts`) doğrular; ayrıca
tutarlılık kontrolleri yapar (parça kütleleri toplamı ≈ `emptyKg`, batarya parçası = `batteryKg`,
payload ≤ `payloadMaxKg` …). Hata/uyarılar üst banttaki “Veri doğrulama” bağlantısında listelenir.
**Kodda ürün verisi yoktur**: ürün eklemek için `products` dizisine yeni kayıt eklemek yeterlidir
(derlenmiş sürümde `dist/specs.json` düzenlenebilir, yeniden derleme gerekmez).

Eksen takımı: **x ileri (burun), y yukarı, z sağ kanat**, orijin gövde orta noktası. Birimler SI;
alan adları birim sonekli (`spanM`, `massKg`, `powerW` …).

| Alan | Açıklama |
|---|---|
| `version`, `title`, `referenceProductId` | Dosya meta verisi; açılışta referans ürün seçilir. |
| `mission.takeoffHoverS`, `landingHoverS`, `transitionS` | Görev profili süreleri (s). |
| `mission.reserveFraction` | Kullanılabilir enerjinin rezerv oranı. |
| `mission.controlReserveFraction` | Hover'da kontrol için ayrılan itki oranı (rüzgâr limiti). |
| `mission.hoverHeightM` | Hover değerlendirme / simülasyon yüksekliği (log rüzgâr profili için). |
| `scenarios[]` | Hazır senaryolar: `env` (irtifa, ISA sapması, rüzgâr hızı @10 m, geldiği yön — buruna göre, türbülans `none/light/moderate/severe`, gust genliği, log profil), `config` (uçuş modu, cruise motor sayısı, batarya doluluğu), `sim` (süre, seed, gust başlangıç/süre). |
| `products[].id/name/sizeClass/variant` | Kimlik; `variant`: `civil` / `military`. |
| `products[].modelUrl` | (Ops.) GLB modeli. Node adları `parts[].id` ile, menteşeler `hinge_left` / `hinge_right` ile eşleşmeli; menteşe node'ları veri eksenine göre döndürülür. |
| `derivedFrom`, `scaleFactor` | Ölçeklenerek türetilmiş ürünler için bilgi alanı. |
| `geometry.spanM, chordRootM, chordTipM, airfoil` | Kanat açıklığı, kök/uç veter, NACA 4 haneli profil (ör. `4415`). |
| `geometry.fuselageLengthM, fuselageDiameterM, landingGearHeightM` | Gövde ve kızak ölçüleri. |
| `geometry.hinge.positionM` | **Sağ** kanat menteşe pivotu [x,y,z]; sol kanat aynalanır (z → −z). |
| `geometry.hinge.axisAzimuthDeg` | Sağ menteşe ekseninin yatay düzlemdeki açısı, **geriden (−x) dışa (+z) doğru**. |
| `geometry.hinge.axisElevationDeg` | Eksenin yatayla açısı (negatif = aşağı). |
| `geometry.hinge.maxFoldDeg` | Tam hover için eksen etrafındaki dönüş açısı; 0 = cruise. |
| `geometry.rotors[]` | `wing` (L/R), `spanFraction` (menteşeden uca, açıkta kalan yarı açıklık üzerinde), `diameterM`, `foldableInCruise` (2 motorlu cruise'da kapanıp katlanan), `hubOffsetM` (pervane düzleminin çeyrek veter çizgisi önündeki mesafesi). |
| `mass.emptyKg, batteryKg, payloadMaxKg` | Boş kütle (batarya ve payload hariç), batarya, maks. faydalı yük. |
| `propulsion.motorMaxPowerW` | Motor başına sürekli **şaft** gücü. |
| `propulsion.figureOfMerit, propEfficiency, motorEscEfficiency` | Hover FM, cruise pervane verimi, motor+ESC verimi. |
| `energy.batteryWh, usableFraction` | Nominal enerji ve kullanılabilir oran. |
| `aero.cd0, oswaldE, clMax` | Sürüklenme poları ve maks. kaldırma katsayısı. |
| `aero.sideCd` | Katlı (hover) gövdenin izdüşüm alanına göre sürüklenme katsayısı. |
| `parts[]` | `id, name, group` (Airframe/Wing/Propulsion/Energy/Avionics/Payload/Landing), `kind` (prosedürel şekil: fuselage, nose, tail, wing, hinge, nacelle, rotor, battery, avionics, payload, gear, box), `massKg`, `positionM` (cruise konumunda parça CG'si), `sizeM` (kutu parçalar), `attach` (`body` / `wingL` / `wingR` — kanada bağlı parçalar menteşe ile döner), `rotorIndex`, `material`, `bomRef`, `estimate`. Payload grubundaki parçanın kütlesi seçili payload'dan gelir. |
| `payloadOptions[]` | `id, name, massKg, dragAreaM2` (cruise'a eklenen CD·S), `variant`, `shape` (gimbal/lidar/box/pod/antenna), `estimate`. |

### Menteşe ekseni nasıl ayarlanır

`src/kinematics/hinge.ts → designHingeForSweep(Λ)` yardımcı fonksiyonu, cruise'daki açıklık yönünü
yatayda Λ kadar geriye süpürülmüş yöne, pervane eksenini (+x) dikeye (+y) taşıyan **tek** dönüşün
eksenini (azimut, elevasyon) ve açısını verir. Λ = 90° tam gövdeye paralel katlanmadır
(eksen (−1,−1,1)/√3, 120°). Varsayılan veri Λ = 65° kullanır (az 57,5°, el −28,25°, 106,8°) — böylece
hover'da sol/sağ pervaneler birbirine çarpmaz. Eksen açıları `specs.json`'da serbestçe değiştirilebilir;
UI'daki katlanma slider'ı ile sonuç anında görülür.

### Ürün ailesi ve ölçekleme

`npm run gen:specs` (`scripts/generate-specs.ts`) 7 m referansı (T7-C) ve askeri varyantını yazar,
5 m / 10 m ürünleri ölçek yasalarıyla türetir (`src/data/scaling.ts`): uzunluk ∝ s, alan ∝ s²,
kütle ∝ s^2,5, motor gücü ∝ s^2,75 (hover T/W'yi yaklaşık sabit tutar), enerji ∝ kütle. Askeri
varyantta aviyonik ×2,2, veri linki ×2,5, gövde ×1,1, kablo demeti ×1,3. Üretici, bataryayı hover
CG'sini rotor itki merkezine yaklaştıracak konuma koyar (gövde sınırında kırpılır). Gerçek verileri
girdikten sonra üreticiyi **çalıştırmayın** (dosyayı üzerine yazar).

## Fizik modeli: varsayımlar ve sınırlar

Her fizik fonksiyonu `{ value, assumptions[] }` döndürür; varsayımlar sonuç kartlarında ve
“Varsayımlar” penceresinde listelenir.

1. **Atmosfer** (`atmosphere.ts`): ISA (11 km'ye kadar troposfer + alt stratosfer). ISA sapması
   yalnızca sıcaklığı değiştirir, basınç standart profildedir. Nem yok.
2. **Hover** (`hover.ts`, `rotor.ts`): rotor başına momentum teorisi `P = T^1.5/√(2ρA)`, FM ve
   motor/ESC verimi ile düzeltme. CG ofseti en küçük-norm diferansiyel itki ile dengelenir; güç marjı
   en yüklü rotordan. Rotor etkileşimi, yer etkisi, gövde blokajı yok.
3. **Cruise** (`cruise.ts`): `CD = CD0 + CL²/(π e AR) + payload CD·S/S`, brüt trapez alan.
   Operasyonel hızlar ≥ 1,2 Vs. 4 motorlu cruise'da pervane verimi ×0,92 (düşük yükte çalışma);
   2 motorda katlı pervanelerin sürüklenmesi CD0 içinde. Sıkıştırılabilirlik / Reynolds yok.
4. **Görev** (`mission.ts`): kalkış hover + 2 geçiş (hover gücünün %85'i) + cruise + iniş hover +
   rezerv. Menzil en iyi menzil hızında, karşı rüzgâr bileşeni düşülerek (tek yön); dayanım cruise
   kısmını minimum güç hızında bekleme olarak hesaplar. Batarya gerilim düşümü, sıcaklık, aviyonik
   tüketimi yok.
5. **Rüzgâr** (`wind.ts`): log profil (z₀ = 0,03 m, referans 10 m); Dryden MIL-HDBK-1797 düşük irtifa
   parametreleri (W20 = 15/30/45 kt), 1000–2000 ft arası yüksek irtifa değerlerine geçiş; bileşenler
   **seed'li** birinci derece Gauss–Markov süreci (u-spektrumu Dryden ile aynı, v/w için yaklaşık);
   1-cosine gust.
6. **Hover rüzgâr limiti**: `φ = atan(D_yan/W)`, gerekli itki `W/cos φ`; maks. eğim 35°, %15 kontrol
   rezervi; basınç merkezinin oluşturduğu moment diferansiyel itkiyle dengelenebilmeli (moment
   otoritesi). Yan alan katlı geometriden (gövde + kanat panelleri |n·d|·A + dikey kuyruk) hesaplanır.
   Not: eğim sınırlı bölgede, D ∝ ρV² olduğundan limit yoğunluk düştükçe (irtifa) artabilir; itki
   sınırı ise ρ ile azalır.
7. **Zaman alanı** (`sim.ts`, RK4, dt = 0,01 s, Web Worker):
   - Hover: düzlemsel 3-DOF (rüzgâr ekseni boyunca yatay konum, irtifa, eğim), kaskad PID, rotor
     başına itki dağıtımı ve [0, Tmax] doyması. Motor dinamiği yok; yalpa ve düzlem dışı hareket yok.
     Pozisyon kaybı: sapma > 5 m veya yere çarpma.
   - Geçiş / cruise: noktasal kütle 2-DOF, gövde yatay; itki ekseni menteşe kinematiğinden, kanat
     kaldırma etkinliği `(n·ŷ)²`; parazit sürüklenme cruise CD0·S ile katlı izdüşüm alanı arasında
     interpolasyon; gust düşey.
   - Aynı seed ⇒ birebir aynı sonuç (testle doğrulanır).

Bu model **ön tasarım ve demo** içindir: aerodinamik etkileşimler, aeroelastisite, motor/ESC
dinamikleri, batarya elektrokimyası, tam 6-DOF uçuş dinamiği ve kontrolcü gerçek değildir.
Karşılaştırma için JSBSim / PX4 SITL / CFD sonuçlarını CSV olarak içe aktarın.

## Harici sonuç içe aktarma (CSV)

Alt paneldeki **⇪ Harici CSV**: `time, x, y, z, roll, pitch, yaw, thrust_1..n` (+ ops. `power_w`,
`fold_deg`). Konum m, açı derece (|açı| ≤ 2π ise radyan algılanır), itki N. Eksen: x ileri, y sağ,
z yukarı; PX4 için **NED (z aşağı)** kutusunu işaretleyin. Veri replay'de oynatılır ve zaman serisi
grafiğinde dahili tahminle (kesikli çizgi) üst üste çizilir. Dışa aktarılan simülasyon CSV'si aynı
formattadır (geri yüklenebilir).

## Kod yapısı

```
public/specs.json          tek veri kaynağı
scripts/generate-specs.ts  yer tutucu veri üreticisi
src/data/                  şema + doğrulama, ölçek yasaları
src/kinematics/hinge.ts    menteşe kinematiği (saf matematik)
src/physics/               atmosfer, rotor, hover, cruise, görev, rüzgâr, sim, zarf (+ testler)
src/workers/               Web Worker (sim + zarf)
src/scene/                 prosedürel geometri, R3F sahne bileşenleri
src/store/                 zustand durumu, türetilmiş sonuçlar ve kartlar
src/lib/                   URL durumu, dışa/içe aktarma, worker istemcisi
src/ui/                    paneller ve sayfalar
```
