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

## Faz 1 — 6-DOF görev simülasyonu (yeni)

**Görev simülasyonu (6-DOF)** sekmesi, referans ürün **Transwing 4.0 (TW4)** ve ondan ölçeklenen aile için
kalkış → hover → ileri geçiş → tırmanış → waypoint seyri → alçalma → geri geçiş → hover → iniş görevini
baştan sona simüle eder (76 km'lik görev ≈ 15–30 s, Web Worker'da).

- `src/physics/fdm/vlm.ts` — açık kanadın düzlemsel vortex-lattice çözümü (CLα, açıklık boyunca yükleme).
- `src/physics/fdm/model.ts` — şeritler (orta kanat, katlanan paneller, V-kuyruk), rotorlar, katlanma
  açısına göre CG / atalet tablosu.
- `src/physics/fdm/forces.ts` — şerit aerodinamiği (±180° kesit modeli), pervane akımı, kanat iniş akımı,
  CT/CP(J) pervane haritası, DC motor + ESC, menteşe momenti.
- `src/physics/fdm/trim.ts` — düz uçuş ve hover trimi, parazit sürüklenme kalibrasyonu, kararlılık /
  kontrol türevleri, dinamik modlar.
- `src/physics/fdm/mission.ts` — RK4 (dt = 0,01 s) görev simülasyonu, QuadPlane-benzeri kontrolcü,
  kontrol dağıtımı, batarya (OCV + R + RC), motor/ESC ısıl modeli, olay / alarm kaydı, ~70 telemetri kanalı.

Sekmede: görev editörü (irtifa, hız, hover süreleri, waypoint'ler, 4 ↔ 2 motor seyir), 3B takip görünümü,
mini harita, faz zaman çizelgesi, çift grafik (14 hazır kanal grubu), sonuç kartları, faz bazında enerji,
olay listesi, telemetri CSV / özet JSON dışa aktarma ve **trim + kararlılık analizi** paneli.

### Transwing 4.0 verisi ve varsayımlar

Tasarım ekibinden gelen değerler `estimate: false` ile, eksik olanlar `estimate: true` ile işaretlidir
(`scripts/generate-specs.ts`). Önemli **varsayımlar** (CAD paketi — Faz 3 — gelince değişecek):

1. **Menteşe kinematiği:** hover'da panel 65° geriye süpürülür, itki ekseni dikeydir (`designHingeForSweep(65)`).
2. **Alçak kanat yerleşimi** (menteşe y = −0,06 m): pervane itki hattı düşey CG'ye yakın. Yüksek kanat
   varsayımıyla itki hattı CG'nin 0,15–0,35 m üstünde kalır ve kanat ≈ %60'tan fazla açıldığında hiçbir rotor
   burun-yukarı moment üretemez; ileri geçiş diferansiyel itkiyle trimlenemez (**geçiş koridoru**).
3. Pervane haritası geneldir (CT0 = 0,105, CP0 = 0,036, sıfır itki J ≈ 0,72) — Lighter GFFA 28×13 test
   verisi gerekir. Motor iç direnci, boşta akımı ve azami akımı (180 A) tahmindir.
4. Kütle grubu toplamları ekip verisidir; grup içi dağılım ve tüm parça konumları tahmindir. Batarya konumu
   seyirde %10 MAC statik marj verecek şekilde seçilir.
5. Parazit sürüklenme, ekibin (L/D)max = 18,07 değerine kalibre edilir; CLmax 1,4 → Vs ≈ 20 m/s.

### Model bulguları (TW4, 40 kg, deniz seviyesi, ISA) — tahmini

- Seyir (23,7 m/s): 4 motorla ≈ 1,38 kW, 2 motorla ≈ 0,89 kW elektrik gücü. 22 N itki için 28" pervaneler
  çok hafif yüklü kalır (pervane verimi ≈ %42 ↔ %64).
- Hover ≈ 8,2 kW (≈ 2,8 C): iç (ön) rotorlar ağırlığın ≈ %75'ini taşır (175 N ↔ 58 N) ve iç rotor akımı
  orta kanat üzerinde ≈ 75 N download üretir. 6 dakikalık hover ≈ 820 Wh.
- 76 km'lik görev (6 dk hover dahil): 4 motorla ≈ 2,13 kWh, son SoC ≈ %28; 2 motorla ≈ 1,70 kWh, son SoC ≈ %44.
- Seyir hızı stall'ın yalnızca %17 üstünde: güvenli yatış ≈ 25–28° ile sınırlı; türbülansta stall /
  Q_ASSIST uyarıları görülür.
- Statik marj %10 MAC; Cnβ ≈ 0,06 (düşük), spiral modu kararsız (T₂ ≈ 6 s) — otopilot gerektirir.
- Hover'da yan rüzgâr: pervane akımı içindeki dikey paneller büyük yan kuvvet üretir; ≈ 8 m/s yan rüzgârda
  kontrol kaybı. Simülasyon bu yüzden rüzgâra burun çevirir (weathervane), ileri ve geri geçişi rüzgâra
  karşı uçar. Şiddetli türbülansta (12 m/s rüzgâr) geri geçiş başarısız olur.
- Ölçeklenmiş türevlerden T10-M (en ağır) ileri geçişten sonra irtifa tutamaz; ölçek yasaları bu boyutta
  itki / kanat yükü dengesini korumuyor.

Sınırlar: kontrolcü ideal durum bilgisi kullanır (sensör / EKF modeli Faz 2), ArduPilot'un kendisi değildir
(Faz 4: SITL köprüsü), katlı konumda aerodinamik etkileşimler ve pervane eğik akış kuvvetleri yaklaşık / ihmal.

## Faz 2 — Tahmini telemetri (yeni)

Görev simülasyonu bittiğinde sağ panelde **Tahmini telemetri** bölümü açılır (`src/telemetry/`):

- **Sensör modelleri** (`sensors.ts`): IMU (bias + gürültü + rotor 1/dev dengesizlik ve kanat geçiş
  titreşim tonları + stall/katlanma buffet'i), GPS (Gauss–Markov hata, 5 Hz), baro (sürüklenme), pitot
  (düşük hızda güvenilmez), ESC, batarya. Seed'li, tekrarlanabilir.
- **ArduPilot DataFlash `.bin`** (`dataflash.ts`): `PARM MSG MODE ATT POS GPS BARO ARSP BAT ESC RCOU CTUN
  QTUN NTUN VIBE IMU` + Specter'e özel `HNG` (menteşe açısı / momenti), `LOAD` (yük faktörü, kanat kökü
  eğilme momenti), `WND`, `PHS`. Mission Planner, MAVExplorer, UAV Log Viewer ve PlotJuggler'da açılır;
  pymavlink ile hatasız ayrıştırıldığı doğrulandı.
- **MAVLink `.tlog`** (`mavlink.ts`): HEARTBEAT, SYS_STATUS, ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD,
  GPS_RAW_INT, SERVO_OUTPUT_RAW, NAV_CONTROLLER_OUTPUT, BATTERY_STATUS, VIBRATION, WIND, STATUSTEXT.
- **Alarm tablosu**: hücre gerilimi, SoC, C oranı, motor akımı / sıcaklığı, ESC ve hücre sıcaklığı, VIBE
  (ArduPilot önerisi: < 30 iyi, > 60 kötü), menteşe momenti, Q_ASSIST / stall, yatış, yük faktörü,
  irtifa hatası — en kötü değer, fazı ve zamanı.
- **Faz bazında aralıklar**: herhangi bir kanal için faz başına min / ortalama / %95 / maks.
- **Monte Carlo**: aynı görevi farklı seed'lerle (rüzgâr, türbülans, sensör) 5–40 kez koşar; başarı
  oranı, sonuç dağılımı, metrik aralıkları ve zaman üzerinde %5–%95 bantları.
- Görev grafiklerine titreşim, IMU ivme / jiroskop, irtifa sensörleri, pitot ve kanat kökü eğilme
  momenti grupları eklendi.

Loglar açıkça **simüle edilmiş tahmin** olarak işaretlenir (MSG ve `SIM_SPECTER` parametresi).
Titreşim genlikleri varsayımdır; ilk gerçek uçuş logu ile kalibre edilmelidir.

### Kontrolcü sağlamlaştırma (Faz 2 sırasında bulunan sorunlar)

Monte Carlo, rüzgârlı koşullarda ileri geçişte kontrol kayıplarını ortaya çıkardı. Düzeltmeler:
kontrol dağıtımında ArduPilot motor karıştırıcısındaki gibi **öncelik** (yalpa / yunuslama önce,
toplam itki sonra, sapma en son), geçiş sırasında **kayma açısı geri beslemesi**, uçak alçalırken
**kanat açılmasını bekletme**, sabit kanatta **düşük hızda hız önceliği, hücum açısı sınırı ve stall
kurtarma**, geçişten hemen sonra ve alçakta **sınırlı yatış**.

TW4, MTOW, 1,5 km görev, 8 seed başına sonuç (tahmini):

| Rüzgâr / türbülans | Başarı | İleri geçiş irtifa kaybı (maks.) |
|---|---|---|
| 0 m/s, yok | 1/1 | 2,5 m |
| 4 m/s, hafif | 7/8 | 3,6 m |
| 6 m/s, hafif | 7/8 | 2,6 m |
| 8 m/s, orta | 8/8 | 3,5 m |
| 10 m/s, orta | 5/8 | 2,5 m |
| 12 m/s, şiddetli | 0/8 | 8,0 m |

Kalan kayıplar sabit kanat tırmanışı / alçalmasında türbülans kaynaklı stall'dan geliyor: seyir hızı
stall'ın yalnızca %17 üstünde olduğu için şiddetli türbülansta marj yetmiyor. Bu, kontrolcüden çok
tasarım noktasının (seyir hızı / kanat yüklemesi) bir sonucu olarak değerlendirilmeli.

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
| `fdm` | 6-DOF veri bloğu: `prop` (CT/CP–J tablosu, hatve, rotor ataleti), `motor` (KV, direnç, boşta ve azami akım, ısıl), `esc`, `battery` (S/P, hücre Ah, OCV tablosu, iç direnç, C limitleri, alarm gerilimleri, rezerv SoC), `hingeActuator` (hız, tork), `section` (kesit kaldırma eğimi, α0, clmax, cd0, cm0), `targetMaxLD`, `transition` (açılma tamamlanma hızı, ARSPD_FBW_MIN, Q_ASSIST_SPEED, başlangıç açılma oranı), `missionDefault`. |
| `geometry.wingAreaM2, wingQcXM, incidenceDeg, aileron, tail` | Referans alan, çeyrek veter x'i, takılma açısı, aileron istasyonları, V-kuyruk (alan, V açısı, konum, panel açıklığı, kumanda veter oranı). |
| `payloadOptions[]` | `id, name, massKg, dragAreaM2` (cruise'a eklenen CD·S), `variant`, `shape` (gimbal/lidar/box/pod/antenna), `estimate`. |

### Menteşe ekseni nasıl ayarlanır

`src/kinematics/hinge.ts → designHingeForSweep(Λ)` yardımcı fonksiyonu, cruise'daki açıklık yönünü
yatayda Λ kadar geriye süpürülmüş yöne, pervane eksenini (+x) dikeye (+y) taşıyan **tek** dönüşün
eksenini (azimut, elevasyon) ve açısını verir. Λ = 90° tam gövdeye paralel katlanmadır
(eksen (−1,−1,1)/√3, 120°). Varsayılan veri Λ = 65° kullanır (az 57,5°, el −28,25°, 106,8°) — böylece
hover'da sol/sağ pervaneler birbirine çarpmaz. Eksen açıları `specs.json`'da serbestçe değiştirilebilir;
UI'daki katlanma slider'ı ile sonuç anında görülür.

### Ürün ailesi ve ölçekleme

`npm run gen:specs` (`scripts/generate-specs.ts`) referans ürün Transwing 4.0'ı (TW4, 4 m) yazar,
5 m / 7 m / 10 m sivil ve askeri ürünleri ölçek yasalarıyla türetir (`src/data/scaling.ts`): uzunluk ∝ s, alan ∝ s²,
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
