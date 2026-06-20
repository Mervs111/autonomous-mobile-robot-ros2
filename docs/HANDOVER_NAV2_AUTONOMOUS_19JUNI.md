# HANDOVER — Nav2 Autonomous Navigation BERHASIL — 19 Juni 2026

**Dari:** Mararevi Subagyo (2040241036) · **Untuk:** Tim AMR (Kelompok PJBL 4-24)
**Status:** ✅ **ROBOT SUDAH BISA JALAN SENDIRI (autonomous) dari goal Nav2.**

---

## 1. TL;DR (ringkasan untuk yang buru-buru)

Robot AMR **berhasil bernavigasi otonom**: kirim goal → planner bikin jalur → robot
jalan sendiri ke tujuan. Ini dicapai setelah membongkar **7 "gerbang" yang berurutan
nge-block**, dari plugin Nav2 yang gagal load sampai gerbang software terakhir di
`stm32_bridge` (`autonomous_enabled` yang default-nya `false`).

Semua perbaikan config sudah **di-commit & push ke repo `Mervs111` (origin)**. Cara
apply ke NUC = `curl` file dari GitHub (karena akun git di NUC bukan punya kita).

> ⚠️ Untuk demo saat ini kita **bypass failover** (Nav2 publish langsung ke `/cmd_vel`).
> Artinya **tidak ada auto emergency-stop** — joystick (tombol R1) WAJIB di tangan
> sebagai rem darurat manual.

---

## 2. Rantai 7 fix Nav2 (urut — semua sempat bikin "Aborting bringup")

Tiap baris ini dulunya error yang menghentikan Nav2. Sekarang sudah beres semua di
`src/amr_slam/config/nav2_params.yaml` & `src/amr_slam/launch/nav2.launch.py`.

| # | Gejala error | Akar masalah | Fix |
|---|---|---|---|
| 1 | `VoxelLayer ... does not exist` | Format plugin `/` vs `::` campur | costmap/controller/smoother/waypoint → `::`, smac_planner & behaviors → `/` |
| 2 | `ID [RemovePassedGoals] already registered` | `plugin_lib_names` didefinisikan eksplisit | **Hapus blok `plugin_lib_names`** → Nav2 auto-load (commit `f423a03`) |
| 3 | `Node not recognized: RateController` | (terpecahkan oleh #2) | — |
| 4 | `Action server spin not available` | behavior_server tanpa `spin`, padahal BT default manggil `<Spin>` | Daftarkan `spin` kembali (commit `a1c0b66`) |
| 5 | `Couldn't open input XML file` | `default_nav_to_pose_bt_xml` cuma nama file | Pakai **path absolut** `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/...` (commit `47dd6a0`) |
| 6 | `collision ahead` / `lethal space` terus | kamera depth (`depth_scan`) salah baca lantai = obstacle hantu | **Matikan depth_scan di costmap** (pakai LiDAR `scan` saja) + inflation 0.45→0.25, robot_radius 0.35→0.28 (commit `bd66131`) |
| 7 | `Failed to make progress` (Nav2 kirim cmd_vel tapi robot diam) | Nav2 publish ke `/cmd_vel_nav`, bridge dengar `/cmd_vel`, failover mati = sambungan putus | **Hapus remap** di `nav2.launch.py` → Nav2 langsung ke `/cmd_vel` (commit `a90a3d8`) |
| 8 | Robot TETAP diam walau `/cmd_vel`=0.3 | **`autonomous_enabled` default `false`** di stm32_bridge → buang semua cmd_vel | `ros2 param set /stm32_bridge autonomous_enabled true` ← **GERBANG TERAKHIR** |

> Catatan #2 & #4: temuan ini selaras dengan kerja Azhar (upstream commit `99373ff`,
> `e047af2`). File `nav2_params.yaml` kita sudah identik dengan punya Azhar + tambahan
> di atas.

---

## 3. Arsitektur cmd_vel (penting dipahami)

```
Nav2 controller ──/cmd_vel──► stm32_bridge ──UART──► STM32 ──► motor + servo
                    ▲
                    │ (gate: autonomous_enabled=true  &  R1 TIDAK ditekan)
```

- **Mode DEMO sekarang (tanpa failover):** Nav2 → `/cmd_vel` langsung. Failover node
  **harus dimatikan** (kalau hidup, dia paksa EMERGENCY_STOP karena salah menilai map
  latched sebagai "basi" + LiDAR return palsu ~0.10 m).
- **`autonomous_enabled`** = "rem parkir software". Default `false` demi keamanan.
  Set `true` supaya bridge mau terima perintah Nav2. Dibaca live (langsung berlaku).
- **R1 joystick** = `manual_override` → kalau ditekan, cmd_vel Nav2 diabaikan (ini cara
  ambil alih manual). Lepas R1 = biarkan autonomous.

---

## 4. SOP Reproduce — bikin robot jalan otonom (dari step 0)

> Tiap terminal baru WAJIB diawali:
> ```bash
> export ROS_DOMAIN_ID=42
> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
> source /opt/ros/humble/setup.bash
> cd ~/amr_starter && source install/setup.bash
> ```

### (Sekali saja) Apply fix terbaru ke NUC
```bash
cd ~/amr_starter
curl -fsSL "https://raw.githubusercontent.com/Mervs111/autonomous-mobile-robot-ros2/main/src/amr_slam/config/nav2_params.yaml" -o src/amr_slam/config/nav2_params.yaml
curl -fsSL "https://raw.githubusercontent.com/Mervs111/autonomous-mobile-robot-ros2/main/src/amr_slam/launch/nav2.launch.py" -o src/amr_slam/launch/nav2.launch.py
colcon build --symlink-install --packages-select amr_slam
source install/setup.bash
```

### TERMINAL 1 — Sensor (TANPA failover)
```bash
ros2 launch amr_bringup amr_full.launch.py use_slam:=false use_nav2:=false use_rtabmap:=false use_vr:=false use_failover:=false
```

### TERMINAL 2 — Localization (load peta)
```bash
ros2 launch amr_3d_mapping rtabmap_localization.launch.py database_path:=$HOME/maps/lab_demo_18jun.db
```

### TERMINAL 3 — Nav2
```bash
ros2 launch amr_slam nav2.launch.py
```
Tunggu sampai: **`Managed nodes are active`** (tanpa error).

### TERMINAL 4 — Buka gerbang autonomous + kirim goal
```bash
# GERBANG WAJIB:
ros2 param set /stm32_bridge autonomous_enabled true

# Kirim goal 1.5 m lurus depan:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```
➡️ Robot jalan sendiri ~1.5 m lalu berhenti.

### Cek cepat kalau robot diam
```bash
ros2 topic echo /cmd_vel --once          # harus ada linear.x != 0
ros2 node list | grep stm32              # bridge harus hidup
# pastikan autonomous_enabled=true & R1 tidak ditekan & failover node MATI
```

---

## 5. Kirim goal lewat RViz (klik di peta, tanpa terminal)

```bash
export DISPLAY=:0 && xhost +local:      # fix display NoMachine
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```
- Set **Fixed Frame = `map`**.
- Klik tool **"2D Goal Pose"** → klik area kosong di peta + seret arah → lepas → robot jalan.
- ⚠️ Klik di **area lapang** (robot butuh ruang muter, radius belok 0.90 m). Jangan nempel tembok.

---

## 6. Known Issues / Batasan saat ini

1. **Peta `lab_demo_18jun.db` vs layout sekarang** — layout ruangan sudah berubah, jadi
   lokalisasi kadang lock-nya tipis + odom agak drift, dan planner sering `no valid path`
   ke titik yang di peta lama dianggap obstacle. **Solusi tuntas: mapping ulang layout
   sekarang.**
2. **Planner SmacPlannerHybrid (Ackermann)** butuh ruang manuver (radius belok 0.90 m).
   Goal di area sempit/dekat tembok → `exceeded maximum iterations / no valid path`.
   Pilih goal di area lapang.
3. **Tanpa failover = tanpa auto e-stop.** Joystick R1 = rem darurat manual. Selalu siap.
4. **`depth_scan` dimatikan di costmap** → obstacle avoidance cuma dari LiDAR (1 bidang
   horizontal). Rintangan sangat rendah/tinggi bisa terlewat. Kamera depth tetap dipakai
   penuh untuk lokalisasi (RTAB-Map), cuma tidak dipakai untuk costmap.
5. **Hardware:** jaga LiPo > 22 V (servo brownout), charge joystick (input drift kalau lowbat).

---

## 7. Catatan koordinasi tim

- **Semua fix ada di repo `Mervs111` (origin), branch `main`.** Apply ke NUC via `curl`
  raw GitHub (akun git NUC bukan punya kita, jadi `git pull` ambil repo Azhar).
- **JANGAN push ke repo Azhar (upstream)** — push upstream sudah di-DISABLE.
- **Folder `~/maps/` di NUC dipakai bareng.** Sepakati `lab_demo_18jun.db` = map kanonik;
  jangan dua orang benahi folder maps bareng. Sudah ada backup `lab_demo_18jun_LOCKED_DEMO.db`.
- File laporan korelasi (PCD, Metnum) bersifat **pribadi**, tidak masuk repo.

---

## 8. Next steps

1. **Mapping ulang layout E105 sekarang** → lokalisasi & planning jauh lebih mulus.
2. Tes goal beruntun / waypoint patrol di area lapang.
3. (Opsional) Benahi failover supaya bisa dipakai lagi: `map_timeout_s` besar (map
   localization itu latched) + filter LiDAR return < 0.15 m (RPLidar A1 min range).
4. (Opsional) Re-enable `depth_scan` di costmap setelah kalibrasi `min_obstacle_height`
   biar tidak bikin obstacle hantu lantai.

**Inti:** pipeline autonomous (sensor → lokalisasi → Nav2 → motor) **sudah tersambung
penuh dan terbukti menggerakkan robot.** Sisanya = kualitas peta + tuning.

---

## 9. Update 20 Juni — Re-map + Temuan Heading/Steering Flip

### Yang dilakukan
- Layout E105 berubah → `lab_demo_18jun.db` nggak cocok → **re-map bersih** ke
  `lab_demo_20jun.db` (1 loop, ~17 menit).
- Localization lock berhasil (loop_closure_id > 0 ✅).
- Nav2 planning jalan, goal via RViz diterima ✅.

### Temuan: robot muter/belok salah arah
Saat autonomous goal dikirim, robot muter jauh & belok ke arah salah. Tes manual:

| Tes | Perintah | Hasil fisik | Verdict |
|---|---|---|---|
| Maju | `linear.x: 0.3` | Robot **maju** | ✅ benar |
| RViz ikon | robot maju | ikon **mundur** | ❌ heading flip |
| Belok | `angular.z: +0.5` (kiri) | robot **belok kanan** | ❌ steering kebalik |
| Odom | maju | `/rtabmap/odom x` **naik** | ✅ VIO odom lokal benar |

**Kesimpulan sementara:** heading di frame `map` ke-flip ~180° (robot tahu *tempat*
tapi *arah hadap* kebalik). Kombinasi 2 kemungkinan:
1. **Steering sign kebalik** di `stm32_bridge.cpp` → perintah belok kiri → roda depan
   belok kanan → Nav2 koreksi ke arah salah → muter.
2. **Heading lokalisasi flip 180°** → RTAB-Map lock posisi benar tapi orientasi kebalik.

### Tes penentu (BELUM SELESAI — robot mati karena switch daya)
```bash
# Di bench (roda ngambang):
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.5}}"
```
**Lihat roda depan:**
- Roda depan belok **KIRI** → servo benar → penyebab: heading lokalisasi flip
- Roda depan belok **KANAN** → **steering sign kebalik di bridge** → fix: flip tanda
  `steer_rad` di `cmd_vel_callback` (1 baris, rebuild ~2 menit)

### Fix steering (kalau roda depan terbukti kebalik)
Di [`src/amr_controller/src/stm32_bridge.cpp:187`](../src/amr_controller/src/stm32_bridge.cpp):
```cpp
// Sebelum (kebalik):
double steer_rad = std::atan(WHEELBASE * w / v);
// Sesudah (flip sign):
double steer_rad = -std::atan(WHEELBASE * w / v);
```
Lalu rebuild: `colcon build --symlink-install --packages-select amr_controller`

### Fix heading lokalisasi (kalau servo terbukti benar)
Taruh robot **persis di titik & arah START waktu mapping** lalu restart T2 localization.
RTAB-Map ambil pose awal dari origin peta — kalau orientasi fisik sama dengan saat
mapping, heading langsung benar dari awal.
