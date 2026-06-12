# HANDOVER — PLAN A: Navigasi Otonom Berbasis Peta (Nav2)

**Tanggal:** 12 Juni 2026
**Untuk:** Rekan tim yang melanjutkan navigasi otonom **berbasis peta**
**Status:** 🟡 Semua komponen SIAP — terganjal 1 blocker: **localization (VIO) butuh tekstur**

> **Bedakan dengan Plan B:** `amr_loop_patrol.py` (Plan B) **TIDAK pakai peta** — itu cuma
> dead-reckoning encoder buat keliling bentuk geometris. **Plan A ini yang BENERAN pakai
> peta acuan** (robot tau posisinya, navigasi ke koordinat peta, hindar obstacle). Plan A =
> deliverable "navigasi otonom berbasis peta" yang diminta dosen.

---

## 1. TUJUAN
Robot bernavigasi **otonom** memakai peta acuan `~/maps/mapping_20260611.db` (sudah ACC dosen):
mengelilingi ruangan ke beberapa titik tujuan lalu kembali ke titik awal, **menghindari
obstacle**, lewat Nav2.

Rantai: `Sensor → Localization (load peta) → Nav2 (plan + control) → cmd_vel → STM32 → motor`

---

## 2. YANG SUDAH SIAP (tinggal dijalankan)

| Komponen | Lokasi / nama | Status |
|---|---|---|
| **Peta acuan** | `~/maps/mapping_20260611.db` (234 node, loop closure, ACC dosen) | ✅ |
| **Backup peta** | `~/maps/backups/mapping_20260611_MASTER.db` *(buat dulu kalau belum!)* | ⚠️ |
| **Launch localization** | `src/amr_3d_mapping/launch/rtabmap_localization.launch.py` (`Mem/IncrementalMemory=false`) | ✅ |
| **Config Nav2 Ackermann** | `src/amr_slam/config/nav2_params.yaml` (SmacHybrid DUBIN, RPP, turn radius 0.90 m, **tanpa Spin**) | ✅ |
| **Launch Nav2** | `src/amr_slam/launch/nav2.launch.py` | ✅ |
| **Jalur autonomous cmd_vel** | `stm32_bridge.cpp` (param `autonomous_enabled`, watchdog 500 ms) | ✅ sudah di NUC |
| **Node patrol waypoint** | `src/amr_slam/scripts/amr_auto_patrol.py` | ✅ sudah di NUC |
| **Kirim 1 goal** | `src/amr_slam/scripts/goal_sender.py` | ✅ |

---

## 3. ⛔ BLOCKER UTAMA: Localization butuh VIO yang bisa "lihat" gerak

Nav2 hanya bisa navigasi kalau robot **tau posisinya di peta** (localization). Localization
butuh **VIO (kamera) menangkap gerakan robot**. Di lab polos, **VIO sering LOST** → pose beku
di titik 0 → `map→odom`, `localization_pose` mentok 0 → Nav2 mati langkah.

**Gejala terverifikasi (12 Juni):** robot didorong di lantai, tapi `tf2_echo odom base_link`
tetap `[0,0,0]` → VIO tidak lihat gerakan.

**AKAR & FIX (sama persis kayak yang bikin MAPPING berhasil):**
1. **Tempel tekstur** (poster/koran/kertas bermotif) di dinding setinggi ~0.3–0.8 m, di area
   yang dilewati robot. Dinding polos = kamera buta.
2. Ruangan terang merata, hadapkan kamera ke area bertekstur.
3. **Verifikasi VIO hidup SEBELUM lanjut:**
   ```bash
   ros2 run tf2_ros tf2_echo odom base_link   # dorong robot → Translation x,y HARUS berubah
   ```
   Kalau berubah → VIO sehat, lanjut. Kalau tetap 0 → tambah tekstur, jangan lanjut.

---

## 4. SOP MENJALANKAN (5 terminal)

> Tiap terminal WAJIB: `export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
> lalu `source ~/amr_starter/install/setup.bash`

**Terminal 0 — Backup peta (sekali, WAJIB):**
```bash
mkdir -p ~/maps/backups
cp ~/maps/mapping_20260611.db ~/maps/backups/mapping_20260611_MASTER.db
```

**Terminal 1 — Sensor:**
```bash
ros2 launch amr_bringup amr_full.launch.py use_slam:=false use_nav2:=false \
  use_rtabmap:=false use_vr:=false use_failover:=false
```
Tunggu: `RPLidar OK` + `RealSense Up!` + `[TX] V:0,S:0`.

**Terminal 2 — LOCALIZATION (load peta acuan):**
```bash
ros2 launch amr_3d_mapping rtabmap_localization.launch.py \
  database_path:=$HOME/maps/mapping_20260611.db
```
✅ **WAJIB cek:** log awal **MEMUAT** DB (sebut ~234 node), **BUKAN** "creating new database".
```bash
ros2 param get /rtabmap Mem/IncrementalMemory     # harus: false
```

**Terminal 3 — Verifikasi localization terkunci** (dorong robot pelan dulu di area bertekstur):
```bash
ros2 run tf2_ros tf2_echo map odom        # harus mengalir; melompat saat relokalisasi
ros2 topic echo /localization_pose        # terbit saat robot dikenali peta (TANPA prefix /rtabmap/)
```
> ⚠️ **Nama topik di setup ini TANPA prefix `/rtabmap/`** (kecuali `/rtabmap/odom`):
> `/map`, `/grid_prob_map`, `/localization_pose`, `/mapData`, `/cloud_map`, `/scan`.

**Terminal 4 — Nav2:**
```bash
ros2 launch amr_slam nav2.launch.py
```
✅ Cek: `ros2 action list | grep navigate_to_pose` → harus muncul.
> Kalau costmap komplain "no map": Nav2 butuh `/map`. Kalau cuma ada `/grid_prob_map`,
> jembatani: `ros2 run topic_tools relay /grid_prob_map /map`.

**Terminal 5 — AKTIFKAN & JALAN SENDIRI:**
```bash
ros2 param set /stm32_bridge autonomous_enabled true

# Tes SATU goal dulu:
ros2 run amr_slam goal_sender.py          # ketik: x y yaw  (mis: 1.5 0.5 0)

# Kalau sukses → patroli keliling + balik START:
ros2 run amr_slam amr_auto_patrol.py --ros-args \
  -p waypoints:="1.0,0.0,0; 2.0,1.0,90; 0.5,1.5,180; 0.0,0.0,270" -p loop:=true
```
> **Ambil koordinat waypoint dari peta:** RViz → toolbar **Publish Point** → klik titik di
> peta → `ros2 topic echo /clicked_point --once` → catat x,y.

---

## 5. ⚠️ HAL YANG BELUM SELESAI / HARUS DICEK

1. **`forward_only` belum ada di stm32_bridge NUC.** Nav2 recovery **BackUp** bisa kirim
   `linear.x < 0` → PWM **negatif** → firmware STM32 belum dukung itu. **Solusi pilih satu:**
   - Tambah param `forward_only` (sudah ada di repo Mervs111 `stm32_bridge.cpp`) secara
     **surgical** (jangan overwrite, ada fix sign Azhar): tambah `declare_parameter("forward_only", true)`
     + clamp `if(forward_only && velocity<0) velocity=0;` di `cmd_vel_callback`. Lalu rebuild.
   - **ATAU** hapus `BackUp` dari `behavior_plugins` di `nav2_params.yaml` (cegah Nav2 mundur).

2. **Arah belok (sign steering) belum 100% terverifikasi.** Bench test (roda diangkat):
   `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear:{x:0.15}, angular:{z:0.13}}' -r 10`
   → roda harus belok **KIRI** (angular.z + = kiri). Kalau kanan, balik tanda di
   `cmd_vel_callback`.

3. **`max_speed_mps` perlu kalibrasi.** Default 1.0 (cmd 0.3 m/s → PWM 1200). Tes: cmd 0.3 m/s
   → robot harus tempuh ±3 m / 10 dtk. Sesuaikan `ros2 param set /stm32_bridge max_speed_mps <nilai>`.

4. **`/depth_scan` ke costmap:** node `depthimage_to_laserscan` ada di launch mapping; pastikan
   juga aktif saat navigasi kalau mau obstacle dari depth (selain `/scan` LiDAR).

---

## 6. 🛑 REM DARURAT (hafalkan, 3 tingkat)
1. Pegang **R1** + stik netral → manual override instan (joystick selalu menang).
2. `ros2 param set /stm32_bridge autonomous_enabled false` → blok semua cmd_vel.
3. Ctrl+C node Nav2/patrol → watchdog stop motor ≤ 0.5 dtk.

---

## 7. TROUBLESHOOTING CEPAT

| Gejala | Penyebab / Solusi |
|---|---|
| `map→odom` & `localization_pose` mentok 0 saat robot jalan | VIO lost — **tambah tekstur dinding**, cek `tf2_echo odom base_link` berubah |
| Log localization "creating new database" | Salah path/config — Ctrl+C, pastikan `database_path` = `mapping_20260611.db` |
| Robot diam saat goal dikirim | `autonomous_enabled` false / R1 ketekan; cek `ros2 param get /stm32_bridge autonomous_enabled` |
| Robot stop sendiri tiap ±0.5 dtk | Watchdog — Nav2 tak publish cmd_vel kontinu; cek `ros2 topic hz /cmd_vel` |
| Nav2 tolak goal | Localization belum lock (TF map→odom) / goal di luar peta |
| Costmap "no map received" | `ros2 run topic_tools relay /grid_prob_map /map` |
| Robot belok terbalik | Balik tanda steering di `cmd_vel_callback` |

---

## 8. REFERENSI
- Peta acuan: `~/maps/mapping_20260611.db` (+ backup di `~/maps/backups/`)
- Repo: `Mervs111/autonomous-mobile-robot-ros2` (main) — `stm32_bridge.cpp` versi `forward_only`
  ada di sini kalau perlu di-merge surgical.
- SOP umum mapping/autonomous: `docs/SOP_MAPPING_DAN_AUTONOMOUS.md`
- Akses NUC: lihat `README.md` bagian "Akses NUC (Step-by-Step)".

**Inti pekerjaan tersisa = bikin LOCALIZATION stabil (tekstur).** Begitu VIO track & robot bisa
relokalisasi di peta, sisanya (Nav2 + patrol) sudah siap jalan.
