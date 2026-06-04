# HANDOVER SESI 4 JUNI 2026
## AMR Ackermann ITS — Proyek Tugas Akhir
**Pemilik:** Muhammad Al Azhar Faradis (NRP 2040241017, TRO ITS Surabaya)
**Ditulis oleh:** Claude Opus 4.6 (sesi 4 Juni 2026)
**Untuk:** Gemini / Claude Sonnet 4.6 — sesi berikutnya

---

## 1. STATUS PROYEK HARI INI

### Progress Keseluruhan (estimasi):
```
████████████████░░░░  75% — Total progress

✅ Hardware setup (URDF, sensor, STM32)              [100%]
✅ TF chain & static transforms                       [100%]
✅ VIO pipeline berjalan (29-30 Hz, quality 300+)     [100%]
✅ IMU debug → konfirmasi tidak rusak                 [100%]
✅ RTAB-Map SLAM engine berjalan penuh                [100%] ← BARU HARI INI
✅ rtabmap_viz terbuka + visualisasi aktif            [100%] ← BARU HARI INI
✅ Course correlation docs (PCD, Metnum, DCS, SCADA)  [100%]
🟡 Mapping quality (sedang di-test saat handover)     [ 60%] ← ONGOING
❌ Localization mode (belum di-test)                  [ 30%]
❌ Nav2 autonomous navigation (belum di-test)         [ 10%]
❌ Depth → Nav2 costmap integration                   [  0%]
❌ Goal sender node untuk demo otonom                 [  0%]
❌ Safety: deadman timeout di STM32 bridge            [  0%]
```

---

## 2. ROOT CAUSE YANG DITEMUKAN DAN SUDAH DIFIX HARI INI

### Bug Kritis 1 — YAML Namespace Mismatch (PENYEBAB UTAMA MAPPING SELALU JELEK)
**Commit:** `778070a`

YAML config `rtabmap_mapping.yaml` menggunakan section name `rtabmap_slam:`,
padahal node name di launch file adalah `rtabmap`. ROS 2 **SILENTLY IGNORE**
semua params kalau section name tidak match node name. Akibatnya:
- Semua loop closure params tidak pernah dimuat
- `publish_grid_map` tidak aktif → `/map` tidak publish
- Semua tuning ICP, Grid, Optimizer tidak aktif
- SLAM berjalan dengan DEFAULT params selama ini

**Fix:** Rename section `rtabmap_slam:` → `rtabmap:` di kedua YAML files.

### Bug Kritis 2 — Params Tidak Dimuat karena CMake Copy (bukan symlink)
**Commit:** `3de48ca`

CMake `install(DIRECTORY)` **COPIES** file ke `install/`, bukan symlink.
Perubahan YAML tidak ter-apply meski sudah rebuild. Solusi: pindahkan
semua params kritis ke **inline dict** di launch file Node() definition.
Ini bypass YAML loading sepenuhnya dan 100% pasti dimuat.

### Bug 3 — subscribe_odom_info menyebabkan sync stuck
**Commit:** `db350ec`

`subscribe_odom_info: true` membuat rtabmap SLAM menunggu topic `/odom_info`
yang tidak di-remap di launch file → sync queue tidak pernah terisi →
`/map` tidak publish. Fix: `subscribe_odom_info: false`.

### Bug 4 — Loop Closure tidak punya search radius
**Commit:** `1623d21`

Param `RGBD/LocalRadius` tidak di-set → default 0 → proximity search
tidak aktif. Robot bisa kembali ke titik start tapi RTAB-Map tidak
"lihat" overlap. Fix: tambahkan:
```
RGBD/LocalRadius: "5.0"
RGBD/ProximityMaxGraphDepth: "50"
RGBD/ProximityPathMaxNeighbors: "10"
```

### Bug 5 — Localization config tidak konsisten dengan Mapping
**Commit:** `1623d21`

`rtabmap_localization.yaml` punya params berbeda dari mapping:
- Reg/Strategy "1" (ICP only) vs mapping "2" (Vis+ICP)
- Optimizer/GravitySigma "0" vs mapping "0.3"
- rgbd_sync approx_sync true vs mapping false
- Vis/MinInliers 15 vs mapping 10

Semua sudah diseragamkan.

### Bug 6 — Robot terlalu cepat + Lab pencahayaan tidak konsisten
**Commit:** `9876dba`

Fix:
- VIO features lebih permissive: MaxFeatures 800, MinInliers 3, GFTT/MinDistance 7
- RealSense exposure tuning: gain=64, exposure=156
- Nav2 velocity limit: 0.4→0.3 m/s linear, 1.0→0.5 rad/s angular
- velocity_smoother max_accel: 2.5→1.0 m/s²

---

## 3. STATE SISTEM SAAT HANDOVER

### Yang sedang berjalan di NUC saat handover:
```bash
# Terminal 1: Full launch mapping
ros2 launch amr_bringup amr_full.launch.py \
    use_slam:=false use_rtabmap:=true rtabmap_mode:=mapping \
    use_nav2:=false use_vr:=false use_failover:=false

# Terminal 2: rtabmap_viz (monitoring visual)
export DISPLAY=:0
ros2 run rtabmap_viz rtabmap_viz
```

### Topic penting yang AKTIF:
```
/map                    ← OccupancyGrid untuk Nav2 (BARU AKTIF)
/info                   ← RTAB-Map SLAM info (loop closure detection)
/cloud_map              ← 3D point cloud
/cloud_obstacles        ← obstacle point cloud
/cloud_ground           ← ground point cloud
/rtabmap/odom           ← VIO pose (28-30 Hz)
/rgbd_image             ← RGB-D synced (30 Hz)
/scan                   ← LiDAR (10 Hz)
/imu/data               ← merged IMU (100 Hz)
```

**CATATAN PENTING:** Topic RTAB-Map tidak pakai prefix `/rtabmap/`:
- `/map` BUKAN `/rtabmap/grid_map`
- `/info` BUKAN `/rtabmap/info`
- `/cloud_map` BUKAN `/rtabmap/cloud_map`

---

## 4. ARSITEKTUR SISTEM (RINGKASAN)

```
[RealSense D455]
    ├── RGB 848×480 30Hz
    ├── Depth (aligned) 848×480 30Hz
    ├── Accel 100Hz → imu_merger_node.py → /imu/data
    └── Gyro 200Hz ↗

    ↓
[rgbd_sync] → /rgbd_image (30Hz)
    ↓
[rgbd_odometry] VIO = RGB-D + IMU → /rtabmap/odom (30Hz)
    + TF: odom → base_link
    ↓
[rtabmap] SLAM engine
    Input: /rgbd_image + /scan + /rtabmap/odom
    Output: /map (OccupancyGrid) + /info + /cloud_map
    + TF: map → odom

[Nav2] ← /map
    → /cmd_vel_nav

[failover_controller]
    Input: /cmd_vel_nav + /cmd_vel_joy + /cmd_vel_visual
    Output: /cmd_vel → STM32
```

### Workspace Structure di NUC:
```
~/amr_starter/        ← workspace utama (= repo GitHub)
  src/
    amr_bringup/      ← launch files (entry point: amr_full.launch.py)
    amr_3d_mapping/   ← RTAB-Map config + launch
    amr_controller/   ← STM32 bridge, odometry, IMU merger
    amr_slam/         ← SLAM Toolbox, Nav2 config
    amr_description/  ← URDF (JANGAN DIUBAH)
    amr_visual_regression/ ← CNN-less obstacle avoidance
    amr_failover/     ← cmd_vel arbitration

~/amr_underlay_ws/    ← dependency pre-built (rtabmap_ros, nav2, dll)
~/amr_data/           ← dataset visual regression
~/maps/               ← saved maps
~/.ros/rtabmap.db     ← RTAB-Map database aktif
```

---

## 5. PARAMETER KRITIS YANG HARUS DIPAHAMI

### VIO (rgbd_odometry) — `src/amr_3d_mapping/config/rtabmap_mapping.yaml`:
```yaml
Odom/Strategy: "0"          # Frame-to-Map (lebih robust)
Odom/GuessMotion: "true"    # IMU bantu prediksi motion
Vis/MaxFeatures: "800"      # Fitur yang di-track per frame
Vis/MinInliers: "3"         # Minimum match (3 = toleran area gelap)
GFTT/MinDistance: "7"       # Jarak pixel antar fitur (7 = lebih rapat)
Reg/Force3DoF: "true"       # Ground vehicle: hanya x, y, yaw
OdomF2M/MaxSize: "1000"     # Local map size
```

### SLAM (rtabmap) — `src/amr_3d_mapping/launch/rtabmap_mapping.launch.py` (INLINE):
```python
'Reg/Strategy': '2',              # Vis+ICP (JANGAN DIUBAH)
'Reg/Force3DoF': 'true',          # 2D floor
'RGBD/LocalRadius': '5.0',        # Loop closure search radius (meter)
'RGBD/ProximityMaxGraphDepth': '50',
'Rtabmap/LoopThr': '0.11',        # Loop closure threshold
'Grid/FromDepth': 'false',        # LiDAR only untuk grid (lebih bersih)
'Grid/CellSize': '0.05',          # 5cm resolution (match Nav2)
'publish_grid_map': True,         # WAJIB True untuk Nav2
'Optimizer/GravitySigma': '0.3',  # IMU gravity constraint
```

### Nav2 — `src/amr_slam/config/nav2_params.yaml`:
```yaml
desired_linear_vel: 0.3      # m/s (JANGAN > 0.3 untuk mapping)
minimum_turning_radius: 0.90 # m (Ackermann: L/tan(30°)=0.866 + safety)
max_velocity: [0.3, 0.0, 0.5]
use_rotate_to_heading: false # Ackermann tidak bisa rotate in place
```

---

## 6. TUGAS YANG BELUM SELESAI (PRIORITAS URUT)

### Prioritas 1 — Selesaikan Mapping (HARI INI / SEGERA)
Setelah mapping run selesai, simpan map:
```bash
# Simpan occupancy grid 2D ke file .pgm
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map

# Backup RTAB-Map database 3D
cp ~/.ros/rtabmap.db ~/maps/lab_3d_$(date +%Y%m%d).db
```

### Prioritas 2 — Verifikasi Loop Closure
Saat robot kembali ke titik start:
```bash
ros2 topic echo /info --once | grep -E "loop_closure|proximity"
# Target: loop_closure_id > 0
```

### Prioritas 3 — Test Localization Mode
Setelah map tersimpan:
```bash
ros2 launch amr_bringup amr_full.launch.py \
    use_slam:=false use_rtabmap:=true rtabmap_mode:=localization \
    rtabmap_db_path:=~/.ros/rtabmap.db \
    use_nav2:=false
```

### Prioritas 4 — Test Nav2 Autonomous (butuh map hasil mapping)
```bash
ros2 launch amr_bringup amr_full.launch.py \
    use_slam:=false use_rtabmap:=true rtabmap_mode:=localization \
    rtabmap_db_path:=~/.ros/rtabmap.db \
    use_nav2:=true
```
Kirim goal via RViz2 "Nav2 Goal" button atau Foxglove.

### Prioritas 5 — Tambah Depth ke Nav2 Costmap
Node `depthimage_to_laserscan` untuk convert depth D455 jadi virtual
laser scan, lalu tambah ke observation_sources di nav2_params.yaml.
Ini agar robot detect obstacle yang tidak terlihat LiDAR.

### Prioritas 6 — Goal Sender Node
Node Python sederhana untuk kirim goal ke Nav2 dari topic/terminal.
Untuk demo otonom tanpa RViz2.

### Prioritas 7 — Safety: Deadman Timeout STM32
**PENTING UNTUK KESELAMATAN:** `stm32_bridge.cpp` tidak punya timeout —
kalau ROS disconnect, STM32 terus jalankan command terakhir → robot
kabur. Perlu tambahkan deadman check: kalau tidak ada cmd_vel baru
dalam 500ms, kirim stop command ke STM32.

---

## 7. PERINTAH DARURAT

```bash
# Stop semua ROS node
pkill -f ros2

# Emergency stop manual (kalau robot kabur)
source ~/amr_starter/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' -1

# Cek semua node aktif
ros2 node list

# Cek TF chain (harus ada: map→odom→base_link→laser_frame,camera_link)
ros2 run tf2_tools view_frames

# Pull latest dari GitHub
cd ~/amr_starter && git pull origin claude/brave-newton-6zvS4
```

---

## 8. CONSTRAINT PENTING (WAJIB DIPATUHI)

1. **JANGAN UBAH URDF** — TF seluruh sistem bergantung padanya
2. **JANGAN REGENERATE launch file dari memori** — edit incremental saja
3. **Backup sebelum edit:** `cp <file> <file>.bak_$(date +%Y%m%d_%H%M%S)`
4. **Satu fix, satu verifikasi** — jangan ubah banyak hal sekaligus
5. **Reg/Strategy: "2" (Vis+ICP)** — ini keputusan deliberate, JANGAN ubah ke "1"
6. **publish_tf: False** di odometry_publisher — jangan aktifkan (TF conflict dengan VIO)
7. **use_slam: false** saat pakai RTAB-Map — jangan aktifkan keduanya

---

## 9. COMMIT HISTORY HARI INI

```
3de48ca Move all RTAB-Map SLAM params inline in launch file
778070a Fix critical YAML namespace mismatch: rtabmap_slam → rtabmap
db350ec Fix rtabmap SLAM stuck: disable subscribe_odom_info
9876dba Fix VIO robustness for fast motion + inconsistent lighting
1623d21 Fix loop closure tuning + localization config consistency
2e4cafb Add comprehensive handover for course correlation documentation
4b92522 Fix SLAM Toolbox vs RTAB-Map TF conflict: default use_slam=false
d7b649d Revert Reg/Strategy to 2 (Vis+ICP) — deliberate decision
0538636 Fix critical mapping bugs: VIO failure, TF conflict, wrong depth topic
a388c48 Upload workspace amr_starter - 3 Juni 2026
```

**GitHub:** `muhammadalazharf/autonomous-mobile-robot-ros2`
**Branch:** `claude/brave-newton-6zvS4`

---

## 10. CATATAN UNTUK AI PENERUS (GEMINI / SONNET 4.6)

### Yang sudah TERBUKTI BEKERJA:
- VIO quality 300+ stable di 30Hz ✅
- `/map` publish aktif ✅
- rtabmap_viz terbuka dan menampilkan 3D map ✅
- Loop ID bertambah saat robot bergerak ✅

### Yang BELUM PERNAH DITEST end-to-end:
- Loop closure saat kembali ke titik start
- Localization mode setelah mapping
- Nav2 autonomous navigation

### Cara debug yang sudah terbukti:
```bash
# Cek apakah SLAM params dimuat (inline params selalu dimuat):
ros2 param get /rtabmap Reg/Strategy     # harus: "2"
ros2 param get /rtabmap publish_tf       # harus: True

# Cek VIO quality:
ros2 topic echo /rtabmap/odom_info --once | head -10

# Cek topic yg publish (nama TANPA prefix /rtabmap/):
ros2 node info /rtabmap  # lihat Publishers section
```

### Jebakan yang sudah ditemukan:
1. YAML section name harus = node name → cek dengan `ros2 param list /nodename`
2. CMake install copy file, bukan symlink → params kritis harus inline di launch
3. Topic RTAB-Map tidak pakai prefix `/rtabmap/` di versi ini
4. `subscribe_odom_info: true` tanpa remap `/odom_info` → sync stuck
5. `git pull` perlu sebelum launch kalau ada perubahan di GitHub

---

*Handover ditulis: 4 Juni 2026, 20:40 WIB*
*Robot terakhir mapping run: sedang berlangsung*
