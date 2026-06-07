# HANDOVER PROYEK AMR ITS — SESI 7 JUNI 2026

**Tanggal:** 7 Juni 2026
**Dari:** Sesi 7 Juni 2026
**Untuk:** Rekan tim yang melanjutkan
**Status:** ⚠️ Mapping belum berhasil — ada masalah rtabmap crash yang perlu diselesaikan

> **Catatan integrasi:** Dokumen ini adalah handover TERBARU (7 Juni), lebih baru dari
> `HANDOVER_4JUNI2026.md`. Ia MENGOREKSI beberapa klaim sebelumnya — lihat bagian
> "Rekonsiliasi" di bawah. Diintegrasikan ke repo pada sesi takeover (8 Juni).

---

## RINGKASAN EKSEKUTIF

Sistem VIO sudah terbukti bekerja dengan baik (27-30 Hz stabil, TF tracking smooth).
Namun **mapping belum berhasil diselesaikan** karena serangkaian masalah teknis yang
sudah diidentifikasi. Dokumen ini mencatat semua masalah dan solusinya agar sesi
berikutnya bisa langsung eksekusi tanpa debugging ulang dari awal.

---

## BAGIAN 1 — INFORMASI AKSES

**IP NUC (LAN lab, BUKAN Tailscale):**

```
10.17.36.151
```

**SSH dari laptop:**

```bash
ssh itssurabaya@10.17.36.151
```

**Akses visual langsung:** NoMachine — buka aplikasi NoMachine di laptop, connect ke IP NUC di atas.

**Environment variables wajib di setiap terminal:**

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## BAGIAN 2 — STATUS TERKINI

### Sudah selesai dan terbukti bekerja ✅
| Item | Status |
|------|--------|
| Hardware terdeteksi (LiDAR, RealSense, STM32) | ✅ |
| `/scan` 10 Hz stabil | ✅ |
| `/imu/data` 100 Hz stabil | ✅ |
| VIO `/rtabmap/odom` 27-30 Hz stabil | ✅ |
| TF tracking diverifikasi (Translation X naik smooth saat robot maju) | ✅ |
| `publish_tf: false` di odometry_publisher (tidak ada TF conflict) | ✅ |
| Nav2 params Ackermann-aware di-deploy | ✅ |
| Peta lama tersimpan di `~/maps/lab_vio_map.pgm` (darurat) | ✅ |

### Belum selesai ❌
| Item | Prioritas |
|------|-----------|
| Mapping lab bersih dengan loop closure | 🔴 Pertama |
| Localization (RTAB-Map mode localization) | 🔴 Kedua |
| Nav2 navigasi otonom A→B | 🔴 Ketiga |
| Obstacle avoidance | 🟡 Keempat |
| Visual Regression (RANSAC) | 🟡 Kelima |
| Video demonstrasi + slide | 🟡 Keenam |

---

## BAGIAN 3 — MASALAH YANG DITEMUKAN DAN SOLUSINYA

### Masalah 1 — rtabmap crash saat launch (TERAKHIR, BELUM SELESAI)
**Gejala:** Dialog "Sorry, the application rtabmap has stopped unexpectedly" saat
menjalankan `rtabmap_mapping.launch.py`. Topic `/rtabmap/grid_map` tidak pernah muncul.

**Kemungkinan penyebab:** database lama corrupt; memory NUC tidak cukup saat semua
sensor + rtabmap jalan bersamaan; dependency missing/versi tidak kompatibel.

**Yang harus dilakukan pertama kali:**
```bash
rm -f ~/maps/lab_vio.db
rm -f ~/.ros/rtabmap.db
ls ~/maps/
ls ~/.ros/*.db 2>/dev/null && echo "masih ada db" || echo "sudah bersih"
```
Lalu cek log crash:
```bash
ls -lt ~/.ros/log/ | head -5
cat ~/.ros/log/latest_*/rtabmap*/stdout.log 2>/dev/null | tail -50
```

### Masalah 2 — `/rtabmap/grid_map` tidak muncul
**Gejala:** `ros2 topic list | grep rtabmap` hanya `/rtabmap/odom` + `/rtabmap/republish_node_data`.
**Penyebab:** `amr_full.launch.py use_rtabmap:=true` tidak meneruskan `publish_grid_map: true` dengan benar.
**Solusi:** JANGAN pakai `amr_full.launch.py use_rtabmap:=true`. Selalu pakai **dua terminal terpisah** (Bagian 4).

### Masalah 3 — Loop closure tidak pernah terpicu
**Gejala:** `loop_closure_id: 0` dan `proximity_detection_id: 0` terus.
**Penyebab:** Robot tidak pernah keliling penuh & kembali ke titik start. Maju-mundur 2m tidak cukup (`Mem/STMSize: 10` → 10 node terakhir dikecualikan dari pencarian).
**Solusi:** Keliling penuh satu putaran menyusuri semua dinding, kembali ke titik start persis.

### Masalah 4 — VIO lost tracking di ruangan kecil
**Gejala:** Point cloud 3D "meledak", warna merah di rtabmap_viz.
**Penyebab:** Dinding polos tanpa tekstur; ruangan kecil → robot terlalu dekat dinding.
**Solusi:** Mapping di ruangan lebih besar dengan furniture; tempel kertas bermotif di dinding bila perlu.

### Masalah 5 — Peta 2D bergaris double
**Gejala:** Dinding di .pgm terlihat dua garis paralel.
**Penyebab:** Area sama dilewati dari dua estimasi posisi berbeda akibat odometri drift sebelum loop closure.
**Solusi:** Pastikan loop closure terpicu sebelum mapping area yang sama dua kali.

---

## BAGIAN 4 — PROSEDUR MAPPING YANG BENAR (DUA TERMINAL)

> ⚠️ WAJIB dua terminal terpisah. JANGAN `amr_full.launch.py use_rtabmap:=true`.

### Persiapan
```bash
rm -f ~/maps/lab_vio.db
rm -f ~/.ros/rtabmap.db
```

### Terminal 1 — Sensor (jangan ditutup)
```bash
export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd ~/amr_starter && source install/setup.bash && \
ros2 launch amr_bringup amr_full.launch.py \
  use_slam:=false use_nav2:=false use_rtabmap:=false use_vr:=false use_failover:=false
```
Tunggu: `RPLidar health status: OK`, `RealSense Node Is Up!`, `[stm32_bridge] [TX] V:0,S:0` berulang.

### Terminal 2 — RTAB-Map mapping (jangan ditutup)
```bash
export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/amr_starter/install/setup.bash
ros2 launch amr_3d_mapping rtabmap_mapping.launch.py database_path:=$HOME/maps/lab_vio.db
```
Tunggu 15 detik. Warning `DepthAsMask` & `Rejected loop closure` → normal.

### Terminal 3 — Verifikasi topic
```bash
source ~/amr_starter/install/setup.bash
ros2 topic list | grep rtabmap     # harus ada /rtabmap/grid_map, /rtabmap/odom, /rtabmap/info
ros2 topic hz /rtabmap/odom        # 27-30 Hz
```

### Terminal 4 — Pantau loop closure
```bash
source ~/amr_starter/install/setup.bash
watch -n 1 "ros2 topic echo /rtabmap/info --once | grep -E 'loop_closure_id|proximity'"
```

### Terminal 5 (opsional) — rtabmap_viz di NoMachine
```bash
source ~/amr_starter/install/setup.bash
ros2 run rtabmap_viz rtabmap_viz
```

---

## BAGIAN 5 — TEKNIK MENGEMUDI UNTUK MAPPING

**Aturan wajib:** Tahan **R1** terus (deadman); stik **10-15%** saja; **berhenti 3 detik** sebelum belok; **jangan keluar ruangan**; **jangan berputar di tempat**.

**Pola jalur ruangan 6 meja (2 kolom):**
1. Susuri dinding kiri → atas
2. Dinding atas → kanan
3. Dinding kanan → bawah
4. Dinding bawah → kembali START → **LOOP CLOSURE #1**
5. Lorong tengah → atas
6. Lorong tengah → bawah
7. Lorong kiri
8. Lorong kanan
9. Kembali START → **LOOP CLOSURE #2**

**Warna rtabmap_viz:** Hijau = tracking bagus (lanjut) · Kuning = mulai sulit (pelankan ke 5%) · Merah = lost (berhenti total, tunggu hijau).

**Tanda loop closure berhasil:** `loop_closure_id` berubah dari `0` ke angka positif.

---

## BAGIAN 6 — SIMPAN PETA

```bash
source ~/amr_starter/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_vio_map
ls -lh ~/maps/      # harus ada lab_vio_map.pgm (≥50K) + .yaml
```
**Peta bagus:** garis dinding jelas, area putih, tanpa garis double, tanpa garis radial berlebihan.

---

## BAGIAN 7 — SETELAH MAPPING BERHASIL

### Localization
```bash
# Terminal 1 tetap (sensor). Terminal 2:
source ~/amr_starter/install/setup.bash
ros2 launch amr_3d_mapping rtabmap_localization.launch.py database_path:=$HOME/maps/lab_vio.db
ros2 param get /rtabmap Mem/IncrementalMemory    # harus: false
```

### Nav2 navigasi otonom
```bash
export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd ~/amr_starter && source install/setup.bash && \
ros2 launch amr_bringup amr_full.launch.py \
  use_slam:=false use_nav2:=true use_rtabmap:=false use_vr:=false use_failover:=false \
  map:=$HOME/maps/lab_vio_map.yaml
```
RViz2 di laptop → **2D Pose Estimate** (set posisi awal) → **2D Goal Pose** (tujuan).

---

## BAGIAN 8 — PARAMETER TEKNIS PENTING (diubah di sesi 7 Juni)

| File | Parameter | Nilai | Alasan |
|------|-----------|-------|--------|
| `rtabmap_mapping.yaml` | `Mem/STMSize` | `10` | Loop closure lebih cepat di ruangan kecil |
| `rtabmap_mapping.yaml` | `Grid/NoiseFilteringRadius` | `0.5` | Buang titik noise sendirian |
| `rtabmap_mapping.yaml` | `Grid/NoiseFilteringMinNeighbors` | `5` | Minimum tetangga untuk titik valid |
| `odometry_publisher.py` | `publish_tf` | `False` | Cegah TF conflict dengan VIO |

> ⚠️ Param di atas mungkin diubah LANGSUNG di NUC dan belum tentu sudah di-push ke GitHub.
> Bandingkan dengan `13ee624` (commit GitHub terakhir, 5 Juni) saat reconcile.

---

## BAGIAN 9 — TROUBLESHOOTING CEPAT

| Masalah | Solusi |
|---------|--------|
| rtabmap crash saat launch | Hapus db lama; cek `~/.ros/log/latest_*/rtabmap*/stdout.log` |
| `/rtabmap/grid_map` tidak muncul | Jangan `amr_full use_rtabmap:=true`; pakai dua terminal |
| Loop closure tidak terpicu | Keliling penuh, kembali ke titik start persis |
| Point cloud 3D hancur (merah) | Ruangan terlalu kecil/polos, pindah ruangan lebih besar |
| Peta 2D bergaris double | Loop closure belum terpicu sebelum area dilewati 2× |
| Joystick tidak merespons | Tahan R1; tekan tombol PS untuk reconnect |
| SSH tidak konek | IP NUC `10.17.36.151`; cek jaringan |

---

## PESAN PENUTUP

Fondasi kuat — VIO bekerja, TF bersih, sensor stabil. Yang tersisa: menyelesaikan mapping.

**Terpenting pertama:** cek log crash rtabmap & pastikan database lama terhapus sebelum launch ulang.

**Urutan prioritas tersisa:**
1. 🔴 Selesaikan rtabmap crash
2. 🔴 Mapping lab baru bersih dengan loop closure
3. 🔴 Localization + Nav2 navigasi otonom
4. 🟡 Obstacle avoidance
5. 🟡 Visual Regression (RANSAC)
6. 🟡 Video demonstrasi + slide
