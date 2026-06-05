# HANDOVER — KORELASI PENGOLAHAN CITRA DIGITAL (PCD) DENGAN PROYEK AMR

**Untuk:** Claude (via claude.ai chat interface) — penyusun laporan korelasi mata kuliah
**Mahasiswa:** Muhammad Al Azhar Faradis, NRP 2040241017
**Mata kuliah:** Pengolahan Citra Digital (PCD)
**Proyek:** Autonomous Mobile Robot (AMR) Sasis Ackermann 2WS — Tugas Akhir TRO ITS
**Sesi tanggal:** 5 Juni 2026
**Repo:** `github.com/muhammadalazharf/autonomous-mobile-robot-ros2` branch `claude/brave-newton-6zvS4`

> Dokumen ini dirancang sebagai briefing self-contained. Setelah membaca ini, Claude chat bisa langsung menyusun bab "Korelasi PCD dengan Proyek AMR" tanpa perlu akses repo. Setiap klaim dilengkapi rujukan file + line untuk verifikasi.

---

## 1. RINGKASAN HARDWARE PENCITRAAN

| Komponen | Spesifikasi PCD-relevan |
|---|---|
| **Kamera Intel RealSense D455** | RGB stereo + active IR projector. Resolusi mapping: 848×480 @ 30 FPS. Depth range 0.6–6 m. Baseline 95 mm (lebih akurat dari D435). |
| **IMU on-board D455** | Accelerometer BMI055 @ 100 Hz, Gyroscope @ 200 Hz. Digunakan sebagai gravity constraint dan motion prior untuk visual tracking. |
| **RPLIDAR C1** | Pencitraan 2D range (bukan kamera) tapi *diperlakukan* sebagai citra polar untuk RANSAC line fitting. |

**Topic ROS 2 (verified working):**
```
/camera/camera/color/image_raw                       # RGB 848×480
/camera/camera/aligned_depth_to_color/image_raw      # Depth aligned ke RGB
/camera/camera/color/camera_info                     # Intrinsics K, distortion
/camera/camera/accel/sample
/camera/camera/gyro/sample
```

Alignment depth-to-color dilakukan di firmware D455 (`align_depth.enable=True`) — depth image berbagi grid pixel yang sama dengan RGB. Ini esensi PCD: **dua modalitas citra dengan registrasi pixel-wise**.

---

## 2. PETA KORELASI: TOPIK PCD → IMPLEMENTASI PROYEK

### 2.1. Model Citra Digital & Sampling

| Konsep PCD | Penerapan di AMR |
|---|---|
| Representasi citra `f(x,y)` matriks intensitas | RGB diterima sebagai `sensor_msgs/Image` dengan encoding `rgb8` (3 channel × 8 bit) |
| Depth image sebagai citra greyscale 16-bit | `aligned_depth_to_color` encoding `16UC1`, nilai pixel = jarak dalam mm |
| Spatial sampling rate | 848×480 dipilih (bukan 1280×720) → trade-off antara density depth dan CPU NUC i7 |
| Temporal sampling | 30 FPS — sinkron dengan rate IMU 100 Hz menggunakan ApproximateTimeSynchronizer |

**Bukti kode:** `src/amr_3d_mapping/launch/rtabmap_mapping.launch.py:122-136` (rgbd_sync node, sinkronisasi RGB+Depth+CameraInfo).

### 2.2. Kalibrasi Kamera & Model Pinhole

D455 sudah ter-factory-calibrated. Intrinsics dipublish di `camera_info`:
```
K = [fx  0  cx;  0  fy  cy;  0  0  1]
fx ≈ 428.5 px, fy ≈ 428.5 px (untuk 848×480)
cx ≈ 424.0,    cy ≈ 240.0
```

**Korelasi PCD:** Model pinhole `[u,v,1]ᵀ = (1/Z) · K · [X,Y,Z]ᵀ` digunakan setiap kali depth image diproyeksikan menjadi point cloud 3D oleh RTAB-Map.

### 2.3. Image Registration (Alignment)

**PCD:** Registrasi dua citra agar pixel/feature berkorespondensi.
**AMR:** Tiga lapis registrasi:
1. **Sensor-level (firmware D455):** depth aligned ke RGB pixel grid.
2. **Temporal (ROS):** RGB+Depth+IMU+LiDAR disinkronkan via `ApproximateTimeSynchronizer` dengan window 50 ms.
3. **Geometric (RTAB-Map):** Frame ke frame visual registration via feature matching (Vis) + ICP point cloud registration.

**Bukti:** `src/amr_3d_mapping/config/rtabmap_mapping.yaml:36` — `approx_sync_max_interval: 0.05`.

### 2.4. Feature Detection & Description

| PCD | Implementasi |
|---|---|
| **Corner detection** (Harris, Shi-Tomasi/GFTT) | `Vis/FeatureType: 8` = GFTT (Good Features To Track) untuk corner detection |
| **Feature descriptor** | BRIEF (Binary Robust Independent Elementary Features) — descriptor 256-bit per fitur |
| **Feature matching** | Brute-force Hamming distance pada deskriptor BRIEF |
| **Outlier rejection** | RANSAC pada hasil matching → MinInliers threshold |

**Parameter aktif** (`rtabmap_mapping.launch.py`):
- `Vis/MaxFeatures: 1000` — maksimum 1000 corner per frame
- `Vis/MinInliers: 2` (rgbd_odometry) / `10` (rtabmap SLAM)
- `GFTT/MinDistance: 5` — minimum jarak antar fitur (pixel)
- `GFTT/QualityLevel: 0.001` — threshold eigenvalue Shi-Tomasi

**Trial & error:** awalnya `MinInliers=5` dan `MaxFeatures=600` di area textureless (dinding polos lab), VIO sering kehilangan tracking → background rtabmap_viz merah. Diturunkan ke `MinInliers=2` dan `MaxFeatures=1000` agar lebih toleran. Diturunkan juga `GFTT/MinDistance` dari 10 → 5 (fitur lebih rapat = lebih banyak corner di area minim tekstur). Ini illustrasi konkret **trade-off precision vs recall pada feature detection**.

### 2.5. Visual Odometry (VIO) — Estimasi Gerak dari Citra

Bab paling padat PCD. Pipeline yang berjalan di node `rgbd_odometry`:

```
Frame_t (RGB + Depth)
    ↓ GFTT corner detection
    ↓ BRIEF descriptor extraction
    ↓ Match dengan local map (Frame-to-Map strategy)
    ↓ RANSAC PnP (Perspective-n-Point) menggunakan depth → 3D point
    ↓ Estimasi transformasi SE(3): R, t
    ↓ IMU prior (Odom/GuessMotion) untuk initial guess
    ↓ Cek confidence (Odom/MaxVariance)
    ↓ Output: pose /rtabmap/odom + TF odom→base_link
```

**Strategy yang dipilih:** `Odom/Strategy: 0` (Frame-to-Map) — setiap frame baru dimatch ke **akumulasi peta visual lokal** sebanyak 1000 keyframe (`OdomF2M/MaxSize: 1000`). Lebih robust dari Frame-to-Frame karena drift terkoreksi setiap frame.

**Korelasi PCD murni:**
- Pose estimation = problem **Perspective-n-Point (PnP)**: diberikan korespondensi 2D-3D, cari kamera pose.
- Sub-problem: solving `argmin Σ ||π(K·[R|t]·X_i) − x_i||²` (reprojection error).

### 2.6. Sensor Fusion (Visual + Inertial)

**Node:** `imu_merger_node.py` (`src/amr_controller/scripts/`)
- Input: `/camera/camera/accel/sample` (100 Hz), `/camera/camera/gyro/sample` (200 Hz) — terpisah karena format D455.
- Proses: gabungkan ke `sensor_msgs/Imu` tunggal dengan timestamp aligned.
- Output: `/imu/data` dikonsumsi `rgbd_odometry`.

**Peran PCD:** Citra → estimasi gerak relatif, IMU → integrasi percepatan/sudut. Fusion mengatasi:
- IMU drift (low-frequency error)
- Visual ambiguity (motion blur, area textureless)

Parameter `Optimizer/GravitySigma: 0.3` mengaktifkan **gravity vector constraint**: arah gravitasi dari IMU dijadikan prior agar peta tidak miring (drift roll/pitch).

### 2.7. Point Cloud sebagai Citra 3D

Setiap depth pixel `D(u,v)` diubah ke titik 3D:
```
Z = D(u,v) / 1000           # mm → meter
X = (u - cx) · Z / fx
Y = (v - cy) · Z / fy
```

Hasil → `sensor_msgs/PointCloud2` di `/cloud_map`. Ini secara harfiah adalah **citra 3D termasuk warna RGB** (XYZ + RGB per titik).

**PCD operasi yang dijalankan RTAB-Map:**
1. **Decimation** (`cloud_decimation: 2`) — subsampling 2:1 per sumbu = 1/4 pixel diambil → density reduction.
2. **Range filtering** (`cloud_min_depth: 0.3`, `cloud_max_depth: 4.0`) — buang depth di luar zona valid sensor.
3. **Voxel grid filter** (`cloud_voxel_size: 0.05`) — dedupe titik dalam grid 5cm³. Analog dengan **2D pixel binning** tapi 3D.
4. **Statistical outlier removal** (`cloud_noise_filtering_radius: 0.05`, `min_neighbors: 5`) — titik dengan tetangga < 5 dalam radius 5cm dianggap noise.

**File:** `rtabmap_mapping.launch.py:266-279`.

### 2.8. ICP — Iterative Closest Point

**PCD klasik untuk 3D:** ICP minimalkan jarak antara dua point cloud secara iteratif:
```
for iter in 1..N:
    1. Pairing: untuk tiap titik P_i di cloud_A, cari titik terdekat Q_i di cloud_B
    2. Transformasi: cari R, t yang minimalkan Σ ||R·P_i + t − Q_i||²  (SVD)
    3. Apply transformasi ke cloud_A
    4. Stop jika error change < epsilon
```

Implementasi di proyek:
- `Reg/Strategy: 2` → Visual features (Vis) untuk initial estimate + ICP LiDAR untuk fine correction.
- `Icp/PointToPlane: true` → varian ICP yang menggunakan normal permukaan (lebih cepat konvergen di lingkungan datar).
- `Icp/Iterations: 15`
- `Icp/MaxCorrespondenceDistance: 0.1` (10 cm)
- `Icp/Epsilon: 0.001`

**Catatan deliberate:** `Reg/Strategy=2` (Vis+ICP) **TIDAK BOLEH** diturunkan ke `1` (ICP only). Ini keputusan arsitektur — visual mempersempit search space sebelum ICP refine.

### 2.9. Loop Closure — Place Recognition via Image

Salah satu aplikasi PCD paling murni: **Bag-of-Words (BoW) image retrieval**:
1. Setiap keyframe diekstrak deskriptor visualnya.
2. Deskriptor di-quantize ke "visual word" via codebook.
3. Setiap frame baru dibandingkan dengan database BoW menggunakan TF-IDF similarity.
4. Jika similarity > `Rtabmap/LoopThr` (= 0.11), kandidat loop closure → diverifikasi geometris dengan PnP+ICP.

**Korelasi PCD:** ini adalah **content-based image retrieval (CBIR)** real-time, salah satu klasik PCD.

### 2.10. Depth Image → 2D LaserScan Conversion

Node `depthimage_to_laserscan`:
- Input: depth image 16UC1
- Proses: ambil baris pixel tertentu (`scan_height: 10` baris di tengah image), konversi ke jarak polar, output sebagai `LaserScan`.
- Output: `/depth_scan` digunakan Nav2 sebagai obstacle source tambahan.

**Korelasi PCD:** **dimensionality reduction** dari citra 2D depth → array 1D range. Berguna karena LiDAR RPLIDAR C1 tidak melihat obstacle yang lebih rendah/tinggi dari bidang scan-nya.

### 2.11. Visual Regression dengan RANSAC (di luar PCD klasik tapi terkait)

Pada lapisan kontrol jalur:
- Input: `/scan` LiDAR (polar range image)
- Proses: titik scan dikonversi ke Cartesian, kemudian **RANSAC line fitting** mendeteksi garis dinding lurus.
- Output: pose koreksi paralel terhadap dinding.

**Korelasi PCD:** RANSAC adalah algoritma robust estimation klasik yang lahir di PCD (Fischler & Bolles, 1981). Aplikasi di proyek = line detection (analog Hough Transform untuk garis).

---

## 3. ALUR DATA — PIPELINE PCD END-TO-END

```
┌──────────────────────────────────────────────────────────────┐
│ RealSense D455 (sensor layer)                                 │
│  - RGB 848×480 @ 30 FPS  -  Depth aligned (16UC1)             │
│  - IMU accel 100Hz + gyro 200Hz                               │
└──────────────────────────────────────────────────────────────┘
                          ↓
          ┌───────────────────────────────────┐
          │  imu_merger_node.py                │  ← sensor fusion
          │  accel+gyro → /imu/data            │
          └───────────────────────────────────┘
                          ↓
          ┌───────────────────────────────────┐
          │  rgbd_sync (rtabmap_sync)          │  ← temporal alignment
          │  RGB+Depth+Info → /rgbd_image      │
          └───────────────────────────────────┘
                          ↓
          ┌───────────────────────────────────┐
          │  rgbd_odometry  (VIO)              │
          │  ─ GFTT feature detection          │  ← corner detection
          │  ─ BRIEF descriptors               │  ← feature description
          │  ─ Frame-to-Map matching           │  ← image registration
          │  ─ PnP RANSAC + IMU prior          │  ← robust pose estimation
          │  ─ MaxVariance reject low-confid.  │  ← uncertainty filtering
          │  → /rtabmap/odom + TF odom→base    │
          └───────────────────────────────────┘
                          ↓
          ┌───────────────────────────────────┐
          │  rtabmap (SLAM engine)             │
          │  ─ Keyframe selection (Rehearsal)  │  ← image deduplication
          │  ─ BoW loop closure detection      │  ← CBIR
          │  ─ Vis+ICP registration            │  ← multi-modal regist.
          │  ─ Pose graph optimization (g2o)   │  ← global consistency
          │  ─ Voxel filter cloud_map          │  ← 3D point cloud DSP
          │  → /cloud_map /grid_map /map       │
          └───────────────────────────────────┘
                          ↓
          ┌───────────────────────────────────┐
          │  depthimage_to_laserscan           │  ← dim. reduction
          │  Depth → /depth_scan               │
          └───────────────────────────────────┘
                          ↓
                    Nav2 costmap
```

---

## 4. TRIAL & ERROR — KASUS DEBUGGING YANG RELEVAN UNTUK LAPORAN

### 4.1. Kasus: Scattered Cloud 8.9 Juta Vertex (5 Juni 2026)

**Gejala:** Hasil mapping diekspor ke `.ply`, dibuka di MeshLab tampak **8.951.244 vertex tersebar acak** — tidak ada struktur dinding/lantai yang koheren. Anti-target dari mapping.

**Penyebab (root cause analysis):**
1. **VIO kehilangan tracking** di area textureless (dinding lab polos warna putih) → estimasi pose meleset jauh.
2. Frame dengan pose buruk tetap di-akumulasi ke `cloud_map` karena filter uncertainty tidak aktif.
3. Setelah audit kode: parameter `Odom/MaxVariance: 0.01` ternyata **diletakkan di node yang salah** (rtabmap, bukan rgbd_odometry) — silently ignored oleh ROS 2.

**Fix:**
- Pindahkan `Odom/MaxVariance` ke node `rgbd_odometry` (lokasi yang benar).
- Tambah `Odom/ResetCountdown: 1` untuk auto-reset saat lost.
- Relaks visual feature: `MaxFeatures 600→1000`, `MinInliers 5→2`, `GFTT/MinDistance 10→5`.
- Aktifkan `Optimizer/GravitySigma: 0.3` untuk constraint IMU.

**Pelajaran PCD:** *Feature-based methods sensitif terhadap area minim tekstur*. Solusinya: relaks threshold dan kombinasikan dengan sensor lain (IMU gravity, LiDAR ICP).

### 4.2. Kasus: Depth-to-Color Misalignment

**Gejala awal:** Point cloud bewarna terlihat "offset" — warna dinding terprediksi di belakang dinding sebenarnya.

**Penyebab:** Depth raw tidak ter-align ke RGB. D455 publish dua depth: `/depth/image_rect_raw` (raw, in depth frame) dan `/aligned_depth_to_color/image_raw` (warped to RGB pixel grid).

**Fix:** Pastikan launch parameter `align_depth.enable: True` di realsense2_camera_node, dan subscribe ke topic yang benar di rgbd_sync.

**Pelajaran PCD:** *Image registration eksplisit wajib sebelum operasi multi-channel*. Topik klasik registrasi citra multispektral.

### 4.3. Kasus: Cloud Map Tidak Muncul Sama Sekali

**Gejala:** Topic `/cloud_map` ada tapi tidak menerbitkan data; di RViz kosong.

**Penyebab:** Parameter `publish_cloud_map` masih `False` di YAML, padahal sudah `True` di config — file YAML di `install/` adalah **stale copy** karena CMake `install(DIRECTORY)` melakukan COPY, bukan symlink.

**Fix:**
1. Inline semua parameter penting langsung di launch file (bypass YAML).
2. Atau build dengan `colcon build --symlink-install`.

**Pelajaran:** Diluar PCD tapi penting → reproduktivitas eksperimen.

### 4.4. Kasus: Comment Salah pada `Rtabmap/LoopThr`

**Gejala bug konseptual:** Kode menaikkan `LoopThr` dari 0.11 → 0.15 dengan komentar "lebih mudah accept loop closure padahal drift". Salah arah — LoopThr lebih tinggi = **lebih sulit** accept (butuh similarity lebih besar).

**Fix:** Kembalikan ke 0.11 (default RTAB-Map). Pelajaran: threshold-based decisions di PCD selalu butuh verifikasi arah inequality.

---

## 5. PARAMETER LENGKAP YANG MENJADI RUJUKAN PCD

Untuk dikutip langsung di laporan:

**File `src/amr_3d_mapping/launch/rtabmap_mapping.launch.py`:**

```python
# Visual feature extraction (Shi-Tomasi / GFTT + BRIEF)
'Vis/FeatureType': '8',          # 8 = GFTT/BRIEF
'Vis/MaxFeatures': '1000',
'Vis/MinInliers': '2',           # rgbd_odometry
'GFTT/MinDistance': '5',
'GFTT/QualityLevel': '0.001',

# Pose estimation (Frame-to-Map PnP)
'Odom/Strategy': '0',
'Odom/GuessMotion': 'true',
'Odom/MaxVariance': '0.01',
'Odom/ResetCountdown': '1',
'OdomF2M/MaxSize': '1000',

# ICP (geometric registration)
'Reg/Strategy': '2',             # Vis+ICP
'Icp/PointToPlane': 'true',
'Icp/Iterations': '15',
'Icp/VoxelSize': '0.05',
'Icp/MaxCorrespondenceDistance': '0.1',

# Loop closure (Bag-of-Words CBIR)
'Rtabmap/LoopThr': '0.11',
'RGBD/LocalRadius': '5.0',
'RGBD/ProximityMaxGraphDepth': '50',

# IMU constraint
'Optimizer/GravitySigma': '0.3',

# Point cloud post-processing
'cloud_decimation': 2,
'cloud_max_depth': 4.0,
'cloud_min_depth': 0.3,
'cloud_voxel_size': 0.05,
'cloud_noise_filtering_radius': 0.05,
'cloud_noise_filtering_min_neighbors': 5,
```

---

## 6. DAFTAR TOPIK PCD YANG DAPAT DI-CROSS-REFERENCE DI LAPORAN

Saat menulis bab korelasi, gunakan pemetaan ini sebagai outline:

| Bab Buku Gonzalez "Digital Image Processing" | Realisasi di Proyek AMR |
|---|---|
| Bab 2 — Digital image fundamentals | Format depth 16UC1, RGB 8UC3, sampling 30 FPS |
| Bab 3 — Intensity transformation & spatial filtering | Tidak digunakan langsung (RTAB-Map internal) |
| Bab 4 — Filtering in frequency domain | Tidak digunakan |
| Bab 5 — Image restoration | Noise filtering pada point cloud (statistical outlier) |
| Bab 6 — Color image processing | RGB pewarna point cloud (XYZRGB) |
| Bab 9 — Morphological processing | Costmap inflation Nav2 (dilation 2D) |
| Bab 10 — Image segmentation | `Grid/NormalsSegmentation` (matikan—2D mode) |
| Bab 11 — Representation & description | Feature descriptor BRIEF |
| Bab 12 — Object recognition | BoW loop closure |
| **(Lanjutan)** — Stereo / depth from images | Stereo D455 → depth image |
| **(Lanjutan)** — Camera calibration | Pinhole model + intrinsics K |
| **(Lanjutan)** — Visual SLAM | Seluruh pipeline RTAB-Map |

---

## 7. REFERENSI AKADEMIK YANG DIREKOMENDASIKAN

Untuk laporan formal, sitasi yang relevan:

1. **Gonzalez & Woods**, *Digital Image Processing*, 4th ed. — referensi utama PCD.
2. **Hartley & Zisserman**, *Multiple View Geometry in Computer Vision* — PnP, kalibrasi.
3. **Labbé & Michaud (2019)**, "RTAB-Map as an Open-Source LiDAR and Visual SLAM Library" — paper RTAB-Map yang dipakai proyek.
4. **Mur-Artal et al. (2015)**, "ORB-SLAM" — landasan teori feature-based visual SLAM.
5. **Besl & McKay (1992)**, "A Method for Registration of 3-D Shapes" — paper original ICP.
6. **Fischler & Bolles (1981)**, "Random Sample Consensus" — RANSAC.
7. **Rublee et al. (2011)**, "ORB: an efficient alternative to SIFT or SURF".
8. **Calonder et al. (2010)**, "BRIEF: Binary Robust Independent Elementary Features" — descriptor yang dipakai.

---

## 8. STRUKTUR LAPORAN YANG DISARANKAN

Untuk bab "Korelasi PCD dengan Proyek AMR" (kira-kira 15–20 halaman):

```
BAB X — Korelasi Mata Kuliah Pengolahan Citra Digital
  X.1 Pengantar
  X.2 Akuisisi Citra Multimodal pada AMR
        X.2.1 Citra RGB (sampling, kuantisasi)
        X.2.2 Citra Depth (16-bit, alignment)
        X.2.3 Sinyal IMU sebagai pelengkap
  X.3 Pre-processing
        X.3.1 Alignment depth-to-color
        X.3.2 Range filtering
        X.3.3 Voxel grid downsampling
  X.4 Feature Detection & Description
        X.4.1 Shi-Tomasi/GFTT
        X.4.2 Deskriptor BRIEF
        X.4.3 Trade-off di lingkungan textureless (kasus lab AMR)
  X.5 Image Registration
        X.5.1 Frame-to-Map PnP+RANSAC
        X.5.2 ICP geometric refinement
        X.5.3 Vis+ICP hybrid (Reg/Strategy=2)
  X.6 Visual SLAM Pipeline
        X.6.1 Visual-Inertial Odometry
        X.6.2 Pose graph optimization
        X.6.3 Bag-of-Words loop closure
  X.7 Studi Kasus Debugging
        X.7.1 Scattered cloud — analisis akar masalah PCD
        X.7.2 Misalignment depth-color
  X.8 Validasi Hasil & Analisis
        X.8.1 Metrik: jumlah vertex, koherensi visual, drift
        X.8.2 Hasil mapping lab (lab_map_20260605)
  X.9 Kesimpulan Korelasi
```

---

## 9. CATATAN PENGGUNAAN DOKUMEN INI DI CLAUDE CHAT

Saat memberikan dokumen ini ke Claude chat di claude.ai, sertakan instruksi:

> "Saya butuh kamu menulis bab `Korelasi Mata Kuliah Pengolahan Citra Digital dengan Proyek Autonomous Mobile Robot` untuk laporan Tugas Akhir, mengikuti outline di Section 8 dokumen briefing terlampir. Gunakan parameter konkret di Section 5 sebagai bukti implementasi. Sitasi formal IEEE style. Bahasa Indonesia formal akademik. Untuk setiap topik PCD yang dibahas, **wajib** sebutkan: (a) teori PCD, (b) realisasi di kode proyek (file:line), (c) parameter aktual yang dipakai, (d) trade-off engineering yang diambil."

Dokumen ini, ditambah file:
- `src/amr_3d_mapping/launch/rtabmap_mapping.launch.py`
- `src/amr_3d_mapping/config/rtabmap_mapping.yaml`
- `src/amr_controller/scripts/imu_merger_node.py`
- Foto-foto hasil mapping (cloud_map RViz, MeshLab export)

sudah cukup untuk menyusun bab korelasi yang substantive.

---

**Versi:** 1.0 — 5 Juni 2026
**Author:** Claude Code session (untuk Muhammad Al Azhar Faradis)
