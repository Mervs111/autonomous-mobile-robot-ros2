# Prototipe Pipeline Pengolahan Citra Digital (PCD) — AMR Ackermann ITS

**Proyek:** Autonomous Mobile Robot (AMR) Ackermann — PjBL VE230414
**Penyusun:** Mararevi Subagyo (NRP 2040241036)
**Platform:** Ubuntu 22.04, Python 3.10+, OpenCV, PyRealSense2
**Status:** *Standalone Prototype* (pra-integrasi ROS 2) + draft node ROS 2

---

## 1. Deskripsi

Prototipe standalone untuk memvalidasi pipeline dasar Pengolahan Citra Digital
pada AMR ITS **tanpa beban middleware ROS 2**:

```
Kamera (D455/webcam)
  → 1. Cropping ROI        (Image Cropping)
  → 2. Resize              (Resizing)
  → 3. Grayscale + Blur    (Smoothing / Gaussian)
  → 4. Otsu Thresholding   (Segmentasi)
  → 5. Opening + Closing   (Operasi Morfologi)
  → 6. Canny               (Edge Detection)
  → 7. Contour Detection   (Analisis Spasial)
  → 8. Logika Zona Navigasi: AMAN / PERINGATAN / BAHAYA
```

Hasil running dipakai sebagai **bukti empiris (screenshot)** untuk Bab Analisa
Korelasi mata kuliah PCD pada Laporan Progres PjBL (Sub-CPMK 2 & 3).

## 2. Isi Folder

| File | Fungsi |
|---|---|
| `amr_pcd_prototype.py` | Prototipe standalone (4 window visualisasi) — **jalankan ini dulu** |
| `amr_perception_node.py` | Draft node ROS 2 (cv_bridge, QoS BEST_EFFORT, publish `/cmd_vel_visual`) — pakai SETELAH parameter valid |
| `README.md` | Dokumen ini |

## 3. Persiapan Environment (di NUC)

```bash
pip install opencv-python numpy pyrealsense2
```

> Fallback otomatis: jika RealSense D455 tidak terdeteksi, skrip memakai
> webcam — berguna untuk debugging di laptop.

⚠️ **Jangan jalankan bersamaan dengan node RealSense ROS** (`sensors_launch.py`)
— kamera hanya bisa dipegang satu proses. Matikan Terminal sensor dulu, atau
pakai webcam.

## 4. Cara Menjalankan (Standalone)

```bash
cd ~/Mervs111-amr/pcd_prototype     # sesuaikan path clone
python3 amr_pcd_prototype.py
```

Muncul **4 window**. Arahkan kamera ke rintangan (kardus/kaki). Tekan `q`
untuk keluar.

**Screenshot untuk laporan:**
- **Window 2 (Binary Mask)** — bukti Otsu + Closing mengisolasi objek dari lantai.
- **Window 4 (Decision Output)** — bounding box, sentroid, zona navigasi
  (Aman/Peringatan/Bahaya) + FPS. Bukti pemenuhan **Sub-CPMK 2 & 3**.

## 5. Parameter Tuning

| Parameter | Default | Arti |
|---|---|---|
| `ROI_CROP_RATIO` | 0.60 | Ambil 40% bagian bawah frame (area lantai) |
| `MIN_AREA` | 500 | Abaikan kontur lebih kecil dari ini (noise) |
| `ZONE_WARN` | 8000 | Total area piksel → zona PERINGATAN |
| `ZONE_DANGER` | 20000 | Total area piksel → zona BAHAYA (stop) |

Tuning di koridor/lab ITS, catat nilai akhirnya untuk laporan.

## 6. Roadmap Integrasi ROS 2

Setelah parameter valid: pakai `amr_perception_node.py` — pipeline yang sama
dibungkus node ROS 2:

- Subscribe `/camera/camera/color/image_raw` (QoS **BEST_EFFORT**)
- Publish `geometry_msgs/Twist` ke **`/cmd_vel_visual`** (BUKAN langsung
  `/cmd_vel`) — supaya melewati arbitrasi `amr_failover` bersama joystick &
  Nav2, sesuai arsitektur AMR.
- Debug visual: topic `/pcd_debug/image` (lihat via RViz/rqt_image_view).

```bash
source ~/amr_starter/install/setup.bash
python3 amr_perception_node.py
```

## 7. Cara Pull di NUC

Repo `~/amr_starter` di NUC menunjuk ke repo Azhar — folder ini ada di repo
**Mervs111**. Clone terpisah sekali saja:

```bash
cd ~
git clone https://github.com/Mervs111/autonomous-mobile-robot-ros2.git Mervs111-amr
cd Mervs111-amr/pcd_prototype
```

Update berikutnya cukup:

```bash
cd ~/Mervs111-amr && git pull
```
