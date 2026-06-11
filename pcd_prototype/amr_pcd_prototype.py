#!/usr/bin/env python3
"""
amr_pcd_prototype.py
====================
Prototipe Pipeline Pengolahan Citra Digital (Standalone) — AMR Ackermann ITS
Proyek  : PjBL VE230414
Penyusun: Mararevi Subagyo (NRP 2040241036)

Pipeline: Cropping ROI -> Resize -> Gaussian Blur -> Otsu Thresholding ->
          Morfologi (Opening+Closing) -> Canny Edge -> Contour Detection ->
          Logika Zona Navigasi (Aman / Peringatan / Bahaya)

Standalone (tanpa ROS 2) — untuk validasi algoritma + bukti empiris
(screenshot) Bab Analisa Korelasi Laporan Progres PjBL.

Jalankan: python3 amr_pcd_prototype.py   (tekan 'q' untuk keluar)
Fallback: jika RealSense D455 tidak terdeteksi, otomatis pakai webcam.
"""
import cv2
import numpy as np
import time

# ==========================================
# KONFIGURASI PARAMETER (Sesuai Laporan PjBL)
# ==========================================
TARGET_WIDTH = 640          # Lebar setelah resize
ROI_CROP_RATIO = 0.60       # Ambil 40% bagian bawah (mulai dari 60% tinggi)
BLUR_KERNEL = (5, 5)        # Gaussian Blur
MORPH_KERNEL = (5, 5)       # Morfologi
MIN_AREA = 500              # Filter noise kontur < 500 piksel
CANNY_LOW, CANNY_HIGH = 50, 150

# Zona Navigasi Ackermann
ZONE_WARN = 8000
ZONE_DANGER = 20000


def get_camera_stream():
    """Inisialisasi kamera: prioritas RealSense D455, fallback ke webcam."""
    try:
        import pyrealsense2 as rs
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        pipeline.start(config)
        print("[SUCCESS] Intel RealSense D455 Terdeteksi & Aktif.")
        return pipeline, "realsense"
    except Exception as e:
        print(f"[WARNING] RealSense tidak ditemukan ({e}). Menggunakan Webcam Bawaan.")
        return cv2.VideoCapture(0), "webcam"


def read_frame(stream, cam_type):
    """Membaca frame dari stream RealSense atau webcam."""
    if cam_type == "realsense":
        frames = stream.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        frame = np.asanyarray(color_frame.get_data())
    else:
        ret, frame = stream.read()
        if not ret:
            return None
    return frame


def main():
    print("Memulai AMR PCD Prototype... Tekan 'q' pada window untuk keluar.")
    stream, cam_type = get_camera_stream()

    while True:
        start_time = time.time()
        frame = read_frame(stream, cam_type)
        if frame is None:
            break

        h, w = frame.shape[:2]

        # 1. CROPPING ROI (Materi: Image Cropping)
        roi_y = int(h * ROI_CROP_RATIO)
        roi_frame = frame[roi_y:h, 0:w]

        # 2. RESIZE (Materi: Resizing)
        scale_factor = TARGET_WIDTH / roi_frame.shape[1]
        new_h = int(roi_frame.shape[0] * scale_factor)
        resized_frame = cv2.resize(roi_frame, (TARGET_WIDTH, new_h),
                                   interpolation=cv2.INTER_AREA)

        # 3. PREPROCESSING: Grayscale & Gaussian Blur (Materi: Smoothing)
        gray = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, BLUR_KERNEL, 0)

        # 4. SEGMENTASI: Thresholding Otsu (Materi: Thresholding)
        _, binary_mask = cv2.threshold(blurred, 0, 255,
                                       cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # 5. MORFOLOGI: Opening & Closing (Materi: Operasi Morfologi)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, MORPH_KERNEL)
        opened = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)
        closed_mask = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)

        # 6. DETEKSI TEPI: Canny (Materi: Edge Detection)
        edges = cv2.Canny(blurred, CANNY_LOW, CANNY_HIGH)

        # 7. ANALISIS SPASIAL: Contour Detection (Materi: Contour)
        contours, _ = cv2.findContours(closed_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        total_obstacle_area = 0
        detection_frame = resized_frame.copy()

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > MIN_AREA:
                total_obstacle_area += area
                x, y, w_box, h_box = cv2.boundingRect(cnt)
                cx, cy = x + w_box // 2, y + h_box // 2

                # Gambar Bounding Box dan Sentroid
                cv2.rectangle(detection_frame, (x, y), (x + w_box, y + h_box),
                              (0, 0, 255), 2)
                cv2.circle(detection_frame, (cx, cy), 4, (0, 255, 0), -1)
                cv2.putText(detection_frame, f"Area: {int(area)}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 8. LOGIKA NAVIGASI (Decision Making)
        status_text = "ZONA AMAN (Maju)"
        status_color = (0, 255, 0)  # Hijau

        if total_obstacle_area >= ZONE_DANGER:
            status_text = "ZONA BAHAYA (E-STOP!)"
            status_color = (0, 0, 255)  # Merah
        elif total_obstacle_area >= ZONE_WARN:
            status_text = "ZONA PERINGATAN (Belok/Pelan)"
            status_color = (0, 165, 255)  # Oranye

        # Tampilkan status di frame
        cv2.putText(detection_frame, f"TOTAL AREA: {int(total_obstacle_area)} px",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(detection_frame, status_text, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

        # Hitung FPS
        fps = 1 / (time.time() - start_time)
        cv2.putText(detection_frame, f"FPS: {int(fps)}",
                    (TARGET_WIDTH - 100, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # VISUALISASI (4 window untuk screenshot laporan)
        cv2.imshow("1. Original ROI (Resized)", resized_frame)
        cv2.imshow("2. Binary Mask (Otsu + Morph)", closed_mask)
        cv2.imshow("3. Canny Edges", edges)
        cv2.imshow("4. AMR Decision Output", detection_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    if cam_type == "realsense":
        stream.stop()
    else:
        stream.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
