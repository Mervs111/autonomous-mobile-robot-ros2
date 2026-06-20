# HANDOVER — Nav2 Demo Siap (Safety Cap + BT Minimal) — 21 Juni 2026

**Dari:** Mararevi Subagyo (2040241036) · **Untuk:** Tim AMR (Kelompok PJBL 4-24)
**Status:** ✅ Robot autonomous **maju + auto-stop** (Plan A Nav2 + Plan B cmd_vel direct sebagai backup demo).

Lanjutan dari [HANDOVER_NAV2_AUTONOMOUS_19JUNI.md](HANDOVER_NAV2_AUTONOMOUS_19JUNI.md) (Nav2 BERHASIL jalan). Handover ini ngerangkum perbaikan kejadian malam 20 Juni (overshoot 7m+ keluar map).

---

## 1. TL;DR

Malam 20 Juni: pipeline Nav2 udah jalan (robot bergerak otonom, heading benar), **tapi overshoot 7m+ sampai keluar map**. Akar masalahnya **bukan** lokalisasi atau steering — itu udah beres. Ada **3 hal terpisah** yang bikin overshoot:

1. **BT default punya recovery liar** (`DriveOnHeading`/`spin`/`backup`). Saat goal aborted, recovery ini publish `cmd_vel` terus → robot lari walau Nav2 udah nyerah.
2. **`stm32_bridge` tidak punya hard-cap runtime**. `autonomous_enabled=true` = bridge taat semua `cmd_vel` apapun, tanpa batas waktu.
3. **Odom tidak ke-reset setelah robot diangkat manual ke start**. Goal "base_link + 0.5m" jadi target di odom yang udah accumulate 10+m → robot harus "lari" sampai odom x = 10.5 supaya nyampe target.

Sekarang **3 layer pengaman** terpasang:

| Layer | Mekanisme | File |
|---|---|---|
| BT minimal | Plan once + Follow once, NO recovery | [`navigate_to_pose_simple.xml`](../src/amr_slam/behavior_trees/navigate_to_pose_simple.xml) |
| Safety cap 10s | `autonomous_max_runtime_s` di bridge → paksa stop + autonomous_enabled=false | [`stm32_bridge.cpp`](../src/amr_controller/src/stm32_bridge.cpp) |
| Reset odom helper | Skrip pre-goal | [`reset_odom.sh`](../src/amr_slam/scripts/reset_odom.sh) |

Plus **Plan B (Plan A fallback):** [`demo_drive_forward.sh`](../src/amr_slam/scripts/demo_drive_forward.sh) — `cmd_vel` direct dengan `--times` limit (predictable, auto-stop, bypass Nav2).

Dan satu skrip bringup: [`run_amr_demo.sh`](../src/amr_slam/scripts/run_amr_demo.sh) — tmux 4-pane (T1 sensor / T2 lokalisasi / T3 Nav2 / T4 command).

---

## 2. Rantai diagnosis malam 20 Juni (yang baru beres pagi 21 Juni)

| Gejala | Tebakan awal | Akar sebenarnya |
|---|---|---|
| Heading kebalik 180° | Steering sign salah | Lokalisasi belum lock → map=odom identity → flip karena posisi awal beda dari mapping. Solved: turunin threshold (LoopThr 0.05, MinInliers 6). |
| `/info` keluar 33 lalu 0 | Threshold ketat | `MinInliers=20` (NUC pakai versi lama). Solved sama. |
| Cek topik `/rtabmap/info` kosong | Topik nggak ada | RTAB-Map di setup ini **tanpa namespace** — `/info`, `/localization_pose` (bukan `/rtabmap/...`). |
| `ros2 node list` kosong | Daemon basi | Kombinasi mati lampu + sempat eksperimen `ROS_LOCALHOST_ONLY=1` → discovery mismatch. Solved: env konsisten (TANPA localhost_only) + `ros2 daemon stop && start`. |
| Robot lari sampai keluar map | Lokalisasi drift | (a) BT recovery `DriveOnHeading` push `cmd_vel` walau goal aborted, (b) odom udah accumulate 10m karena temen angkat robot manual ke start. |

**Pelajaran besarnya:** lokalisasi sebenernya OK. Yang bikin chaos: **kombinasi recovery BT + autonomous tanpa hard-cap + odom yang nggak reset**. Tiga-tiganya udah dipatch.

---

## 3. Arsitektur safety sekarang

```
  Nav2 BT minimal              stm32_bridge                      STM32
  ────────────────             ────────────                       ─────
  ComputePathToPose            cmd_vel callback
       │                            │
       ▼                            ▼
  FollowPath ─── cmd_vel ──► gate: autonomous_enabled
       │ (gagal? abort)              │  & !manual_override (R1)
       │                             ▼
       │                       SAFETY: runtime > 10s?
       │                             │ ya → STOP + autonomous_enabled=false
       │                             │ tidak
       │                             ▼
       │                       Twist → Ackermann → V:{pwm},S:{sudut}
       │                                                  │
       ▼                                                  ▼
  goal SUCCEEDED / ABORTED                          motor + servo
```

**Layer pengaman dari atas ke bawah:**
1. **BT minimal** — `FollowPath` gagal = abort. Tidak ada `DriveOnHeading`/`spin` yang push `cmd_vel`.
2. **Manual override** — R1 ditekan = `manual_override_=true` → cmd_vel diabaikan.
3. **Gate `autonomous_enabled`** — default false, harus di-set true via `ros2 param set`.
4. **Watchdog 500ms** — kalau `cmd_vel` berhenti datang, motor auto-stop.
5. **Safety cap 10s** — kalau autonomous nyala terus > 10s, paksa stop + gate ditutup. **Ini perlindungan baru pasca-20 Juni.**

Lima-limanya independen. Salah satu kerja = robot stop.

---

## 4. SOP Reproduce (dari NUC mati)

### (a) Apply patch ke NUC (sekali saja, atau setiap kali ada update)
```bash
cd ~/amr_starter

# 5 file: bridge + nav2_params + BT + 3 skrip
curl -fsSL "https://raw.githubusercontent.com/Mervs111/autonomous-mobile-robot-ros2/main/src/amr_controller/src/stm32_bridge.cpp" \
  -o src/amr_controller/src/stm32_bridge.cpp
curl -fsSL "https://raw.githubusercontent.com/Mervs111/autonomous-mobile-robot-ros2/main/src/amr_slam/CMakeLists.txt" \
  -o src/amr_slam/CMakeLists.txt
curl -fsSL "https://raw.githubusercontent.com/Mervs111/autonomous-mobile-robot-ros2/main/src/amr_slam/config/nav2_params.yaml" \
  -o src/amr_slam/config/nav2_params.yaml
mkdir -p src/amr_slam/behavior_trees src/amr_slam/scripts
curl -fsSL "https://raw.githubusercontent.com/Mervs111/autonomous-mobile-robot-ros2/main/src/amr_slam/behavior_trees/navigate_to_pose_simple.xml" \
  -o src/amr_slam/behavior_trees/navigate_to_pose_simple.xml
for s in reset_odom.sh demo_drive_forward.sh run_amr_demo.sh; do
  curl -fsSL "https://raw.githubusercontent.com/Mervs111/autonomous-mobile-robot-ros2/main/src/amr_slam/scripts/$s" \
    -o "src/amr_slam/scripts/$s"
  chmod +x "src/amr_slam/scripts/$s"
done

colcon build --symlink-install --packages-select amr_controller amr_slam
source install/setup.bash

# wajib: pastikan tmux ada (sekali aja)
which tmux || sudo apt install -y tmux
```

### (b) Bringup satu klik
```bash
bash $(ros2 pkg prefix amr_slam)/lib/amr_slam/run_amr_demo.sh
```
Ini buka tmux 4-pane. Tunggu sampai pane Nav2 (kanan-bawah) muncul `Managed nodes are active`. Ctrl-b lalu d = detach (sesi terus jalan di background).

### (c) Plan A — Demo Nav2 (recommended)
Di pane T4 (kiri-bawah) ATAU terminal baru dengan env sama:
```bash
# WAJIB pertama: reset odom (akar masalah 20 Jun: robot diangkat → odom geser)
bash $(ros2 pkg prefix amr_slam)/lib/amr_slam/reset_odom.sh

# buka gerbang
ros2 param set /stm32_bridge autonomous_enabled true

# goal 0.5m lurus depan robot (frame base_link = "0.5m di depan moncong robot")
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```
🕹️ Pegang joystick, jari di **R1 = rem darurat manual**.

### (d) Plan B — Demo fallback (cmd_vel direct)
Kalau Plan A masih nakal:
```bash
bash $(ros2 pkg prefix amr_slam)/lib/amr_slam/demo_drive_forward.sh 0.5 0.2
# 0.5m maju @ 0.2 m/s, auto-stop, gerbang ditutup di akhir
```
Plan B **bypass Nav2 sama sekali** — predictable & anti-overshoot karena `--times` limit. Demo ini valid sebagai "robot otonom dengan jarak ditentukan dan berhenti otomatis."

### (e) Bunuh semua (kalau perlu reset bersih)
```bash
bash $(ros2 pkg prefix amr_slam)/lib/amr_slam/run_amr_demo.sh kill
```

---

## 5. Kirim goal lewat RViz (klik di peta)

```bash
export DISPLAY=:0 && xhost +local:
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```
- Set **Fixed Frame = `odom`** (penting! `map` masih bisa kedip)
- Klik tombol **Nav2 Goal** → klik area kosong di sekitar robot → seret arah hadap → lepas

⚠️ Sebelum klik goal, **jangan lupa reset_odom + autonomous_enabled=true**.

---

## 6. Yang berubah di kode (commit `a91d937`)

### `stm32_bridge.cpp`
```cpp
// param baru
this->declare_parameter("autonomous_max_runtime_s", 10.0);

// state baru
rclcpp::Time autonomous_started_;

// di cmd_vel_callback: catat saat autonomous mulai
if (!autonomous_active_) {
  autonomous_started_ = this->now();
  RCLCPP_INFO(..., "[AUTO] Started — max runtime cap: %.1fs", max_rt);
}

// di watchdog: hard-cap
double elapsed_s = (this->now() - autonomous_started_).seconds();
if (elapsed_s > max_rt) {
  autonomous_active_ = false;
  send_command(0, STEER_TRIM);
  this->set_parameter(rclcpp::Parameter("autonomous_enabled", false));
  RCLCPP_WARN(..., "[SAFETY] runtime > cap — STOP & autonomous_enabled=false");
}
```

### `nav2_params.yaml`
```yaml
bt_navigator:
  ros__parameters:
    # ganti BT default → custom minimal
    default_nav_to_pose_bt_xml: "/home/itssurabaya/amr_starter/install/amr_slam/share/amr_slam/behavior_trees/navigate_to_pose_simple.xml"
```

### `navigate_to_pose_simple.xml` (BT minimal)
```xml
<PipelineSequence name="NavigateNoRecovery">
  <RateController hz="1.0">
    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
  </RateController>
  <FollowPath path="{path}" controller_id="FollowPath"
              goal_checker_id="general_goal_checker"/>
</PipelineSequence>
```
No `RecoveryFallback`, no `DriveOnHeading`, no `spin`. Fail = abort bersih.

---

## 7. Known issues & batasan

1. **Lokalisasi RTAB-Map masih kedip** di layout sekarang (lock 33 → 0 → 33). Nav2 sekarang pakai `global_frame: odom` jadi nggak terpengaruh, **tapi** kalau mau tampilan map di RViz mulus → re-map lagi. Saat ini nggak penting buat demo.
2. **Safety cap 10s** → goal yang butuh lebih dari 10 detik bakal ke-paksa stop. Untuk goal jauh, naikkan param: `ros2 param set /stm32_bridge autonomous_max_runtime_s 30.0`.
3. **Plan B (cmd_vel direct)** = open-loop. Robot maju lurus tanpa peka obstacle. Cuma dipakai di ruang kosong, sebagai backup demo.
4. **`base_link` goal** transform sekali pas accept ke odom. Kalau robot tergeser SEBELUM goal di-accept (joystick dipake), target juga geser. **Mulai goal HANYA setelah robot diam.**
5. **Charge joystick** — input drift kalau lowbat.

---

## 8. Koordinasi tim

- Semua fix di repo `Mervs111` (origin) branch `main`. Push upstream (Azhar) **DISABLED**.
- Apply ke NUC via `curl` raw GitHub (akun git NUC bukan kita).
- **`maps/` folder dipakai bareng**: `lab_demo_20jun.db` = kanonik saat ini.
- File laporan (PCD, Metnum, Bengkel Otomasi) = **pribadi**, tidak masuk repo.

---

## 9. Roadmap singkat

- [x] Lokalisasi lock (17 Juni)
- [x] Re-map bersih (18 Juni → `lab_demo_18jun.db`)
- [x] Nav2 BERHASIL jalan (19 Juni)
- [x] Re-map layout baru (20 Juni → `lab_demo_20jun.db`)
- [x] Diagnosis steering & heading flip (20 Juni)
- [x] **Safety cap + BT minimal + reset_odom helper** (21 Juni — ini)
- [ ] Demo final (besok pagi)
- [ ] (Opsional) Tuning RTAB-Map biar lock stabil → nav2 di frame `map` lagi
- [ ] (Opsional) Re-enable depth_scan di costmap setelah kalibrasi

---

## 10. Kontak / Eskalasi

Kalau robot ngaco saat demo:
1. **Lepas R1, jangan tekan apa-apa lagi.** Watchdog 500ms auto-stop.
2. Kalau masih jalan: `ros2 param set /stm32_bridge autonomous_enabled false` di pane mana aja.
3. Kalau itu nggak juga: cabut kabel daya motor.

WhatsApp Mararevi kalau bingung. SOP step 0 ada di tiap handover sebelumnya.

**Inti 21 Juni:** safety net terpasang berlapis. Nav2 boleh ngaco, robot tetap stop dalam 10 detik max.
