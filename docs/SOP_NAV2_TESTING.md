# SOP Pengetesan Nav2 — Plan A (Navigasi Otonom Berbasis Peta)

**Prasyarat mutlak:** localization sudah LOCK (lihat `HANDOVER_LOKALISASI_17JUNI.md`).

## Pra-test (jangan skip)
- [ ] Localization LOCK (covariance <1, `loop_closure_id` >0)
- [ ] Joystick di-charge penuh; servo kencang; baterai LiPo >22 V
- [ ] Area luas & bersih (radius belok Ackermann 0,90 m — butuh ruang)
- [ ] E-STOP siap: tangan di R1

## Step 0 — tiap terminal
```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
cd ~/amr_starter && source install/setup.bash
```

## Urutan launch
```bash
# T1 — sensor
ros2 launch amr_bringup amr_full.launch.py use_slam:=false use_nav2:=false use_rtabmap:=false use_vr:=false use_failover:=false
# T2 — localization (VERIFY LOCK dulu sebelum lanjut!)
ros2 launch amr_3d_mapping rtabmap_localization.launch.py database_path:=$HOME/maps/lab_demo_17jun.db
ros2 topic echo /info | grep loop_closure_id                        # harus > 0
ros2 topic echo /localization_pose --field pose.covariance --once    # elemen pertama << 1
# T3 — Nav2
ros2 launch amr_slam nav2.launch.py
ros2 action list | grep navigate_to_pose                            # harus muncul
# T4 — cek costmap
ros2 topic echo /global_costmap/costmap --once | grep -E "width|height"
# kalau "no map": ros2 run topic_tools relay /grid_prob_map /map
```

## Tes bertahap (jangan langsung patrol)
1. **Costmap valid** di RViz (Map + costmap + pose benar).
2. **1 goal pendek:**
   ```bash
   ros2 param set /stm32_bridge autonomous_enabled true
   ros2 param set /stm32_bridge max_speed_mps 0.4
   ```
   RViz → "2D Goal Pose" → titik dekat & kosong (~1–2 m depan). TANGAN DI R1.
3. **Patrol** (kalau sukses):
   ```bash
   ros2 run amr_slam amr_auto_patrol.py --ros-args -p waypoints:="1.0,0.0,0; 2.0,1.0,90; 0.0,0.0,180" -p loop:=true
   ```

## E-STOP (3 tingkat)
1. R1 + stik netral (manual override) · 2. `ros2 param set /stm32_bridge autonomous_enabled false` · 3. Ctrl+C Nav2 (watchdog ≤0,5 dtk)

## Troubleshooting
| Gejala | Fix |
|---|---|
| costmap "no map received" | `ros2 run topic_tools relay /grid_prob_map /map` |
| robot diam pas goal | `autonomous_enabled` true? cek `ros2 topic hz /cmd_vel` |
| robot belok kebalik | cek sign steering (bench test roda diangkat) |
| goal ditolak | localization belum lock / goal di luar peta / di obstacle |
| stop tiap ~0,5 dtk | watchdog — Nav2 tak publish cmd_vel kontinu |
| robot mau mundur | `allow_reversing:false` → goal harus di DEPAN |

> Config Ackermann (SmacPlannerHybrid DUBIN, RPP, footprint, turning radius 0.90, tanpa Spin) ada di `amr_slam/config/nav2_params.yaml`.
