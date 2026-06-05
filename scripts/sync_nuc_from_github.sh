#!/usr/bin/env bash
# =====================================================================
# sync_nuc_from_github.sh
# =====================================================================
# Tujuan: re-sync workspace NUC ke versi terbaru di GitHub
#         (branch claude/brave-newton-6zvS4), MENGGANTI versi yang
#         sudah di-push sebelumnya dari laptop.
#
# Yang dilakukan:
#   1. Backup database mapping & file lokal yang TIDAK boleh hilang
#   2. Fetch versi terbaru dari origin
#   3. RESET HARD ke origin/claude/brave-newton-6zvS4 (overwrite local)
#   4. Bersihkan artefak build lama
#   5. Rebuild dengan --symlink-install (agar YAML changes apply)
#   6. Source environment
#
# Pakai: jalankan dari root workspace di NUC
#   cd ~/autonomous-mobile-robot-ros2
#   bash scripts/sync_nuc_from_github.sh
# =====================================================================

set -e  # exit on any error

BRANCH="claude/brave-newton-6zvS4"
WS_DIR="$HOME/autonomous-mobile-robot-ros2"
BACKUP_DIR="$HOME/amr_backup_$(date +%Y%m%d_%H%M%S)"

echo "============================================="
echo "  AMR NUC Re-Sync from GitHub"
echo "  Branch: $BRANCH"
echo "  Backup dir: $BACKUP_DIR"
echo "============================================="
echo ""

# ---- 0. Sanity check ----
if [ ! -d "$WS_DIR/.git" ]; then
  echo "[ERROR] $WS_DIR bukan git repo. Abort."
  exit 1
fi
cd "$WS_DIR"

# ---- 1. Backup hal-hal yang tidak boleh hilang ----
echo "[1/6] Backup file penting ke $BACKUP_DIR ..."
mkdir -p "$BACKUP_DIR"

# Database RTAB-Map (hasil mapping)
if [ -f "$HOME/.ros/rtabmap.db" ]; then
  cp "$HOME/.ros/rtabmap.db" "$BACKUP_DIR/rtabmap.db"
  echo "  - rtabmap.db dibackup"
fi

# Folder maps (output mapping .pgm/.yaml)
if [ -d "$WS_DIR/maps" ]; then
  cp -r "$WS_DIR/maps" "$BACKUP_DIR/maps"
  echo "  - folder maps/ dibackup"
fi

# Backup file lokal yg di-modify (uncommitted changes) — kalau ada
if ! git diff --quiet || ! git diff --cached --quiet; then
  echo "  - ada uncommitted changes, simpan ke patch ..."
  git diff HEAD > "$BACKUP_DIR/local_uncommitted.patch"
fi

# ---- 2. Fetch versi terbaru ----
echo ""
echo "[2/6] Fetch dari origin ..."
git fetch origin "$BRANCH"

# ---- 3. Hard reset ke versi GitHub (overwrite local commit yg sebelumnya dipush) ----
echo ""
echo "[3/6] Reset hard ke origin/$BRANCH ..."
git checkout "$BRANCH" 2>/dev/null || git checkout -b "$BRANCH" "origin/$BRANCH"
git reset --hard "origin/$BRANCH"

# Bersihkan untracked file (kecuali maps/ dan .db yg sudah dibackup)
git clean -fd -e maps -e "*.db"

echo "  ✓ Local branch sekarang match dengan GitHub"
git log --oneline -5

# ---- 4. Bersihkan build artifacts lama ----
echo ""
echo "[4/6] Hapus build/ install/ log/ lama ..."
rm -rf build/ install/ log/

# ---- 5. Rebuild dengan symlink-install ----
echo ""
echo "[5/6] colcon build --symlink-install ..."
echo "      (symlink-install KRITIS: tanpa ini perubahan YAML"
echo "       di config/ tidak akan apply ke runtime)"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---- 6. Source environment ----
echo ""
echo "[6/6] Source install/setup.bash ..."
source install/setup.bash

echo ""
echo "============================================="
echo "  ✓ SYNC SELESAI"
echo "============================================="
echo ""
echo "Versi sekarang di NUC:"
git log --oneline -1
echo ""
echo "Backup data lama: $BACKUP_DIR"
echo ""
echo "Langkah selanjutnya:"
echo "  1. Verifikasi parameter VIO sudah benar:"
echo "     grep -A2 'Odom/MaxVariance\\|Odom/ResetCountdown' \\"
echo "         src/amr_3d_mapping/launch/rtabmap_mapping.launch.py"
echo "     (harus muncul di section rgbd_odometry_node, BUKAN rtabmap_slam_node)"
echo ""
echo "  2. Hapus database lama sebelum fresh mapping:"
echo "     rm ~/.ros/rtabmap.db"
echo ""
echo "  3. Jalankan mapping:"
echo "     ros2 launch amr_bringup amr_full.launch.py"
echo "     ros2 launch amr_3d_mapping rtabmap_mapping.launch.py"
echo ""
