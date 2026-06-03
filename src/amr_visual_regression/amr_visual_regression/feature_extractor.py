"""
feature_extractor.py
=====================
Modul shared untuk ekstraksi fitur dari depth image.
Dipakai di kedua tempat: data_collector_node, vr_inference_node, train.

Fitur per region:
  1. mean_depth        - rata-rata jarak (m)
  2. min_depth         - obstacle terdekat di region (m)
  3. free_space_ratio  - persentase pixel dengan depth > free_threshold
  4. std_depth         - standar deviasi (mengukur kompleksitas)

Default: 9 region vertikal x 4 fitur = 36 fitur per frame.
"""
import numpy as np


def extract_features(depth_image_uint16,
                     num_regions: int = 9,
                     roi: tuple = (200, 360),
                     free_threshold_m: float = 1.5,
                     max_depth_m: float = 6.0) -> np.ndarray:
    """
    Args:
        depth_image_uint16 : 2D numpy array (uint16), depth dalam mm
                             (RealSense default; pixel 0 = invalid)
        num_regions        : jumlah region vertikal (kolom)
        roi                : (top_row, bottom_row) untuk crop ROI
        free_threshold_m   : ambang free-space (m); pixel di atas ini dianggap kosong
        max_depth_m        : clamp depth maksimum (m)

    Returns:
        feature vector 1D shape = (num_regions * 4,)
    """
    # Crop ROI horizontal-strip (rows roi[0]..roi[1])
    depth_roi = depth_image_uint16[roi[0]:roi[1], :].astype(np.float32)

    # Convert mm -> m, mark invalid (==0 atau > max) as NaN
    depth_m = depth_roi / 1000.0
    depth_m[depth_m == 0.0] = np.nan
    depth_m[depth_m > max_depth_m] = np.nan

    # Bagi rata jadi num_regions kolom
    regions = np.array_split(depth_m, num_regions, axis=1)

    feats = np.zeros(num_regions * 4, dtype=np.float32)
    for i, r in enumerate(regions):
        # NaN-safe statistics
        valid_mask = ~np.isnan(r)
        n_valid = int(valid_mask.sum())

        if n_valid == 0:
            # Region full of invalid pixels -> assume "very close obstacle"
            feats[i*4 + 0] = 0.0   # mean
            feats[i*4 + 1] = 0.0   # min
            feats[i*4 + 2] = 0.0   # free-space ratio
            feats[i*4 + 3] = 0.0   # std
            continue

        valid_pixels = r[valid_mask]
        feats[i*4 + 0] = float(valid_pixels.mean())
        feats[i*4 + 1] = float(valid_pixels.min())
        feats[i*4 + 2] = float((valid_pixels > free_threshold_m).mean())
        feats[i*4 + 3] = float(valid_pixels.std()) if n_valid > 1 else 0.0

    return feats


def feature_names(num_regions: int = 9) -> list:
    """List nama fitur untuk debugging atau plot importance."""
    names = []
    for i in range(num_regions):
        names.extend([
            f'r{i}_mean',
            f'r{i}_min',
            f'r{i}_free',
            f'r{i}_std',
        ])
    return names


if __name__ == '__main__':
    # Quick self-test
    fake_depth = (np.random.rand(480, 640) * 5000).astype(np.uint16)
    feats = extract_features(fake_depth)
    print(f'Feature vector shape: {feats.shape}')
    print(f'Feature names: {feature_names()}')
    print(f'First 4 features (region 0): {feats[:4]}')
