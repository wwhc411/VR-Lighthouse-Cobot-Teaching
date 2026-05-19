"""Quick MLS diagnostic script"""
import json
import numpy as np

path = r"C:\Users\15421\Desktop\lighthouse_1.12\Assets\StreamingAssets\TrackerRecordings\mls_transform.json"

with open(path, 'r', encoding='utf-8') as f:
    data = json.load(f)

src = np.array(data['training_data']['source_points'])
tgt = np.array(data['training_data']['target_points'])

src_d = np.diff(src, axis=0)
tgt_d = np.diff(tgt, axis=0)
src_arc = np.sum(np.linalg.norm(src_d, axis=1))
tgt_arc = np.sum(np.linalg.norm(tgt_d, axis=1))

print(f"Source shape: {src.shape}, Target shape: {tgt.shape}")
print(f"Source arc length: {src_arc:.2f}mm")
print(f"Target arc length: {tgt_arc:.2f}mm")
print(f"Target/Source ratio: {tgt_arc/src_arc:.4f}")

print(f"\nSource range:")
print(f"  X: [{src[:,0].min():.1f}, {src[:,0].max():.1f}]")
print(f"  Y: [{src[:,1].min():.1f}, {src[:,1].max():.1f}]")
print(f"  Z: [{src[:,2].min():.1f}, {src[:,2].max():.1f}]")

print(f"\nTarget range:")
print(f"  X: [{tgt[:,0].min():.1f}, {tgt[:,0].max():.1f}]")
print(f"  Y: [{tgt[:,1].min():.1f}, {tgt[:,1].max():.1f}]")
print(f"  Z: [{tgt[:,2].min():.1f}, {tgt[:,2].max():.1f}]")

# Grid transforms analysis
grids = np.array(data['grid']['grid_transforms'])
print(f"\nGrid transforms shape: {grids.shape}")

translations = grids[:, :3, 3]
print(f"\nTranslation vector magnitudes:")
print(f"  |t| min: {np.linalg.norm(translations, axis=1).min():.1f}mm")
print(f"  |t| max: {np.linalg.norm(translations, axis=1).max():.1f}mm")
print(f"  |t| mean: {np.linalg.norm(translations, axis=1).mean():.1f}mm")

# Check rotation angles
for idx in [0, 50, 100, 150, 199]:
    R = grids[idx, :3, :3]
    angle_rad = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    angle_deg = np.degrees(angle_rad)
    t = grids[idx, :3, 3]
    print(f"  Grid[{idx}]: rotation={angle_deg:.2f}deg, |t|={np.linalg.norm(t):.1f}mm")

# Simulate what happens when applying grid transform to a source-like point
print("\n--- Simulating transform application ---")
test_idx = 750  # middle of trajectory
test_point = src[test_idx]
test_target = tgt[test_idx]
norm_s = test_idx / len(src)

# Find nearest grid
gpos = np.array(data['grid']['grid_positions'])
grid_idx = np.searchsorted(gpos, norm_s) - 1
grid_idx = np.clip(grid_idx, 0, len(gpos) - 2)

T = grids[grid_idx]
result = (T @ np.append(test_point, 1))[:3]
print(f"  Input point (src[{test_idx}]): {test_point}")
print(f"  Expected output (tgt[{test_idx}]): {test_target}")
print(f"  Actual output (T @ input): {result}")
print(f"  Error: {np.linalg.norm(result - test_target):.4f}mm")

# Now test with a DIFFERENT point at same arc position
offset = np.array([5.0, 5.0, 5.0])  # 5mm offset
test_point2 = test_point + offset
result2 = (T @ np.append(test_point2, 1))[:3]
print(f"\n  Offset input (src[{test_idx}]+5mm): {test_point2}")
print(f"  Output with offset: {result2}")
print(f"  Error from expected target: {np.linalg.norm(result2 - test_target):.4f}mm")

# Check what training source and target arc length the JSON stores
print(f"\n--- JSON stored values ---")
print(f"  total_arc_length: {data['total_arc_length']:.2f}")
print(f"  bandwidth: {data['bandwidth']}")
