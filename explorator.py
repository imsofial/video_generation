import numpy as np
import open3d as o3d
import json
from scipy.ndimage import binary_opening, binary_closing

def load_ply(path):
    pcd = o3d.io.read_point_cloud(path)
    return pcd

def detect_floor_plane(pcd, distance_thresh=0.03):
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_thresh,
        ransac_n=3,
        num_iterations=500
    )
    [a, b, c, d] = plane_model
    if abs(c) < 0.8:
        raise RuntimeError("Floor plane not horizontal")
    floor_points = pcd.select_by_index(inliers)
    floor_height = np.mean(np.asarray(floor_points.points)[:, 2])
    return floor_height

def auto_fix_axes(pcd):
    pts = np.asarray(pcd.points)
    ranges = pts.max(axis=0) - pts.min(axis=0)

    vertical = np.argmin(ranges)

    if vertical == 2:
        perm = (0, 1, 2)
    elif vertical == 0:
        perm = (1, 2, 0)
    else:
        perm = (0, 2, 1)

    if perm != (0, 1, 2):
        rot = pts[:, perm]
        pcd.points = o3d.utility.Vector3dVector(rot)

    inv_perm = tuple(np.argsort(perm))
    return pcd, perm, inv_perm

def build_occupancy_grid(pcd, floor_z, res=0.1, person_height=1.0, floor_band=0.1, clearance_band=0.3, floor_threshold=1, obstacle_threshold=20, smooth_iters=1):
    pts = np.asarray(pcd.points)

    xmin, ymin = pts[:,0].min(), pts[:,1].min()
    xmax, ymax = pts[:,0].max(), pts[:,1].max()
    xmin = 0

    W = int((xmax - xmin) / res) + 1
    H = int((ymax - ymin) / res) + 1

    grid = np.ones((W,H), dtype=np.uint8)
    floor_support = np.zeros((W,H), dtype=np.uint16)
    obstacle_count = np.zeros((W,H), dtype=np.uint16)

    max_walkable_z = floor_z + person_height

    for x,y,z in pts:
        gx = int((x - xmin) / res)
        gy = int((y - ymin) / res)
        if not (0 <= gx < W and 0 <= gy < H):
            continue
        if abs(z - floor_z) <= floor_band:
            floor_support[gx, gy] += 1
        if floor_z + clearance_band <= z <= max_walkable_z:
            obstacle_count[gx, gy] += 1

    walkable = (floor_support >= floor_threshold) & (obstacle_count <= obstacle_threshold)

    if smooth_iters > 0:
        structure = np.array([[0,1,0],[1,1,1],[0,1,0]], dtype=bool)
        walkable = binary_opening(walkable, structure=structure, iterations=smooth_iters)
        walkable = binary_closing(walkable, structure=structure, iterations=smooth_iters)

    grid[walkable] = 0

    free_pts = []
    for gx in range(W):
        for gy in range(H):
            if grid[gx, gy] == 0:
                xw = xmin + gx * res
                yw = ymin + gy * res
                zw = floor_z
                free_pts.append([float(xw), float(yw), float(zw)])

    return free_pts

if __name__ == "__main__":
    PLY_FILE = "raw_data/ConferenceHall.ply"
    OUTPUT_JSON = "free_points.json"

    print("Loading PLY...")
    pcd = load_ply(PLY_FILE)

    print("Auto-fixing axes...")
    pcd, perm, inv_perm = auto_fix_axes(pcd)  

    print("Detecting floor plane...")
    floor_z = detect_floor_plane(pcd)
    print(f"Floor detected at Z = {floor_z:.2f}")

    print("Building occupancy grid and extracting free points...")
    free_pts = build_occupancy_grid(pcd, floor_z, res=0.5)
    print(f"Found {len(free_pts)} free points")

    print(f"Saving to {OUTPUT_JSON}...")
    with open(OUTPUT_JSON, "w") as f:
        json.dump(free_pts, f, indent=2)

    print("Done.")