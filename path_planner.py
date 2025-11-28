import json
import numpy as np

PANORAMA_FILE = "free_points.json"
OUTPUT_FILE = "smooth_panorama_path.json"
NUM_POINTS = 300
STEP_MIN = 0.01
STEP_MAX = 0.05

print("Loading free-space points...")
with open(PANORAMA_FILE, "r") as f:
    free_pts_raw = json.load(f)

free_pts = np.array(free_pts_raw)
if len(free_pts) == 0:
    raise RuntimeError("No free points found in panorama_path.json")

# ------------------------------------------------------
# 1. Starting point
# ------------------------------------------------------
start_idx = np.random.randint(0,300)
start_pt = free_pts[start_idx]
start_pt[2]-=1 ## as two RES variables (two blocks of ply)
path = [start_pt]

cur_pt = start_pt.copy()

# -------------------------------------------------------
# 2. Generating the path
# -------------------------------------------------------
axis = 0  # 0 = X, 1 = Y
direction1, direction2 = 1, 1
for i in range(NUM_POINTS - 1):
    step = np.random.uniform(STEP_MIN, STEP_MAX)
    cur_pt = cur_pt.copy()
    cur_pt[0] += step * direction1
    cur_pt[1] += step 
    # if not in available points - skip
    if cur_pt in free_pts:
        path.append(cur_pt)
    else:
        # two times for smooth redirection
        path.append(path[-1])
        path.append(path[-1])
    
    if i % 70 == 0:
        direction1 *= (-1)

path = np.array(path)
print("Generated simple path with", len(path), "points")

# ------------------------------------------------------
# 3. Save points in file
# ------------------------------------------------------
with open(OUTPUT_FILE, "w") as f:
    json.dump(path.tolist(), f, indent=2)

print(f"Saved simple path to {OUTPUT_FILE}")
