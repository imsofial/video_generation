
# Video Tour Generation from PLY Point Cloud

This repository provides a system for generating a video tour from a given `.ply` point cloud file. It includes several scripts for processing the point cloud, planning a camera path, and rendering the tour.  

**Before running any scripts, make sure all libraries in `requirements.txt` are installed in your environment.**

---

## 1. `explorator.py`

- Loads a `.ply` file.
- Detects the floor plane.
- Automatically fixes axis orientation.
- Builds an occupancy grid.
- Saves the free points to a JSON file named `free_points.json`.

**To run:**  
```bash
py explorator.py
````

---

## 2. `path_planner.py`

* Loads `free_points.json`.
* Finds starting points.
* Generates a smooth path using a simple algorithm.
* Saves the path points into `smooth_path.json`.

**To run:**

```bash
py path_planner.py
```

---

## 3. `index.html` (renderer)

* Renders the scene from the `.ply` points.
* Sets the camera on every point along the path.
* Provides smooth transitions between points.
* Records the video of the tour.

**To run the renderer server:**

```bash
py -m http.server 8000
```

**To open and start the video recording:**

* Open in browser: [http://localhost:8000](http://localhost:8000)
* When the recording is finished, the video will be downloaded in `.webm` format to your local storage (a notification will appear).

---

### Additional Notes

* All `.ply` were converted using **superspl.at** but was not loaded because of size.
* For path planning, random steps in the range `(0.01, 0.05)` were used for every new point.
* Video are saved in webm format, but converted manually for github.*

