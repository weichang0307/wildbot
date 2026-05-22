import cv2
import numpy as np
import csv
import yaml
import os

_HERE   = os.path.dirname(os.path.abspath(__file__))
MAP_DIR = os.path.join(_HERE, "..", "maps")
TPL_DIR = os.path.join(_HERE, "..", "templates")
RAW_MAP = os.path.join(MAP_DIR, "raw_map.pgm")

TPL_RES   = 0.001   # PNG template resolution (1 mm)
FINAL_RES = 0.01    # map resolution (10 mm)
FIELD_SIZE = 4.0


def load_scaled_template(filename, target_res):
    """Load a PNG template and resize it to match target_res."""
    img = cv2.imread(os.path.join(TPL_DIR, filename), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Missing template: {filename}")
    scale = TPL_RES / target_res
    w, h  = int(img.shape[1] * scale), int(img.shape[0] * scale)
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_NEAREST)


def create_wall_template(size=FIELD_SIZE, thickness=0.1, res=FINAL_RES):
    """Generate the 4×4 m arena boundary as a template."""
    p_size, p_thick = int(size / res), int(thickness / res)
    img = np.zeros((p_size + p_thick*2, p_size + p_thick*2), dtype=np.uint8)
    cv2.rectangle(img, (p_thick, p_thick), (p_size+p_thick, p_size+p_thick), 255, p_thick)
    return img


def fit_template(map_img, template, angles):
    """Slide and rotate template to find highest correlation. Returns (cx, cy, yaw)."""
    best_val, best_loc, best_angle = -1, (0, 0), 0
    h, w = template.shape
    for angle in angles:
        M       = cv2.getRotationMatrix2D((w//2, h//2), angle, 1.0)
        rotated = cv2.warpAffine(template, M, (w, h))
        res     = cv2.matchTemplate(map_img, rotated, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)
        if max_val > best_val:
            best_val, best_loc, best_angle = max_val, max_loc, angle
    return best_loc[0] + w//2, best_loc[1] + h//2, best_angle


def draw_on_map(canvas, template, cx, cy, yaw):
    """Stamp a rotated template (black=obstacle) onto canvas."""
    h, w = template.shape
    M       = cv2.getRotationMatrix2D((w//2, h//2), yaw, 1.0)
    rotated = cv2.warpAffine(template, M, (w, h))
    y1, y2  = max(0, cy - h//2), min(canvas.shape[0], cy + h//2 + h%2)
    x1, x2  = max(0, cx - w//2), min(canvas.shape[1], cx + w//2 + w%2)
    ry1, ry2 = max(0, h//2 - cy), h - max(0, cy + h//2 + h%2 - canvas.shape[0])
    rx1, rx2 = max(0, w//2 - cx), w - max(0, cx + w//2 + w%2 - canvas.shape[1])
    mask = rotated[ry1:ry2, rx1:rx2]
    canvas[y1:y2, x1:x2][mask > 0] = 0


def main():
    os.makedirs(MAP_DIR, exist_ok=True)
    raw_map = cv2.imread(RAW_MAP, cv2.IMREAD_GRAYSCALE)
    if raw_map is None:
        raise FileNotFoundError(f"raw_map.pgm not found at {RAW_MAP}")

    # Occupied cells → white (255), free/unknown → black (0)
    _, binary_map = cv2.threshold(raw_map, 200, 255, cv2.THRESH_BINARY_INV)

    # Dilate to fill gaps from sparse angular scan intervals before template matching
    binary_map = cv2.dilate(binary_map, np.ones((5, 5), np.uint8))

    # 1. Fit wall boundary to establish metric origin
    wall_tmp = create_wall_template()
    w_cx, w_cy, w_yaw = fit_template(binary_map, wall_tmp, range(0, 90, 1))
    origin_x = int(w_cx - (FIELD_SIZE / FINAL_RES) / 2)
    origin_y = int(w_cy + (FIELD_SIZE / FINAL_RES) / 2)

    # 2. Object list: (name, scan-height template, run-height template, base template, angles)
    #    scan template → used for fitting (matching against the raw SLAM map)
    #    run template  → drawn onto lidar_map  (what the lidar actually sees during the run)
    #    base template → drawn onto planner_map (physical footprint for path planning)
    objects = [
        ("Bridge",    "bridge_lidar_scan.png",   "bridge_lidar_run.png",   "bridge_base.png",   range(0, 360, 2)),
        ("Pyramid_1", "pyramid_lidar_scan.png",  "pyramid_lidar_run.png",  "pyramid_base.png",  range(0, 360, 2)),
        ("Pyramid_2", "pyramid_lidar_scan.png",  "pyramid_lidar_run.png",  "pyramid_base.png",  range(0, 360, 2)),
    ]

    pixels = int(FIELD_SIZE / FINAL_RES)
    planner_canvas = np.full((pixels, pixels), 255, dtype=np.uint8)
    lidar_canvas   = np.full((pixels, pixels), 255, dtype=np.uint8)
    cv2.rectangle(planner_canvas, (0, 0), (pixels, pixels), 0, 2)
    cv2.rectangle(lidar_canvas,   (0, 0), (pixels, pixels), 0, 2)

    landmarks  = []
    search_map = binary_map.copy()   # progressively masked to prevent duplicate matches

    for name, scan_png, run_png, base_png, angles in objects:
        scan_tmp = load_scaled_template(scan_png, FINAL_RES)
        run_tmp  = load_scaled_template(run_png,  FINAL_RES)
        base_tmp = load_scaled_template(base_png, FINAL_RES)

        # Match using scan-height template (what the lidar saw while mapping)
        cx, cy, oyaw = fit_template(search_map, scan_tmp, angles)

        # Mask out the matched region so the next pyramid can't match here
        sh, sw = scan_tmp.shape
        mx1 = max(0, cx - sw//2)
        mx2 = min(search_map.shape[1], cx + sw//2 + sw%2)
        my1 = max(0, cy - sh//2)
        my2 = min(search_map.shape[0], cy + sh//2 + sh%2)
        search_map[my1:my2, mx1:mx2] = 0

        # Convert pixel centre to metric coordinates (relative to wall origin)
        mx  = (cx - origin_x) * FINAL_RES
        my  = (origin_y - cy) * FINAL_RES
        rel_yaw = (oyaw - w_yaw) % 360
        landmarks.append((name, mx, my, rel_yaw))

        # Pixel coords within the final canvas
        px_x = int(mx / FINAL_RES)
        px_y = pixels - int(my / FINAL_RES)

        # Draw run-height cross-section on lidar_map (bridge invisible at 0.42 m)
        draw_on_map(lidar_canvas,   run_tmp,  px_x, px_y, oyaw)
        # Draw physical footprint on planner_map
        draw_on_map(planner_canvas, base_tmp, px_x, px_y, oyaw)

    # 3. Export
    with open(os.path.join(MAP_DIR, "landmarks.csv"), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Object_Type", "X", "Y", "Yaw_Degrees"])
        writer.writerows(landmarks)

    cv2.imwrite(os.path.join(MAP_DIR, "planner_map.pgm"), planner_canvas)
    cv2.imwrite(os.path.join(MAP_DIR, "lidar_map.pgm"),   lidar_canvas)

    yaml_base = {"resolution": FINAL_RES, "origin": [0.0, 0.0, 0.0],
                 "occupied_thresh": 0.65, "free_thresh": 0.25, "negate": 0}
    for name in ["planner_map", "lidar_map"]:
        with open(os.path.join(MAP_DIR, f"{name}.yaml"), 'w') as f:
            yaml.dump({**yaml_base, "image": f"{name}.pgm"}, f, default_flow_style=False)

    print(f"Done — landmarks, planner_map, lidar_map written to {MAP_DIR}")


if __name__ == "__main__":
    main()
