import cv2
import numpy as np
import csv
import yaml
import os

# --- CONFIGURATION ---
MAP_DIR = "../maps"
TPL_DIR = "../templates"
RAW_MAP = os.path.join(MAP_DIR, "raw_map.pgm")

TPL_RES = 0.001    # Original PNG template resolution (1mm)
FINAL_RES = 0.005  # Recommended Nav2 and SLAM resolution (5mm)
FIELD_SIZE = 4.0

def load_scaled_template(filename, target_res):
    """Loads a PNG template and resizes it to match the target map resolution."""
    img = cv2.imread(os.path.join(TPL_DIR, filename), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Missing template: {filename}")
    
    scale = TPL_RES / target_res
    w, h = int(img.shape[1] * scale), int(img.shape[0] * scale)
    # Use INTER_NEAREST to preserve binary 0/255 edges
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_NEAREST)

def create_wall_template(size=FIELD_SIZE, thickness=0.1, res=FINAL_RES):
    """Generates the 4x4 wall boundaries."""
    p_size, p_thick = int(size / res), int(thickness / res)
    img = np.zeros((p_size + p_thick*2, p_size + p_thick*2), dtype=np.uint8)
    cv2.rectangle(img, (p_thick, p_thick), (p_size+p_thick, p_size+p_thick), 255, p_thick)
    return img

def fit_template(map_img, template, angles):
    """Slides and rotates template to find highest correlation (x, y, yaw)."""
    best_val, best_loc, best_angle = -1, (0,0), 0
    h, w = template.shape
    for angle in angles:
        M = cv2.getRotationMatrix2D((w//2, h//2), angle, 1.0)
        rotated = cv2.warpAffine(template, M, (w, h))
        res = cv2.matchTemplate(map_img, rotated, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)
        
        if max_val > best_val:
            best_val, best_loc, best_angle = max_val, max_loc, angle
    return best_loc[0] + w//2, best_loc[1] + h//2, best_angle

def draw_on_map(canvas, template, x, y, yaw):
    """Stamps a rotated template onto the final map canvas."""
    h, w = template.shape
    M = cv2.getRotationMatrix2D((w//2, h//2), yaw, 1.0)
    rotated = cv2.warpAffine(template, M, (w, h))
    
    y1, y2 = max(0, y - h//2), min(canvas.shape[0], y + h//2 + (h%2))
    x1, x2 = max(0, x - w//2), min(canvas.shape[1], x + w//2 + (w%2))
    ry1, ry2 = max(0, h//2 - y), h - max(0, (y + h//2 + (h%2)) - canvas.shape[0])
    rx1, rx2 = max(0, w//2 - x), w - max(0, (x + w//2 + (w%2)) - canvas.shape[1])
    
    mask = rotated[ry1:ry2, rx1:rx2]
    canvas[y1:y2, x1:x2][mask > 0] = 0 # 0 is occupied

def main():
    os.makedirs(MAP_DIR, exist_ok=True)
    raw_map = cv2.imread(RAW_MAP, cv2.IMREAD_GRAYSCALE)
    _, binary_map = cv2.threshold(raw_map, 200, 255, cv2.THRESH_BINARY_INV)

    # 1. Fit Walls to establish origin
    wall_tmp = create_wall_template()
    w_x, w_y, w_yaw = fit_template(binary_map, wall_tmp, range(0, 90, 1))
    origin_x = int(w_x - (FIELD_SIZE / FINAL_RES) / 2)
    origin_y = int(w_y + (FIELD_SIZE / FINAL_RES) / 2)

    # 2. Match Objects
    objects = [
        ("Bridge", "bridge_lidar.png", "bridge_base.png", range(0, 360, 2)),
        ("Pyramid_1", "pyramid_lidar.png", "pyramid_base.png", range(0, 360, 2)),
        ("Pyramid_2", "pyramid_lidar.png", "pyramid_base.png", range(0, 360, 2))
    ]
    
    pixels = int(FIELD_SIZE / FINAL_RES)
    planner_canvas = np.full((pixels, pixels), 255, dtype=np.uint8)
    lidar_canvas = np.full((pixels, pixels), 255, dtype=np.uint8)
    cv2.rectangle(planner_canvas, (0,0), (pixels, pixels), 0, 2)
    cv2.rectangle(lidar_canvas, (0,0), (pixels, pixels), 0, 2)
    
    landmarks = []

    for name, lidar_png, base_png, angles in objects:
        lidar_tmp = load_scaled_template(lidar_png, FINAL_RES)
        base_tmp = load_scaled_template(base_png, FINAL_RES)
        
        # Match using the lidar cross-section against the SLAM map
        ox, oy, oyaw = fit_template(binary_map, lidar_tmp, angles)
        
        # Convert pixel to metric relative to bottom-left origin
        mx = (ox - origin_x) * FINAL_RES
        my = (origin_y - oy) * FINAL_RES
        rel_yaw = (oyaw - w_yaw) % 360
        landmarks.append((name, mx, my, rel_yaw))

        # Draw Base Footprint for Path Planning
        px_x, px_y = int(mx / FINAL_RES), pixels - int(my / FINAL_RES)
        draw_on_map(planner_canvas, base_tmp, px_x, px_y, oyaw)
        
        # Draw Lidar Cross-section for AMCL
        draw_on_map(lidar_canvas, lidar_tmp, px_x, px_y, oyaw)

    # 3. Export Output
    with open(os.path.join(MAP_DIR, "landmarks.csv"), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Object_Type", "X", "Y", "Yaw_Degrees"])
        writer.writerows(landmarks)

    cv2.imwrite(os.path.join(MAP_DIR, "planner_map.pgm"), planner_canvas)
    cv2.imwrite(os.path.join(MAP_DIR, "lidar_map.pgm"), lidar_canvas)

    yaml_data = {"resolution": FINAL_RES, "origin": [0.0, 0.0, 0.0], "occupied_thresh": 0.65, "free_thresh": 0.25, "negate": 0}
    for m in ["planner_map", "lidar_map"]:
        yaml_data["image"] = f"{m}.pgm"
        with open(os.path.join(MAP_DIR, f"{m}.yaml"), 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)

if __name__ == "__main__":
    main()