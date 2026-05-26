import cv2
import numpy as np
import csv
import yaml
import os

_HERE   = os.path.dirname(os.path.abspath(__file__))
MAP_DIR = os.path.join(_HERE, "..", "maps")
TPL_DIR = os.path.join(_HERE, "..", "templates")
RAW_MAP = os.path.join(MAP_DIR, "raw_map.pgm")

TPL_RES   = 0.001
FINAL_RES = 0.01
PLANNER_RES = 0.05
FIELD_SIZE = 4.0

SCAN_BRIDGE = False

def load_scaled_template(filename, target_res):
    img = cv2.imread(os.path.join(TPL_DIR, filename), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Missing template: {filename}")
    img = np.where(img < 200, 255, 0).astype(np.uint8)  # black geometry → white mask
    if int(np.max(img)) == 0:
        return None
    ys, xs = np.where(img > 0)
    img = img[ys.min():ys.max()+1, xs.min():xs.max()+1]
    scale = TPL_RES / target_res
    w = max(1, int(round(img.shape[1] * scale)))
    h = max(1, int(round(img.shape[0] * scale)))
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_NEAREST)


def rotate_template_bound(template, angle):
    h, w = template.shape
    cx, cy = w / 2.0, h / 2.0
    M = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
    cos, sin = abs(M[0, 0]), abs(M[0, 1])
    nw = int(np.ceil(h * sin + w * cos))
    nh = int(np.ceil(h * cos + w * sin))
    M[0, 2] += nw / 2.0 - cx
    M[1, 2] += nh / 2.0 - cy
    return cv2.warpAffine(template, M, (nw, nh), flags=cv2.INTER_NEAREST, borderValue=0)


def ransac_detect_walls(binary_map, iterations=2500, threshold=2.0):
    """
    Uses RANSAC to greedily extract the 4 dominant wall lines.
    Computes intersections for the center and averages the 90-degree modulo angles for yaw.
    """
    ys, xs = np.where(binary_map > 0)
    if len(ys) == 0:
        return binary_map.shape[1]//2, binary_map.shape[0]//2, 0.0, 0.0
        
    pts = np.column_stack((xs, ys)).astype(np.float32)
    lines = []
    remaining_pts = pts.copy()
    
    for _ in range(4):
        if len(remaining_pts) < 10:
            break
            
        idx1 = np.random.randint(0, len(remaining_pts), iterations)
        idx2 = np.random.randint(0, len(remaining_pts), iterations)
        valid = idx1 != idx2
        p1 = remaining_pts[idx1[valid]]
        p2 = remaining_pts[idx2[valid]]
        
        dx = p2[:, 0] - p1[:, 0]
        dy = p2[:, 1] - p1[:, 1]
        norms = np.hypot(dx, dy)
        valid_norms = norms > 0
        
        p1, p2 = p1[valid_norms], p2[valid_norms]
        dx, dy, norms = dx[valid_norms], dy[valid_norms], norms[valid_norms]
        
        nx = -dy / norms
        ny = dx / norms
        c = -(nx * p1[:, 0] + ny * p1[:, 1])
        
        pts_x = remaining_pts[:, 0][:, np.newaxis]
        pts_y = remaining_pts[:, 1][:, np.newaxis]
        dists = np.abs(pts_x * nx + pts_y * ny + c)
        
        inliers = dists < threshold
        inlier_counts = np.sum(inliers, axis=0)
        
        best_idx = np.argmax(inlier_counts)
        best_mask = inliers[:, best_idx]
        
        # Refit with SVD for precision on the inliers
        inlier_pts = remaining_pts[best_mask]
        if len(inlier_pts) > 2:
            mean = np.mean(inlier_pts, axis=0)
            centered = inlier_pts - mean
            _, _, v = np.linalg.svd(centered)
            best_nx, best_ny = -v[0, 1], v[0, 0]
            best_c = -(best_nx * mean[0] + best_ny * mean[1])
        else:
            best_nx, best_ny, best_c = nx[best_idx], ny[best_idx], c[best_idx]
            
        lines.append((best_nx, best_ny, best_c))
        remaining_pts = remaining_pts[~best_mask]  # Remove inliers for the next wall
        
    if len(lines) < 2:
         return int(np.mean(pts[:, 0])), int(np.mean(pts[:, 1])), 0.0, 0.0
         
    # Compute the global orientation (average angle modulo 90 degrees)
    angles = [np.arctan2(ny, nx) for nx, ny, c in lines]
    angles_deg = np.degrees(angles) % 90
    angles_rad_90 = np.radians(angles_deg * 4) 
    mean_angle_90 = np.arctan2(np.mean(np.sin(angles_rad_90)), np.mean(np.cos(angles_rad_90)))
    w_yaw = np.degrees(mean_angle_90 / 4)
    w_yaw = ((w_yaw + 45) % 90) - 45
    
    # Compute intersections of non-parallel lines to find the center
    corners = []
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            nx1, ny1, c1 = lines[i]
            nx2, ny2, c2 = lines[j]
            det = nx1 * ny2 - ny1 * nx2
            if abs(det) > 0.5:  # Intersect if lines are roughly orthogonal
                x = (ny1 * c2 - ny2 * c1) / det
                y = (nx2 * c1 - nx1 * c2) / det
                corners.append((x, y))
                
    corners = np.array(corners)
    if len(corners) > 0:
        w_cx, w_cy = np.median(corners[:, 0]), np.median(corners[:, 1])
    else:
        w_cx, w_cy = np.mean(pts[:, 0]), np.mean(pts[:, 1])
        
    return int(w_cx), int(w_cy), w_yaw, 1.0


def fit_template_overlap(map_img, template, angles, scales=(1.0,)):
    map01 = (map_img > 0).astype(np.float32)
    best = (-np.inf, 0, 0, 0, None) 
    for scale in scales:
        tpl = template if scale == 1.0 else cv2.resize(
            template, (max(1, int(round(template.shape[1]*scale))),
                       max(1, int(round(template.shape[0]*scale)))),
            interpolation=cv2.INTER_NEAREST)
        for angle in angles:
            rot  = rotate_template_bound(tpl, angle)
            rot01 = (rot > 0).astype(np.float32)
            denom = float(rot01.sum())
            if denom == 0 or rot.shape[0] > map01.shape[0] or rot.shape[1] > map01.shape[1]:
                continue
            resp = cv2.matchTemplate(map01, rot01, cv2.TM_CCORR) / denom
            _, score, _, loc = cv2.minMaxLoc(resp)
            if score > best[0]:
                rh, rw = rot.shape
                best = (score, loc[0] + rw//2, loc[1] + rh//2, angle, rot)
    score, cx, cy, angle, rotated = best
    return cx, cy, angle, score, rotated


def detect_square_pyramid(search_map, template):
    h, w = template.shape
    u_template = np.zeros((h, w), dtype=np.uint8)
    t = 2 
    
    u_template[:t, :] = 255      
    u_template[:, :t] = 255      
    u_template[:, -t:] = 255     
    
    map01 = (search_map > 0).astype(np.float32)
    best = (-np.inf, 0, 0, 0, None)
    
    for angle in range(0, 360, 2):
        rot = rotate_template_bound(u_template, angle)
        rot01 = (rot > 0).astype(np.float32)
        denom = float(rot01.sum())
        
        if denom == 0 or rot.shape[0] > map01.shape[0] or rot.shape[1] > map01.shape[1]:
            continue
            
        resp = cv2.matchTemplate(map01, rot01, cv2.TM_CCORR) / denom
        _, score, _, loc = cv2.minMaxLoc(resp)
        
        if score > best[0]:
            rh, rw = rot.shape
            best = (score, loc[0] + rw//2, loc[1] + rh//2, angle, rot)
            
    score, cx, cy, angle, rotated = best
    
    if score < 0.10:
        return None
        
    return int(round(cx)), int(round(cy)), float(angle), score, rotated


def erase_rotated_footprint(map_img, rotated_template, cx, cy, pad=6):
    fp = cv2.dilate((rotated_template > 0).astype(np.uint8) * 255,
                    np.ones((pad*2+1, pad*2+1), np.uint8))
    h, w = fp.shape
    y1 = max(0, cy-h//2);  y2 = min(map_img.shape[0], cy+h//2+h%2)
    x1 = max(0, cx-w//2);  x2 = min(map_img.shape[1], cx+w//2+w%2)
    fy1 = max(0, h//2-cy); fy2 = h - max(0, cy+h//2+h%2 - map_img.shape[0])
    fx1 = max(0, w//2-cx); fx2 = w - max(0, cx+w//2+w%2 - map_img.shape[1])
    map_img[y1:y2, x1:x2][fp[fy1:fy2, fx1:fx2] > 0] = 0


def draw_on_map(canvas, template, cx, cy, yaw):
    if template is None:
        return
    rotated = rotate_template_bound(template, yaw)
    h, w = rotated.shape
    y1, y2  = max(0, cy-h//2), min(canvas.shape[0], cy+h//2+h%2)
    x1, x2  = max(0, cx-w//2), min(canvas.shape[1], cx+w//2+w%2)
    ry1, ry2 = max(0, h//2-cy), h - max(0, cy+h//2+h%2-canvas.shape[0])
    rx1, rx2 = max(0, w//2-cx), w - max(0, cx+w//2+w%2-canvas.shape[1])
    canvas[y1:y2, x1:x2][rotated[ry1:ry2, rx1:rx2] > 0] = 0


def main():
    os.makedirs(MAP_DIR, exist_ok=True)
    raw_map = cv2.imread(RAW_MAP, cv2.IMREAD_GRAYSCALE)
    if raw_map is None:
        raise FileNotFoundError(f"raw_map.pgm not found at {RAW_MAP}")
    
    # --- NEW: Crop away all unvisited gray space (value 205) ---
    mapped_pixels = np.where(raw_map != 205)
    if len(mapped_pixels[0]) > 0:
        y1, y2 = np.min(mapped_pixels[0]), np.max(mapped_pixels[0])
        x1, x2 = np.min(mapped_pixels[1]), np.max(mapped_pixels[1])
        
        # Add a small 10px buffer so we don't clip the edges
        h_raw, w_raw = raw_map.shape
        y1, y2 = max(0, y1 - 10), min(h_raw, y2 + 10)
        x1, x2 = max(0, x1 - 10), min(w_raw, x2 + 10)
        
        raw_map = raw_map[y1:y2, x1:x2]
        print(f"[map_processor] Cropped raw_map to mapped bounds: {raw_map.shape}")
    # -----------------------------------------------------------

    _, binary_map = cv2.threshold(raw_map, 200, 255, cv2.THRESH_BINARY_INV)
    object_map = binary_map.copy()
    wall_map   = cv2.dilate(binary_map, np.ones((5, 5), np.uint8))

    # Replaced Template Matching with RANSAC
    w_cx, w_cy, w_yaw, w_score = ransac_detect_walls(wall_map)
    print(f'[map_processor] RANSAC Wall fit: center=({w_cx},{w_cy}) yaw={w_yaw:.2f}°')

    h_img, w_img = object_map.shape
    # This aligns the entire map using the RANSAC orientation BEFORE finding inner objects
    M_align = cv2.getRotationMatrix2D((w_cx, w_cy), w_yaw, 1.0)
    aligned_map = cv2.warpAffine(object_map, M_align, (w_img, h_img), flags=cv2.INTER_NEAREST)

    half     = (FIELD_SIZE / FINAL_RES) / 2
    origin_x = int(w_cx - half)
    origin_y = int(w_cy + half)
    pixels   = int(FIELD_SIZE / FINAL_RES)

    margin = int(round(0.1 / FINAL_RES))
    sy1 = max(0, origin_y - pixels + margin);  sy2 = min(aligned_map.shape[0], origin_y - margin)
    sx1 = max(0, origin_x + margin);           sx2 = min(aligned_map.shape[1], origin_x + pixels - margin)
    search_map = np.zeros_like(aligned_map)
    search_map[sy1:sy2, sx1:sx2] = aligned_map[sy1:sy2, sx1:sx2]

    objects = [
        ("Bridge",    "bridge_lidar_scan.png",  "bridge_lidar_run.png",  "bridge_base.png"),
        ("Pyramid_1", "pyramid_lidar_scan.png", "pyramid_lidar_run.png", "pyramid_base.png"),
        ("Pyramid_2", "pyramid_lidar_scan.png", "pyramid_lidar_run.png", "pyramid_base.png"),
    ]
    bridge_scales = np.round(np.arange(0.85, 1.55, 0.05), 2)

    canvas_size = pixels + 2 * margin
    planner_canvas = np.zeros((canvas_size, canvas_size), dtype=np.uint8)
    lidar_canvas   = np.zeros((canvas_size, canvas_size), dtype=np.uint8)
    planner_canvas[margin:margin+pixels, margin:margin+pixels] = 255
    lidar_canvas[margin:margin+pixels,   margin:margin+pixels] = 255

    landmarks = []

    for name, scan_png, run_png, base_png in objects:
        if SCAN_BRIDGE:
            scan_tmp = load_scaled_template(scan_png, FINAL_RES)
        else:
            scan_tmp = load_scaled_template(run_png, FINAL_RES)
        run_tmp  = load_scaled_template(run_png, FINAL_RES)
        base_tmp = load_scaled_template(base_png, FINAL_RES)

        if SCAN_BRIDGE and scan_tmp is None:
            print(f'[map_processor] {name}: scan template empty, skipping')
            continue

        is_pyramid = name.startswith("Pyramid")
        is_bridge  = name.startswith("Bridge")

        if is_pyramid:
            result = detect_square_pyramid(search_map, scan_tmp)
            if result is None:
                print(f'[map_processor] {name}: no square contour found, skipping')
                continue
            cx, cy, oyaw, score, rotated = result
        elif SCAN_BRIDGE and is_bridge:
            cx, cy, oyaw, score, rotated = fit_template_overlap(
                search_map, scan_tmp, range(0, 360, 2), scales=bridge_scales)
        else:
            cx, cy, oyaw, score, rotated = 0.0, 0.0, 0.0, 0.0, None

        print(f'[map_processor] {name}: center=({cx},{cy}) yaw={oyaw}° score={score:.3f}')
        
        if SCAN_BRIDGE and score < 0.10:
            print(f'[map_processor] {name}: score below 0.10, skipping')
            continue

        if rotated is not None:
            erase_rotated_footprint(search_map, rotated, cx, cy)

        if not SCAN_BRIDGE and is_bridge:
            mx, my, rel_yaw = 1.5, 2.0, 0.0
        else:
            mx      = (cx - origin_x) * FINAL_RES
            my      = (origin_y - cy) * FINAL_RES
            rel_yaw = oyaw % 360

        landmarks.append((name, mx, my, rel_yaw))

        px_x = int(mx / FINAL_RES) + margin
        px_y = pixels - int(my / FINAL_RES) + margin

        if is_pyramid:
            def make_hollow(shape, t=2):
                h, w = shape
                img = np.zeros((h, w), dtype=np.uint8)
                img[:t, :] = 255
                img[-t:, :] = 255
                img[:, :t] = 255
                img[:, -t:] = 255
                return img
            run_tmp  = make_hollow(run_tmp.shape)  if run_tmp  is not None else None
            base_tmp = np.full(base_tmp.shape, 255, dtype=np.uint8) if base_tmp is not None else None

        draw_on_map(lidar_canvas,   run_tmp,  px_x, px_y, rel_yaw)
        draw_on_map(planner_canvas, base_tmp, px_x, px_y, rel_yaw)

    scale = FINAL_RES / PLANNER_RES
    new_size = (int(planner_canvas.shape[1] * scale), int(planner_canvas.shape[0] * scale))
    planner_canvas = cv2.resize(planner_canvas, new_size, interpolation=cv2.INTER_NEAREST)

    with open(os.path.join(MAP_DIR, "landmarks.csv"), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Object_Type", "X", "Y", "Yaw_Degrees"])
        writer.writerows(landmarks)

    cv2.imwrite(os.path.join(MAP_DIR, "planner_map.pgm"), planner_canvas)
    cv2.imwrite(os.path.join(MAP_DIR, "lidar_map.pgm"),   lidar_canvas)

    yaml_lidar = {"resolution": FINAL_RES, "image": "lidar_map.pgm", "origin": [-2.1, -2.1, 0.0],
                 "occupied_thresh": 0.65, "free_thresh": 0.25, "negate": 0}
    yaml_planner = {"resolution": PLANNER_RES, "image": "planner_map.pgm", "origin": [-2.1, -2.1, 0.0],
                 "occupied_thresh": 0.65, "free_thresh": 0.25, "negate": 0}
    
    with open(os.path.join(MAP_DIR, "lidar_map.yaml"), 'w') as f:
        yaml.dump(yaml_lidar, f, default_flow_style=False)
    with open(os.path.join(MAP_DIR, "planner_map.yaml"), 'w') as f:
        yaml.dump(yaml_planner, f, default_flow_style=False)
        
    print(f"Done — landmarks, planner_map, lidar_map written to {MAP_DIR}")


if __name__ == "__main__":
    main()