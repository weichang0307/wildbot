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
FIELD_SIZE = 4.0


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


def create_wall_template(size=FIELD_SIZE, thickness=0.1, res=FINAL_RES):
    p_size, p_thick = int(size / res), int(thickness / res)
    img = np.zeros((p_size + p_thick*2, p_size + p_thick*2), dtype=np.uint8)
    cv2.rectangle(img, (p_thick, p_thick), (p_size+p_thick, p_size+p_thick), 255, p_thick)
    return img


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


def fit_template(map_img, template, angles):
    best_val, best_loc, best_angle = -1.0, (0, 0), 0
    h, w = template.shape
    for angle in angles:
        M       = cv2.getRotationMatrix2D((w//2, h//2), angle, 1.0)
        rotated = cv2.warpAffine(template, M, (w, h))
        res     = cv2.matchTemplate(map_img, rotated, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)
        if max_val > best_val:
            best_val, best_loc, best_angle = max_val, max_loc, angle
    return best_loc[0] + w//2, best_loc[1] + h//2, best_angle, best_val


def fit_template_overlap(map_img, template, angles, scales=(1.0,)):
    map01 = (map_img > 0).astype(np.float32)
    best = (-np.inf, 0, 0, 0, None)  # score, cx, cy, angle, rotated
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
    tpl_area = float(template.shape[0] * template.shape[1])
    contours, _ = cv2.findContours(search_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in contours:
        (cx, cy), (w, h), angle = cv2.minAreaRect(cnt)
        box_area = w * h
        if not (0.25 * tpl_area < box_area < 4.0 * tpl_area):
            continue
        if min(w, h) < 1 or min(w, h) / max(w, h) < 0.55:
            continue
        score = 1.0 - abs(box_area - tpl_area) / tpl_area
        if best is None or score > best[0]:
            best = (score, cx, cy, angle)
    if best is None:
        return None
    score, cx, cy, angle = best
    yaw = float((-angle) % 90)
    return int(round(cx)), int(round(cy)), yaw, score, rotate_template_bound(template, yaw)


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

    _, binary_map = cv2.threshold(raw_map, 230, 255, cv2.THRESH_BINARY_INV)
    object_map = binary_map.copy()
    wall_map   = cv2.dilate(binary_map, np.ones((5, 5), np.uint8))

    wall_tmp = create_wall_template()
    w_cx, w_cy, w_yaw, w_score = fit_template(wall_map, wall_tmp, range(0, 90, 1))
    w_yaw = ((w_yaw + 45) % 90) - 45  # normalize 90° symmetry
    print(f'[map_processor] Wall fit: center=({w_cx},{w_cy}) yaw={w_yaw}° score={w_score:.3f}')

    h_img, w_img = object_map.shape
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

    # Canvas: arena interior (free=255) surrounded by 0.2 m solid wall (occupied=0)
    canvas_size = pixels + 2 * margin
    planner_canvas = np.zeros((canvas_size, canvas_size), dtype=np.uint8)
    lidar_canvas   = np.zeros((canvas_size, canvas_size), dtype=np.uint8)
    planner_canvas[margin:margin+pixels, margin:margin+pixels] = 255
    lidar_canvas[margin:margin+pixels,   margin:margin+pixels] = 255

    landmarks = []

    for name, scan_png, run_png, base_png in objects:
        scan_tmp = load_scaled_template(scan_png, FINAL_RES)
        run_tmp  = load_scaled_template(run_png,  FINAL_RES)
        base_tmp = load_scaled_template(base_png, FINAL_RES)
        if scan_tmp is None:
            print(f'[map_processor] {name}: scan template empty, skipping')
            continue

        is_pyramid = name.startswith("Pyramid")
        if is_pyramid:
            result = detect_square_pyramid(search_map, scan_tmp)
            if result is None:
                print(f'[map_processor] {name}: no square contour found, skipping')
                continue
            cx, cy, oyaw, score, rotated = result
        else:
            cx, cy, oyaw, score, rotated = fit_template_overlap(
                search_map, scan_tmp, range(0, 360, 2), scales=bridge_scales)

        print(f'[map_processor] {name}: center=({cx},{cy}) yaw={oyaw}° score={score:.3f}')
        if score < 0.30:
            print(f'[map_processor] {name}: score below 0.30, skipping')
            continue

        if rotated is not None:
            erase_rotated_footprint(search_map, rotated, cx, cy)

        mx      = (cx - origin_x) * FINAL_RES
        my      = (origin_y - cy) * FINAL_RES
        rel_yaw = oyaw % 360
        landmarks.append((name, mx, my, rel_yaw))

        px_x = int(mx / FINAL_RES) + margin
        px_y = pixels - int(my / FINAL_RES) + margin

        if is_pyramid:
            run_tmp  = np.full(run_tmp.shape,  255, dtype=np.uint8) if run_tmp  is not None else None
            base_tmp = np.full(base_tmp.shape, 255, dtype=np.uint8) if base_tmp is not None else None

        draw_on_map(lidar_canvas,   run_tmp,  px_x, px_y, rel_yaw)
        draw_on_map(planner_canvas, base_tmp, px_x, px_y, rel_yaw)

    with open(os.path.join(MAP_DIR, "landmarks.csv"), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Object_Type", "X", "Y", "Yaw_Degrees"])
        writer.writerows(landmarks)

    cv2.imwrite(os.path.join(MAP_DIR, "planner_map.pgm"), planner_canvas)
    cv2.imwrite(os.path.join(MAP_DIR, "lidar_map.pgm"),   lidar_canvas)

    wall_m = margin * FINAL_RES
    yaml_base = {"resolution": FINAL_RES, "origin": [-wall_m, -wall_m, 0.0],
                 "occupied_thresh": 0.65, "free_thresh": 0.25, "negate": 0}
    for nm in ["planner_map", "lidar_map"]:
        with open(os.path.join(MAP_DIR, f"{nm}.yaml"), 'w') as f:
            yaml.dump({**yaml_base, "image": f"{nm}.pgm"}, f, default_flow_style=False)

    print(f"Done — landmarks, planner_map, lidar_map written to {MAP_DIR}")


if __name__ == "__main__":
    main()
