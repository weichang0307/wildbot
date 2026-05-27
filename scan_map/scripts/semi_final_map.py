import cv2
import numpy as np
import csv
import yaml
import os

_HERE   = os.path.dirname(os.path.abspath(__file__))
MAP_DIR = os.path.join(_HERE, "..", "maps")
TPL_DIR = os.path.join(_HERE, "..", "templates")

TPL_RES     = 0.001
FINAL_RES   = 0.01
PLANNER_RES = 0.05
FIELD_SIZE  = 4.0

LANDMARKS = [
    ("Bridge",    1.4, 2.0, 0.0, "bridge_base.png"),
    ("Pyramid_1", 3.0, 2.0, 0.0, "pyramid_base.png"),
]


def load_scaled_template(filename, target_res):
    img = cv2.imread(os.path.join(TPL_DIR, filename), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Missing template: {filename}")
    img = np.where(img < 200, 255, 0).astype(np.uint8)
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

    pixels      = int(FIELD_SIZE / FINAL_RES)    # 400
    margin      = int(round(0.1 / FINAL_RES))    # 10
    canvas_size = pixels + 2 * margin            # 420

    planner_canvas = np.zeros((canvas_size, canvas_size), dtype=np.uint8)
    planner_canvas[margin:margin+pixels, margin:margin+pixels] = 255

    for name, mx, my, yaw, base_png in LANDMARKS:
        base_tmp = load_scaled_template(base_png, FINAL_RES)

        if name.startswith("Pyramid") and base_tmp is not None:
            base_tmp = np.full(base_tmp.shape, 255, dtype=np.uint8)

        px_x = int(mx / FINAL_RES) + margin
        px_y = pixels - int(my / FINAL_RES) + margin

        draw_on_map(planner_canvas, base_tmp, px_x, px_y, yaw)
        print(f"[semi_final_map] {name}: world=({mx},{my}) px=({px_x},{px_y})")

    scale    = FINAL_RES / PLANNER_RES
    new_size = (int(planner_canvas.shape[1] * scale), int(planner_canvas.shape[0] * scale))
    planner_canvas = cv2.resize(planner_canvas, new_size, interpolation=cv2.INTER_NEAREST)

    with open(os.path.join(MAP_DIR, "landmarks.csv"), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Object_Type", "X", "Y", "Yaw_Degrees"])
        for name, mx, my, yaw, _ in LANDMARKS:
            writer.writerow([name, mx, my, yaw])

    cv2.imwrite(os.path.join(MAP_DIR, "planner_map.pgm"), planner_canvas)

    yaml_planner = {"resolution": PLANNER_RES, "image": "planner_map.pgm", "origin": [-2.1, -2.1, 0.0],
                    "occupied_thresh": 0.65, "free_thresh": 0.25, "negate": 0}
    with open(os.path.join(MAP_DIR, "planner_map.yaml"), 'w') as f:
        yaml.dump(yaml_planner, f, default_flow_style=False)

    print(f"Done — landmarks and planner_map written to {MAP_DIR}")


if __name__ == "__main__":
    main()
