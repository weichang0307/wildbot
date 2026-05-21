import cv2
import numpy as np
import trimesh
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import os

RESOLUTION = 0.001

def slice_template(stl_path, height, resolution=RESOLUTION):
    """Slices a 3D STL at height and returns a rasterized 2D template and its physical bounds."""
    
    if not os.path.exists(stl_path):
        print(f"Warning: File not found: {stl_path}")
        return None, None
        
    mesh = trimesh.load(stl_path)

    # If the square base is on the XZ plane, rotate it to the XY plane
    if "pyramid" in stl_path.lower():
        rotation = trimesh.transformations.rotation_matrix(np.radians(90), [1, 0, 0])
        mesh.apply_transform(rotation)
    
    mesh.apply_scale(0.001)  # Convert mm to meters
    mesh.apply_translation([-mesh.centroid[0], -mesh.centroid[1], 0])  # Force center
    
    slice_3d = mesh.section(plane_origin=[0, 0, height], plane_normal=[0, 0, 1])
    
    # Handle case where the slicing plane is completely above or below the object
    if slice_3d is None:
        return np.full((1, 1), 255, dtype=np.uint8), [-resolution, resolution, -resolution, resolution]
        
    slice_2d, _ = slice_3d.to_2D()
    bounds = slice_2d.bounds
    
    # Force centralize
    center = (bounds[1] + bounds[0]) / 2.0
    padding_m = 10 * resolution
    xmin = bounds[0][0] - center[0] - padding_m
    xmax = bounds[1][0] - center[0] + padding_m
    ymin = bounds[0][1] - center[1] - padding_m
    ymax = bounds[1][1] - center[1] + padding_m
    
    w = int(round((xmax - xmin) / resolution))
    h = int(round((ymax - ymin) / resolution))
    
    # Initialize WHITE background (255)
    img = np.full((h, w), 255, dtype=np.uint8)
    
    for poly in slice_2d.polygons_full:
        pts = np.array(poly.exterior.coords)
        pts = pts - center
        pts = ((pts - [xmin, ymin]) / resolution).astype(np.int32)
        # Fill obstacles as BLACK (0)
        cv2.fillPoly(img, [pts], 0)
        
    return img, [xmin, xmax, ymin, ymax]

def main():

    stl_files = {
        "Bridge": "./templates/bridge.stl",
        "Pyramid": "./templates/pyramid.stl"
    }
    
    heights = {
        "Base":       0.0,   # physical footprint — used for planner_map
        "Lidar Scan": 0.3,   # scanning phase lidar height — used for map fitting
        "Lidar Run":  0.42,  # competition run lidar height — used for lidar_map
    }
    
    results = []
    
    for name, path in stl_files.items():
        for h_label, z in heights.items():
            img, extent = slice_template(path, z)
            if img is not None:
                results.append((f"{name} - {h_label} ({z}m)", img, extent))

    if not results:
        print("No templates generated. Please check STL file paths.")
        return

    # Export the 4 individual grid maps to ./templates/
    out_dir = "./templates"
    os.makedirs(out_dir, exist_ok=True)
    
    for title, img, _ in results:
        # Formats "Bridge - Base (0.0m)" into "bridge_base.png"
        filename = title.split(' (')[0].lower().replace(' - ', '_').replace(' ', '_') + ".png"
        filepath = os.path.join(out_dir, filename)
        cv2.imwrite(filepath, img)
        print(f"Exported grid map: {filepath}")

    # Calculate global axis limits to enforce unified scale
    global_xmin = min([ext[0] for _, _, ext in results])
    global_xmax = max([ext[1] for _, _, ext in results])
    global_ymin = min([ext[2] for _, _, ext in results])
    global_ymax = max([ext[3] for _, _, ext in results])

    # Visualization (sharex/sharey ensures they all sync)
    fig, axes = plt.subplots(1, len(results), figsize=(16, 10), sharex=True, sharey=True)
    if len(results) == 1:
        axes = [axes]
        
    for ax, (title, img, extent) in zip(axes, results):
        # Display image; apply extent mapping so the axis ticks represent meters
        ax.imshow(img, cmap='gray', origin='lower', extent=extent, vmin=0, vmax=255)
        ax.set_title(title)
        
        # Enforce unified scale bounding box
        ax.set_xlim(global_xmin, global_xmax)
        ax.set_ylim(global_ymin, global_ymax)
        ax.set_aspect('equal', adjustable='box')
        
        # Lock the grid interval to 0.1 meters to ensure consistent visual density
        ax.xaxis.set_major_locator(ticker.MultipleLocator(0.1))
        ax.yaxis.set_major_locator(ticker.MultipleLocator(0.1))
        
        # Add labels and grid
        ax.set_xlabel("X (meters)")
        if ax == axes[0]:
            ax.set_ylabel("Y (meters)")
        ax.grid(True, linestyle='--', alpha=0.5)
        
    plt.tight_layout(pad=2.0, w_pad=0.5) 
    output_path = "cross_sections.png"
    plt.savefig(output_path, bbox_inches='tight', dpi=150)
    print(f"Saved visualization to {output_path}")

if __name__ == "__main__":
    main()