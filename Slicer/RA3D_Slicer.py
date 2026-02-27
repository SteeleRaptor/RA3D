'''This slicer is simple. It converts a stl file into 6 axis gcode
Currently it prints conventional horizontal layers and puts the tool head normal to the service
Also no accounting for printing, and is at a fixed speed'''
import trimesh
import numpy as np
import os
import trimesh
import numpy as np 
import os 
from scipy.spatial.transform import Rotation as R
def normal_to_euler_zyx(normal, degrees=True):
    """ Convert a surface normal vector into Euler angles (Z, Y, X) a.k.a. ZYX which maps to A, B, C as roll-pitch-yaw. Returns (A, B, C). """
    n = np.asarray(normal, dtype=float)
    n /= np.linalg.norm(n) + 1e-12 # guard against divide-by-zero
    # Build a frame whose Z-axis aligns with the normal.
    # Choose a temp vector not parallel to n to construct orthonormal basis.
    tmp = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(tmp, n)
    x_norm = np.linalg.norm(x_axis)
    if x_norm < 1e-12:
        # Fallback if the cross-product is degenerate
        tmp = np.array([0.0, 1.0, 0.0])
        x_axis = np.cross(tmp, n)
        x_norm = np.linalg.norm(x_axis)
        if x_norm < 1e-12:
            # Last resort: use a fixed frame
            x_axis = np.array([1.0, 0.0, 0.0])
    x_axis /= (np.linalg.norm(x_axis) + 1e-12)
    y_axis = np.cross(n, x_axis)
    y_axis /= (np.linalg.norm(y_axis) + 1e-12)

    # Rotation matrix columns are the basis vectors [x y z]
    R_world = np.column_stack([x_axis, y_axis, n])
    rot = R.from_matrix(R_world)
    # ZYX (returns [z, y, x]) → map to A, B, C
    zyx = rot.as_euler('zyx', degrees=degrees)
    A, B, C = zyx  # Using A=Z, B=Y, C=X consistent with ZYX order
    return A, B, C

def normal_to_euler_zyx_with_offset(normal, offset_zyx_deg=(0.0, 0.0, 0.0), degrees=True):
    """ Map a surface normal to an orientation, corrected by a machine base offset in ZYX (deg).
      Returns Euler ZYX angles (A=Z, B=Y, C=X) in degrees by default. """
    n = np.asarray(normal, dtype=float)
    n /= (np.linalg.norm(n) + 1e-12)
    # Build a right-handed frame with z-axis = normal
    tmp = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(tmp, n); x_axis /= (np.linalg.norm(x_axis) + 1e-12)
    y_axis = np.cross(n, x_axis); y_axis /= (np.linalg.norm(y_axis) + 1e-12)

    R_calculated = np.column_stack([x_axis, y_axis, n])

    # Apply inverse of hardware/base offset (ZYX = 0,90,0 facing down)
    R_offset = R.from_euler('zyx', offset_zyx_deg, degrees=True)
    # This used to multiply by the inverse of the offset
    R_final = R.from_matrix(R_calculated) * R_offset.inv()

    zyx = R_final.as_euler('zyx', degrees=degrees)  # [Z, Y, X]
    A, B, C = zyx
    return A, B+90, C

def process_stl_slices(file_path, layer_height=0.2, output_filename="output.gcode", feedrate=1500):

    print(f"--- Processing: {file_path} ---")
    
    # 1. Load and Validate Mesh
    try:
        mesh = trimesh.load(file_path)
    except Exception as e:
        print(f"CRITICAL: Could not load file. Error: {e}")
        return

    # Handle Scene objects (multi-part STLs)
    if isinstance(mesh, trimesh.Scene):
        print("Info: File is a Scene. Merging into single mesh...")
        mesh = mesh.dump(concatenate=True)

    if mesh.is_empty:
        print("CRITICAL: Mesh is empty.")
        return

    # Check mesh health
    if not mesh.is_watertight:
        print("WARNING: Mesh is not watertight (has holes). Slicing may produce gaps.")
    
    # 2. Check Scale and Bounds
    z_min, z_max = mesh.bounds[:, 2]
    z_height = z_max - z_min
    print(f"Mesh Z-Range: {z_min:.3f} to {z_max:.3f} (Height: {z_height:.3f}mm)")

    if z_height < layer_height:
        print(f"CRITICAL: Mesh height ({z_height}) is smaller than layer height ({layer_height}). No slices possible.")
        return

    # Generate Z levels
    # Start slightly above bottom to avoid grazing exact faces
    z_levels = np.arange(z_min + 0.01, z_max, layer_height)
    print(f"Attempting to generate {len(z_levels)} slices...")

    # 3. Slice the Mesh
    # We use section_multiplane for speed
    try:
        sections = mesh.section_multiplane(
            plane_origin=[0, 0, 0],
            plane_normal=[0, 0, 1],
            heights=z_levels
        )
    except Exception as e:
        print(f"CRITICAL: Slicing operation failed. Error: {e}")
        return

    # 4. Extract Coordinates (Defensive)
    all_vertices = []
   

    print("Extracting vertices from slices...")
    # 4. Extract Coordinates (Updated for 2D/3D safety)
    all_vertices = []

    for i, section in enumerate(sections):
        if section is None:
            continue

        current_z = z_levels[i]

        # Try to get points from the section
        points = None
        if hasattr(section, 'discrete') and len(section.discrete) > 0:
            # Combine all paths in this slice into one array
            points = np.vstack(section.discrete)
        elif hasattr(section, 'vertices') and len(section.vertices) > 0:
            points = section.vertices

        if points is not None and len(points) > 0:
            # CHECK: If points are (N, 2), they are missing the Z component
            if points.shape[1] == 2:
                # Create a column of the current Z height
                z_column = np.full((len(points), 1), current_z)
                # Stack X, Y with Z to make it (N, 3)
                points = np.hstack((points, z_column))

            all_vertices.append(points)
    
    # Now stacking will result in a clean (N, 3) array
    if all_vertices:
        stacked_points = np.vstack(all_vertices)
        print(f"Total slice points generated: {len(stacked_points)} with shape {stacked_points.shape}")
    else:
        print("No slice data generated.")
        return
    
    # 5. Stack and Validate Shape
    try:
        stacked_points = np.vstack(all_vertices)
    except ValueError as e:
        print(f"CRITICAL: Error stacking point arrays: {e}")
        return

    # Force (N, 3) shape
    if stacked_points.ndim != 2 or stacked_points.shape[1] != 3:
        print(f"Warning: Reshaping stacked points from {stacked_points.shape}...")
        stacked_points = stacked_points.reshape(-1, 3)

    print(f"Total points to process: {len(stacked_points)}")

    # 6. Calculate Normals
    print("Calculating surface normals...")
    try:
        # nearest.on_surface returns (points, distances, triangle_ids)
        _, _, triangle_ids = mesh.nearest.on_surface(stacked_points)
        normals = mesh.face_normals[triangle_ids]
    except Exception as e:
        print(f"CRITICAL: Normal calculation failed. Error: {e}")
        return
    # 7. Generate 6-axis G-code
    print(f"Generating G-code for {len(stacked_points)} points...")
    try:
        with open("Slicer/"+output_filename, 'w') as f:
            # Header
            f.write("; G-code generated for 6-axis toolpath\n")
            f.write("; Axes: X Y Z A B C (A,B,C = Euler ZYX in degrees)\n")
            f.write("G21 ; units in millimeters\n")
            f.write("G90 ; absolute positioning\n")
            if feedrate is not None:
                f.write(f"F{float(feedrate):.3f}\n")

            for i in range(len(stacked_points)):
                x, y, z = stacked_points[i]
                #Make sure print is not negative
                low_point = min(z)
                if low_point < 0:
                    z = z + abs(low_point)
                nx, ny, nz = normals[i]
                A, B, C = normal_to_euler_zyx_with_offset([nx, ny, nz], degrees=True)
                if feedrate is not None:
                    f.write(f"G1 X{x:.4f} Y{y:.4f} Z{z:.4f} A{A:.4f} B{B:.4f} C{C:.4f} F{float(feedrate):.3f}\n")
                else:
                    f.write(f"G1 X{x:.4f} Y{y:.4f} Z{z:.4f} A{A:.4f} B{B:.4f} C{C:.4f}\n")

        print(f"Success! G-code saved to {output_filename}")
    except Exception as e:
        print(f"Error writing G-code: {e}")
        return

if __name__ == "__main__":
    # Create a dummy file for demonstration if one doesn't exist
    fileName = "Baseplate spacer long.stl"
    if not os.path.exists(fileName):
        print("No file found. Creating a test sphere...")
        trimesh.creation.icosphere(radius=10).export("model.stl")
        fileName = "model.stl"
    process_stl_slices(fileName, layer_height=0.2)