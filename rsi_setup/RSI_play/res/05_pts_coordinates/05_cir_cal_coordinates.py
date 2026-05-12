import numpy as np
import pandas as pd

# ===== Configuration =====
CENTER_X = 750.0        # Circle center X (mm)
CENTER_Y = 0.0          # Circle center Y (mm)
CENTER_Z = 650.0        # Circle center Z (mm)
INNER_RADIUS = 100.0    # Inner circle radius (mm)
OUTER_RADIUS = 150.0    # Outer circle radius (mm)
NUM_POINTS = 48         # Number of points
ANGLE_STEP = 360.0 / NUM_POINTS  # 7.5 degrees per point

# Tool and Base orientation (constant for all points)
TOOL_NO = 3
BASE_NO = 0
ORIENT_A = 0
ORIENT_B = 90
ORIENT_C = 0

# ===== Calculate Inner Circle Points =====
print("=" * 80)
print("[INNER CIRCLE CALCULATION]")
print(f"Center: ({CENTER_X}, {CENTER_Y}, {CENTER_Z})")
print(f"Radius: {INNER_RADIUS}mm")
print(f"Points: {NUM_POINTS}")
print("=" * 80)

inner_points = []
for i in range(NUM_POINTS):
    angle_deg = i * ANGLE_STEP
    angle_rad = np.radians(angle_deg)
    
    x = CENTER_X + INNER_RADIUS * np.cos(angle_rad)
    y = CENTER_Y + INNER_RADIUS * np.sin(angle_rad)
    z = CENTER_Z
    
    inner_points.append({
        'Point_ID': i,
        'Angle_deg': angle_deg,
        'X': round(x, 3),
        'Y': round(y, 3),
        'Z': z,
        'A': ORIENT_A,
        'B': ORIENT_B,
        'C': ORIENT_C
    })

inner_df = pd.DataFrame(inner_points)
print("\nInner Circle Points (first 10):")
print(inner_df.head(10).to_string(index=False))
print(f"\n[Total: {len(inner_df)} points]")

# ===== Calculate Outer Circle Points =====
print("\n" + "=" * 80)
print("[OUTER CIRCLE CALCULATION]")
print(f"Center: ({CENTER_X}, {CENTER_Y}, {CENTER_Z})")
print(f"Radius: {OUTER_RADIUS}mm")
print(f"Points: {NUM_POINTS}")
print("=" * 80)

outer_points = []
for i in range(NUM_POINTS):
    angle_deg = i * ANGLE_STEP
    angle_rad = np.radians(angle_deg)
    
    x = CENTER_X + OUTER_RADIUS * np.cos(angle_rad)
    y = CENTER_Y + OUTER_RADIUS * np.sin(angle_rad)
    z = CENTER_Z
    
    outer_points.append({
        'Point_ID': i,
        'Angle_deg': angle_deg,
        'X': round(x, 3),
        'Y': round(y, 3),
        'Z': z,
        'A': ORIENT_A,
        'B': ORIENT_B,
        'C': ORIENT_C
    })

outer_df = pd.DataFrame(outer_points)
print("\nOuter Circle Points (first 10):")
print(outer_df.head(10).to_string(index=False))
print(f"\n[Total: {len(outer_df)} points]")

# ===== Calculate Position Offsets (Outer - Inner) =====
print("\n" + "=" * 80)
print("[OFFSET CALCULATION (Outer - Inner)]")
print("=" * 80)

offset_points = []
for i in range(NUM_POINTS):
    delta_x = outer_df.loc[i, 'X'] - inner_df.loc[i, 'X']
    delta_y = outer_df.loc[i, 'Y'] - inner_df.loc[i, 'Y']
    delta_z = outer_df.loc[i, 'Z'] - inner_df.loc[i, 'Z']
    
    offset_points.append({
        'Point_ID': i,
        'Delta_X': round(delta_x, 3),
        'Delta_Y': round(delta_y, 3),
        'Delta_Z': delta_z
    })

offset_df = pd.DataFrame(offset_points)
print("\nOffset values (first 10):")
print(offset_df.head(10).to_string(index=False))
print(f"\n[Total: {len(offset_df)} offsets]")

# ===== Save to CSV =====
inner_csv = 'inner_circle_points.csv'
outer_csv = 'outer_circle_points.csv'
offset_csv = 'circle_offsets.csv'

inner_df.to_csv(inner_csv, index=False)
outer_df.to_csv(outer_csv, index=False)
offset_df.to_csv(offset_csv, index=False)

print("\n" + "=" * 80)
print("[FILES SAVED]")
print(f"  {inner_csv}")
print(f"  {outer_csv}")
print(f"  {offset_csv}")
print("=" * 80)

# ===== Export as Python dict for direct use =====
print("\n[PYTHON DICT FORMAT - Copy to your code]")
print("\ninner_circle = {")
for i in range(5):  # Show first 5 as example
    pt = inner_df.iloc[i]
    print(f"    {i}: {{'x': {pt['X']}, 'y': {pt['Y']}, 'z': {pt['Z']}}},")
print("    ... (48 total)")
print("}")

print("\nouter_circle = {")
for i in range(5):  # Show first 5 as example
    pt = outer_df.iloc[i]
    print(f"    {i}: {{'x': {pt['X']}, 'y': {pt['Y']}, 'z': {pt['Z']}}},")
print("    ... (48 total)")
print("}")

# Save as Python code
with open('circle_coordinates.py', 'w') as f:
    f.write("# Auto-generated circle coordinates\n\n")
    f.write("INNER_CIRCLE = {\n")
    for i, row in inner_df.iterrows():
        f.write(f"    {i}: {{'x': {row['X']}, 'y': {row['Y']}, 'z': {row['Z']}}},\n")
    f.write("}\n\n")
    
    f.write("OUTER_CIRCLE = {\n")
    for i, row in outer_df.iterrows():
        f.write(f"    {i}: {{'x': {row['X']}, 'y': {row['Y']}, 'z': {row['Z']}}},\n")
    f.write("}\n\n")
    
    f.write("CIRCLE_OFFSETS = {\n")
    for i, row in offset_df.iterrows():
        f.write(f"    {i}: {{'dx': {row['Delta_X']}, 'dy': {row['Delta_Y']}, 'dz': {row['Delta_Z']}}},\n")
    f.write("}\n")

print("\n[circle_coordinates.py generated]")
print("=" * 80)
