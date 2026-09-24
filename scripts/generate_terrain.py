#!/usr/bin/env python

import random
import os

grid_size = 10
box_half_size = 0.1
grid_elements = ""

offset = (grid_size - 1) * box_half_size

for i in range(grid_size):
    for j in range(grid_size):
        is_light = (i + j) % 2 == 0
        material = "mat_light" if is_light else "mat_dark"
        
        # Generate a lower random height for the boxes
        hz = round(random.uniform(0.01, 0.07), 3)
        
        # Apply offset to center the grid
        pos_x = i * (box_half_size * 2) - offset
        pos_y = j * (box_half_size * 2) - offset
        
        grid_elements += f'      <geom type="box" size="{box_half_size} {box_half_size} {hz}" pos="{pos_x} {pos_y} {hz}" material="{material}" contype="1" conaffinity="1" friction="0.001"/>\n'

xml_content = f"""<mujoco model="generated_scene">
  <compiler texturedir="assets/textures"/>
  
  <asset>
    <material name="mat_light" rgba=".9 .9 .9 1" reflectance=".2"/>
    <material name="mat_dark" rgba=".7 .7 .7 1" reflectance=".2"/>
  </asset>
  
  <worldbody>
    <!-- Directional light with disabled shadows to prevent acne artifacts -->
    <light pos="5 5 5" dir="-1 -1 -1" directional="true" diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2" castshadow="false"/>
{grid_elements}
  </worldbody>
</mujoco>
"""

output_path = "mujoco/scene-outrun.xml"

os.makedirs(os.path.dirname(output_path), exist_ok=True)

with open(output_path, "w") as f:
    f.write(xml_content)
print(f"Terrain generated in {output_path}")