# import pyassimp

# file_path = "/home/kwang/catkin_ws/src/pick_place_python/scripts/models/bin.stl"  # Replace with your model's file path

# # Use the context manager to load the model
# with pyassimp.load(file_path) as scene:
#     # Access and process meshes
#     for i, mesh in enumerate(scene.meshes):
#         print(f"Mesh {i}:")
#         print("Vertices:")
#         print(mesh.vertices)  # Access vertex data
#         print("Faces (indices):")
#         print(mesh.faces)     # Access face indices
#         print("Normals:")
#         print(mesh.normals)   # Access normal data
#         print("Texture Coordinates:")
#         print(mesh.texturecoords)  # Access texture coordinates (if available)

# # No need to explicitly release resources when using a context manager
import trimesh

# Load the mesh

# Scale the mesh
scale_factor = 0.0005
mesh = trimesh.load('/home/kwang/catkin_ws/src/pick_place_python/scripts/models/caliper.stl')

mesh.apply_scale(scale_factor)

# Save the scaled mesh
mesh.export('/home/kwang/catkin_ws/src/pick_place_python/scripts/models/caliper_scaled.stl')


mesh = trimesh.load('/home/kwang/catkin_ws/src/pick_place_python/scripts/models/mallet.stl')
scale_factor = 0.0005

mesh.apply_scale(scale_factor)

# Save the scaled mesh
mesh.export('/home/kwang/catkin_ws/src/pick_place_python/scripts/models/mallet_scaled.stl')


mesh = trimesh.load('/home/kwang/catkin_ws/src/pick_place_python/scripts/models/pliers.stl')
scale_factor = 0.001

mesh.apply_scale(scale_factor)

# Save the scaled mesh
mesh.export('/home/kwang/catkin_ws/src/pick_place_python/scripts/models/pliers_scaled.stl')


mesh = trimesh.load('/home/kwang/catkin_ws/src/pick_place_python/scripts/models/screwdriver.stl')
scale_factor = 0.001

mesh.apply_scale(scale_factor)

# Save the scaled mesh
mesh.export('/home/kwang/catkin_ws/src/pick_place_python/scripts/models/screwdriver_scaled.stl')

# Add the scaled mesh
# scene.add_mesh(bin_name, bin_pose, '/home/kwang/catkin_ws/src/pick_place_python/scripts/models/bin_scaled.stl')
