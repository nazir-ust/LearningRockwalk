import numpy as np
import trimesh

def generate_cone_vertices(a,b,p):
    all_vertices = [p]
    theta = np.linspace(0, 2*np.pi, 100)
    for i in range(np.size(theta)):
        x = a*np.cos(theta[i])
        y = b*np.sin(theta[i])
        vert = [x,y,0]
        all_vertices.append(vert)

    return all_vertices

# mesh = trimesh.Trimesh(vertices=[[0, 0, 0], [0, 0, 1], [0, 1, 0]])

# mesh = trimesh.creation.annulus(0.1,1,0.1) + trimesh.Trimesh(vertices=[[0, 0, 1]])

apex_vertex = [0.,-0.35,1.5]

mesh = trimesh.Trimesh(vertices= generate_cone_vertices(0.35,0.35,apex_vertex))

mesh = trimesh.convex.convex_hull(mesh)
transform_matrix = np.eye(4)
transform_matrix[:3,3] = -np.array(apex_vertex)
mesh.apply_transform(transform_matrix)


assert mesh.is_watertight, 'mesh is not watertight'
mesh.density=10
# print(mesh.mass)
# print(mesh.center_mass)
mesh_mass_properties = mesh.mass_properties

mesh_mass = mesh_mass_properties['mass']
mesh_center_of_mass = mesh_mass_properties['center_mass']
mesh_inertia_tensor = mesh_mass_properties['inertia']

print(mesh_mass)
print(mesh_center_of_mass)
print(mesh_inertia_tensor)


mesh.export('mesh1.obj')

# mesh.show()
