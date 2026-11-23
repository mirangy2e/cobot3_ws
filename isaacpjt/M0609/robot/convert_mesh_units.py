import glob
import re
import trimesh

SCALE = 0.001

# DAE: edit numbers in place, keep the XML structure
pos_pattern = re.compile(r'(<float_array[^>]*id="[^"]*positions-array"[^>]*>)([^<]*)(</float_array>)')
mat_pattern = re.compile(r'(<matrix[^>]*>)([^<]*)(</matrix>)')

def fmt(v):
    return f"{v:.7g}"

def scale_positions(m):
    values = [float(v) * SCALE for v in m.group(2).split()]
    return m.group(1) + " ".join(fmt(v) for v in values) + m.group(3)

def scale_matrix(m):
    values = [float(v) for v in m.group(2).split()]
    for i in (3, 7, 11):
        values[i] *= SCALE
    return m.group(1) + " ".join(fmt(v) for v in values) + m.group(3)

for f in glob.glob("m0609/meshes/*/*.dae"):
    src = open(f, encoding="utf-8").read()
    out = pos_pattern.sub(scale_positions, src)
    out = mat_pattern.sub(scale_matrix, out)
    open(f, "w", encoding="utf-8").write(out)
    print(f, "scaled")

# STL: binary mesh, rescale with trimesh
for f in glob.glob("rg2/meshes/*/*.stl"):
    mesh = trimesh.load(f, force="mesh")
    mesh.apply_scale(SCALE)
    mesh.export(f)
    print(f, mesh.bounds[1] - mesh.bounds[0])
