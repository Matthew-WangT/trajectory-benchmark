"""Render the actual URDF meshes: python robots/ar5_dual/preview.py [--zero]."""
from pathlib import Path
import json
import sys
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = Path(__file__).resolve().parent
URDF = HERE / 'urdf/ar5_dual.urdf'
root = ET.parse(URDF).getroot()
pose = {} if '--zero' in sys.argv else json.loads((HERE / 'example_pose.json').read_text())
links = {link.get('name'): link for link in root.findall('link')}
joints = root.findall('joint')
assert len(links) == len(root.findall('link')), 'Duplicate links'
assert len({j.get('name') for j in joints}) == len(joints), 'Duplicate joints'
children = [j.find('child').get('link') for j in joints]
assert len(set(children)) == len(children), 'Multiple parents'
assert set(links) - set(children) == {'benchmark_base'}
assert sum(j.get('type') == 'revolute' for j in joints) == 14

def origin(element):
    node = element.find('origin')
    if node is None:
        return np.eye(4)
    transform = trimesh.transformations.euler_matrix(*map(float, node.get('rpy', '0 0 0').split()))
    transform[:3, 3] = list(map(float, node.get('xyz', '0 0 0').split()))
    return transform

frames = {'benchmark_base': np.eye(4)}
pending = list(joints)
while pending:
    count = len(pending)
    for joint in pending[:]:
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        assert parent in links and child in links
        if parent not in frames:
            continue
        transform = origin(joint)
        if joint.get('type') == 'revolute':
            q = pose.get(joint.get('name'), 0)
            limit = joint.find('limit')
            assert float(limit.get('lower')) <= q <= float(limit.get('upper'))
            axis = list(map(float, joint.find('axis').get('xyz').split()))
            transform = transform @ trimesh.transformations.rotation_matrix(q, axis)
        frames[child] = frames[parent] @ transform
        pending.remove(joint)
    assert len(pending) < count, 'Disconnected tree or cycle'
assert len(frames) == len(links)
# Keep this assembly preview in the shoulder midpoint frame.
world_to_shoulder = np.linalg.inv(frames['base_link'])
frames = {name: world_to_shoulder @ frame for name, frame in frames.items()}
left = frames['AR5-5_08L-W4C1C5_base']
right = frames['AR5-5_08R-W4C1C5_base']
assert np.isclose(np.linalg.norm(left[:3, 3] - right[:3, 3]), 0.24)
assert np.allclose(left[:3, 2], [0, 1, 0])
assert np.allclose(right[:3, 2], [0, -1, 0])
# Positive J4 rotates local +Z toward local +X, which must face forward.
assert np.allclose(left[:3, 0], [1, 0, 0])
assert np.allclose(right[:3, 0], [1, 0, 0])
for node in root.iter('mesh'):
    assert (URDF.parent / node.get('filename')).is_file()

fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
for name, link in links.items():
    for visual in link.findall('visual'):
        geometry = visual.find('geometry')
        node = geometry.find('mesh')
        if node is not None:
            mesh = trimesh.load_mesh(URDF.parent / node.get('filename'))
            mesh.apply_scale(list(map(float, node.get('scale', '1 1 1').split())))
        elif geometry.find('sphere') is not None:
            mesh = trimesh.creation.icosphere(subdivisions=2, radius=float(geometry.find('sphere').get('radius')))
        else:
            node = geometry.find('cylinder')
            mesh = trimesh.creation.cylinder(radius=float(node.get('radius')), height=float(node.get('length')), sections=48)
        mesh.apply_transform(frames[name] @ origin(visual))
        light = np.maximum(0, mesh.face_normals @ np.array([0.4, -0.3, 0.85]))
        color = np.array([0.72, 0.81, 0.89]) if ('08L' in name or name.startswith('left_')) else np.array([0.89, 0.79, 0.64])
        if name == 'base_link':
            color = np.array([0.35, 0.4, 0.46])
        colors = color[None, :] * (0.45 + 0.55 * light[:, None])
        ax.add_collection3d(Poly3DCollection(mesh.triangles, facecolors=colors, linewidths=0, rasterized=True))
ax.plot([0, 0], [-0.12, 0.12], [0.13, 0.13], color='crimson', marker='|')
ax.text(0, 0, 0.18, '240 mm', color='crimson', ha='center')
ax.set(xlim=(-0.65, 0.9), ylim=(-1.05, 1.05), zlim=(-0.8, 0.35), xlabel='X / forward (m)', ylabel='Y / left (m)', zlabel='Z / up (m)')
ax.set_box_aspect((1.55, 2.1, 1.15))
ax.view_init(elev=32, azim=32)
ax.set_title('AR5 dual arm | 14 DoF | ' + ('zero pose' if '--zero' in sys.argv else 'example pose'))
fig.tight_layout()
output = HERE / ('preview_zero.png' if '--zero' in sys.argv else 'preview.png')
fig.savefig(output, dpi=160)
print(f'Validated {len(links)} links, {len(joints)} joints, 14 DoF, all mesh paths; base spacing = 0.240 m.\n{output}')
