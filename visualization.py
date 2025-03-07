import sys, os
import argparse
import numpy as np
from occwl.solid import Solid
from occwl.viewer import Viewer
from occwl.io import load_step
from occwl.edge import Edge
from occwl.uvgrid import uvgrid
from occwl.graph import face_adjacency
from datakit.shape_builder import CadModelCreator
from datakit.utils import solid_count, solid_edges

FEATURE_COLORS_MAP = {
    "rect_slot": (1.0, 1.0, 0.0),
    "tri_slot": (0.0, 0.0, 1.0),
    "cir_slot": (0.0, 1.0, 0.0),
    "rect_psg": (1.0, 0.7, 0.8),
    "tri_psg": (0.0, 1.0, 0.0),
    "hexa_psg": (1.0, 0.0, 0.0),
    "hole": (0.0, 0.0, 1.0),
    "rect_step": (0.5, 0.0, 0.5),
    "tside_step": (1.0, 0.0, 0.0),
    "slant_step": (1.0, 0.6, 0.0),
    "rect_b_step": (0.5, 0.0, 0.5),
    "tri_step": (1.0, 0.7, 0.8),
    "cir_step": (0.0, 1.0, 0.0),
    "rect_b_slot": (1.0, 0.6, 0.0),
    "cir_b_slot": (1.0, 0.0, 0.0),
    "u_b_slot": (1.0, 0.6, 0.0),
    "rect_pkt": (1.0, 0.0, 0.0),
    "key_pkt": (0.0, 0.0, 1.0),
    "tri_pkt": (1.0, 1.0, 0.0),
    "hexa_pkt": (0.0, 1.0, 0.0),
    "o_ring": (1.0, 0.6, 0.0),
    "b_hole": (1.0, 1.0, 0.0),
    "chamfer": (0.5, 0.0, 0.5),
    "fillet": (1.0, 0.7, 0.8),
}

def display_feature(v, shape, face_map):
    for face in face_map.keys():
        label = face_map[face]
        if label == "primitive":
            v.display(face, transparency=0.0, color=(0.4,0.4,0.4))
        else:
            v.display(face, transparency=0.0, color=FEATURE_COLORS_MAP[label])
    
    for edge in solid_edges(shape):
        v.display(edge, color=(0.2,0.2,0.2), update=True)

def display_uv_net(v, solid):
    g = face_adjacency(solid)
    bbox = solid.box()
    point_radius = bbox.max_box_length() * 0.03
    arrow_radius = point_radius * 0.85
    arrow_length = arrow_radius * 4

    face_grids = {}
    for face_idx in g.nodes:
        face = g.nodes[face_idx]["face"]
        points = uvgrid(face, num_u=10, num_v=10, method="point")
        mask = uvgrid(face, num_u=10, num_v=10, method="inside")
        normals = uvgrid(face, num_u=10, num_v=10, method="normal")
        face_grids[face_idx] = {"points": points, "normals": normals, "mask": mask}

    print(f"Number of nodes (faces): {len(g.nodes)}")
    print(f"Number of edges: {len(g.edges)}")

    # Get the points at each face's center for visualizing edges
    face_centers = {}
    for face_idx in g.nodes():
        # Display a sphere for each UV-grid point
        face = g.nodes[face_idx]["face"]
        grid = face_grids[face_idx]
        # Display points
        face_points = grid["points"].reshape((-1, 3))
        face_mask = grid["mask"].reshape(-1)
        face_points = face_points[face_mask, :]
        v.display_points(face_points, marker="point", color="GREEN")
        # Display normals
        face_normals = grid["normals"].reshape((-1, 3))
        face_normals = face_normals[face_mask, :]
        lines = [Edge.make_line_from_points(pt, pt + arrow_length * nor) for pt, nor in zip(face_points, face_normals)]
        for l in lines:
            v.display(l, color="RED")
        face_centers[face_idx] = grid["points"][4, 4]

    for fi, fj in g.edges():
        pt1 = face_centers[fi]
        pt2 = face_centers[fj]
        dist = np.linalg.norm(pt2 - pt1)
        if dist > 1e-3:
            v.display(Edge.make_line_from_points(pt1, pt2), color=(51.0 / 255.0, 0, 1))


parser = argparse.ArgumentParser("CAD model creation")
parser.add_argument("--json", type=str, default=None, help="Path to json file")
parser.add_argument("--shape", type=int, default=-1, help="The index of shape to display")
parser.add_argument("--step", type=str, default=None, help="Path to step file")
parser.add_argument("--uvgrid", action='store_true', help="Whether to display uv grid")   

if __name__ == "__main__":
    args = parser.parse_args()
    filename = args.json if args.json else args.step
    v = Viewer(backend="wx")
    v._display.get_parent().GetParent().SetTitle(filename)
    v.display_points(np.array([[0, 0, 0]]), marker="point", color="RED")

    if args.step is not None and os.path.exists(args.step):
        solid = load_step(args.step)[0]
        v.display(solid, transparency=0.0, color=(0.4, 0.4, 0.4))
        if args.uvgrid:
            display_uv_net(v, solid)
    elif args.json is not None and os.path.exists(args.json):
        cadCreator = CadModelCreator(args.json)
        shape, label = cadCreator.get_model(args.shape) 
        display_feature(v, shape, label)
        if args.uvgrid and solid_count(shape) == 1: 
            solid = Solid(shape)
            v.display(solid, transparency=0.0, color=(0.2,0.2,0.2), update=True)
            display_uv_net(v, solid)
    # show the viewer
    v.fit()
    v.show()