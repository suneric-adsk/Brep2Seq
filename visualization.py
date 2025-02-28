import sys
import json
import argparse
import numpy as np
from math import cos, sin, pi
from OCC.Core.gp import gp_Trsf, gp_Vec, gp_Ax1, gp_Pnt, gp_Quaternion
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakePrism, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeCone, BRepPrimAPI_MakeSphere
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace
from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier
from OCC.Core.BRepAlgoAPI import (
    BRepAlgoAPI_Fuse,
    BRepAlgoAPI_Common,
    BRepAlgoAPI_Section,
    BRepAlgoAPI_Cut,
)
from OCC.Core.TopAbs import TopAbs_SOLID
from OCC.Core.TopoDS import topods_Solid, TopoDS_Shape

from occwl.solid import Solid
from occwl.viewer import Viewer
from occwl.edge import Edge
from occwl.uvgrid import uvgrid
from occwl.graph import face_adjacency

class Feature:
    def __init__(self, feature_type, param):
        self.feature_type = feature_type
        self.param = param

    @staticmethod
    def get_value(level, min=-1.0, max=1.0):
        if level < 0 or level >= 256:
            return -1
        return float((max-min)*level/256  + min)
    
    @staticmethod
    def transform_shape(shape, param):
        tx = Feature.get_value(param["T_x"])
        ty = Feature.get_value(param["T_y"])
        tz = Feature.get_value(param["T_z"])
        qw = Feature.get_value(param["Q_0"])
        qx = Feature.get_value(param["Q_1"])
        qy = Feature.get_value(param["Q_2"])
        qz = Feature.get_value(param["Q_3"])
        t = gp_Trsf()
        t.SetTranslationPart(gp_Vec(tx, ty, tz))
        quaternion = gp_Quaternion(qx, qy, qz, qw)
        t.SetRotationPart(quaternion)
        #brep_trans = BRepBuilderAPI_Transform(shape, t)
        #brep_trans.Build()
        #return brep_trans.Shape()
        return shape

    def type(self):
        return self.feature_type
    
    def shape(self):
        print("create shape", self.feature_type)
        pass

class PrimitiveBox(Feature):
    def __init__(self, feature_type, param):
        super().__init__(feature_type, param)

    def shape(self):
        l1 = Feature.get_value(self.param["L1"],0.0,2.0)
        l2 = Feature.get_value(self.param["L2"],0.0,2.0)
        l3 = Feature.get_value(self.param["L3"],0.0,2.0)
        box = BRepPrimAPI_MakeBox(l1, l2, l3).Shape()  
        return Feature.transform_shape(box, self.param)

class PrimitiveCylinder(Feature):
    def __init__(self, feature_type, param):
        super().__init__(feature_type, param)

    def shape(self):
        l1 = Feature.get_value(self.param["L1"],0.0,2.0)
        l2 = Feature.get_value(self.param["L2"],0.0,2.0)
        cylinder = BRepPrimAPI_MakeCylinder(l1, l2).Shape()
        return Feature.transform_shape(cylinder, self.param)
    
class PrimitivePrism(Feature):
    def __init__(self, feature_type, param):
        super().__init__(feature_type, param)

    def shape(self):
        l1 = Feature.get_value(self.param["L1"],0.0,2.0)
        l2 = Feature.get_value(self.param["L2"],0.0,2.0)
        e = int(self.param["E"])
        polygon = BRepBuilderAPI_MakePolygon()
        for i in range(e):
            x = l1 * cos(i*2*pi/e)
            y = l1 * sin(i*2*pi/e)
            polygon.Add(gp_Pnt(x, y, 0))
        polygon.Close()
        profile = BRepBuilderAPI_MakeFace(polygon.Wire()).Face()
        extrusion_vec = gp_Vec(0, 0, l2)
        prism = BRepPrimAPI_MakePrism(profile, extrusion_vec).Shape()
        return Feature.transform_shape(prism, self.param)
    
class PrimitiveCone(Feature):
    def __init__(self, feature_type, param):
        super().__init__(feature_type, param)

    def shape(self):
        l1 = Feature.get_value(self.param["L1"],0.0,2.0)
        l2 = Feature.get_value(self.param["L2"],0.0,2.0)
        cone = BRepPrimAPI_MakeCone(l1, 0., l2).Shape()
        return Feature.transform_shape(cone, self.param)
    
class PrimitiveSphere(Feature):
    def __init__(self, feature_type, param):
        super().__init__(feature_type, param)

    def shape(self):
        l1 = Feature.get_value(self.param["L1"],0.0,2.0)
        tx = Feature.get_value(self.param["T_x"])
        ty = Feature.get_value(self.param["T_y"])
        tz = Feature.get_value(self.param["T_z"])
        return BRepPrimAPI_MakeSphere(gp_Pnt(tx,ty,tz), l1).Shape()

class CadModelCreator:
    def __init__(self, json_file):
        self.json_file = json_file
        self.shapes = []
        self.create_model()

    def create_model(self):
        with open(self.json_file, "r") as f:
            data = json.load(f)
            for primative in data.get("principal_primitives",[]):
                shape = self.create_primitve(primative["type"], primative["param"])
                self.shapes.append(shape)
            for feature in data.get("detail_features",[]):
                print (feature["type"])
                pass
    
    def create_primitve(self, feature_type, param):
        if feature_type == "box":
            return PrimitiveBox(feature_type, param).shape()
        elif feature_type == "cylinder":
            return PrimitiveCylinder(feature_type, param).shape()
        elif feature_type == "prism":
            return PrimitivePrism(feature_type, param).shape()
        elif feature_type == "cone":
            return PrimitiveCone(feature_type, param).shape()
        elif feature_type == "sphere":
            return PrimitiveSphere(feature_type, param).shape()
        else:
            print("Unknown feature type: ", feature_type)
            pass 

    def model(self):
        base = self.shapes[0]
        for shape in self.shapes[1:]:
            fuse = BRepAlgoAPI_Fuse(base, shape)
            fuse.Build()
            base = fuse.Shape()
        
        solid_classifier = BRepClass3d_SolidClassifier(base)
        solid_classifier.PerformInfinitePoint(1e-7)
        if solid_classifier.State() == TopAbs_SOLID:
            return topods_Solid(base)
        return self.shapes[0]
    

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
parser.add_argument("--json", type=str, help="Path to json file")

if __name__ == "__main__":
    args = parser.parse_args()
    v = Viewer(backend="wx")
    v._display.get_parent().GetParent().SetTitle(args.json)

    cad = CadModelCreator(args.json)
    solid = Solid(cad.model())
    solid.set_transform_to_identity()
    v.display(solid, transparency=0.0, color=(0.2, 0.2, 0.2))
    display_uv_net(v, solid)
    
    # show the viewer
    v.fit()
    v.show()