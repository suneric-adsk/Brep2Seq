import sys
import json
import argparse
import numpy as np
from abc import ABC, abstractmethod
from math import cos, sin, pi
from OCC.Core.gp import gp_Trsf, gp_Vec, gp_Pnt, gp_Quaternion
from OCC.Core.BRepPrimAPI import (
    BRepPrimAPI_MakeBox, 
    BRepPrimAPI_MakePrism, 
    BRepPrimAPI_MakeCylinder, 
    BRepPrimAPI_MakeCone, 
    BRepPrimAPI_MakeSphere,
)
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_Transform, 
    BRepBuilderAPI_MakePolygon, 
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
    BRepBuilderAPI_MakeSolid,
)
from OCC.Core.BRepAlgoAPI import (
    BRepAlgoAPI_Fuse,
    BRepAlgoAPI_Common,
    BRepAlgoAPI_Section,
    BRepAlgoAPI_Cut,
)
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.TopAbs import TopAbs_SOLID, TopAbs_FACE
from OCC.Core.TopoDS import topods_Solid, topods_Shell, topods_Face, TopoDS_Compound, TopoDS_Shell 
from OCC.Core.BRep import BRep_Builder, BRep_Tool, BRep_Tool_Surface
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain

from occwl.solid import Solid
from occwl.viewer import Viewer
from occwl.io import load_step
from occwl.edge import Edge
from occwl.uvgrid import uvgrid
from occwl.graph import face_adjacency

def level_to_value(level, min=-1.0, max=1.0, max_level=255):
    """
    Convert level to value
    The original values of features parameters are discretized into [0, max_level] levels
    The non-negative values are in range of [0.0,2.0], other vaules are in range of [-1.0,1.0]
    """
    step = (max-min)/(max_level+1)
    value = level * step + min
    return value

class Primitive(ABC):
    def __init__(self, type, param):
        self.type = type
        self.param = param

    def rotation(self):
        qw = level_to_value(int(self.param["Q_0"]))
        qx = level_to_value(int(self.param["Q_1"]))
        qy = level_to_value(int(self.param["Q_2"]))
        qz = level_to_value(int(self.param["Q_3"]))
        trns = gp_Trsf()
        trns.SetRotation(gp_Quaternion(qx,qy,qz,qw))
        return trns
    
    def translation(self):
        tx = level_to_value(int(self.param["T_x"]))
        ty = level_to_value(int(self.param["T_y"]))
        tz = level_to_value(int(self.param["T_z"]))
        trns = gp_Trsf()
        trns.SetTranslation(gp_Vec(tx, ty, tz))
        return trns

    def transform_shape(self, shape, translation, rotation = None):
        trns = translation
        if rotation is not None:
            trns = trns.Multiplied(rotation)
        shape_trans = BRepBuilderAPI_Transform(shape, trns, True)
        shape_trans.Build()
        return shape_trans.Shape()
    
    @abstractmethod
    def shape(self):
        print("create shape", self.type)
        pass

class PrimitiveBox(Primitive):
    def __init__(self, type, param):
        super().__init__(type, param)

    def shape(self):
        l1 = level_to_value(int(self.param["L1"])+1,0.0,2.0)
        l2 = level_to_value(int(self.param["L2"])+1,0.0,2.0)
        l3 = level_to_value(int(self.param["L3"])+1,0.0,2.0)
        print("box: length {:2f}, width {:2f}, height {:2f}".format(l1, l2, l3))
        box = BRepPrimAPI_MakeBox(l1, l2, l3).Solid()
        translation = self.translation()
        rotation = self.rotation()  
        return self.transform_shape(box, translation, rotation)

class PrimitiveCylinder(Primitive):
    def __init__(self, type, param):
        super().__init__(type, param)

    def shape(self):
        l1 = level_to_value(int(self.param["L1"])+1,0.0,2.0)
        l2 = level_to_value(int(self.param["L2"])+1,0.0,2.0)
        cylinder = BRepPrimAPI_MakeCylinder(l1, l2).Solid()
        print("cylinder: radius {:2f}, height {:2f}".format(l1, l2))
        translation = self.translation()
        rotation = self.rotation()  
        return self.transform_shape(cylinder, translation, rotation)
    
class PrimitiveCone(Primitive):
    def __init__(self, type, param):
        super().__init__(type, param)

    def shape(self):
        l1 = level_to_value(int(self.param["L1"])+1,0.0,2.0)
        l2 = level_to_value(int(self.param["L2"])+1,0.0,2.0)
        cone = BRepPrimAPI_MakeCone(l1, 0., l2).Solid()
        print("cone: radius {:2f}, height {:2f}".format(l1, l2))
        translation = self.translation()
        rotation = self.rotation()  
        return self.transform_shape(cone, translation, rotation)    
    
class PrimitiveSphere(Primitive):
    def __init__(self, type, param):
        super().__init__(type, param)

    def shape(self):
        l1 = level_to_value(int(self.param["L1"])+1,0.0,2.0)
        print("sphere: randius {:2f}".format(l1))
        sphere = BRepPrimAPI_MakeSphere(l1).Solid()
        translation = self.translation()
        return self.transform_shape(sphere, translation)
    
def print_transformation(face):
    loc = TopLoc_Location()
    BRep_Tool_Surface(face, loc)
    mat = np.eye(4)
    for i in range(3):
        for j in range(4):
            mat[i,j] = loc.Transformation().Value(i+1,j+1)
    print(mat)

class PrimitivePrism(Primitive):
    def __init__(self, type, param):
        super().__init__(type, param)

    def shape(self):
        l1 = level_to_value(int(self.param["L1"])+1,0.0,2.0)
        l2 = level_to_value(int(self.param["L2"])+1,0.0,2.0)
        e = int(self.param["E"])
        print("prism:", l1, l2, e)
        polygon_builder = BRepBuilderAPI_MakePolygon()
        for i in range(e):
            x = l1 * cos(i*2*pi/e)
            y = l1 * sin(i*2*pi/e)
            pt = gp_Pnt(x, y, 0)
            polygon_builder.Add(pt)
        polygon_builder.Close()
        polygon = polygon_builder.Wire()
        face_builder = BRepBuilderAPI_MakeFace(polygon, True)
        profile = face_builder.Face()
        vec = gp_Vec(0, 0, l2)
        prism = BRepPrimAPI_MakePrism(profile, vec).Shape()
        translation = self.translation()
        rotation = self.rotation()
        shape = self.transform_shape(prism, translation, rotation)

        # the prism contain a face with translation part due to the extrucsion
        # bake the translation to the face geometry and set the location to identity
        sewing = BRepBuilderAPI_Sewing()
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        while explorer.More():
            face = explorer.Current()
            loc = TopLoc_Location()
            BRep_Tool_Surface(face, loc)
            if loc.IsIdentity():
                sewing.Add(face)
            else:
                print_transformation(face)
                builder = BRepBuilderAPI_Transform(face, loc.Transformation(), True)
                new_face = builder.Shape()
                new_face.Location(TopLoc_Location())
                sewing.Add(new_face)
            explorer.Next()
        sewing.Perform()
        shell = topods_Shell(sewing.SewedShape())
        solid_builder = BRepBuilderAPI_MakeSolid(shell)
        return solid_builder.Solid()

class CadModelCreator:
    """
    Create a CAD model from a json file containing a list of principal primitives and detail features
    """
    def __init__(self, json_file):
        self.json_file = json_file
        self.primitives = []
        self.features = []
        self.create_model()

    def create_model(self):
        with open(self.json_file, "r") as f:
            data = json.load(f)
            for item in data.get("principal_primitives",[]):
                primitive = self.create_primitive(item)
                if primitive is not None:
                    shape = primitive.shape()
                    self.fuse_primitive(shape)
            for item in data.get("detail_features",[]):
                self.create_feature(item)
    
    def create_primitive(self, item):
        """
        Five types of principal primitives: box, cylinder, prism, cone, sphere
        """
        type = item["type"]
        param = item["param"]
        if type == "box":
            return PrimitiveBox(type, param)
        elif type == "cylinder":
            return PrimitiveCylinder(type, param)
        elif type == "prism":
            return PrimitivePrism(type, param)
        elif type == "cone":
            return PrimitiveCone(type, param)
        elif type == "sphere":
            return PrimitiveSphere(type, param)
        else:
            print("Unknown feature type: ", type)
            return None
        
    def create_feature(self, feature):
        pass

    def solid_count(self, shape):
        explorer = TopExp_Explorer(shape, TopAbs_SOLID)
        count = 0
        while explorer.More():
            count += 1
            explorer.Next()
        return count    

    def fuse_primitive(self, shape):
        """
        fuse current primitive shape with the existing shape
        """
        if len(self.primitives) < 1:
            self.primitives.append(shape)
            return
        
        base = self.primitives[-1]
        fused = BRepAlgoAPI_Fuse(base, shape).Shape()
        # sew the fused shape and upgrade the faces (merge them into same domain)
        if self.solid_count(fused) == 1:
            print("fuse one solid")
            sewing = BRepBuilderAPI_Sewing()
            sewing.Add(fused)    
            sewing.Perform()
            shell = topods_Shell(sewing.SewedShape())
            solid_builder = BRepBuilderAPI_MakeSolid(shell)
            solid = solid_builder.Solid()
            unify = ShapeUpgrade_UnifySameDomain(solid, True, True, True)
            unify.Build()
            fused = unify.Shape()
        self.primitives.append(fused)
        