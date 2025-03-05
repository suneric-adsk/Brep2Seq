
import numpy as np
from math import cos, sin, pi
from OCC.Core.gp import gp_Trsf, gp_Vec, gp_Pnt, gp_Quaternion, gp_Ax2, gp_Dir
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCC.Core.TopoDS import topods
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_Transform, 
    BRepBuilderAPI_MakePolygon, 
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_Sewing,
    BRepBuilderAPI_MakeSolid,
)

def print_transformation(face):
    loc = TopLoc_Location()
    BRep_Tool.Surface(face, loc)
    mat = np.eye(4)
    for i in range(3):
        for j in range(4):
            mat[i,j] = loc.Transformation().Value(i+1,j+1)
    print(mat)

def level_to_value(level, min=-1.0, max=1.0, max_level=255):
    """
    Convert level to value
    The original values of features parameters are discretized into [0, max_level] levels
    The non-negative values are in range of [0.0,2.0], other vaules are in range of [-1.0,1.0]
    """
    step = (max-min)/(max_level+1)
    value = level * step + min
    return value

def make_solid_and_upgrade(shell):
    ms = BRepBuilderAPI_MakeSolid()
    ms.Add(topods.Shell(shell))
    su = ShapeUpgrade_UnifySameDomain(ms.Solid(), True, True, True)
    su.Build()
    return su.Shape()

def update_prism_shape(shape):
    """ 
    The prism contain a face with translation part due to the extrucsion
    Bake the translation to the face geometry and set the location to identity
    """
    sewing = BRepBuilderAPI_Sewing()
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = explorer.Current()
        loc = TopLoc_Location()
        BRep_Tool.Surface(face, loc)
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
    return make_solid_and_upgrade(sewing.SewedShape())

def make_polygon_face(r, n):
    """
    Make a polygon face with a radius r and number of sides n
    """
    polygon_builder = BRepBuilderAPI_MakePolygon()
    for i in range(n):
        x = r * cos(i*2*pi/n)
        y = r * sin(i*2*pi/n)
        pt = gp_Pnt(x, y, 0)
        polygon_builder.Add(pt)
    polygon_builder.Close()
    polygon = polygon_builder.Wire()
    face_builder = BRepBuilderAPI_MakeFace(polygon, True)
    return face_builder.Face()
    
def solid_count(shape):
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    count = 0
    while explorer.More():
        count += 1
        explorer.Next()
    return count
