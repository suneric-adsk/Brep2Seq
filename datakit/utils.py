
import numpy as np
from math import cos, sin, pi
from OCC.Core.gp import (
    gp_Trsf, 
    gp_Vec, 
    gp_Pnt, 
    gp_Quaternion, 
    gp_Ax2, 
    gp_Dir, 
    gp_Ax3,
    gp_Circ, 
    gp_Pln, 
    gp_Pnt2d,
)
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_SOLID, TopAbs_EDGE, TopAbs_VERTEX
from OCC.Core.TopoDS import topods, TopoDS_Face
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_Transform, 
    BRepBuilderAPI_MakePolygon, 
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_Sewing,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeVertex,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_MakeEdge,
)
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Extend.ShapeFactory import make_edge
from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Core.BRepFeat import BRepFeat_MakePrism
from OCC.Core.GC import GC_MakeArcOfCircle, GC_MakeSegment

PT_Tol = 1e-6

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
            # print_transformation(face)
            builder = BRepBuilderAPI_Transform(face, loc.Transformation(), True)
            new_face = builder.Shape()
            new_face.Location(TopLoc_Location())
            sewing.Add(new_face)
        explorer.Next()
    sewing.Perform()
    return make_solid_and_upgrade(sewing.SewedShape())
    
def solid_count(shape):
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    count = 0
    while explorer.More():
        count += 1
        explorer.Next()
    return count

def make_2d_polygon_points(n, r):
    points = []
    for i in range(n):
        x = r * cos(i*2*pi/n)
        y = r * sin(i*2*pi/n)
        points.append(gp_Pnt2d(x, y))
    return points

def find_closet_edge(shape, pt1, pt2):
    min_dist = float("inf")
    closest_edge = None
    edge_from_points = make_edge(pt1, pt2)
    for edge in TopologyExplorer(shape).edges():
        dist = BRepExtrema_DistShapeShape(edge_from_points, edge).Value()
        if dist < min_dist:
            min_dist = dist
            closest_edge = edge
    return closest_edge

def apply_feature(base, feat_face, depth_dir, fuse=False):
    fm = BRepFeat_MakePrism()
    fm.Init(base, feat_face, TopoDS_Face(), depth_dir, fuse, False)
    fm.Build()
    fm.Perform(np.linalg.norm(depth_dir))
    return fm.Shape()

def transform_to_3d_point(pt2d, c, dir):
    plane = gp_Ax3(c, dir)
    x = plane.XDirection()
    y = plane.YDirection()
    return gp_Pnt(c.X() + pt2d.X() * x.X() + pt2d.Y() * y.X(),
                  c.Y() + pt2d.X() * x.Y() + pt2d.Y() * y.Y(),
                  c.Z() + pt2d.X() * x.Z() + pt2d.Y() * y.Z())

def transform_to_3d_points(pt2ds, c, dir):
    pt3ds = []
    for pt2d in pt2ds:
        pt3d = transform_to_3d_point(pt2d, c, dir)
        pt3ds.append(pt3d)
    return pt3ds

def make_face_polygon(points):
    vertx = [BRepBuilderAPI_MakeVertex(pt).Vertex() for pt in points]
    wm = BRepBuilderAPI_MakeWire()
    for i in range(len(vertx)):
        j = (i+1)%len(vertx)
        wm.Add(BRepBuilderAPI_MakeEdge(vertx[i], vertx[j]).Edge())    
    return BRepBuilderAPI_MakeFace(wm.Wire()).Face()

def make_face_circle(r, c, dir):
    circ = gp_Circ(gp_Ax2(c, dir), r)
    edge = BRepBuilderAPI_MakeEdge(circ, 0., 2*pi).Edge()
    wire = BRepBuilderAPI_MakeWire(edge).Wire()
    return BRepBuilderAPI_MakeFace(wire).Face()

def make_face_circle_slot(r, c, dir):
    pt0 = transform_to_3d_point(gp_Pnt2d(r, 0), c, dir)
    pt1 = transform_to_3d_point(gp_Pnt2d(-r, 0), c, dir)
    circ = gp_Circ(gp_Ax2(c, dir), r)
    edge0 = BRepBuilderAPI_MakeEdge(pt0, pt1).Edge()
    edge1 = BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(circ,pt1,pt0,True).Value()).Edge()
    wm = BRepBuilderAPI_MakeWire(edge0, edge1)
    return BRepBuilderAPI_MakeFace(wm.Wire()).Face()

def make_face_oring(r, c, dir, ratio=0.5):
    outer_circ = gp_Circ(gp_Ax2(c, dir), r)
    inner_circ = gp_Circ(gp_Ax2(c, dir), r*ratio)
    outer_edge = BRepBuilderAPI_MakeEdge(outer_circ, 0., 2*pi).Edge()
    inner_edge = BRepBuilderAPI_MakeEdge(inner_circ, 0., 2*pi).Edge()
    outer_wire = BRepBuilderAPI_MakeWire(outer_edge).Wire()
    inner_wire = BRepBuilderAPI_MakeWire(inner_edge).Wire()
    fm = BRepBuilderAPI_MakeFace(outer_wire)
    fm.Add(inner_wire)
    return fm.Face()

def make_face_key_hole(w, h, c, dir):
    if abs(w-h) < PT_Tol:
        return make_face_circle(w/2, c, dir)
    
    r = min(w,h)/2
    pt0 = transform_to_3d_point(gp_Pnt2d((w-h)/2,-h/2), c, dir)
    pt1 = transform_to_3d_point(gp_Pnt2d((w-h)/2,h/2), c, dir)
    pt2 = transform_to_3d_point(gp_Pnt2d((h-w)/2,h/2), c, dir)
    pt3 = transform_to_3d_point(gp_Pnt2d((h-w)/2,-h/2), c, dir)
    c01 = transform_to_3d_point(gp_Pnt2d((w-h)/2,0), c, dir)
    c23 = transform_to_3d_point(gp_Pnt2d((h-w)/2,0), c, dir)
    if w < h:
        pt0 = transform_to_3d_point(gp_Pnt2d(w/2,(h-w)/2), c, dir)
        pt1 = transform_to_3d_point(gp_Pnt2d(-w/2,(h-w)/2), c, dir)
        pt2 = transform_to_3d_point(gp_Pnt2d(-w/2,(w-h)/2), c, dir)
        pt3 = transform_to_3d_point(gp_Pnt2d(w/2,(w-h)/2), c, dir)
        c01 = transform_to_3d_point(gp_Pnt2d(0,(h-w)/2), c, dir)
        c23 = transform_to_3d_point(gp_Pnt2d(0,(w-h)/2), c, dir)
    cir01 = gp_Circ(gp_Ax2(c01, dir), r)
    cir23 = gp_Circ(gp_Ax2(c23, dir), r)
    sm = [GC_MakeArcOfCircle(cir01, pt0, pt1, True), GC_MakeSegment(pt1,pt2),
          GC_MakeArcOfCircle(cir23, pt2, pt3, True), GC_MakeSegment(pt3,pt0)]
    wm = BRepBuilderAPI_MakeWire()
    for seg in sm:
        wm.Add(BRepBuilderAPI_MakeEdge(seg.Value()).Edge())
    return BRepBuilderAPI_MakeFace(wm.Wire()).Face()

def make_face_circ_slot_x(w, h, c, dir):
    r = w/2
    if abs(w/2-h) < PT_Tol:
        return make_face_circle_slot(r, c, dir)
    
    sm = []
    pt0 = transform_to_3d_point(gp_Pnt2d(r, 0), c, dir)
    pt1 = transform_to_3d_point(gp_Pnt2d(-r, 0), c, dir)
    if h < w/2:
        angle = np.arctan2(h,r)
        x, y = r*cos(angle), r*sin(angle)
        pt2 = transform_to_3d_point(gp_Pnt2d(-x, -y), c, dir)
        pt3 = transform_to_3d_point(gp_Pnt2d(x, -y), c, dir)
        circ = gp_Circ(gp_Ax2(c, dir), r)
        sm = [GC_MakeSegment(pt0, pt1), GC_MakeArcOfCircle(circ, pt1, pt2, True), 
              GC_MakeSegment(pt2, pt3), GC_MakeArcOfCircle(circ, pt3, pt0, True)]
    else:
        pt2 = transform_to_3d_point(gp_Pnt2d(-r, -h), c, dir)
        pt3 = transform_to_3d_point(gp_Pnt2d(r, -h), c, dir)
        c23 = transform_to_3d_point(gp_Pnt2d(0, -h), c, dir)
        cir23 = gp_Circ(gp_Ax2(c23, dir), r)
        sm = [GC_MakeSegment(pt0, pt1), GC_MakeSegment(pt1, pt2), 
               GC_MakeArcOfCircle(cir23, pt2, pt3), GC_MakeSegment(pt3, pt0)]
        
    wm = BRepBuilderAPI_MakeWire()
    for seg in sm:
        wm.Add(BRepBuilderAPI_MakeEdge(seg.Value()).Edge())
    fm = BRepBuilderAPI_MakeFace(wm.Wire())
    return fm.Face()