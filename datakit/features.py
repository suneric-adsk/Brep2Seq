from datakit.utils import *
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Core.BRepFilletAPI import BRepFilletAPI_MakeFillet, BRepFilletAPI_MakeChamfer

class Feature:
    """
    Base class for all machine features
    """
    def __init__(self, type, param, base=None):
        self.type = type
        self.param = param
        self.base = base

    def is_valid(self, s, e, w, h = None):
        if gp_Vec(s,e).Magnitude() < PT_Tol:
            return False
        if w < PT_Tol:
            return False
        if h is not None and h < PT_Tol:    
            return False
        return True
    
    def start_point(self):
        x = level_to_value(int(self.param["x1"]))
        y = level_to_value(int(self.param["y1"]))
        z = level_to_value(int(self.param["z1"]))
        return gp_Pnt(x, y, z)
    
    def end_point(self):
        x = level_to_value(int(self.param["x2"]))
        y = level_to_value(int(self.param["y2"]))
        z = level_to_value(int(self.param["z2"]))
        return gp_Pnt(x, y, z)
    
    def add_feature(self):
        raise NotImplementedError("Subclass must implement abstract method")
    
class RectSlot(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("RectSlot: width {:.2f}, height {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, h, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, w, h):
            print("RectSlot: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        pt2ds = [gp_Pnt2d(-w/2, 0), gp_Pnt2d(-w/2, -h), gp_Pnt2d(w/2, -h), gp_Pnt2d(w/2, 0)]
        pt3ds = transform_to_3d_points(pt2ds, s, dir)
        face = make_face_polygon(pt3ds)
        return apply_feature(self.base, face, dir)   

class TriSlot(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("TriSlot: width {:.2f}, height {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, h, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, w, h):
            print("TriSlot: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        pt2ds = [gp_Pnt2d(-w/2, 0), gp_Pnt2d(0, -h), gp_Pnt2d(w/2, 0)]
        pt3ds = transform_to_3d_points(pt2ds, s, dir)
        face = make_face_polygon(pt3ds)
        return apply_feature(self.base, face, dir)

class CircSlot(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        # h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("CircSlot: width {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, w):
            print("CircSlot: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        face = make_face_circle_x(s, w, w/2, dir)
        return apply_feature(self.base, face, dir)
    
class RectPassage(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("RectPassage: width {:.2f}, height {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, h, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, w, h):
            print("RectPassage: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        pt2ds = [gp_Pnt2d(-w/2, h/2), gp_Pnt2d(-w/2, -h/2), gp_Pnt2d(w/2, -h/2), gp_Pnt2d(w/2, h/2)]
        pt3ds = transform_to_3d_points(pt2ds, s, dir)
        face = make_face_polygon(pt3ds)
        return apply_feature(self.base, face, dir)

class TriPassage(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("TriPassage: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, r):
            print("TriPassage: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        pt2ds = make_2d_polygon_points(3, r)
        pt3ds = transform_to_3d_points(pt2ds, s, dir)
        face = make_face_polygon(pt3ds)
        return apply_feature(self.base, face, dir)

class HexPassage(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("HexPassage: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, r):
            print("HexPassage: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        pt2ds = make_2d_polygon_points(6, r)
        pt3ds = transform_to_3d_points(pt2ds, s, dir)
        face = make_face_polygon(pt3ds)
        return apply_feature(self.base, face, dir)
    
class Hole(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("Hole: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, r):
            print("Hole: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        face = make_face_circle(s, r, dir)
        return apply_feature(self.base, face, dir)
    
class RectStep(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)
    
    def add_feature(self):
        d = level_to_value(int(self.param["dep"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("RectStep: depth {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(d, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, d):
            print("RectStep: Invalid parameters")
            return self.base
        
        edge = find_closet_parallel_edge(self.base, s, e)
        if edge is None:
            print("RectStep: No parallel edge found")
            return self.base 
        
        pt1 = find_intersection_point(edge, s, gp_Dir(gp_Vec(s, e)))
        pt2 = find_intersection_point(edge, e, gp_Dir(gp_Vec(s, e)))
        if pt1 is None or pt2 is None:
            print("RectStep: No intersection found")
            return self.base
        
        points = [s, pt1, pt2, e]
        offset_pts, dir = get_offset_points(points, -d)
        face = make_face_polygon(offset_pts)
        return apply_feature(self.base, face, dir)
    
class TwoSideStep(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)
    
    def add_feature(self):
        w = level_to_value(int(self.param["wid_s1"])+1,0.0,2.0)
        d = level_to_value(int(self.param["dep"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("TwoSideStep: width {:.2f}, depth {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, d, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, w, d):
            print("TwoSideStep: Invalid parameters")
            return self.base
        
        edge = find_closet_parallel_edge(self.base, s, e)
        if edge is None:
            print("TwoSideStep: No parallel edge found")
            return self.base 

        pt1 = find_intersection_point(edge, s, gp_Dir(gp_Vec(s, e)))
        pt2 = find_intersection_point(edge, e, gp_Dir(gp_Vec(s, e)))
        if pt1 is None or pt2 is None:
            print("TwoSideStep: No intersection found")
            return self.base
        
        refpt1 = middle_point(s,e)
        refpt2 = middle_point(pt1,pt2)
        pt3 = calculate_point(refpt2, gp_Dir(gp_Vec(refpt2, refpt1)), w)
        points = [s, pt1, pt2, e, pt3]
        offset_pts, dir = get_offset_points(points, -d)
        face = make_face_polygon(offset_pts)
        return apply_feature(self.base, face, dir)
    
class SlantStep(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)
    
    def add_feature(self):
        d = level_to_value(int(self.param["dep"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("SlantStep: depth {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(d, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, d):
            print("SlantStep: Invalid parameters")
            return self.base

        face = find_closet_face(self.base, s, e)
        if face is None:
            print("SlantStep: No face found")
            return self.base
        
        edge = find_closet_parallel_edge(face, s, e)
        if edge is None:
            print("SlantStep: No parallel edge found")
            return self.base
        
        pt1 = find_intersection_point(edge, s, edge_direction(edge))
        pt2 = find_intersection_point(edge, e, edge_direction(edge))
        if pt1 is None or pt2 is None:
            print("SlantStep: No intersection found")
            return self.base
        
        points = [s, pt1, pt2, e]
        offset_pts, dir = get_offset_points(points, -d)
        face = make_face_polygon(offset_pts)
        return apply_feature(self.base, face, dir)
    
class RectBlindStep(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)
    
    def add_feature(self):
        d = level_to_value(int(self.param["dep"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("RectBlindStep: depth {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(d, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, d):
            print("RectBlindStep: Invalid parameters")
            return self.base

        face = find_closet_face(self.base, s, e)
        if face is None:
            print("RectBlindStep: No face found")
            return self.base

        pt1 = find_closet_vertex_on_face(face, s, e)
        ref_vec = gp_Vec(pt1, e)
        pt2 = calculate_point(s, gp_Dir(ref_vec), ref_vec.Magnitude())
        points = [s, pt1, e, pt2]
        offset_pts, dir = get_offset_points(points, -d)
        face = make_face_polygon(offset_pts)
        return apply_feature(self.base, face, dir)

class TriBlindStep(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)
    
    def add_feature(self):
        d = level_to_value(int(self.param["dep"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("TriBlindStep: depth {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
                format(d, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, d):
            print("TriBlindStep: Invalid parameters")
            return self.base
        
        face = find_closet_face(self.base, s, e)
        if face is None:
            print("TriBlindStep: No face found")
            return self.base
        
        pt = find_closet_vertex_on_face(face, s, e)
        points = [s, pt, e]
        offset_pts, dir = get_offset_points(points, -d)
        face = make_face_polygon(offset_pts)
        return apply_feature(self.base, face, dir)

class CircBlindStep(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)
    
    def add_feature(self):
        d = level_to_value(int(self.param["dep"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("CircBlindStep: depth {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
                format(d, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, d):
            print("CircBlindStep: Invalid parameters")
            return self.base
        
        face = find_closet_face(self.base, s, e)
        if face is None:
            print("CircBlindStep: No face found")
            return self.base
        
        c = find_closet_vertex_on_face(face, s, e)
        points = [c, e, s]
        offset_pts, dir = get_offset_points(points, -d)
        face = make_face_fan(offset_pts[0], offset_pts[1], offset_pts[2])
        return apply_feature(self.base, face, dir)

class RectBlindSlot(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("RectBlindStep: width {:.2f}, height {:.2f}, start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, h, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, w, h):
            print("RectBlindSlot: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        pt2ds = [gp_Pnt2d(-w/2,0), gp_Pnt2d(-w/2,-h), gp_Pnt2d(w/2,-h), gp_Pnt2d(w/2,0)]
        pt3ds = transform_to_3d_points(pt2ds, s, dir)
        face = make_face_polygon(pt3ds)
        return apply_feature(self.base, face, dir)

class HCircBlindSlot(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        # h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("HCircBlindSlot: width {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, w):
            print("HCircBlindSlot: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        face = make_face_circle_x(s, w, w/2, dir)
        return apply_feature(self.base, face, dir)

class VCircBlineSlot(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)
    
    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("VCircBlindSlot: width {:.2f}, height {:.2f}, start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, h, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, w, h):
            print("VCircBlindSlot: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        face = make_face_circle_x(s, w, h, dir)
        return apply_feature(self.base, face, dir)

class RectPocket(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("RectPocket: width {:.2f}, height {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, h, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, w, h):
            print("RectPocket: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        pt2ds = [gp_Pnt2d(-w/2,h/2), gp_Pnt2d(-w/2,-h/2), gp_Pnt2d(w/2,-h/2), gp_Pnt2d(w/2,h/2)]
        pt3ds = transform_to_3d_points(pt2ds, s, dir)
        face = make_face_polygon(pt3ds)
        return apply_feature(self.base, face, dir)    
    
class KeyPocket(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("KeyPocket: width {:.2f}, height {:.2f}, start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, h, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z())) 
        if not self.is_valid(s, e, w, h):
            print("KeyPocket: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        face = make_face_key_hole(s, w, h, dir)
        return apply_feature(self.base, face, dir)
    
class TriPocket(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("TriPocket: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, r):
            print("TriPocket: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        pt2ds = make_2d_polygon_points(3, r)
        pt3ds = transform_to_3d_points(pt2ds, s, dir)
        face = make_face_polygon(pt3ds)
        return apply_feature(self.base, face, dir)

class HexPocket(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("HexPocket: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, r):
            print("HexPocket: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        pt2ds = make_2d_polygon_points(6, r)
        pt3ds = transform_to_3d_points(pt2ds, s, dir)
        face = make_face_polygon(pt3ds)
        return apply_feature(self.base, face, dir)
    
class ORing(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)
    
    def add_feature(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("ORing: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, r):
            print("ORing: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        face = make_face_oring(s, r, dir)
        return apply_feature(self.base, face, dir)

class BlindHole(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("BlindHole: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, r):
            print("BlindHole: Invalid parameters")
            return self.base
        
        dir = gp_Dir(gp_Vec(s, e))
        face = make_face_circle(s, r, dir) 
        return apply_feature(self.base, face, dir)
    
class Chamfer(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("Chamfer: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, r):
            print("Chamfer: Invalid parameters")
            return self.base
        
        edge = find_closet_parallel_edge(self.base, s, e)
        if edge is None:
            print("Chamfer: No parallel edge found")
            return self.base 
        
        cm = BRepFilletAPI_MakeChamfer(self.base)
        cm.Add(r, edge)
        if cm.IsDone():
            return cm.Shape()
        else:
            print("Chamfer: Failed to add chamfer")
            return self.base
    
class Fillet(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)
    
    def add_feature(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("Fillet: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        if not self.is_valid(s, e, r):
            print("Fillet: Invalid parameters")
            return self.base
        
        edge = find_closet_parallel_edge(self.base, s, e)
        if edge is None:
            print("Fillet: No parallel edge found")
            return self.base 
        
        fm = BRepFilletAPI_MakeFillet(self.base)
        fm.Add(r, edge)
        if fm.IsDone():
            return fm.Shape()
        else:
            print("Fillet: Failed to add fillet")
            return self.base