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
    
class BlindHole(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        dir = gp_Dir(gp_Vec(s, e))
        print("BlindHole: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        face = make_face_circle(r, s, dir) 
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
        edge = find_closet_edge(self.base, s, e)
        cm = BRepFilletAPI_MakeChamfer(self.base)
        cm.Add(r, edge)
        chamfer = cm.Shape()
        return chamfer
    
class Fillet(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)
    
    def add_feature(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        print("Fillet: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        edge = find_closet_edge(self.base, s, e)
        fm = BRepFilletAPI_MakeFillet(self.base)
        fm.Add(r, edge)
        fillet = fm.Shape()
        return fillet
    
class ORing(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)
    
    def add_feature(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        dir = gp_Dir(gp_Vec(s, e))
        print("ORing: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        face = make_face_oring(r, s, dir)
        return apply_feature(self.base, face, dir)

class RectSlot(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        dir = gp_Dir(gp_Vec(s, e))
        print("RectSlot: width {:.2f}, height {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, h, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        pt2ds = [gp_Pnt2d(-w/2, 0), gp_Pnt2d(w/2, 0), gp_Pnt2d(w/2, h), gp_Pnt2d(-w/2, h)]
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
        dir = gp_Dir(gp_Vec(s, e))
        print("TriSlot: width {:.2f}, height {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, h, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        pt2ds = [gp_Pnt2d(-w/2, 0), gp_Pnt2d(w/2, 0), gp_Pnt2d(0, h)]
        pt3ds = transform_to_3d_points(pt2ds, s, dir)
        face = make_face_polygon(pt3ds)
        return apply_feature(self.base, face, dir)

class CircSlot(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        r = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        dir = gp_Dir(gp_Vec(s, e))
        print("CircSlot: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        face = make_face_circle(r, s, dir)
        return apply_feature(self.base, face, dir)
    
class RectPassage(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        dir = gp_Dir(gp_Vec(s, e))
        print("RectPassage: width {:.2f}, height {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, h, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        pt2ds = [gp_Pnt2d(-w/2, 0), gp_Pnt2d(w/2, 0), gp_Pnt2d(w/2, h), gp_Pnt2d(-w/2, h)]
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
        dir = gp_Dir(gp_Vec(s, e))
        print("TriPassage: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
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
        dir = gp_Dir(gp_Vec(s, e))
        print("HexPassage: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
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
        dir = gp_Dir(gp_Vec(s, e))
        print("Hole: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        face = make_face_circle(r, s, dir)
        return apply_feature(self.base, face, dir)
    
class RectBlindSlot(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        dir = gp_Dir(gp_Vec(s, e))
        print("RectBlindStep: width {:.2f}, height {:.2f}, start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, h, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        pt2ds = [gp_Pnt2d(-w/2,0), gp_Pnt2d(w/2,0), gp_Pnt2d(w/2,h), gp_Pnt2d(-w/2,h)]
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
        dir = gp_Dir(gp_Vec(s, e))
        print("HCircBlindSlot: width {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        face = make_face_circle_slot(w/2, s, dir)
        return apply_feature(self.base, face, dir)

class VCircBlineSlot(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)
    
    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        dir = gp_Dir(gp_Vec(s, e))
        print("VCircBlindSlot: width {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        face = make_face_circle(w/2, s, dir)
        return apply_feature(self.base, face, dir)

class RectPocket(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        w = level_to_value(int(self.param["wid"])+1,0.0,2.0)
        h = level_to_value(int(self.param["len"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        dir = gp_Dir(gp_Vec(s, e))
        print("RectPocket: width {:.2f}, height {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, h, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        pt2ds = [gp_Pnt2d(-w/2,-h/2), gp_Pnt2d(w/2,-h/2), gp_Pnt2d(w/2,h/2), gp_Pnt2d(-w/2,h/2)]
        pt3ds = transform_to_3d_points(pt2ds, s, dir)
        face = make_face_polygon(pt3ds)
        return apply_feature(self.base, face, dir)    
    
class TriPocket(Feature):
    def __init__(self, type, param, base):
        super().__init__(type, param, base)

    def add_feature(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        dir = gp_Dir(gp_Vec(s, e))
        print("TriPocket: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
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
        dir = gp_Dir(gp_Vec(s, e))
        print("HexPocket: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        pt2ds = make_2d_polygon_points(6, r)
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
        dir = gp_Dir(gp_Vec(s, e))
        print("KeyPocket: width {:.2f}, height {:.2f}, start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(w, h, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))   
        face = make_face_key_hole(w, h, s, dir)
        return apply_feature(self.base, face, dir)
    