from datakit.utils import *
from OCC.Core.BRepPrimAPI import (
    BRepPrimAPI_MakeCylinder, 
)
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut

class Feature:
    """
    Base class for all machine features
    """
    def __init__(self, type, param):
        self.type = type
        self.param = param
    
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
    
    def add_feature(self, base):
        if base is None:
            return None
        
        feat = self.shape()
        if feat is None:
            print("no feature shape to add")
            return base
        
        cut = BRepAlgoAPI_Cut(base, feat).Shape()
        return cut
    
    def shape(self):
        pass
    
class BlindHole(Feature):
    def __init__(self, type, param):
        super().__init__(type, param)

    def shape(self):
        r = level_to_value(int(self.param["rad"])+1,0.0,2.0)
        s = self.start_point()
        e = self.end_point()
        v = gp_Vec(s, e)
        h = v.Magnitude()
        dir = gp_Dir(v)
        print("BlindHole: radius {:.2f} start ({:.2f},{:.2f},{:.2f}), end ({:.2f},{:.2f},{:.2f})".
              format(r, s.X(), s.Y(), s.Z(), e.X(), e.Y(), e.Z()))
        return BRepPrimAPI_MakeCylinder(gp_Ax2(s, dir), r, h).Solid()
