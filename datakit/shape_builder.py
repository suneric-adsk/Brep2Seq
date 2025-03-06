import json
from datakit.primitives import *
from datakit.features import *

class CadModelCreator:
    """
    Create a CAD model from a json file containing a list of principal primitives and detail features
    """
    def __init__(self, json_file):
        self.json_file = json_file
        self.shapes = []
        self.create_model()

    def get_model(self, idx):
        if idx >= len(self.shapes):
            print(f"Invalid index {idx}, the model contains {len(self.shapes)} shapes")
            return None
        return self.shapes[idx] 

    def create_model(self):
        with open(self.json_file, "r") as f:
            data = json.load(f)
            # load primitives and fuse them together
            prims = self.read_primitive(data)
            for item in prims:
                prim = self.create_primitive(item[0], item[1])
                if prim is None:
                    continue
                
                shape = prim.shape()
                if len(self.shapes) == 0:
                    self.shapes.append(shape)
                    continue
                
                shape = prim.fuse_primitive(self.shapes[-1])
                self.shapes.append(shape)

            if len(self.shapes) == 0:
                return

            # load features and apply them to the fused primitive shape
            feats = self.read_features(data)
            for item in feats:
                feature = self.create_feature(item[0], item[1], self.shapes[-1])
                if feature is None:
                    continue

                shape = feature.add_feature()
                self.shapes.append(shape)

    def read_primitive(self, data):
        prims = []
        for item in data.get("principal_primitives",[]):
            if item["type"] in ["box", "cylinder", "prism", "cone", "sphere"]:
                prims.append((item["type"],item["param"]))
        return prims

    def read_features(self, data):
        transition_feats = []
        step_feats = []
        slot_feats = []
        through_feats = []
        blind_feats = []
        o_ring_feats = []
        for item in data.get("detail_features",[]):
            if item["type"] in ["chamfer", "fillet"]:
                transition_feats.append((item["type"],item["param"]))
            elif item["type"] in ["rect_step", "tside_step", "slant_step", "rect_b_step", "tri_step", "cir_step"]:
                step_feats.append((item["type"],item["param"]))
            elif item["type"] in ["rect_slot", "tri_slot", "cir_slot", "rect_b_slot", "cir_b_slot", "u_b_slot"]:
                slot_feats.append((item["type"],item["param"]))
            elif item["type"] in ["hole", "tri_psg", "rect_psg","hexa_psg"]:
                through_feats.append((item["type"],item["param"]))
            elif item["type"] in ["b_hole", "tri_pkt", "rect_pkt", "key_pkt", "hexa_pkt"]:
                blind_feats.append((item["type"],item["param"]))
            elif item["type"] == "o_ring":
                o_ring_feats.append((item["type"],item["param"]))
        feats = step_feats + slot_feats + through_feats + blind_feats + o_ring_feats + transition_feats
        return feats
    
    def create_primitive(self, type, param):
        """
        Five types of principal primitives: box, cylinder, prism, cone, sphere
        """
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
            return None
        
    def create_feature(self, type, param, base):
        if type == "rect_slot":
            return RectSlot(type, param, base)
        elif type == "tri_slot":
            return TriSlot(type, param, base)
        elif type == "cir_slot":
            return CircSlot(type, param, base)
        elif type == "rect_psg":
            return RectPassage(type, param, base)
        elif type == "tri_psg":
            return TriPassage(type, param, base)
        elif type == "hexa_psg":
            return HexPassage(type, param, base)
        elif type == "hole":
            return Hole(type, param, base)
        elif type == "rect_step":
            return None
        elif type == "tside_step":
            return None
        elif type == "slant_step":
            return None
        elif type == "rect_b_step":
            return None
        elif type == "tri_step":
            return None
        elif type == "cir_step":
            return None
        elif type == "rect_b_slot":
            return RectBlindSlot(type, param, base)
        elif type == "cir_b_slot":
            return HCircBlindSlot(type, param, base)    
        elif type == "u_b_slot":
            return VCircBlineSlot(type, param, base)
        elif type == "rect_pkt":
            return RectPocket(type, param, base)
        elif type == "key_pkt":
            return KeyPocket(type, param, base)
        elif type == "tri_pkt":
            return TriPocket(type, param, base)
        elif type == "hexa_pkt":
            return HexPocket(type, param, base)
        elif type == "o_ring":
            return ORing(type, param, base)
        elif type == "b_hole":
            return BlindHole(type, param, base)
        elif type == "chamfer":
            return Chamfer(type, param, base)
        elif type == "fillet":
            return Fillet(type, param, base)
        else:
            return None
        