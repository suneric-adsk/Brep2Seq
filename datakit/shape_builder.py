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
            prims = []
            for item in data.get("principal_primitives",[]):
                prim = self.create_primitive(item)
                if prim is not None:
                    prims.append(prim)
            
            for primitive in prims:
                shape = primitive.shape()
                if len(self.shapes) == 0:
                    self.shapes.append(shape)
                else:
                    shape = primitive.fuse_primitive(self.shapes[-1])
                    self.shapes.append(shape)

            # load features and apply them to the fused primitive shape
            feats = []
            for item in data.get("detail_features",[]):
                feature = self.create_feature(item)
                if feature is not None:
                    feats.append(feature)

            rearranged_feats = self.rearrange_features(feats)
            for feature in rearranged_feats:
                base = self.shapes[-1]
                shape = feature.add_feature(base)
                self.shapes.append(shape)

    def create_primitive(self, item):
        """
        Five types of principal primitives: box, cylinder, prism, cone, sphere
        """
        type, param = item["type"], item["param"]
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
        
    def create_feature(self, item):
        type, param = item["type"], item["param"]
        if type == "b_hole":
            return BlindHole(type, param)
        else:
            print("Unknown feature type: ", type)
            return None

    def rearrange_features(self, feats):
        """
        Rearrange the features to make sure the cutting order is correct
        """
        if len(feats) == 0:
            return
        
        transition_feats = []
        step_feats = []
        slot_feats = []
        through_feats = []
        blind_feats = []
        o_ring_feats = []
        for feat in feats:
            if feat.type in ["chamfer", "fillet"]:
                transition_feats.append(feat)
            elif feat.type in ["rect_step", "tside_step", "slant_step", "rect_b_step", "tri_step", "cir_step"]:
                step_feats.append(feat)
            elif feat.type in ["rect_slot", "tri_slot", "cir_slot", "rect_b_slot", "cir_b_slot", "u_b_slot"]:
                slot_feats.append(feat)
            elif feat.type in ["hole", "tri_psg", "rect_psg","hexa_psg"]:
                through_feats.append(feat)
            elif feat.type in ["b_hole", "tri_pkt", "rect_pkt", "key_pkt", "hexa_pkt"]:
                blind_feats.append(feat)
            elif feat.type == "o_ring":
                o_ring_feats.append(feat)
        rarranged_feats = step_feats + slot_feats + through_feats + blind_feats + o_ring_feats + transition_feats
        return rarranged_feats