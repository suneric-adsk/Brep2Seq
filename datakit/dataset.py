import os 
from tqdm import tqdm
from torch.utils.data import Dataset, DataLoader
from data.collator import collator
from datakit.graph_builder import GraphBuilder
from occwl.io import load_step
from torch_geometric.data import Data as PYGGraph
import torch

"""
STEP dataset for test only
"""
class STEPDataSet(Dataset):
    def __init__(self, root_dir):
        self.files = self.load_files(root_dir)

    def load_files(self, root_dir):
        """Load files in root_dir"""
        files = os.listdir(root_dir)
        return [f for f in files if f.endswith(".stp") or f.endswith(".step")]
        
    def __len__(self):
        return len(self.files)
    
    def load_one_graph(self, filepath):
        graph_builder = GraphBuilder()
        graph_builder.build_graph(load_step(filepath)[0], 10, 10, 10)
        graph = graph_builder.dgl_graph
        label = graph_builder.label
        dense_adj = graph.adj().to_dense().type(torch.int)
        N = graph.num_nodes()

        pyg_graph = PYGGraph()
        pyg_graph.graph = graph 
        pyg_graph.node_data = graph.ndata["x"].type(torch.FloatTensor)   #node_data[num_nodes, U_grid, V_grid, pnt_feature]
        pyg_graph.face_area = graph.ndata["y"].type(torch.int)     #face_area[num_nodes]
        pyg_graph.face_type = graph.ndata["z"].type(torch.int)     #face_type[num_nodes]
        pyg_graph.edge_data = graph.edata["x"].type(torch.FloatTensor)
        pyg_graph.in_degree = dense_adj.long().sum(dim=1).view(-1)       
        pyg_graph.attn_bias = torch.zeros([N + 1, N + 1], dtype=torch.float)
        pyg_graph.edge_path = ["edges_path"]           # edge_input[num_nodes, num_nodes, max_dist, 1, U_grid, pnt_feature]
        pyg_graph.spatial_pos = label["spatial_pos"]        # spatial_pos[num_nodes, num_nodes]
        pyg_graph.d2_distance = label["d2_distance"]        # d2_distance[num_nodes, num_nodes, 64]
        pyg_graph.angle_distance = label["angle_distance"]  # angle_distance[num_nodes, num_nodes, 64]

        if ("commands_primitive" in label):
            pyg_graph.label_commands_primitive = label["commands_primitive"]
            pyg_graph.label_args_primitive = label["args_primitive"]
            pyg_graph.label_commands_feature = label["commands_feature"]
            pyg_graph.label_args_feature = label["args_feature"]
        else:
            pyg_graph.label_commands_primitive = torch.zeros([10])
            pyg_graph.label_args_primitive = torch.zeros([10, 11])
            pyg_graph.label_commands_feature = torch.zeros([12])
            pyg_graph.label_args_feature = torch.zeros([12, 12])

        # pyg_graph.data_id = int(os.path.basename(filepath).splitext()[0])
        return pyg_graph
    
    def __getitem__(self, index):
        fn = self.files[index]
        sample = self.load_one_graph(fn)
        return sample
    
    def _collate(self, batch):
        return collator(
            items = batch,
            split = "test"
        )
    
    def get_dataloader(self, batch_size, shuffle=True, num_workers=0, drop_last=True): 
        return DataLoader(
            dataset = self,
            batch_size=batch_size,
            shuffle=shuffle,
            collate_fn=self._collate,
            num_workers=num_workers,
            drop_last=drop_last
        )