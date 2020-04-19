import numpy as np
from cvxopt import matrix, solvers


def nearby_nodes(state,G,r_search):
    r_min = float('inf')
    Nearest_node = None
    key_Nearest_node = None
    Near_nodes = {}
    for key in G.keys():
        d = np.linalg.norm(G[key]['state'][0:3] - state[0:3])
        if d < r_search:
            Near_nodes[key]=G[key]
        if d < r_min:
            Nearest_node = G[key]
            key_Nearest_node = key
            r_min = d
    return Near_nodes,Nearest_node, key_Nearest_node


def get_total_cost(G, node_idx):
    cost = G[node_idx]['cost']
    parent = G[node_idx]['parent']
    while parent != None:
        cost += G[parent]['cost'] 
        parent = G[parent]['parent'] 
        
    return cost


def truncate(p, p_nearest, step):
    delta = p-p_nearest
    p_new = p_nearest + step * delta/np.linalg.norm(delta)
    return p_new    











