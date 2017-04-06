'''
.. module:: scc
   :synopsis: TODO:

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    TODO:
    Copyright (C) 2014-2016  Cristian Ioan Vasile <cvasile@bu.edu>
    Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Laboratory,
    Boston University

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''


'''
'TODO:
- implement efficiently as described in the paper
- use deques -- sort of circular doubly-linked list
- maybe use heapq or bisect libraries
- FIXME: exception when removing a self-loop in the SCC graph
'''

import networkx as nx


def first_out(G, n):
    return next(G.out_edges_iter([n], data=True), None)

def first_in(G, n):
    return next(G.in_edges_iter([n], data=True), None)

def next_out(G, u):
    return next(G.node[u]['next-out'], None)

def next_in(G, z):
    return next(G.node[z]['next-in'], None)

def initVertexGuidedSearch(sccG):
    for n in sccG.nodes_iter():
        sccG.node[n]['next-out'] = sccG.out_edges_iter([n], data=True)
        next_out(sccG, n)
        sccG.node[n]['next-in'] = sccG.in_edges_iter([n], data=True)
        next_in(sccG, n)

def unite(G, sccG, canonical, newSCC):
    # update canonical vertex for each node in the new SCC
    for n in newSCC:
        G.node[n]['canonical'] = canonical
    
    # TODO: implement more efficiently
    for p in G.nodes_iter():
        if G.node[p]['canonical'] in newSCC:
            G.node[p]['canonical'] = canonical
    
    # save in-going and out-going edges from the newSCC
    outEdges = sccG.out_edges(newSCC, data=True)
    inEdges = sccG.in_edges(newSCC, data=True)
    
    # remove canonical from set of SCC nodes
    newSCC.remove(canonical)
    # update size of SCC
    sccG.node[canonical]['size']  += sum(map(lambda n: sccG.node[n]['size'], newSCC))
    # remove nodes except canonical
    sccG.remove_nodes_from(newSCC)
        
    for _, dest, data in outEdges:
        sccG.add_edge(canonical, G.node[dest]['canonical'], **data)
    
    for source, _, data in inEdges:
        sccG.add_edge(G.node[source]['canonical'], canonical, **data)

def reorderVertexGuidedSearch(G, sccG, v, w, order, F, B, key):
    # compute minimum
    f_v = G.node[v]['canonical']
    f_w = G.node[w]['canonical']
    
    t, minVal = f_v, key[f_v]
    for x in F:
        if sccG.node[x]['out'] and (minVal > key[x]):
            t, minVal = x, key[x]
        
    F_l = filter(lambda x: key[x] < minVal, F)
    B_g = filter(lambda y: key[y] > minVal, B)
        
    X = set(F_l + [t] + B_g)
    Y = set([(u, x) for u, x in sccG.out_edges_iter(F_l) if u != x and x in X]) \
        | set([(y, z) for y, z in sccG.in_edges_iter(B_g) if y != z and y in X]) | set([(f_v, f_w)])
        
    subSccG = nx.DiGraph()
    subSccG.add_nodes_from(X)
    subSccG.add_edges_from(Y)
    
    newSCCs = nx.strongly_connected_components(subSccG)
    
    newSCC = []
    canonical = None
    if len(newSCCs) != len(X):
        # unite
        newSCC = filter(lambda x: len(x)>1, newSCCs)[0]
        canonical = f_v
        
        unite(G, sccG, canonical, newSCC)

    # reorder
    F_l = filter(lambda x: x in F_l, order)
    B_g = filter(lambda x: x in B_g, order)
    
    if t == f_v:
        order = filter(lambda x: (key[x] <= minVal) and (x not in F_l), order) + F_l \
            + filter(lambda x: (key[x] > minVal) and (x not in F_l), order)
    else:
        order = filter(lambda x: (key[x] < minVal) and (x not in F_l) and (x not in B_g), order) \
            + B_g + F_l + [t] \
            + filter(lambda x: (key[x] > minVal) and (x not in F_l) and (x not in B_g), order)
    order = filter(lambda x: x not in newSCC, order)
    
    return order

#-------------------------------------------------------------------------------
def softThresholdSearch(G, key, v, w):
    F = set([w])
    B = set([v])
    
    G.node[w]['out'] = first_out(G, w)
    G.node[v]['in'] = first_in(G, v)
    s = v
    
    if not G.node[w]['out']:
        F_A = set([])
    else:
        F_A = set([w])
    F_P = set([])
    
    if not G.node[v]['in']:
        B_A = set([])
    else:
        B_A = set([v])
    B_P = set([])
    
    while F_A and B_A:
        u = next(iter(F_A))
        z = next(iter(B_A))
        
        if key[u] < key[z]:
            f_q, f_g, _ = G.node[u]['out']
            f_h, f_r, _ = G.node[z]['in']

            
            G.node[u]['out'] = next_out(G, u)
            G.node[z]['in'] = next_in(G, z)
            
            x = f_g
            y = f_h
            
            if not G.node[u]['out']:
                F_A.remove(u)
            if not G.node[z]['in']:
                B_A.remove(z)
            
#            if u == x:
#                G.remove_edge(f_q, f_g)
#            if y == z:
#                G.remove_edge(f_h, f_r)
            
            if x not in F:
                F.add(x)
                G.node[x]['out'] = first_out(G, x)
                if G.node[x]['out']:
                    F_A.add(x)

            if y not in B:
                B.add(y)
                G.node[y]['in'] = first_in(G, y)
                if G.node[y]['in']:
                    B_A.add(y)
        else:
            if key[u] > key[s]:
                F_A.remove(u)
                F_P.add(u)
            elif key[z] < key[s]:
                B_A.remove(z)
                B_P.add(z)
            else:
                F_A.remove(u)
                F_P.add(u)
                B_A.remove(z)
                B_P.add(z)
                
        if not F_A:
            B_P = set([])
            B_A -= set([s])
            if F_P:
                s = next(iter(F_P))
                F_A = set(filter(lambda x: key[x] <= key[s], F_P))
                F_P = F_P - F_A
        
        if not B_A:
            F_P = set([]) 
            F_A -= set([s])
            if B_P:
                s = next(iter(B_P))
                B_A = set(filter(lambda x: key[x] >= key[s], B_P))
                B_P = B_P - B_A
        
    return F, B

#-------------------------------------------------------------------------------
def initNodes(G, sccG, order, Q=None):
    if Q is None:
        Q = G.nodes_iter()
    
    sccG.add_nodes_from(Q)
    for n in Q:
        G.node[n]['canonical'] = n
        sccG.node[n]['size'] = 1
    
    order.extend(Q)

def scc(G, E, sccG, order):
    for v, w in E:
        initVertexGuidedSearch(sccG)
        
        # re-keying
        key = {}
        for n, i in zip(order, range(len(order))):
            key[n] = i
        
        G.add_edge(v, w)
        f_v = G.node[v]['canonical']
        f_w = G.node[w]['canonical']
        sccG.add_edge(G.node[v]['canonical'], G.node[w]['canonical'], e=(v,w))
        
        if key[f_v] > key[f_w]:
            F, B = softThresholdSearch(sccG, key, f_v, f_w)
            
            order = reorderVertexGuidedSearch(G, sccG, v, w, order, F, B, key)

    return order