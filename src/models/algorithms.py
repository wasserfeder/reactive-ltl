'''
.. module:: algorithms
   :synopsis: # TODO:

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''
'''
    #TODO:
    Copyright (C) 2013  Cristian Ioan Vasile <cvasile@bu.edu>
    Hybrid and Networked Systems (HyNeSs) Laboratory, Boston University

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

__all__ = ['self_reachable_final_states', 'has_empty_language']


def self_reachable_final_states(model, trivial=False):
    '''
    Returns the list of self-reachable final states of the given model. The
    algorithms only considers those final states which are reachable from some
    initial state of the model. 
    
    Adapted from networkx.networkx.algorithms.components.strongly_connected.strongly_connected_components.

    Parameters
    ----------
    model : Model
       A finite system model.
    trivial: boolean
       Specify if self-loops are allowed in the definition of self-reachability.
    By default, self-loops are not allowed.

    Returns
    -------
    self_reachable_final_states : list
       The list of self-reachable final states.
    
    See Also
    --------
    networkx.networkx.algorithms.components.strongly_connected.strongly_connected_components
    has_empty_language

    Notes
    -----
    Uses Tarjan's algorithm with Nuutila's modifications.
    Nonrecursive version of algorithm.

    References
    ----------
    .. [1] Depth-first search and linear graph algorithms, R. Tarjan
       SIAM Journal of Computing 1(2):146-160, (1972).

    .. [2] On finding the strongly connected components in a directed graph.
       E. Nuutila and E. Soisalon-Soinen
       Information Processing Letters 49(1): 9-14, (1994).
    '''
    preorder = {}
    lowlink = {}
    scc_found = {}
    scc_queue = []
    self_reachable_final_states = set()
    i = 0     # Preorder counter
    for source in model.init: # only use nodes reachable from some initial state
        if source not in scc_found:
            queue = [source]
            while queue:
                v = queue[-1]
                if v not in preorder:
                    i += 1
                    preorder[v] = i
                done = True
                v_nbrs = model.g[v]
                for w in v_nbrs:
                    if w not in preorder:
                        queue.append(w)
                        done = False
                        break
                if done:
                    lowlink[v] = preorder[v]
                    for w in v_nbrs:
                        if w not in scc_found:
                            if preorder[w] > preorder[v]:
                                lowlink[v] = min([lowlink[v], lowlink[w]])
                            else:
                                lowlink[v] = min([lowlink[v], preorder[w]])
                    queue.pop()
                    if lowlink[v] == preorder[v]:
                        scc_found[v] = True
                        scc_trivial = True
                        scc_final = [v] if v in model.final else []
                        while scc_queue and preorder[scc_queue[-1]] > preorder[v]:
                            k = scc_queue.pop()
                            scc_found[k] = True
                            scc_trivial = False
                            if k in model.final:
                                scc_final.append(k)
                        if trivial or (not trivial and not scc_trivial):
                            # if self-loops are allowed or the scc is not
                            # trivial (i.e. its length is greater than 1)
                            self_reachable_final_states.extend(scc_final)
                    else:
                        scc_queue.append(v)
    return self_reachable_final_states

def has_empty_language(model, trivial=False):
    '''
    Checks if the language associated with the model is empty. It verifies if
    there are any self-reachable final states of the model which are also
    reachable from some initial state.
    
    Parameters
    ----------
    model : Model
       A finite system model.
    trivial: boolean
       Specify if self-loops are allowed in the definition of self-reachability.
    By default, self-loops are not allowed.

    Returns
    -------
    empty : boolean
       True if the language is empty.
    
    See Also
    --------
    networkx.networkx.algorithms.components.strongly_connected.strongly_connected_components
    self_reachable_final_states
    product
    
    Note
    ----
    This function is intended to be used on product automata.
    '''
    return len(self_reachable_final_states(model, trivial)) == 0

def product():
    pass

if __name__ == '__main__':
    pass