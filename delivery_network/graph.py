class Graph:
    """
    A class representing graphs as adjacency lists and implementing various algorithms on the graphs. Graphs in the class are not oriented. 
    Attributes: 
    -----------
    nodes: NodeType
        A list of nodes. Nodes can be of any immutable type, e.g., integer, float, or string.
        We will usually use a list of integers 1, ..., n.
    graph: dict
        A dictionnary that contains the adjacency list of each node in the form
        graph[node] = [(neighbor1, p1, d1), (neighbor1, p1, d1), ...]
        where p1 is the minimal power on the edge (node, neighbor1) and d1 is the distance on the edge
    nb_nodes: int
        The number of nodes.
    nb_edges: int
        The number of edges. 
    """

    def __init__(self, nodes=[]):
        """
        Initializes the graph with a set of nodes, and no edges. 
        Parameters: 
        -----------
        nodes: list, optional
            A list of nodes. Default is empty.
        """
        self.nodes = nodes
        self.graph = dict([(n, []) for n in nodes])
        self.nb_nodes = len(nodes)
        self.nb_edges = 0
    

    def __str__(self):
        """Prints the graph as a list of neighbors for each node (one per line)"""
        if not self.graph:
            output = "The graph is empty"            
        else:
            output = f"The graph has {self.nb_nodes} nodes and {self.nb_edges} edges.\n"
            for source, destination in self.graph.items():
                output += f"{source}-->{destination}\n"
        return output
    
    def add_edge(self, node1, node2, power_min, dist=1):
        """
        Adds an edge to the graph. Graphs are not oriented, hence an edge is added to the adjacency list of both end nodes. 

        Parameters: 
        -----------
        node1: NodeType
            First end (node) of the edge
        node2: NodeType
            Second end (node) of the edge
        power_min: numeric (int or float)
            Minimum power on this edge
        dist: numeric (int or float), optional
            Distance between node1 and node2 on the edge. Default is 1.
        """
        if node1 not in self.graph:
            self.graph[node1] = []
            self.nb_nodes += 1
            self.nodes.append(node1)
        if node2 not in self.graph:
            self.graph[node2] = []
            self.nb_nodes += 1
            self.nodes.append(node2)

        self.graph[node1].append((node2, power_min, dist))
        self.graph[node2].append((node1, power_min, dist))
        self.nb_edges += 1


    def get_path_with_power(self, src, dest, power):
        chemins = trajet(self,src,dest)
        nb_chemins = len(chemins)
        chemins_possibles=[]
        for i in range (nb_chemins) :
            if power >= max([powermin(self,chemins[i][j-1],chemins[i][j]) for j in range(1,len(chemins[i]))]):
                chemins_possibles.append(chemins[i])
        if chemins_possibles == [] :
            return None
        else :
            return chemins_possibles

    
    def connected_components(self):
        linked_nodes_list = []
        l=[]
        for node in self.nodes :
            if node not in l :
                linked_nodes_list.append(set((explore(self, node))))
                l+=explore(self,node) 
        return linked_nodes_list

    def connected_components_set(self): 
        """
        The result should be a set of frozensets (one per component), 
        For instance, for network01.in: {frozenset({1, 2, 3}), frozenset({4, 5, 6, 7})}

        Complexity = O(V+E+k*n)
        """
        return set(frozenset(s) for s in self.connected_components())
    
    def min_power(self, src, dest):
        """
        We start by returning all possible paths between the nodes. Then, we create a list
        giving the highest powermin for each edge in the path, for each path. Finally we return
        the first path where this value is the lowest. 
        Should return path, min_power. 
        """
        paths = trajet(self, src, dest)
        paths_list = [(paths[i],max([powermin(self,paths[i][j-1],paths[i][j]) for j in range(1,len(paths[i]))])) for i in range(len(paths))]
        return min(paths_list, key = lambda x : x[1])

def graph_from_file(filename):
    """
    Reads a text file and returns the graph as an object of the Graph class.

    The file should have the following format: 
        The first line of the file is 'n m'
        The next m lines have 'node1 node2 power_min dist' or 'node1 node2 power_min' (if dist is missing, it will be set to 1 by default)
        The nodes (node1, node2) should be named 1..n
        All values are integers.

    Parameters: 
    -----------
    filename: str
        The name of the file

    Outputs: 
    -----------
    g: Graph
        An object of the class Graph with the graph from file_name.
    """
    with open(filename, "r") as file:
        n, m = map(int, file.readline().split())
        g = Graph(range(1, n+1))
        for _ in range(m):
            edge = list(map(int, file.readline().split()))
            if len(edge) == 3:
                node1, node2, power_min = edge
                g.add_edge(node1, node2, power_min) # will add dist=1 by default
            elif len(edge) == 4:
                node1, node2, power_min, dist = edge
                g.add_edge(node1, node2, power_min, dist)
            else:
                raise Exception("Format incorrect")
    return g

def sorted_list_edge(g) :
    """Retourne la liste des aretes rang??es par ordre de puissance croissante.

    Args:
        g (Graph): 

    Returns:
        Liste: Liste de Tuples contenant les deux noeuds de l'arete ainsi que la puissance de celle ci
    """
    mylist=[]
    for node in g.nodes() :
        for voisin in g.graph[node]:
            if (node,voisin[0],voisin[1]) and (voisin[0],node,voisin[1]) not in mylist :
                mylist+=[(node,voisin[0],voisin[1])]
    return mylist.sort(key=lambda x : x[2])    


def explore(g, start, visited=None):
    """On ajoute le n??ud de d??part ?? la liste visited par d??faut vide (Sa valeur est actualis??e au cours de 
        l'ex??cution de la fonction). 
        Ensuite, pour chaque voisin du n??ud de d??part dans le graphe qui n'est pas encore dans visited, 
        la fonction explore est r??cursivement appliqu??e avec ce voisin comme nouveau n??ud 
        de d??part et la liste visited se compl??te au fur et ?? mesure.
        Si le graphe est vide on retourne la liste vide.

        Args:
        ----------
            start (NodeType): Le N??ud duquel on part
            visited (list, optional): Liste stockant les nouveaux n??us visit??s. Defaults to [].

        Returns:
        ----------
            Visited: Liste contenant tous les n??uds visit??s.
    """
    if visited is None :
        visited=[]
    visited.append(start)
    for node in g.graph[start]:
        if node[0] not in visited:
            explore(g, node[0], visited)
    return visited 
    
from graphviz import Digraph

def plot_path(g, start, target, shortest_path:list):
    """
    Plot the graph, the path to the target node, and the shortest path found by the BFS algorithm.
    """
    m = g.nodes
    dot = Digraph(comment='Graph')
    # Add nodes to the graph
    for n in m:
        dot.node(str(n))
    # Add edges to the graph
    for n in m :
        for neighbor in g.graph[n]:
            dot.edge(str(n), str(neighbor[0]))
    # Mark the start node
    dot.node(str(start), color='green', style='filled')
    # Mark the target node
    dot.node(str(target), color='red', style='filled')
    """
    # Highlight the shortest path found by the BFS algorithm
    for i in range(len(shortest_path)-1):
        print(type(shortest_path[i]))
        dot.edge(str(shortest_path[i]), str(shortest_path[i+1]), color='green', penwidth=3)"""
    # Render the graph
    dot.render('graph')  
    
    
def trajet(g, src, dest, path=[]):
    """
    Return a list of all possible paths between the start and target nodes in the graph.
    """
    path = path + [src]
    if src == dest:
        return [path]
    if src not in g.nodes :
        return []
    paths = []
    for neighbor in g.graph[src]:
        if neighbor[0] not in path:
            new_paths = trajet(g, neighbor[0], dest, path)
            for new_path in new_paths:
                paths.append(new_path)
    return paths 

    
def powermin(g,node1,node2):
    """Cette fonction est sp??cialement cr??e pour compl??ter la fonction trajet
        Il n'y a pas ?? s'inqu??ter de l'adjacence des n??uds si l'on suppose que la fonction trajet fait
        bien son boulot. Ainsi les n??uds consid??r??s dans les cas pratiques o?? nous utiliserons la fonction 
        seront toujours adjacent.
        """
    voisins = g.graph[node1]
    c=len(voisins)
    i=0
    while i < c and voisins[i][0] != node2 :
        i+=1
    return voisins[i][1]



