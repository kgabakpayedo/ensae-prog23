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

    
    def powermin(self,node1,node2)::
        """Cette fonction est spécialement crée pour compléter la fonction trajet
        Il n'y a pas à s'inquéter de l'adjacence des nœuds si l'on suppose que la fonction trajet fait
        bien son boulot. Ainsi les nœuds considérés dans les cas pratiques où nous utiliserons la fonction 
        seront toujours adjacent.
        """
        voisins = self.graph[node1][i]
        return [voisins for i in range len(voisins) if i == node2][1]



    def get_path_with_power(self, src, dest, power):
        p= False
        i = 0
        chemins = trajet(self,src,destination)
        nb_chemins = len(chemins)
        while p = False and i != len(chemins):
            p max([powermin(self,chemins[i][j],chemins[i][j+1]) for j in range(1,nb_chemins)])
            i+=1
        if p == True :
            output = f"Le camion peut aller de {src} à {dest} en faisant le parcours {chemin[i-1]}"
        else :
            output = f"Le camion ne peut pas couvrir le trajet entre {src} et {dest}"
        return output

        
    def parcours(self, start, visited=[]):
        """On le nœud de départ à la liste visited par défaut vide (Sa valeur est actualisée au cours de 
        l'exécution de la fonction). 
        Ensuite, pour chaque voisin du nœud de départ dans le graphe qui n'est pas encore dans visited, 
        la fonction parcours est récursivement appliquée avec ce voisin comme nouveau nœud 
        de départ et la liste visited se complète au fur et à mesure.
        Si le graphe est vide on retourne la liste vide.

        Args:
        ----------
            start (NodeType): Le Nœud duquel on part
            visited (list, optional): Liste stockant les nouveaux nœus visités. Defaults to [].

        Returns:
        ----------
            Visited: Liste contenant tous les nœuds visités.
        """
        if not self.graph :
            return []
        visited.append(start)
        for node in self.graph[start]:
            if node[0] not in visited:
                parcours(self, node, visited)
        return visited 

    

    def connected_components(self):
        """La realtion "est connecté à" définit une relation d'équivalence sur l'ensemble des nœuds 
        du graphe. Ainsi il suffit d'identifier un représentant d'une composante connexe par arc et
        de trouver tous les nœuds de sa classe d'équivalence avec la fonction parcours.
        On commence donc par le premier nœud du graphe puis on choisit le prochain sur la liste des nœuds
        n'est pas joignable à partir de nœud précedent ; Il appartient à une autre composante connexe par arc.

        Returns:
            set: un set de set stockant chacun tous les nœuds appartenant à une composante connexe 
            du graphe.  
        """
        linked_nodes = set()
        l=[]
        for node in self.nodes and not in l :
             linked_nodes.add(set(parcours(self, node)))
             l+=parcours(self,node) 
        return linked_nodes 


    def connected_components_set(self):
        """
        The result should be a set of frozensets (one per component), 
        For instance, for network01.in: {frozenset({1, 2, 3}), frozenset({4, 5, 6, 7})}
        """
        return set(map(frozenset, self.connected_components()))
    
    def min_power(self, src, dest):
        """
        Should return path, min_power. 
        """
        raise NotImplementedError


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
