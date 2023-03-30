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
    
    def min_powerr(self, src, dest):
        """
        Should return path, min_power. 
        """
        a=0
        b=0
        c=0
        m=0
        t=[]
        for i in self.graph:
            for j in self.graph[i]:
                if j[1]!=0:
                    t.append(j[1])
        b=sum(t)
        m=min(t)
        while b-a>=m:
            c=(a+b)/2
            if self.get_path_with_power(src,dest,c)!=None:
                b=c
            else:
                a=c
        v=[]
        U=self.get_path_with_power(src,dest,b)
        if U==None:
            return None
        for i in range(len(U)-1):
            for j in self.graph[U[i]]:
                if j[0]==U[i+1]:
                    v.append(j[1])
        return [U,max(v)]

    


    def get_path_with_power(self, src, dest, power):
        """
        On commence par enregistrer la liste de liste donnant tous les trajets possibles 
        Ensuite pour chaque chemin on compare la puissance du camion avec la plus grande puissance 
        qu'il rencontrera entre deux nœuds voisins du chemin. Le chemin est naturellement enregistré 
        si la puissance du camion est supérieur à cette valeur maximale.
        Si le camion ne peut prendre aucun chemin on retourne None. Sinon on retourne une liste de liste 
        avec chaque trajet possible.

        Complexité = O(m*n)
    
        """
        chemins = trajet(self,src,dest)
        nb_chemins = len(chemins)
        chemins_possibles=[]
        for i in range (nb_chemins) :
            if power >= max([powermin(self,chemins[i][j-1],chemins[i][j]) for j in range(1,len(chemins[i]))]):
                chemins_possibles.append(chemins[i])
        if chemins_possibles == [] :
            return None
        else :
            return (chemins_possibles, dist_min(self, chemins_possibles))

    
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
    """Retourne la liste des aretes rangées par ordre de puissance croissante.

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
    """On ajoute le nœud de départ à la liste visited par défaut vide (Sa valeur est actualisée au cours de 
        l'exécution de la fonction). 
        Ensuite, pour chaque voisin du nœud de départ dans le graphe qui n'est pas encore dans visited, 
        la fonction explore est récursivement appliquée avec ce voisin comme nouveau nœud 
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
    if visited is None :
        visited=[]
    visited.append(start)
    for node in g.graph[start]:
        if node[0] not in visited:
            explore(g, node[0], visited)
    return visited 
    
from graphviz import Graph as GG

def plot_graph(g):
    """
    Retourne le graphe
    """
    l = []
    m = g.nodes
    dot = GG('Graph')
    # Add nodes to the graph
    for n in m:
        dot.node(str(n))
    # Add edges to the graph
    for n in m :
        for neighbor in g.graph[n]:
            if {n,neighbor[0]} not in l :
                 dot.edge(str(n), str(neighbor[0]), xlabel= str(neighbor[2]))
                 l.append({n,neighbor[0]})

    # Render the graph
    dot.render('graph')  
    

def plot_path(g, start, target,power, shortest_path=[]):
    """
    Plot the graph, the path to the target node, and the shortest path found by the BFS algorithm.
    """
    if g.get_path_with_power(start,target,power) is None :
        raise ValueError("Trajet impossible")
    shortest_path = g.get_path_with_power(start,target,power)[1][0]
    l=[]
    m = g.nodes
    dot = GG('Graph')
    # Add nodes to the graph
    for n in m:
        dot.node(str(n))
    # Add edges to the graph
    for n in m :
        for neighbor in g.graph[n]:
            if {n,neighbor[0]} not in l :
                 dot.edge(str(n), str(neighbor[0]), xlabel= str(neighbor[2]))
                 l.append({n,neighbor[0]})
    
    # Colore en vert le nœud de départ
    dot.node(str(start), color='green', style='filled')
    
    # Colore en rouge le nœud d'arrivée 
    dot.node(str(target), color='red', style='filled')
    
    #Colore en bleu les nœuds intermédiaires
    for i in range(1,len(shortest_path)-1):
        dot.node(str(shortest_path[i]), color='blue', style='filled')
    """
    # Highlight the shortest path found by the BFS algorithm
    for i in range(len(shortest_path)-1):
        print(type(shortest_path[i]))
        dot.edge(str(shortest_path[i]), str(shortest_path[i+1]), color='green', penwidth=3)"""
    # Render the graph
    dot.render('graph')  
    
    
def trajet(g, src, dest, path=[]):
    """
    Retourne tous les trajets possibles entre les nœuds src et dest en utilisant à peu 
    près le même principe de récursivité que dans le parcours en profondeur du graphe.

    Complexité = O(m*n)
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
    """Cette fonction est spécialement crée pour compléter la fonction trajet
        Il n'y a pas à s'inquéter de l'adjacence des nœuds si l'on suppose que la fonction trajet fait
        bien son boulot. Ainsi les nœuds considérés dans les cas pratiques où nous utiliserons la fonction 
        seront toujours adjacent.

        Complexité = O(1)
        """
    voisins = g.graph[node1]
    c=len(voisins)
    i=0
    while i < c and voisins[i][0] != node2 :
        i+=1
    return voisins[i][1]

def dist(g,node1,node2):
    """Prend en paramètre un graphe et deux nœuds de celui-ci puis retourne la distance entre ceux-ci 
    On utilisera cette fonction dans des cas où nos nœuds sont nécéssairement adjacents donc aucun problème .
    """
    edge = list(filter(lambda x : x[0]==node2 , g.graph[node1]))
    return edge[0][2]

    
    
def dist_min(g, chemins:list):
    """Prend en paramètre une liste donnant les trajets entre deux noeuds d'un graphe g 
    et retourne le trajet de distance minimale sous forme de tuple (chemin,distance).
    On appelle distmin dans get_path_with_power uniquement lorsqu'il y a au moins un chemin possible 
    pour le camion c'est-à-dire lorsque chemins_possibles != []"""

    for i in range(len(chemins)) :
        chemins[i] = (chemins[i], sum([dist(g,chemins[i][j],chemins[i][j+1]) for j in range((len(chemins[i])-1))]))
    
    chemins.sort(key = lambda x : x[1])
    
    return chemins[0]




#Pour utiliser l'algorithme de Kruskal, on crée d'abord des fonctions Union et Find.
#Find prend en argument le tableau des parents des noeuds d'un arbre et un noeud, et renvoie la racine de l'arbre ou est le noeud.
#Union prend en argument ce même tableau et deux noeuds.
#Il trouve d'abord les racines des deux noeuds et si elles sont différentes, c'est que les noeuds sont dans des arbres différents.
#La fonction Union réunit alors les deux arbres, en définissant une des deux racines comme parent de l'autre.

def Union(a,b,P):
    A=Find(P,a)
    B=Find(P,b)
    if A!=B:
        P[A-1]=B

def Find(P,i):
    if P[i-1]==i:
        return i
    return Find(P,P[i-1])

def kruskal(g):
    T=[]
    P=[i for i in range(1,len(g.graph)+1)]
    Q=[]
    for i in range(1,len(g.graph)+1):
        for j in range(len(g.graph[i])):
            T.append([i,g.graph[i][j][0],g.graph[i][j][1]])
    T.sort(key=lambda u:u[2])
    
    #T est un tableau représentant le graphe g dans le format [noeud1,noeud2,puissance], trié par puissance des arêtes croissante.
    #P est le tableau des parents des noeuds de P.
    #Q est un tableau qui regroupe les noeuds et arêtes qui conviennent, et sera converti en un graphe G.
    
    c1,c2=0,0
    while c2<len(P)-1:
        
        #On parcours les noeuds de g et lorsque l'on trouve deux noeuds ayant des parents différents, on les rassemble avec Union.
        #Comme T était trié, on traite bien les arêtes de puissance minimale en premier.
        #Lorsque c2=len(P)-1, cela veut dire que Q possède nb_noeuds-1 éléments, et l'arbre peut être défini par les nb_noeuds-1 arêtes définies par Q. Donc la boucle while s'arrête.
        
        c1=c1+1
        p=Find(P,T[c1][0]-1)
        q=Find(P,T[c1][1]-1)
        if p!=q:
            c2=c2+1
            Q.append(T[c1])
            Union(p,q,P)
    
    #On peut alors définir le graphe G représentant cet arbre, et on le renvoie.
    
    G=Graph([i for i in range(1,len(Q)+1)])
    for i in range(len(Q)):
        G.add_edge(Q[i][0],Q[i][1],Q[i][2])
    return G

    #L'algorithme de Kruskal est de complexité O(E*ln(E)), qui est la complexité du tri de T.


def min_power_K(g,src,dest):
    
    #Ainsi, on peut redéfinir une fonction min_power_K qui cette fois applique min_power à l'arbre couvrant minimal de g.
    #Sa complexité sera alors O(E*ln(E)) pour la même raison que celle de min_power est O(E*V).

    G=kruskal(g)
    h=G.min_power(src,dest)
    return h


#Pour les tests de temps, on crée d'abord une fonction pour récupérer les trajets des dossiers routes.x.in, puis on crée les fonctions de test

def trajets_from_route(a):
    t=[]
    with open("input/routes."+str(a)+".in", "r") as file:
        n=int(file.readline().split()[0])
        for _ in range(n):
            t.append(list(map(int, file.readline().split())))
    return t


def time_test(N):
    U=[]
    V=[[0] for i in range(10)]
    for k in range(1,11):
        g=graph_from_file("input/network."+str(k)+".in")
        h=trajets_from_route(k)
        U.append(h[0])
        for _ in range(N):
            w=numpy.random.randint(0,h[0]-1)
            src=h[1][w][0]
            dest=h[1][w][1]
            t1=time.perf_counter()
            t=g.min_power(src,dest)
            t2=time.perf_counter()
            V[k].append(t2-t1)
    return [U[i]*sum(V[i])/N for i in range(10)]

def time_test_K(N):
    U=[]
    V=[[0] for i in range(10)]
    for k in range(1,11):
        g=graph_from_file("input/network."+str(k)+".in")
        h=trajets_from_route(k)
        U.append(h[0])
        for _ in range(N):
            w=numpy.random.randint(0,h[0]-1)
            src=h[1][w][0]
            dest=h[1][w][1]
            t1=time.perf_counter()
            t=min_power_K(g,src,dest)
            t2=time.perf_counter()
            V[k].append(t2-t1)
    return [U[i]*sum(V[i])/N for i in range(10)]

#pour routes.1.in, on trouve un temps moyen de 0.55s pour min_power et 0.04s pour min_power_K.


def list_paths(filename):
    """Allow us to obtain a list of a list from a routes.x.in file, containing all the paths with their utility
    """
    t=[]
    with open(filename, "r") as file:
        n = int(file.readline())
        for _ in range(n):
            t.append(list(map(int, file.readline().split())))
               
    return t

def routes_out(filename,graphname) : 
    t=[]
    l=list_paths(filename)
    g=graph_from_file(graphname)
    for i in range(len(l)) :
        t.append(min_power_K(g, l[i][0], l[i][1]))

    return t