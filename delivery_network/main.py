from graph import *


data_path = "input/"
file_name = "network.01.in"

g = graph_from_file(data_path + file_name)
print(explore(g,4))
