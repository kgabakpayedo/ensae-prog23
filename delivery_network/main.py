from graph import *

data_path = "input/"
file_name = "network.00.in"

g = graph_from_file(data_path + file_name)
print(g.get_path_with_power(1, 4, 8))
