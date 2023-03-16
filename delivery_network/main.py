from graph import *

data_path = "input/"
file_name = "network.1.in"

g = graph_from_file(data_path + file_name)

print(catalog_from_trucks(1))