from graph import graph_from_file , plot_path

data_path = "input/"
file_name = "network.00.in"

g = graph_from_file(data_path + file_name)
shortest_path = g.min_power(1, 4)[0]

print(plot_path(g, 1, 4, shortest_path))



