from graph import graph_from_file , plot_path
from time import perf_counter

data_path = "input/"
file_name = "network.1.in"

g = graph_from_file(data_path + file_name)

d = time.perf_counter()

a = g.min_powerr(1,4)

f = time.perf_counter()

print(a,f-d)
