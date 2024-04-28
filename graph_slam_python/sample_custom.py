from graphslam.graph import Graph

g = Graph.from_g2o("output.g2o")  # https://lucacarlone.mit.edu/datasets/
g.plot()
g.calc_chi2()
g.optimize()
g.plot()