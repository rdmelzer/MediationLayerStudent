all:
	g++-8 -std=c++11 main.cc node.h node.cc directed_edge.h directed_edge.cc undirected_edge.h undirected_edge.h graph.h graph.cc dijkstra.h dijkstra.cc

clean: 
	rm *.gch *.out
