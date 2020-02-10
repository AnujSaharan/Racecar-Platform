
#ifndef MSP_H_
#define MSP_H_

#include <deque>
#include <map>
#include <vector>
#include <utility>
#include <string>
#include "kshortestpaths/Graph.h"
#include "Tree.h"
#include "State.h"
#include "Key.h"

template <unsigned int DIM> class MSP{
public:
	MSP(Tree<DIM>* tree);
	~MSP();
	void clear();
	bool init(State<DIM> start,State<DIM> end);
	bool step();
	bool run();
	std::deque<State<DIM>> getPath();
	std::deque<State<DIM>> getSmoothedPath();
	double getPathCost(){return std::accumulate(m_path_cost.begin(),m_path_cost.end(),0.0);}
	void setAlpha(double a){m_alpha=a;}
	void setEpsilon(double a){m_epsilon=a;}
	void setSpeedUp(bool a){m_speed_up=a;}
	void setNewNeighboorCheck(bool a){m_newNeighboorCheck=a;}
	void setMapLearning(bool a, int n=0, bool (*isObstacle)(State<DIM> s)=NULL);
	bool isEpsilonObstacle(Node<DIM>* n);

protected:
	void iterationDetails(kshortestpaths::BasePath* result=NULL);
	void drawTree(std::ostream& stream);
	void drawTreeRec(std::ostream& stream, Key<DIM> k, Node<DIM>* n, int size);
	bool inPath(Key<DIM> pt,int size);
	void reducedGraph();
	bool neighboor(std::pair<Key<DIM>,int> &na,std::pair<Key<DIM>,int> &nb);
	bool is_start(std::pair<Key<DIM>,int> &node){return is_in(m_current_coord,node);}
	bool is_goal(std::pair<Key<DIM>,int> &node){return is_in(m_end_coord,node);}
	bool is_in(Key<DIM>& pt,std::pair<Key<DIM>,int> node);
	double cost(Node<DIM>* n);
	void add_node_to_reduced_vertices(Node<DIM>* node,Node<DIM>* nodeReducedTree,Key<DIM> key, int size);
	Key<DIM> m_start_coord;
	Key<DIM> m_end_coord;
	Key<DIM> m_current_coord;
	int m_current_size;
	Tree<DIM>* m_tree;
	long m_start_index;
	int m_nb_backtrack;
	long m_end_index;
	bool m_speed_up;
	bool m_path_found;
	std::deque<Key<DIM>> m_current_path;
	kshortestpaths::Graph m_graph;
	std::map<Key<DIM>,std::set<Key<DIM>>> m_misleading;
	std::set<Key<DIM>> m_current_forbidden;
	double m_alpha;//used in reduced graph as parameter for decomposition
	std::vector<std::pair<Key<DIM>,int> > m_nodes; //coord,size
	std::vector<double> m_cost;
	std::vector<double> m_path_cost;
	double m_epsilon;
	double m_M;
	double m_lambda1;
	double m_lambda2;
	int m_nb_step;

	std::vector<std::vector<Key<DIM>>> m_nodesByDepth;
	std::vector<std::vector<double>> m_costByDepth;
	std::map<long, std::pair<int,int>> m_hashToIndices;
	long hash(Key<DIM> k);
	bool m_newNeighboorCheck;
	Tree<DIM> m_reducedGraphTree;

	bool m_mapLearning;
	bool (*m_isObstacle)(State<DIM> s);
	int m_nbDraw;
};

#include "MSP.hpp"

#endif /* MSP_H_ */
