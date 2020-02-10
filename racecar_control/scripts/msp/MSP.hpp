#include "kshortestpaths/DijkstraShortestPathAlg.h"
#include "kshortestpaths/YenTopKShortestPathsAlg.h"
#include <set>
#include <cmath>
#include <numeric>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <algorithm>


template <unsigned int DIM> MSP<DIM>::MSP(Tree<DIM>* tree):m_tree(tree),m_reducedGraphTree() {
	m_M=100*pow(2,DIM*tree->getMaxDepth());
	m_epsilon=0.5;
	m_current_size=0;
	m_end_index=0;
	m_nb_backtrack=0;
	m_nb_step=0;
	m_start_index=0;
	m_speed_up=false;
	m_path_found=false;
	m_newNeighboorCheck=false;
	m_nbDraw=0;
	m_isObstacle=NULL;
	m_mapLearning=false;
	m_alpha=0.55*sqrt(DIM);
	m_lambda1=0.999;
	m_lambda2=0.001;
	m_reducedGraphTree.copyParams(m_tree);
}

template <unsigned int DIM> MSP<DIM>::~MSP() {
	clear();
}

template <unsigned int DIM> bool MSP<DIM>::isEpsilonObstacle(Node<DIM>* n){
	if(n->getValue()>1-m_epsilon/m_tree->getVolume(n->getDepth()))
		return true;
	return false;
}

template <unsigned int DIM> void MSP<DIM>::setMapLearning(bool a, int n, bool (*isObstacle)(State<DIM> s)){
	m_mapLearning=a;
	if(a){
		srand (time(NULL));
		m_newNeighboorCheck=true;
		m_nbDraw=n;
		m_isObstacle=isObstacle;
	}
}

template <unsigned int DIM> void MSP<DIM>::clear(){
	m_reducedGraphTree.clear();
	m_misleading.clear();
	m_current_path.clear();
	m_path_cost.clear();
	m_cost.clear();
	m_current_forbidden.clear();
	m_nodes.clear();
	m_nodesByDepth.clear();
	m_costByDepth.clear();
	m_hashToIndices.clear();
}


template <unsigned int DIM> bool MSP<DIM>::init(State<DIM> start,State<DIM> end){
	Key<DIM> startKey;
	Key<DIM> goalKey;
	if(!(m_tree->getKey(start,startKey,!m_mapLearning) && m_tree->getKey(end,goalKey,!m_mapLearning))){
		std::cout << "Error converting state to key" << std::endl;
		return false;
	}
	m_current_coord=startKey;
	m_current_size=m_tree->getSize(startKey);
	m_start_coord=startKey;
	m_end_coord=goalKey;
	m_current_path.clear();
	m_path_cost.clear();
	m_current_path.push_back(m_start_coord);
	m_misleading.clear();
	m_misleading[m_current_coord]=std::set<Key<DIM>>();
	m_nb_step=0;
	m_nb_backtrack=0;
	if(m_mapLearning){
		return true;
	}
	Node<DIM>* nstart=m_tree->getNode(startKey);
	Node<DIM>* ngoal=m_tree->getNode(goalKey);
	if(nstart->isLeaf() && ngoal->isLeaf() && !isEpsilonObstacle(nstart) && !isEpsilonObstacle(ngoal)){
		return true;
	}else{
		/*
		std::cout << "start or goal not leaf on free space" << std::endl;
		std::cout << "desired start " << start <<std::endl;
		std::cout << "start " << startKey << " , "
				<< m_tree->getState(startKey) << " , leaf : " <<nstart->isLeaf()
				<< " , epsilon obstacle : " << isEpsilonObstacle(nstart) <<std::endl;
		std::cout << "desired goal " << end <<std::endl;
		std::cout << "goal " << goalKey << " , "
				<< m_tree->getState(goalKey) << " , leaf : " <<ngoal->isLeaf()
				<< " , epsilon obstacle : " << isEpsilonObstacle(ngoal) <<std::endl;//*/
		//exit(1);
	}
	return false;
}

template <unsigned int DIM> bool MSP<DIM>::step(){
	reducedGraph();
	kshortestpaths::YenTopKShortestPathsAlg yenAlg(m_graph, m_graph.get_vertex(m_start_index),m_graph.get_vertex(m_end_index));
	//if solution
	if(yenAlg.has_next()){
		//go forward // if goal return false;
		kshortestpaths::BasePath* result =yenAlg.next();
		iterationDetails(result);
		int next_point_id=result->GetVertex(1)->getID();
		//do stuff to prepare next iteration
		if(m_newNeighboorCheck){
			auto it=m_hashToIndices.find(next_point_id);
			m_misleading[m_current_coord].insert(m_nodesByDepth[it->second.first][it->second.second]);
			m_current_path.push_back(m_nodesByDepth[it->second.first][it->second.second]);
			m_path_cost.push_back(m_costByDepth[it->second.first][it->second.second]);

			if(m_speed_up){
				int mv_fwd=2;
				while(result->length()>mv_fwd){
					int next_point_id2=result->GetVertex(mv_fwd)->getID();
					auto it2=m_hashToIndices.find(next_point_id2);
					if((!m_mapLearning && m_tree->getNode(m_nodesByDepth[it2->second.first][it2->second.second])->isLeaf())
							|| (m_mapLearning && it2->second.first==m_tree->getMaxDepth())){
						m_misleading[m_nodesByDepth[it->second.first][it->second.second]].insert(m_nodesByDepth[it2->second.first][it2->second.second]);
						m_current_path.push_back(m_nodesByDepth[it2->second.first][it2->second.second]);
						m_path_cost.push_back(m_costByDepth[it2->second.first][it2->second.second]);
						next_point_id=next_point_id2;
						it=it2;
						++mv_fwd;
					}else{
						break;
					}
				}
			}

			m_current_coord=m_nodesByDepth[it->second.first][it->second.second];
//			std::cout << "next id " << next_point_id << std::endl;
//			std::cout << "it " << it->first << " , " << it->second.first << " , " << it->second.second << std::endl;
//			std::cout << "new nkipi " << m_current_coord <<std::endl;
			m_current_size=m_tree->getSize(it->second.first);

		}else{
			m_misleading[m_current_coord].insert(m_nodes[next_point_id].first);
			m_current_path.push_back(m_nodes[next_point_id].first);
			m_path_cost.push_back(m_cost[next_point_id]);

			if(m_speed_up){
				int mv_fwd=2;
				while(result->length()>mv_fwd){
					int next_point_id2=result->GetVertex(mv_fwd)->getID();
					if(m_tree->getNode(m_nodes[next_point_id2].first)->isLeaf()){
						m_misleading[m_nodes[next_point_id].first].insert(m_nodes[next_point_id2].first);
						m_current_path.push_back(m_nodes[next_point_id2].first);
						m_path_cost.push_back(m_cost[next_point_id2]);
						next_point_id=next_point_id2;
						++mv_fwd;
					}else{
						break;
					}
				}
			}

			m_current_coord=m_nodes[next_point_id].first;
			m_current_size=m_nodes[next_point_id].second;

		}

		if(next_point_id==m_end_index){
			//std::cout << "goal reached in " << m_nb_step << " iterations" << std::endl;
			m_path_found=true;
			return false;
		}else{
			return true;
		}

	}else{
		iterationDetails();
		m_misleading[m_current_coord].clear();
		m_nb_backtrack++;
		m_current_path.pop_back();
		if(m_current_path.size()==0){
			//no possible path
			return false;
		}else{
			m_current_coord=m_current_path.back();
			m_current_size=m_tree->getSize(m_current_coord);
			m_path_cost.pop_back();
			return true;
		}
	}
}

template <unsigned int DIM> bool MSP<DIM>::run(){
	while(step()){/*std::cout<<*/++m_nb_step;}
	//std::cout<< "NB backtrack : " << m_nb_backtrack << std::endl;
	if(m_path_found){
		return true;
	}else{
		return false;
	}
}

template <unsigned int DIM> std::deque<State<DIM>> MSP<DIM>::getPath(){
	std::deque<State<DIM>> path(m_current_path.size());
	std::transform(m_current_path.begin(),m_current_path.end(),path.begin(),[this](Key<DIM> k){return m_tree->getState(k);});
	return path;
}

template <unsigned int DIM> std::deque<State<DIM>> MSP<DIM>::getSmoothedPath(){
	if(m_mapLearning){
//		std::cout << "banana" << std::endl;
		std::deque<State<DIM>> path=getPath();
		std::deque<State<DIM>> smoothedPath;
		smoothedPath.push_back(path[0]);
		State<DIM> scur=path[0];
		State<DIM> su=m_tree->getState(m_tree->getRootKey()+Key<DIM>(1))-m_tree->getState(m_tree->getRootKey());
		double inc=0.2*su.min();
//		double inc=0.02;
		int i=2;
		while(i<path.size()){
			State<DIM> snext=path[i];
//			std::cout << "scur " <<scur << " snext " << snext << std::endl;
			int jmax=(int)((snext-scur).norm()/inc);
//			std::vector<int> v(jmax+1);
//			std::iota(v.begin(), v.end(), 0);
			bool safe=true;
			for(int j=0;safe && j<=jmax;++j){
//				std::cout << "testing " << scur+(snext-scur)*(j*inc/(snext-scur).norm()) << std::endl;
				safe=!m_isObstacle(scur+(snext-scur)*(j*inc/(snext-scur).norm()));
			}
//			if(std::any_of(v.begin(),v.end(),[this,scur,snext](int j){return m_isObstacle(scur+(snext-scur)*((double)j/(snext-scur).norm()));})){
			if(!safe){
				scur=path[i-1];
				smoothedPath.push_back(scur);
			}
			++i;
		}
		smoothedPath.push_back(path.back());
		return smoothedPath;
	}
	//	m_tree->updateRec();
	std::deque<State<DIM>> sPath;
	sPath.push_back(m_tree->getState(m_current_path[0]));
	Key<DIM> cur=m_current_path[0];
	int i=2;
	while(i<m_current_path.size()){
		Key<DIM> cur2=m_current_path[i];
		auto keys=m_tree->getRayKeys(cur,cur2);
		if(std::any_of(keys.begin(),keys.end(),[this](Key<DIM> k){return this->isEpsilonObstacle(m_tree->getNode(k));})){
			cur=m_current_path[i-1];
			sPath.push_back(m_tree->getState(cur));
		}
		i++;
	}
	sPath.push_back(m_tree->getState(m_current_path[m_current_path.size()-1]));
	return sPath;
}

template <unsigned int DIM> long MSP<DIM>::hash(Key<DIM> k){
	long hash=0;
	for(int i=0;i<DIM;++i){
		hash+=k[i];
		if(i<(DIM-1)){
			hash=hash<<(m_tree->getMaxDepth()+1);
		}
	}
	return hash;
}

template <unsigned int DIM> bool MSP<DIM>::inPath(Key<DIM> pt,int size){
	return std::any_of(m_current_path.begin(),m_current_path.end(),
			[pt,size,this](Key<DIM> it){return this->is_in(it,std::pair<Key<DIM>,int>(pt,size)) && it!=m_current_path.back();});
}

template <unsigned int DIM> void MSP<DIM>::add_node_to_reduced_vertices(Node<DIM>* node,Node<DIM>* nodeReducedTree,Key<DIM> coord, int size){
	//	std::cout << coord << " , " << scale << " , " << node->getValue() << " , " << node->isEpsilonObstacle() << " , " << inPath(coord,scale) << " , " << (m_current_forbidden.find(coord)==m_current_forbidden.end()) << std::endl;
	if( ( ((coord-m_current_coord).normSq()>(m_alpha*(size<<1)+sqrt(DIM)*m_current_size)*(m_alpha*(size<<1)+sqrt(DIM)*m_current_size)) || (!m_mapLearning && node->isLeaf()) || m_tree->getSize(coord)==1)
			&& !inPath(coord,size)
			&& (m_mapLearning || !isEpsilonObstacle(node))
			&& m_current_forbidden.find(coord)==m_current_forbidden.end()
	){
		if(m_newNeighboorCheck){
			nodeReducedTree->clear();
			int i=nodeReducedTree->getDepth();
			int j=m_nodesByDepth[i].size();

			if(m_mapLearning){
				if(!nodeReducedTree->isSampled()){
					State<DIM> base,inc,draw;
					base=m_tree->getState(coord-(*(m_tree->getDirections()))[0]*size);
					inc=m_tree->getState(coord+(*(m_tree->getDirections()))[0]*size)-base;
					int hitCount=0;
					for(int test=0;test<m_nbDraw;test++){
						for(int d=0;d<DIM;++d){
							draw[d]=base[d]+inc[d]*((double) rand() / (RAND_MAX));
						}
						if(m_isObstacle(draw)){
							++hitCount;
						}
					}
					nodeReducedTree->setValue((double)hitCount / m_nbDraw);
					nodeReducedTree->setSampled(true);
				}
				//what if epsilon obstacle
				if(isEpsilonObstacle(nodeReducedTree)){
					return; // that should be enough
				}
				m_costByDepth[i].push_back(cost(nodeReducedTree));
			}else{
				m_costByDepth[i].push_back(cost(node));
			}
			m_nodesByDepth[i].push_back(coord);
			m_hashToIndices.insert(std::pair<long,std::pair<int,int>>(hash(coord),std::pair<int,int>(i,j)));

			m_graph.add_vertex(hash(coord),m_lambda2*(coord-m_end_coord).norm());  //TODO: use square of the cost to remove square roots
			std::pair<Key<DIM>,int> pair(coord,size);
			if(is_start(pair)){
				if(m_start_index!=-1){
					std::cout << "2 start nodes, fail" << std::endl;
					return;
				}
				m_start_index=hash(coord);
			}
			if(is_goal(pair)){
				if(m_end_index!=-1){
					std::cout << "2 end nodes, fail" << std::endl;
					return;
				}
				m_end_index=hash(coord);
			}
		}else{
			m_nodes.push_back(std::pair<Key<DIM>,int>(coord,size));
			m_cost.push_back(cost(node));
		}
	}else{
		int s=size>>1;
		if(m_mapLearning){
			if(!isEpsilonObstacle(nodeReducedTree) && nodeReducedTree->getDepth()<m_tree->getMaxDepth()){
				for(int i=0;i<TwoPow<DIM>::value;++i){
					add_node_to_reduced_vertices(node,nodeReducedTree->getChild(i),coord+(*(m_tree->getDirections()))[i]*s,size>>1);
				}
				nodeReducedTree->update(false,false);
				nodeReducedTree->setSampled(true);
			}
		}else{
			if(!node->isLeaf() && !isEpsilonObstacle(node)){
				for(int i=0;i<TwoPow<DIM>::value;++i){
					if(m_newNeighboorCheck){
						add_node_to_reduced_vertices(node->getChild(i),nodeReducedTree->getChild(i),coord+(*(m_tree->getDirections()))[i]*s,size>>1);
					}else{
						add_node_to_reduced_vertices(node->getChild(i),nodeReducedTree,coord+(*(m_tree->getDirections()))[i]*s,size>>1);
					}
				}
			}
		}
	}
}

template <unsigned int DIM> void MSP<DIM>::reducedGraph(){
	m_graph.clear();
	if(m_newNeighboorCheck){
		m_nodesByDepth.assign(m_tree->getMaxDepth()+1,std::vector<Key<DIM>>());
		m_costByDepth.assign(m_tree->getMaxDepth()+1,std::vector<double>());
		m_hashToIndices.clear();
		//m_reducedGraphTree.clear(); // instead of recreating the tree at each iteration, we remove just the  unnescessary nodes during the reduced graph construction
	}else{
		m_nodes.clear();
		m_cost.clear();
	}
	m_start_index=-1;
	m_end_index=-1;
	try {
		m_current_forbidden=m_misleading.at(m_current_coord);
	}catch (const std::out_of_range& oor) {
		m_current_forbidden=std::set<Key<DIM>>();
	}
	add_node_to_reduced_vertices(m_tree->getRoot(),m_reducedGraphTree.getRoot(),m_tree->getRootKey(),m_tree->getRootKey()[0]);

	if(m_newNeighboorCheck){
		//graph insertion and start end finding in node selection
		//neighboor finding
		Key<DIM> k;
		Key<DIM> kInTree;
		int maxVal=1<<(m_tree->getMaxDepth()+1);
		for(int i=m_tree->getMaxDepth();i>=0;i--){
			int sideLength=m_tree->getSize(i)*2;
			for(int j=0;j<m_nodesByDepth[i].size();++j){
				k=m_nodesByDepth[i][j];
				for(int d=0;d<DIM;++d){
					k[d]+=sideLength;
					//test positif neighboor
					if(k[d]<maxVal){
						m_reducedGraphTree.getKey(k,kInTree,true);
						auto it=m_hashToIndices.find(hash(kInTree));
						if(it!=m_hashToIndices.end()){
							m_graph.add_edge(hash(m_nodesByDepth[i][j]),it->first,m_costByDepth[it->second.first][it->second.second]);
							m_graph.add_edge(it->first,hash(m_nodesByDepth[i][j]),m_costByDepth[i][j]);
						}
					}
					k[d]-=2*sideLength;
					//test negative neighboor
					if(k[d]>0){
						m_reducedGraphTree.getKey(k,kInTree,true);
						auto it=m_hashToIndices.find(hash(kInTree));
						if(it!=m_hashToIndices.end()){
							m_graph.add_edge(hash(m_nodesByDepth[i][j]),it->first,m_costByDepth[it->second.first][it->second.second]);
							m_graph.add_edge(it->first,hash(m_nodesByDepth[i][j]),m_costByDepth[i][j]);
						}
					}
					//set k back to its original value for next iteration
					k[d]+=sideLength;
				}
			}
		}
	}else{
		int l=m_nodes.size();
		for(int i=0;i<l;++i){
			m_graph.add_vertex(i,m_lambda2*(m_nodes[i].first-m_end_coord).norm());  //TODO: use square of the cost to remove square roots
			if(is_start(m_nodes[i])){
				if(m_start_index!=-1){
					std::cout << "2 start nodes, fail" << std::endl;
					return;
					//exit(1);
				}
				m_start_index=i;
			}
			if(is_goal(m_nodes[i])){
				if(m_end_index!=-1){
					std::cout << "2 end nodes, fail" << std::endl;
					return;
					//exit(1);
				}
				m_end_index=i;
			}
		}
		for(int i=0;i<l;++i){
			for(int j=i+1;j<l;++j){
				if(neighboor(m_nodes[i],m_nodes[j])){
					m_graph.add_edge(i,j,m_cost[j]);
					m_graph.add_edge(j,i,m_cost[i]);
				}
			}
		}
	}
	if(m_start_index==-1){
		std::cout << "0 start node, fail" << std::endl;
		return;
	}
	if(m_end_index==-1){
		std::cout << "0 end node, fail" << std::endl;
		return;
	}
}

template <unsigned int DIM> bool MSP<DIM>::is_in(Key<DIM>& pt,std::pair<Key<DIM>,int> node){
	Key<DIM> k=pt-node.first;
	if(std::all_of(k.begin(),k.end(),[&node](int ki){return ki>-node.second && ki<node.second;}))
		return true;
	return false;
}

template <unsigned int DIM> double MSP<DIM>::cost(Node<DIM>* n){
	if (!isEpsilonObstacle(n)){
		return (m_lambda1*n->getValue()+m_lambda2)*m_tree->getVolume(n->getDepth());
	}else{
		return m_M;
	}
}

template <unsigned int DIM> bool MSP<DIM>::neighboor(std::pair<Key<DIM>,int> &na,std::pair<Key<DIM>,int> &nb){
	int l=na.second+nb.second;
	Key<DIM> diff=(na.first-nb.first).abs();
	std::partial_sort(diff.begin(),diff.begin()+2,diff.end(),std::greater<int>());
	if(diff[0]==l && diff[1]!=l)
		return true;
	return false;
}

template <unsigned int DIM> void MSP<DIM>::iterationDetails(kshortestpaths::BasePath* result){
	bool console=false;
	if(console){
		if(m_newNeighboorCheck){
			std::cout << std::endl << std::endl << "Iteration " << m_nb_step << std::endl
					<< "nkipi: " << m_current_coord << " with scale factor " << m_current_size << std::endl;
			std::cout<< "rejects : ";
			for(auto& c : m_current_forbidden)
				std::cout << c << " , ";
			std::cout << std::endl;
			std::cout << "Gi:" <<std::endl;
			//			std::streamsize prev=std::cout.width(0);
			//			std::cout.flags(std::ios_base::right);
			//			std::cout<<*(m_reducedGraphTree.getRoot())<<std::endl;
			//			std::cout.width(prev);
			//*
			for(int i=0;i<m_tree->getMaxDepth()+1;++i){
				std::cout << "depth " << i << std::endl;
				for(int j=0;j<m_nodesByDepth[i].size();++j){
					std::cout << "Vertex " << hash(m_nodesByDepth[i][j]) << " at " << m_nodesByDepth[i][j] << " and cost " << m_costByDepth[i][j] << ", neighbor with ";
					std::set<kshortestpaths::BaseVertex*> vertex_set;
					m_graph.get_adjacent_vertices(m_graph.get_vertex(hash(m_nodesByDepth[i][j])),vertex_set);
					for(auto it:vertex_set){
						std::cout << it->getID() << " , ";
					}
					std::cout << std::endl;
				}
			}//*/
		}else{
			std::cout << std::endl << std::endl << "Iteration " << m_nb_step << std::endl
					<< "nkipi: " << m_current_coord << " with scale factor " << m_current_size << std::endl;
			std::cout<< "rejects : ";
			for(auto& c : m_current_forbidden)
				std::cout << c << " , ";
			std::cout << std::endl;
			std::cout << "Gi:" <<std::endl;
			for(int i=0;i<m_nodes.size();++i){
				std::cout << "Vertex " << i << " at " << m_nodes[i].first << " with size " << m_nodes[i].second << " and cost " << m_cost[i] << ", neighbor with ";
				for(int j=i+1;j<m_nodes.size();++j){
					if(neighboor(m_nodes[i],m_nodes[j])){
						std::cout << j << " , ";
					}
				}
				std::cout << std::endl;
			}
		}
	}
	bool latex=false;
	if(latex && DIM==2){
		if(m_nb_step==0){
			//remove previous results
			std::stringstream ss2;
			ss2 << "rm -r " << RESDIR << "/iterationFiles";
			system(ss2.str().c_str());
			std::stringstream ss3;
			ss3 << "mkdir -p " << RESDIR << "/iterationFiles/";
			system(ss3.str().c_str());
			std::stringstream ss;
			ss << RESDIR << "/iterationFiles/environment.tex";
			std::fstream file(ss.str(),std::fstream::out);
			file << "\\begin{tikzpicture}[scale=\\picScale*32/" << (1<<m_tree->getMaxDepth()+1) << "]" << std::endl
					<< "\\tikzstyle{treenodes}=[black,thick,fill=white]" <<std::endl
					<< "\\tikzstyle{every node}=[circle,draw,minimum size=2pt,inner sep=1pt];" <<std::endl
					<< "\\node[rectangle,draw] at (" << (1<<m_tree->getMaxDepth()) << "," << (1<<m_tree->getMaxDepth()+1)+2 << ") {Environment};" <<std::endl
					<< "\\draw[black,thick,fill=white] (0,0) rectangle (" << (1<<m_tree->getMaxDepth()+1) << "," << (1<<m_tree->getMaxDepth()+1) << ");" <<std::endl;
			drawTree(file);
			file << "\\end{tikzpicture}" << std::endl;
			file.close();
		}
		std::stringstream ss;
		ss << RESDIR << "/iterationFiles/iteration" << m_nb_step << ".tex";
		std::fstream file(ss.str(),std::fstream::out);
		file << "\\begin{tikzpicture}[scale=\\picScale*32/" << (1<<m_tree->getMaxDepth()+1) << "]" << std::endl
				<< "\\tikzstyle{treenodes}=[black,thick,fill=white]" <<std::endl
				<< "\\tikzstyle{every node}=[circle,draw,minimum size=2pt,inner sep=1pt];" <<std::endl
				<< "\\node[rectangle,draw] at (" << (1<<m_tree->getMaxDepth()) << "," << (1<<m_tree->getMaxDepth()+1)+2 << ") {Iteration " << m_nb_step << "};" <<std::endl
				<< "\\draw[black,thick,fill=blue] (0,0) rectangle (" << (1<<m_tree->getMaxDepth()+1) << "," << (1<<m_tree->getMaxDepth()+1) << ");" <<std::endl;
		if(m_newNeighboorCheck){
			for(int i=0;i<m_tree->getMaxDepth()+1;++i){
				int nodeSize=m_tree->getSize(i);
				for(int j=0;j<m_nodesByDepth[i].size();++j){
					if(m_mapLearning){
						file << "\\draw[treenodes, fill=red!" << m_reducedGraphTree.getNode(m_nodesByDepth[i][j])->getValue()*100.0 << "] "
								<< m_nodesByDepth[i][j]-(*(m_tree->getDirections()))[0]*nodeSize
								<< " rectangle "
								<< m_nodesByDepth[i][j]+(*(m_tree->getDirections()))[0]*nodeSize
								<< ";" << std::endl;
					}else{
						file << "\\draw[treenodes, fill=red!" << m_tree->getNode(m_nodesByDepth[i][j])->getValue()*100.0 << "] "
								<< m_nodesByDepth[i][j]-(*(m_tree->getDirections()))[0]*nodeSize
								<< " rectangle "
								<< m_nodesByDepth[i][j]+(*(m_tree->getDirections()))[0]*nodeSize
								<< ";" << std::endl;
					}
				}
			}
			for(int i=0;i<m_tree->getMaxDepth()+1;++i){
				int nodeSize=m_tree->getSize(i);
				for(int j=0;j<m_nodesByDepth[i].size();++j){
					file << "\\node";
					if(hash(m_nodesByDepth[i][j])==m_start_index)
						file << "[green,thick]";
					if(hash(m_nodesByDepth[i][j])==m_end_index)
						file << "[red,thick]";
					file << " at " << m_nodesByDepth[i][j] << " (" << hash(m_nodesByDepth[i][j]) << ") {" << hash(m_nodesByDepth[i][j]) << "};" << std::endl;
				}
			}
			for(int i=0;i<m_tree->getMaxDepth()+1;++i){
				int nodeSize=m_tree->getSize(i);
				for(int j=0;j<m_nodesByDepth[i].size();++j){
					std::set<kshortestpaths::BaseVertex*> vertex_set;
					m_graph.get_adjacent_vertices(m_graph.get_vertex(hash(m_nodesByDepth[i][j])),vertex_set);
					for(auto it:vertex_set){
						file << "\\path[draw] (" << hash(m_nodesByDepth[i][j]) << ") -- (" << it->getID() << ");" << std::endl;
					}
				}
			}
		}else{
			for(int i=0;i<m_nodes.size();++i){
				file << "\\draw[treenodes, fill=red!" << m_tree->getNode(m_nodes[i].first)->getValue()*100.0 << "] "
						<< m_nodes[i].first-(*(m_tree->getDirections()))[0]*m_nodes[i].second
						<< " rectangle "
						<< m_nodes[i].first+(*(m_tree->getDirections()))[0]*m_nodes[i].second
						<< ";" << std::endl;
			}
			for(int i=0;i<m_nodes.size();++i){
				file << "\\node";
				if(i==m_start_index)
					file << "[green,thick]";
				if(i==m_end_index)
					file << "[red,thick]";
				file << " at " << m_nodes[i].first << " (" << i << ") {" << i << "};" << std::endl;
			}
			for(int i=0;i<m_nodes.size();++i){
				for(int j=i+1;j<m_nodes.size();++j){
					if(neighboor(m_nodes[i],m_nodes[j])){
						file << "\\path[draw] (" << i << ") -- (" << j << ");" << std::endl;
					}
				}
			}
		}
		if(m_current_path.size()>1){
			file << "\\path[draw,thick] " << m_current_path.front();
			if(m_current_path.size()>2){
				for(typename std::deque<Key<DIM>>::iterator it=m_current_path.begin()+1,end=m_current_path.end()-1;it!=end;++it){
					file << " -- " << *it ;
				}
			}
			file << "-- (" << m_start_index << ");" << std::endl;
		}
		if(result!=NULL){
			file << "\\path[draw,thick,orange] (" << m_start_index <<")";
			for(int i=1;i<result->length()-1;++i){
				file << " -- (" << result->GetVertex(i)->getID() << ")";
			}
			file << "-- (" << m_end_index << ");" << std::endl;
		}
		file << "\\end{tikzpicture}" << std::endl;
		file.close();
	}
}

template <unsigned int DIM> void MSP<DIM>::drawTreeRec(std::ostream& stream, Key<DIM> k, Node<DIM>* n, int size){
	if(n->isLeaf()){
		if(isEpsilonObstacle(n)){
			stream << "\\draw[treenodes, fill=red] "
					<< k-(*(m_tree->getDirections()))[0]*size
					<< " rectangle "
					<< k+(*(m_tree->getDirections()))[0]*size
					<< ";" << std::endl;
		}
	}else{
		int s=size>>1;
		for(int i=0; i<TwoPow<DIM>::value;++i){
			if(n->childExists(i)){
				drawTreeRec(stream,k+(*(m_tree->getDirections()))[i]*s,n->getChild(i),s);
			}
		}
	}
}

template <unsigned int DIM> void MSP<DIM>::drawTree(std::ostream& stream){
	drawTreeRec(stream, m_tree->getRootKey(), m_tree->getRoot(),m_tree->getRootKey()[0]);
}
