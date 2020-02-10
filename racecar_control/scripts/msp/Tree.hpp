#include <iostream>     // std::cout

template <unsigned int DIM> Tree<DIM>::Tree() {
	root_=new Node<DIM>(0.0f,0);
	maxDepth_=0;
	directions_[0]=Key<DIM>(1);
	int index=1;
	int pindex=1;
	std::array<Key<DIM>,TwoPow<DIM>::value> temp;
	temp[0]=Key<DIM>(1);
	for(int i=0;i<DIM;++i){
		for(int j=0;j<pindex;++j){
			Key<DIM> k(temp[j]);
			k[i]=-k[i];
			temp[index]=k;
			directions_[unitKeyHash(k)]=k;
			++index;
		}
		pindex=index;
	}
	/*std::cout << "printing directions" << std::endl;
	for(int i=0;i<TwoPow<DIM>::value;++i){
		std::cout << temp[i] << " with hash " << keyHash(temp[i]) << std::endl;
	}
	std::cout << "end of directions" << std::endl;*/
}

template <unsigned int DIM> Tree<DIM>::~Tree() {
	delete root_;
}

template <unsigned int DIM> void Tree<DIM>::clear(){
	delete root_;
	root_=new Node<DIM>(0.0f,0);
}

template <unsigned int DIM> void Tree<DIM>::copyParams(Tree<DIM>* t){
	setMaxDepth(t->maxDepth_);
	stateMin_=t->stateMin_;
	stateInc_=t->stateInc_;
}


template <unsigned int DIM> void Tree<DIM>::setMaxDepth(int depth){
	maxDepth_=depth;
	int c=1<<depth;
	rootKey_=Key<DIM>(c);
}

template <unsigned int DIM> void Tree<DIM>::addObstacle(State<DIM>& s){
	Key<DIM> k;
	if(getKey(s,k)){
		addObstacle(k);
	}
}

template <unsigned int DIM> int	Tree<DIM>::unitKeyHash(const Key<DIM> k){
	int hash=0;
	for(int i=0;i<DIM;++i){
		if(k[i]<0)
			hash+=1;
		hash=hash<<1;
	}
	hash=hash>>1;
	return hash;
}

template <unsigned int DIM> void Tree<DIM>::addObstacle(Key<DIM>& obs){
	//	std::cout << "Adding obstacle at " << obs << std::endl;
	Node<DIM>* node=root_;
	Key<DIM> k(rootKey_);
	int height=maxDepth_-1;
	int size=k[0]>>1;
	Key<DIM> k2;
	while(height>=0){
		k2=obs-k;
		if(std::any_of(k2.begin(),k2.end(),[](int ki){return ki==0;})){
			break;
		}
		int childIndex=unitKeyHash(k2);
		/*
		//		std::cout << "Key " << k << " , height " << height << " , size " << size << std::endl;
		int i=0;
		for(;i<TwoPow<DIM>::value;++i){
			k2=k+(directions_[i]<<height)-obs;
			//			std::cout << "k2 " << k2 << std::endl;
			if(std::all_of(k2.begin(),k2.end(),[size](int ki){return ki>-size && ki<size;})){
				break;
			}
		}
		if(i==TwoPow<DIM>::value){
			std::cout << obs << " , " << k2 << " , " << size << " , " << height << std::endl;
			std::cout << "can only add obstacles at max depth for now" << std::endl;
			return;
		}
		*/
		k=k+(directions_[childIndex]<<height);
		node=node->getChild(childIndex);
		height--;
		size=size>>1;
	}
	node->setValue(1.0f);
}

template <unsigned int DIM> Node<DIM>* Tree<DIM>::getNode(const Key<DIM>& kt){
	Node<DIM>* node=root_;
	Key<DIM> k(rootKey_);
	int height=maxDepth_-1;
	int size=k[0]>>1;
	Key<DIM> k2;
	while(height>=0 && !node->isLeaf() && k!=kt){
		k2=kt-k;
		int childIndex=unitKeyHash(k2);
		/*int i=0;
		for(;i<TwoPow<DIM>::value;++i){
			k2=k+(directions_[i]<<height)-kt;
			if(std::all_of(k2.begin(),k2.end(),[size](int ki){return ki>-size && ki<size;})){
				break;
			}
		}*/
		k=k+(directions_[childIndex]<<height);
		node=node->getChild(childIndex);
		height--;
		size=size>>1;
	}
	return node;
}

template <unsigned int DIM> void Tree<DIM>::setStateBounds(const State<DIM>& minState,const State<DIM>& maxState){
	stateMin_=minState;
	stateInc_=maxState-minState;
}

template <unsigned int DIM> int Tree<DIM>::getVolume(int depth){
	return 1<<(DIM*(maxDepth_-depth));
}

template <unsigned int DIM> int Tree<DIM>::getSize(int depth){
	return 1<<(maxDepth_-depth);
}

template <unsigned int DIM> int Tree<DIM>::getSize(Key<DIM> k){
	return k[0]&-k[0];
}

template <unsigned int DIM> bool Tree<DIM>::getKey(const Key<DIM>& kt,Key<DIM>& k, bool inTree){
	k=rootKey_;
	Node<DIM>* node=root_;
	int height=maxDepth_-1;
	int size=k[0]>>1;
	Key<DIM> k2;
	while(height>=0 && k!=kt && (!inTree || !node->isLeaf())){
		k2=kt-k;
		if(std::any_of(k2.begin(),k2.end(),[](int ki){return ki==0;})){
			break;
		}
		int childIndex=unitKeyHash(k2);
		/*int i=0;
		std::array<double,TwoPow<DIM>::value> dist;
		for(;i<TwoPow<DIM>::value;++i){
			dist[i]=(getState(k+(directions_[i]<<height))-s).normSq();
		}
		i=std::distance(dist.begin(),std::min_element(dist.begin(),dist.end()));*/
		k=k+(directions_[childIndex]<<height);
		if(inTree){
			node=node->getChild(childIndex);
		}
		height--;
		size=size>>1;
	}
	return true;
}

template <unsigned int DIM> bool Tree<DIM>::getKey(const State<DIM>& s,Key<DIM>& k, bool inTree){
	if(s<stateMin_){
		return false;
	}
	State<DIM> ts=s-stateMin_;
	if(ts>stateInc_){
		return false;
	}
	Key<DIM> ktemp;
	std::transform(ts.begin(),ts.end(),stateInc_.begin(),ktemp.begin(),
			[this](float si,float sInci){double val=si/sInci*(1<<(maxDepth_+1));
											int intval=(int)std::lround(val);
											if(intval%2==0){
												if(val<intval){
													return intval-1;
												}else{
													return intval+1;
												}
											}return intval;});
//	std::cout << stateMin_ << " , " << stateInc_ << std::endl;
//	std::cout << ts << " , " << ktemp << std::endl;
	return getKey(ktemp,k,inTree);
	/*k=rootKey_;
	Node<DIM>* node=root_;
	int height=maxDepth_-1;
	int size=k[0]>>1;
	while(height>=0 && getState(k)!=s && (!inTree || !node->isLeaf())){
		int i=0;
		std::array<double,TwoPow<DIM>::value> dist;
		for(;i<TwoPow<DIM>::value;++i){
			dist[i]=(getState(k+(directions_[i]<<height))-s).normSq();
		}
		i=std::distance(dist.begin(),std::min_element(dist.begin(),dist.end()));
		k=k+(directions_[i]<<height);
		if(inTree){
			node=node->getChild(i);
		}
		height--;
		size=size>>1;
	}
	return true;*/
}

template <unsigned int DIM> State<DIM> Tree<DIM>::getState(const Key<DIM>& k){
	State<DIM> s;
	std::transform(k.begin(),k.end(),stateInc_.begin(),s.begin(),[this](int ki,float sInci){return (ki*sInci/(1<<(maxDepth_+1)));});
	return stateMin_+s;
}

template <unsigned int DIM>  std::forward_list<Key<DIM>> Tree<DIM>::getRayKeys(const Key<DIM>& k1,const Key<DIM>& k2){
	//TODO: DEBUG
	//assumptions: k1 and k2 are valid distinct keys of the tree
	//	std::cout << "getRaysKeys from " << k1 << " to " << k2 << std::endl;
	std::forward_list<Key<DIM>> list;
	Key<DIM> k(k1),lastKey(k1),targetKey;
	if(k[0]%2==0){
		k=k+Key<DIM>(1); // finest resolution nodes are always centered on odd positions
	}
	State<DIM> v1,v2,dir,distToTarget;
	std::transform(k1.cbegin(),k1.cend(),v1.begin(),[](const int ki){return float(ki);});
	std::transform(k2.cbegin(),k2.cend(),v2.begin(),[](const int ki){return float(ki);});
	dir=v2-v1;
	Key<DIM> inc;
	std::transform(dir.begin(),dir.end(),inc.begin(),[](const int diri){if(diri>0){
		return 2;
	}else{
		if(diri<0){
			return -2;
		}else{return 0;}}});
	State<DIM> alpha;
	int i;
	//	int count=0;
	//	while(count<10){
		while(lastKey!=k2){
			targetKey=k+inc;
			//		std::cout << "target " << targetKey << std::endl;
			std::transform(targetKey.begin(),targetKey.end(),v1.begin(),distToTarget.begin(),[](int ki, float v1i){return ki-v1i;});
			//		std::cout << "distance to target " << distToTarget << std::endl;
			std::transform(distToTarget.begin(),distToTarget.end(),dir.begin(),alpha.begin(),[](float di,float vi){if(vi==0){return 1.0f;}return di/vi;});
			//		std::cout << "alpha " << alpha << std::endl;
			float alphaMin =*std::min_element(alpha.begin(),alpha.end());
			auto itb=alpha.begin();
			while(true){
				itb=std::find(itb,alpha.end(),alphaMin);
				if(itb!=alpha.end()){
					i=std::distance(alpha.begin(),itb);
					//				std::cout << "min at i=" <<i<<std::endl;
					k[i]+=inc[i];
				}else{
					break;
				}
				itb++;
			}
			v1=v1+dir*alphaMin;
			dir=v2-v1;
			getKey(getState(k),lastKey,true);
			if(list.empty() || lastKey!=list.front()){
				list.push_front(lastKey);
			}
			//		std::cout << "lastKey " << lastKey << " from " << k << std::endl;
			//		++count;
		}
		return list;
}
