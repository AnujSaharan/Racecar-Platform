/*
 * Key.h
 *
 *  Created on: Feb 12, 2015
 *      Author: florian
 */

#ifndef KEY_H_
#define KEY_H_

#include "Params.h"
#include <array>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <initializer_list>
#include <cstdlib>

template<unsigned int DIM>
class Key : public std::array<int, DIM> {
public:
	Key(){}
	Key(int n){std::fill(this->begin(),this->end(),n);}
	Key(const Key& s){std::copy(s.begin(),s.end(),this->begin());}
	Key operator + ( const Key& b) const{
		Key c;
		std::transform(this->begin(), this->end(), b.begin(), c.begin(),
				[](int a, int b){return a + b;});
		return c;
	}
	Key operator - ( const Key& b) const{
		Key c;
		std::transform(this->begin(), this->end(), b.begin(), c.begin(),
				[](int a, int b){return a - b;});
		return c;
	}
	Key operator << (const int s) const{
		Key c;
		std::transform(this->begin(), this->end(), c.begin(),
				[s](int a){return a<<s;});
		return c;
	}
	Key operator * (const int s) const{
		Key c;
		std::transform(this->begin(), this->end(), c.begin(),
				[s](int a){return a*s;});
		return c;
	}
	float normSq () const{
		return std::inner_product(this->begin(), this->end(), this->begin(), 0);
	}
	float norm () const{
		return sqrt(normSq());
	}
	Key abs () const{
		Key c;
		std::transform(this->begin(), this->end(), c.begin(),
				[](int a){return std::abs(a);});
		return c;
	}
	//print operator
	friend std::ostream& operator<< (std::ostream& stream, const Key& st) {
		stream << "(";
		for(int i=0;i<DIM-1;++i)
					stream << st[i] << ", ";
				stream << st[DIM-1] << ")";
		return stream;
	}
};

#endif /* KEY_H_ */
