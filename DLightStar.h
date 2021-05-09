/*
 * DLightStar.h
 *
 *  Created on: 19 déc. 2020
 *      Author: hdd
 */

#include <queue>
#include <vector>
#include <iostream>
#include <list>
#include <utility>
#include <algorithm>
#include <complex.h>
#include <math.h>
#include <map>
#include "Grid.h"


#ifndef DLIGHTSTAR_H_
#define DLIGHTSTAR_H_



using namespace std;

namespace nav_platform {
typedef list<pair<pair<int,int>,pair<double,double>>> op_lst;






class D_Light_Star {
public:
	D_Light_Star();
	virtual ~D_Light_Star();

	static pair<int, int> get_min_neighbors_dlite(pair<int, int>, Grid&);
	static void get_neighbors_dlite(list<pair<int, int>>&, pair<int, int>, Grid&);
	static pair<double,double> TopKey(op_lst&);
	static pair<int,int> Pop(op_lst&);
	static bool findElem(op_lst&, pair<int,int>);
	static bool removeElem(op_lst&, pair<int,int>);

	static pair<double,double> calculateKey(pair<int, int>, pair<int, int>, pair<int, int>, Grid &);
	static void initialize(Grid&, op_lst&,pair<int, int>,pair<int, int>);
	static void UpdateVertex(pair<int,int>,pair<int, int>, pair<int, int>, Grid&,op_lst&);
	static void computeShortestPath(pair<int, int>, pair<int, int>, Grid &, op_lst &, int&);

	static void prit_debug(Grid &);
	static void prit_open_list(op_lst &);

	static int find_path(pair<int,int>, pair<int,int>, Grid &, list<pair<int,int>>&, int&);


};

} /* namespace mpal */

#endif /* DLIGHTSTAR_H_ */
