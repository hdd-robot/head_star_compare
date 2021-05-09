/*
 * AStar.h
 *
 *  Created on: 21 déc. 2020
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


#ifndef ASTAR_H_
#define ASTAR_H_


namespace nav_platform {

class AStar {
public:
	AStar();
	virtual ~AStar();


	static void init(pair<int,int>, pair<int,int>, list<pair<int,int>>&, list<pair<int,int>>&, Grid &);
	static int exapend_a_star(pair<int,int>, pair<int,int>, Grid &, int &);
	static void get_path(pair<int,int>, pair<int,int>, Grid &,list<pair<int,int>>&, int &);

	static pair<int, int> remove_lowest_open_list(list<pair<int, int>>&,Grid& );


};

} /* namespace nav_platform */

#endif /* ASTAR_H_ */
