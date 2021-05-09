/*
 * HeadStar.h
 *
 *  Created on: 03 dec. 2020
 *      Author: ros
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



#ifndef HEADSTAR_H_
#define HEADSTAR_H_



namespace nav_platform {

using namespace std;


class HeadStar {
private:
	static void add_to_open_list (pair<int,int>, list<pair<int,int>>&, list<pair<int,int>>&, list<pair<int,int>>&, Grid&);
	static void add_to_inconsist_list(pair<int,int>, list<pair<int,int>>&, list<pair<int,int>>&, list<pair<int,int>>&);
	static void add_to_close_list (pair<int,int>, list<pair<int,int>>&, list<pair<int,int>>&, list<pair<int,int>>&);
	static void remove_in_open_lists_and_inconsist_list(pair<int,int>, list<pair<int,int>>&, list<pair<int,int>>&);
	static pair<int,int> find_min_cell(pair<int, int> , pair<int, int>, list<pair<int,int>>&, list<pair<int,int>>&, Grid &);



public:

	static int find_path(pair<int,int> start_pos, pair<int,int> goal_pos, Grid &grd, list<pair<int,int>>&, int& nb_node);
	static bool get_list_inconsistant(pair<int,int>, pair<int,int>, list<pair<int, int>>&, Grid &);



};

} /* namespace nav_platform */

#endif /* HEADSTAR_H_ */





