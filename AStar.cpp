/*
 * AStar.cpp
 *
 *  Created on: 21 déc. 2020
 *      Author: hdd
 */

#include "AStar.h"

namespace nav_platform {

AStar::AStar() {
	// TODO Auto-generated constructor stub

}

AStar::~AStar() {
	// TODO Auto-generated destructor stub
}


void AStar::init(pair<int, int> start, pair<int, int> goal, list<pair<int, int> > &open_lst, list<pair<int, int> > &close_lst, Grid& grd) {

	for (map<pair<int, int>, t_cell>::iterator it = grd.grd_map.begin(); it != grd.grd_map.end(); ++it)
	{
	  it->second.a_star_g = std::numeric_limits<double>::infinity();
	  it->second.a_star_h = std::numeric_limits<double>::infinity();
	  it->second.a_star_f = std::numeric_limits<double>::infinity();
	  it->second.a_star_pred = make_pair(-1,-1);
	}

	grd.grd_map[start].a_star_g = 0;
	grd.grd_map[start].a_star_h = grd.get_dist(goal, start) ;
	grd.grd_map[start].a_star_f = grd.grd_map[start].a_star_g + grd.grd_map[start].a_star_h;
	grd.grd_map[start].a_star_pred = start;
	open_lst.clear();
	close_lst.clear();
	open_lst.push_back(start);
}


pair<int, int> AStar::remove_lowest_open_list(list<pair<int, int>> &lst_open, Grid& grd) {
	if (lst_open.empty() == true) {
		return make_pair(-1,-1);
	}
	std::list<pair<int,int>>::iterator it, it_tmp;
	it_tmp = lst_open.begin();
	pair<int,int> min_cell = *(it_tmp);
	for (it = lst_open.begin(); it != lst_open.end(); it++) {
		double h_min = grd.grd_map[min_cell].a_star_h;
		double g_min = grd.grd_map[min_cell].a_star_g;
		double h_curr = grd.grd_map[(*it)].a_star_h;
		double g_curr = grd.grd_map[(*it)].a_star_g;

		if( (h_min+g_min) > (h_curr + g_curr)) {
			min_cell = *it;
			it_tmp = it;
		}
	}

	//cout << "nbr" << count(lst_open.begin(), lst_open.end(), (*it_tmp)) << endl;

	lst_open.erase(it_tmp);
	return min_cell;
}


int AStar::exapend_a_star(pair<int, int> start, pair<int, int> goal, Grid& grd, int &nb_node) {
	list<pair<int, int>> open_list;
	list<pair<int, int>> close_list;

	AStar::init(start, goal,open_list,close_list,grd);
	nb_node =1;
	pair<int,int> curent_cell;
	while ( (open_list.empty() != true) && (find(open_list.begin(), open_list.end(),goal) == open_list.end()) ) {
		curent_cell = AStar::remove_lowest_open_list(open_list, grd);
		if (curent_cell == goal) {
			return 1;
		}
		close_list.push_back(curent_cell);

		list<pair<int,int>> lst_neighbors;
		grd.get_neighbors(lst_neighbors, curent_cell,grd);
		for(auto e : lst_neighbors){
			nb_node++;
			if(grd.is_obstacle(e) == true || find(close_list.begin(), close_list.end(),e) != close_list.end()){
				continue;
			}

			list<pair<int, int>>::iterator itr = find(open_list.begin(), open_list.end(),e);
			if (itr != open_list.end()){
				if(grd.grd_map[(*itr)].a_star_h + grd.grd_map[(*itr)].a_star_g  < grd.get_dist(curent_cell, e) ){
					grd.grd_map[(*itr)].a_star_g = grd.get_dist(curent_cell, e);
					grd.grd_map[(*itr)].a_star_h = grd.get_dist(e, goal);
					grd.grd_map[e].a_star_pred = curent_cell;
				}
			}
			else{
				grd.grd_map[e].a_star_g = grd.get_dist(curent_cell, e);
				grd.grd_map[e].a_star_h = grd.get_dist(e, goal);
				grd.grd_map[e].a_star_pred = curent_cell;
				open_list.push_back(e);

			}

		}
	}

	if(find(open_list.begin(), open_list.end(),goal) == open_list.end()){
		return  -1;
	}

	return 1;
}


void AStar::get_path(pair<int, int> start, pair<int, int> goal, Grid& grd, list<pair<int, int>> &path, int &nb_node) {
	pair<int, int> e_path = goal;
	exapend_a_star(start,goal,grd,nb_node);
	while (grd.grd_map[e_path].a_star_pred != e_path){
		path.push_back(e_path);
		e_path = grd.grd_map[e_path].a_star_pred;
		grd.set_flag(e_path, ROBOT);
	}
	path.push_back(e_path);

}


} /* namespace nav_platform */
