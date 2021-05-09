/*
 * HeadStar.cpp
 *
 *  Created on: 12 avr. 2020
 *      Author: ros
 */

#include "HeadStar.h"

namespace nav_platform {

void HeadStar::add_to_open_list(pair<int, int> elem, list<pair<int, int>> &open_list, list<pair<int, int>> &inconsist_list, list<pair<int, int>> &close_list,Grid& grd) {

	if (grd.is_obstacle(elem)){
		return;
	}

	list<pair<int, int>>::iterator it;

	it = find(open_list.begin(), open_list.end(),elem);
	if (it != open_list.end()){ return; }

	it = find(close_list.begin(), close_list.end(),elem);
	if (it != close_list.end()){ return; }

	it = find(inconsist_list.begin(), inconsist_list.end(),elem);
	if (it != inconsist_list.end()){ return; }

	open_list.push_back(elem);
}

void HeadStar::add_to_inconsist_list(pair<int, int> elem, list<pair<int, int> > &open_list, list<pair<int, int> > &inconsist_list, list<pair<int, int> > &close_list) {
	list<pair<int, int>>::iterator it;

	it = find(close_list.begin(), close_list.end(),elem);
	if (it != close_list.end()){ return; }

	it = find(inconsist_list.begin(), inconsist_list.end(),elem);
	if (it != inconsist_list.end()){ return; }

	it = find(open_list.begin(), open_list.end(),elem);
	if (it != open_list.end()){
		open_list.erase(it);
	}
	inconsist_list.push_back(elem);
}

void HeadStar::add_to_close_list(pair<int, int> elem, list<pair<int, int> > &open_list, list<pair<int, int> > &inconsist_list, list<pair<int, int> > &close_list) {
		list<pair<int, int>>::iterator it;
	it = find(close_list.begin(), close_list.end(),elem);
	if (it != close_list.end()){ return; }
	close_list.push_back(elem);

	it = find(inconsist_list.begin(), inconsist_list.end(),elem);
	if (it != inconsist_list.end()){
		inconsist_list.erase(it);
	}

	it = find(open_list.begin(), open_list.end(),elem);
	if (it != open_list.end()){
		open_list.erase(it);
	}
}



void HeadStar::remove_in_open_lists_and_inconsist_list(pair<int, int> elem, list<pair<int, int> > &open_list, list<pair<int, int> > &inconsist_list) {
	list<pair<int, int>>::iterator it;
	it = find(open_list.begin(), open_list.end(),elem);

	if (it != open_list.end()){
		open_list.erase(it);
	}

	if (it != inconsist_list.end()){
		inconsist_list.erase(it);
	}
}

pair<int,int> HeadStar::find_min_cell(pair<int, int> start, pair<int, int> goal,list<pair<int, int>> &open_list, list<pair<int, int>> &inconsist_list, Grid &grd) {
	list<pair<int, int>>::iterator it;
	pair<int,int> min_cell = make_pair(-1,-1);

	if (open_list.size() > 0){
		min_cell = open_list.front();
	}

	for (auto elem: open_list){
		if(grd.get_h(elem,goal) < grd.get_h(min_cell,goal)){
			min_cell = elem;
		}
	}

	if (min_cell != make_pair(-1,-1)){
		return min_cell;
	}

	// dans le cas ou la open_list est vide
	min_cell = make_pair(-1,-1);

	if (inconsist_list.size() > 0){
		min_cell = inconsist_list.front();;
	}

	for (auto elem: inconsist_list){
		if(grd.get_h(elem,goal) < grd.get_h(min_cell,goal)){
			min_cell = elem;
		}
	}

	return min_cell;
}



bool HeadStar::get_list_inconsistant(pair<int, int> start_cell, pair<int, int> goal_cell, list<pair<int, int>> &lst_inconsistant, Grid &grd) {
	//  (-1,-1) ( 0,-1) ( 1,-1)
	//  (-1, 0) ( 0, 0) ( 1, 0)
	//  (-1, 1) ( 0, 1) ( 1, 1)
	list<pair<int, int>> lst_inconsist;
	int dx = goal_cell.first  - start_cell.first ;
	int dy = goal_cell.second - start_cell.second ;

	int new_pos_x = start_cell.first + dx;
	int new_pos_y = start_cell.second + dy;
	if (new_pos_x < 0 || new_pos_x >= grd.tx) { return false; } //déplacement impossible sur x
	if (new_pos_y < 0 || new_pos_y >= grd.ty) { return false; } //déplacement impossible sur y

	if (dx == 0 && dy == -1) {
		lst_inconsist.push_back(make_pair(start_cell.first - 1, start_cell.second  + 0));
		lst_inconsist.push_back(make_pair(start_cell.first + 1, start_cell.second + 0));
	}
	if (dx == 0 && dy == +1) {
		lst_inconsist.push_back(make_pair(start_cell.first - 1, start_cell.second  + 0));
		lst_inconsist.push_back(make_pair(start_cell.first + 1, start_cell.second + 0));
	}
	if (dx == -1 && dy == 0) {
		lst_inconsist.push_back(make_pair(start_cell.first + 0, start_cell.second  - 1));
		lst_inconsist.push_back(make_pair(start_cell.first + 0, start_cell.second + 1));
	}
	if (dx == +1 && dy == 0) {
		lst_inconsist.push_back(make_pair(start_cell.first + 0, start_cell.second  - 1));
		lst_inconsist.push_back(make_pair(start_cell.first + 0, start_cell.second + 1));
	}
	if (dx == -1 && dy == -1) {
		lst_inconsist.push_back(make_pair(start_cell.first - 1, start_cell.second  + 0));
		lst_inconsist.push_back(make_pair(start_cell.first + 0, start_cell.second - 1));
	}
	if (dx == +1 && dy == +1) {
		lst_inconsist.push_back(make_pair(start_cell.first + 1, start_cell.second  + 0));
		lst_inconsist.push_back(make_pair(start_cell.first + 0, start_cell.second + 1));
	}
	if (dx == +1 && dy == -1) {
		lst_inconsist.push_back(make_pair(start_cell.first + 0, start_cell.second  - 1));
		lst_inconsist.push_back(make_pair(start_cell.first + 1, start_cell.second + 0));
	}
	if (dx == -1 && dy == +1) {
		lst_inconsist.push_back(make_pair(start_cell.first - 1, start_cell.second  + 0));
		lst_inconsist.push_back(make_pair(start_cell.first + 0, start_cell.second + 1));
	}

	// vérifier si les point inconsistants sont dans les cases et ne sont pas des obstacles
	for (auto e : lst_inconsist){
		if (e.first  < 0 || e.first  >= grd.tx) { continue; }
		if (e.second < 0 || e.second >= grd.ty) { continue; }
		if(grd.is_obstacle(e) == true)          { continue; }
		lst_inconsistant.push_back(e);
	}
	return true;
}





int HeadStar::find_path(pair<int, int> start_pos, pair<int, int> goal_pos, Grid &grd, list<pair<int, int>> &path, int& nb_node) {
	list<pair<int, int>> lst_open;
	list<pair<int, int>> lst_incost;
	list<pair<int, int>> lst_close;

	pair<int, int> robot_pos = start_pos;
	lst_open.push_back(make_pair(robot_pos.first, robot_pos.second));
	nb_node ++;

	//grd.grd_map[robot_pos].cout_h = Grid::get_dist(robot_pos, goal_pos);

	while (lst_open.size() > 0   || lst_incost.size() > 0) {
		add_to_close_list(robot_pos, lst_open, lst_incost,lst_close); // remove from all lists and put in close list
		path.push_back(robot_pos); // just draw the path

		grd.set_flag(robot_pos, ROBOT);

		if(robot_pos == goal_pos){
			return (1);
		}


		// récupérer la lste des voisins et les rajouter daans la open_list
		list<pair<int, int>> list_neighbors;
		Grid::get_neighbors(list_neighbors, robot_pos, grd);

		for (auto e : list_neighbors) {
			if(e == goal_pos){
				path.push_back(goal_pos);
				return (1);
			}
			add_to_open_list(e, lst_open, lst_incost, lst_close,grd);
			nb_node ++;
		}

		pair<int,int> next_robot_pos  = find_min_cell(robot_pos, goal_pos, lst_open, lst_incost, grd);
		if(next_robot_pos == make_pair(-1,-1)){
			// pas de chemin
			return (-1);
		}
		float dst = Grid::get_dist(robot_pos, next_robot_pos);
		if(dst < 1.5){
			list<pair<int,int>> lst_inconsist_tmp;
			get_list_inconsistant(robot_pos, next_robot_pos, lst_inconsist_tmp, grd);

			for(auto e : lst_inconsist_tmp){
					add_to_inconsist_list(e,lst_open,lst_incost,lst_close);
			}
			robot_pos = next_robot_pos;
			grd.set_flag(robot_pos, PATH);
		}
		else{
			if (find_path(robot_pos,next_robot_pos,grd,path,nb_node) == 1){
				robot_pos = next_robot_pos;
			}

		}
	}

	return 0;
}


} /* namespace nav_platform */

