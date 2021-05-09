/*
 * grid.cpp
 *
 *  Created on: 3 déc. 2020
 *      Author: hdd
 */

#include "Grid.h"

namespace nav_platform {

Grid::Grid(int tx, int ty) {
	this->tx = tx;
	this->ty = ty;
	for(int i=0 ; i < tx ; i++){
		for(int j=0 ; j < ty ; j++){
			t_cell& cell = *(new t_cell());
			grd_map.insert(make_pair(make_pair(i,j), cell));
		}
	}
}


void Grid::set_obstacle(pair<int, int> pos) {
	grd_map[pos].type = OBSTACLE;
}

void Grid::set_flag(pair<int, int> pos, cell_flags flg) {
	grd_map[pos].flag = flg;
}


bool Grid::is_obstacle(pair<int, int> pos) {
	return grd_map[pos].type == OBSTACLE;
}

void Grid::set_start(pair<int, int> pos) {
	start_x = pos.first;
	start_y = pos.second;
	set_flag(pos, START);
}

void Grid::set_goal(pair<int, int> pos) {
	goal_x = pos.first;
	goal_y = pos.second;
	set_flag(pos, GOAL);
}



float Grid::get_h(pair<int, int> pos_start, pair<int, int> pos_goal) {
	if (is_obstacle(pos_start)){
		return numeric_limits<float>::infinity();
	}
	float dist = get_dist(pos_start,pos_goal);
	return dist;
}


void Grid::print_grid() {
	for(int i=0 ; i < ty ; i++){
		cout << "         " ;
		for(int j=0 ; j < tx ; j++){


			if (grd_map[make_pair(j,i)].flag == PATH){
				cout << "X" ; continue;
			}
			if (grd_map[make_pair(j,i)].flag == ROBOT){
				cout << "R" ; continue;
			}
			if (grd_map[make_pair(j,i)].flag == START){
				cout << "S" ; continue;
			}
			if (grd_map[make_pair(j,i)].flag == GOAL){
				cout << "G" ; continue;
			}
			if (grd_map[make_pair(j,i)].type == FREE){
				cout << "." ; continue;
			}
			if (grd_map[make_pair(j,i)].type == OBSTACLE){
				cout << "%" ; continue;
			}
		}
		cout << endl;
	}

	cout << "---"  << endl;
}

void Grid::get_neighbors(list<pair<int, int>>&lst, pair<int, int> pos, Grid& grd) {
	int h = grd.ty - 1;
	int w = grd.tx - 1;
	int x = pos.first;
	int y = pos.second;



	if((x-1 >=0  and x-1 <= w) and (y-1 >=0  and y-1 <= h)) {lst.push_back(std::make_pair(x-1,y-1));}
	if((x   >=0  and x   <= w) and (y-1 >=0  and y-1 <= h)) {lst.push_back(std::make_pair(x  ,y-1));}
	if((x+1 >=0  and x+1 <= w) and (y-1 >=0  and y-1 <= h)) {lst.push_back(std::make_pair(x+1,y-1));}
	if((x-1 >=0  and x-1 <= w) and (y   >=0  and y   <= h)) {lst.push_back(std::make_pair(x-1,y  ));}
	if((x+1 >=0  and x+1 <= w) and (y   >=0  and y   <= h)) {lst.push_back(std::make_pair(x+1,y  ));}
	if((x-1 >=0  and x-1 <= w) and (y+1 >=0  and y+1 <= h)) {lst.push_back(std::make_pair(x-1,y+1));}
	if((x   >=0  and x   <= w) and (y+1 >=0  and y+1 <= h)) {lst.push_back(std::make_pair(x  ,y+1));}
	if((x+1 >=0  and x+1 <= w) and (y+1 >=0  and y+1 <= h)) {lst.push_back(std::make_pair(x+1,y+1));}
}

double Grid::get_dist(pair<int, int> pos1, pair<int, int> pos2) {
	std::complex<double> dif =	 std::complex<double>(pos2.first, pos2.second) -
							 	 std::complex<double>(pos1.first, pos1.second) ;
	return abs(dif);
}

pair<int, int> Grid::get_min_h(list<pair<int, int>> &lst, t_grid& grid) {
	pair<int,int> min;

	if (lst.size() == 0){
		min.first  = -1;
		min.second = -1;
		return min;
	}

	min.first  = lst.front().first;
	min.second = lst.front().second;

	for (auto e : lst ){
		if (grid[make_pair(e.first,e.second)].cout_h < grid[make_pair(min.first, min.second)].cout_h){
			min.first = e.first;
			min.second = e.second;
		}
	}
	return min;
}


void Grid::gen_map(int nbr_obs) {
	while (nbr_obs--){
		int randx = rand() % tx;
		int randy = rand() % ty;
		if((randx == start_x) && (randy == start_y)){ continue; }
		if((randx == goal_x ) && (randy == goal_y )){ continue; }
		set_obstacle(make_pair(randx,randy));
	}

}

double Grid::get_length_path(list<pair<int, int>> &lst_path) {
	if (lst_path.size() <= 1){
		return 0;
	}
	double dst = 0;

	list<pair<int, int>>::iterator it;
	for(it = next(lst_path.begin()) ; it != lst_path.end(); it++){
		dst =  dst + Grid::get_dist(*(prev(it)),(*it));

	}
	return dst;
}

} /* namespace mpal */
