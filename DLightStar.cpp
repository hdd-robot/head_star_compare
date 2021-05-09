/*
 * DLightStar.cpp
 *
 *  Created on: 19 déc. 2020
 *      Author: hdd
 */

#include "DLightStar.h"

namespace nav_platform {

bool comparison(pair<pair<int,int>,pair<double,double>> x1, pair<pair<int,int>,pair<double,double>> x2) {
  return (x1.second.first <= x2.second.first) && (x1.second.second <= x2.second.second);
}


D_Light_Star::D_Light_Star() {
	// TODO Auto-generated constructor stub

}

D_Light_Star::~D_Light_Star() {
	// TODO Auto-generated destructor stub
}

pair<double, double> D_Light_Star::TopKey(op_lst &ls_open){
	if(ls_open.empty()){
		return make_pair(numeric_limits<double>::infinity(),numeric_limits<double>::infinity());
	}
	ls_open.sort(comparison);
	ls_open.sort(comparison);
	return ls_open.front().second;
}

pair<int, int> D_Light_Star::Pop(op_lst &ls_open) {
	if(ls_open.empty()){
		return make_pair(-1,-1);
	}
	ls_open.sort(comparison);
	ls_open.sort(comparison);
	pair<double,double> res = ls_open.front().first;
	ls_open.pop_front();
	return res;
}

bool D_Light_Star::findElem(op_lst &ls_open, pair<int,int> elem){
	for(auto e : ls_open){
		if(e.first == elem){
			return true;
		}
	}
	return false;
}

bool D_Light_Star::removeElem(op_lst &ls_open, pair<int,int> elem){
	op_lst::iterator it;
	for(it =ls_open.begin(); it != ls_open.end(); it++){
		if((*it).first == elem){
			ls_open.erase(it);
			return true;
		}
	}
	return false;
}



pair<double,double> D_Light_Star::calculateKey(pair<int, int> s, pair<int, int> start_pos, pair<int, int> goal_pos, Grid &grd){
	pair<double,double> key;

	if(s == make_pair(-1,-1)){
		return make_pair(numeric_limits<double>::infinity(),numeric_limits<double>::infinity());
	}

	key.first  = fmin(grd.grd_map[s].d_lite_g, grd.grd_map[s].d_lite_rhs) + Grid::get_dist(start_pos,s);
	key.second = fmin(grd.grd_map[s].d_lite_g, grd.grd_map[s].d_lite_rhs);
	return key;
}

void D_Light_Star::initialize(Grid& grd, op_lst &op_list, pair<int, int> start_pos, pair<int, int> goal_pos) {
	op_list.clear();
	for (auto& e : grd.grd_map){
		e.second.d_lite_g   = numeric_limits<double>::infinity() ;
		e.second.d_lite_rhs = numeric_limits<double>::infinity();
		e.second.d_lite_succ = make_pair(-1,-1);
	}
	grd.grd_map[goal_pos].d_lite_rhs = 0;
	op_list.push_back(make_pair(goal_pos,calculateKey(goal_pos,start_pos, goal_pos,grd)));
	//prit_debug(grd);
}

void D_Light_Star::UpdateVertex(pair<int, int> u, pair<int, int> start_pos, pair<int, int> goal_pos, Grid &grd, op_lst &op_list) {
	if (u != goal_pos){
		double min = Grid::get_dist(u, grd.grd_map[u].d_lite_succ) +  grd.grd_map[grd.grd_map[u].d_lite_succ].d_lite_g;
		min = floor(min * 10.0)/10.0;
		grd.grd_map[u].d_lite_rhs = min;
	}
	if(findElem(op_list, u)){
		removeElem(op_list,u);
	}

	if(grd.grd_map[u].d_lite_g != grd.grd_map[u].d_lite_rhs){
		op_list.push_back(make_pair(u,calculateKey(u,start_pos, goal_pos,grd)));
	}

}

void D_Light_Star::computeShortestPath(pair<int, int> start_pos, pair<int, int> goal_pos, Grid &grd, op_lst &op_list, int& nb_node) {

	while((TopKey(op_list) < calculateKey(start_pos,start_pos, goal_pos,grd) ) || ( grd.grd_map[start_pos].d_lite_rhs  != grd.grd_map[start_pos].d_lite_g )){

		nb_node++;
		pair<double,double> k_old = TopKey(op_list);
		pair<int,int> u  = Pop(op_list);


		//cout << k_old.first << " "<< k_old.second << " - " <<  calculateKey(u,start_pos, goal_pos,grd).first << " " <<  calculateKey(u,start_pos, goal_pos,grd).second << endl;

		if(k_old < calculateKey(u,start_pos, goal_pos,grd)){
			op_list.push_back(make_pair(u,calculateKey(u,start_pos, goal_pos,grd)));

		}
		else if(grd.grd_map[u].d_lite_g > grd.grd_map[u].d_lite_rhs){
			grd.grd_map[u].d_lite_g = grd.grd_map[u].d_lite_rhs ;
			list<pair<int,int>> lst_pred;
			get_neighbors_dlite(lst_pred,u,grd);
			for(auto e : lst_pred){
				if (grd.grd_map[e].type == OBSTABLE_TMP){ continue; }
				grd.grd_map[e].d_lite_succ = u;
				UpdateVertex(e,start_pos,goal_pos,grd,op_list);

			}
		}
		else{
			grd.grd_map[u].d_lite_g = numeric_limits<double>::infinity();
			list<pair<int,int>> lst_pred;
			get_neighbors_dlite(lst_pred,u,grd);
	//		lst_pred.push_back(u);
			for(auto e : lst_pred){
				if (grd.grd_map[e].type == OBSTABLE_TMP){ continue; }
				grd.grd_map[e].d_lite_succ = u;
				UpdateVertex(e,start_pos,goal_pos,grd,op_list);
			}
		}


	}
//	prit_open_list(op_list);
//	prit_debug(grd);
}


int D_Light_Star::find_path(pair<int, int> start_pos, pair<int, int> goal_pos, Grid &grd, list<pair<int, int>> &path, int& nb_node) {
	op_lst open_list;
	pair<int, int> s_start = start_pos;
	pair<int, int> s_next = start_pos;
	initialize(grd, open_list, start_pos, goal_pos);
	nb_node++;
	computeShortestPath(start_pos,goal_pos,grd,open_list,nb_node);



//	grd.set_obstacle(make_pair(2,1));
//	grd.set_obstacle(make_pair(1,1));
	path.push_back(s_start);
	while(s_start != goal_pos){
		if(path.back() != s_start){
			path.push_back(s_start);
			grd.grd_map[s_start].flag = ROBOT;
		}

		if(grd.grd_map[s_start].d_lite_g == numeric_limits<double>::infinity()){
			cout << "no path" << endl;
			return (-1);
		}

		// get min neighbors
		s_next = get_min_neighbors_dlite(s_start, grd);
		if(grd.grd_map[s_next].type != OBSTACLE){
			s_start = s_next;
		}
		else{
			grd.grd_map[s_next].d_lite_g  = numeric_limits<double>::infinity();
			grd.grd_map[s_next].d_lite_rhs  = numeric_limits<double>::infinity();

			list<pair<int,int>> lst_nbr;
			Grid::get_neighbors(lst_nbr, s_next,grd);
			for (auto &e : lst_nbr ){
				if(grd.grd_map[e].d_lite_succ == s_next){
					grd.grd_map[e].d_lite_succ = make_pair(-1,-1);
					grd.grd_map[e].d_lite_g  = numeric_limits<double>::infinity();
					grd.grd_map[e].d_lite_rhs  = numeric_limits<double>::infinity();

					//update_adge
					list<pair<int,int>> lst_nbr_nbr;
					Grid::get_neighbors(lst_nbr_nbr, e,grd);

					pair<int,int> new_arrw ;
					if (lst_nbr_nbr.empty()) {
						new_arrw =  make_pair(-1, -1);
					}
					new_arrw = lst_nbr_nbr.front();
					for (auto ee : lst_nbr_nbr) {
						double dst1 = grd.grd_map[new_arrw].d_lite_g;
						double dst2 = grd.grd_map[ee].d_lite_g ;
						if (dst1 > dst2) {
							new_arrw = ee;
						}
					}

					grd.grd_map[e].d_lite_succ = new_arrw;
					grd.grd_map[e].d_lite_g  = grd.grd_map[new_arrw].d_lite_g + Grid::get_dist(grd.grd_map[e].d_lite_succ ,new_arrw);
					grd.grd_map[e].d_lite_rhs  = grd.grd_map[new_arrw].d_lite_rhs + Grid::get_dist(grd.grd_map[e].d_lite_succ ,new_arrw);


					//UpdateVertex(e,start_pos, goal_pos,grd, open_list);
				}
			}
			//computeShortestPath(start_pos,goal_pos,grd,open_list);
		}

	}
	path.push_back(goal_pos);
	return (0);
}


void D_Light_Star::prit_debug(Grid& grd) {
	int tx = grd.tx;
	int ty = grd.ty;
	for(int i=0 ; i < ty ; i++){
		cout << "\t" ;
		for(int j=0 ; j < tx ; j++){
			cout << j<<i<<"(" << grd.grd_map[make_pair(j,i)].d_lite_g << " , " << grd.grd_map[make_pair(j,i)].d_lite_rhs << ")"<< grd.grd_map[make_pair(j,i)].d_lite_succ.first << grd.grd_map[make_pair(j,i)].d_lite_succ.second <<"\t";

		}
		cout << endl;
	}

	cout << "---"  << endl;
}

void D_Light_Star::prit_open_list(op_lst &op_list) {
	cout << "OpenList: "  << endl;
	for(auto a:op_list){
		cout << "(" << a.first.first << ","<< a.first.second << ") " <<  a.second.first << " , " << a.second.second ;
	}
	cout << endl;
}

pair<int, int> D_Light_Star::get_min_neighbors_dlite( pair<int, int> pos, Grid& grd) {
	// get min neighbors
	pair<int, int> min;

	list<pair<int,int>> lst_neighb;
	Grid::get_neighbors(lst_neighb, pos, grd);
	if(lst_neighb.empty()){
		return make_pair(-1,-1);
	}
	min = lst_neighb.front();
	for(auto e : lst_neighb){
		double dst1 = grd.grd_map[min].d_lite_g + Grid::get_dist(pos,min);
		double dst2 = grd.grd_map[e].d_lite_g + Grid::get_dist(pos,e);
		if( dst1 > dst2) {
			min = e;
		}
	}
	return min;
}


void D_Light_Star::get_neighbors_dlite(list<pair<int, int>>&lst, pair<int, int> pos, Grid& grd) {
	int h = grd.ty - 1;
	int w = grd.tx - 1;
	int x = pos.first;
	int y = pos.second;

	list<pair<int, int>> lst_tmp;

	if((x-1 >=0  and x-1 <= w) and (y-1 >=0  and y-1 <= h)) {lst_tmp.push_back(std::make_pair(x-1,y-1));}
	if((x   >=0  and x   <= w) and (y-1 >=0  and y-1 <= h)) {lst_tmp.push_back(std::make_pair(x  ,y-1));}
	if((x+1 >=0  and x+1 <= w) and (y-1 >=0  and y-1 <= h)) {lst_tmp.push_back(std::make_pair(x+1,y-1));}
	if((x-1 >=0  and x-1 <= w) and (y   >=0  and y   <= h)) {lst_tmp.push_back(std::make_pair(x-1,y  ));}
	if((x+1 >=0  and x+1 <= w) and (y   >=0  and y   <= h)) {lst_tmp.push_back(std::make_pair(x+1,y  ));}
	if((x-1 >=0  and x-1 <= w) and (y+1 >=0  and y+1 <= h)) {lst_tmp.push_back(std::make_pair(x-1,y+1));}
	if((x   >=0  and x   <= w) and (y+1 >=0  and y+1 <= h)) {lst_tmp.push_back(std::make_pair(x  ,y+1));}
	if((x+1 >=0  and x+1 <= w) and (y+1 >=0  and y+1 <= h)) {lst_tmp.push_back(std::make_pair(x+1,y+1));}

	for(auto e  : lst_tmp){
		if(grd.grd_map[e].d_lite_g != numeric_limits<double>::infinity() || grd.grd_map[e].d_lite_rhs != numeric_limits<double>::infinity()){
			continue;
		}
		lst.push_back(e);
	}
}

} /* namespace mpal */
