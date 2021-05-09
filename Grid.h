/*
 * grid.h
 *
 *  Created on: 3 déc. 2020
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

#ifndef GRID_H_
#define GRID_H_

namespace nav_platform {

using namespace std;
enum type_cell {
	FREE, OBSTACLE, OBSTABLE_TMP
};

enum cell_flags {
	NONE, ROBOT, START, GOAL, PATH
};

class t_cell {
public:
	float cout_h;
	type_cell type;
	cell_flags flag;

	double a_star_g;
	double a_star_h;
	double a_star_f;
	pair<int,int> a_star_pred;

	double d_lite_g;
	double d_lite_rhs;
	pair<int,int> d_lite_succ;

	t_cell() {
		cout_h = 0;
		type = FREE;
		flag = NONE;
	}
	virtual ~t_cell(){}
};

typedef map<pair<int, int>, t_cell> t_grid;

class Grid {
public:
	int tx, ty;
	int goal_x, goal_y;
	int start_x, start_y;

	t_grid grd_map;

	void set_start(pair<int, int> pos);
	void set_goal(pair<int, int> pos);

	void set_obstacle(pair<int, int> pos);
	bool is_obstacle(pair<int, int> pos);

	void set_flag(pair<int, int> pos, cell_flags);

	float get_h(pair<int, int>, pair<int, int>);

	Grid(int, int);
	void print_grid();
	void gen_map(int);

	static double get_dist(pair<int,int>, pair<int,int>);
	static pair<int,int> get_min_h(list<pair<int, int>>&, t_grid&);
	static void get_neighbors(list<pair<int, int>>&, pair<int, int>, Grid&);

	static double get_length_path(list<pair<int, int>>&);

	virtual ~Grid(){};
};

} /* namespace mpal */

#endif /* GRID_H_ */
