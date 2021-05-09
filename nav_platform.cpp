#include <time.h>
#include "Grid.h"
#include "AStar.h"
#include "HeadStar.h"
#include "DLightStar.h"

using namespace nav_platform;

int main(int argc, char **argv) {
	clock_t t1, t2;
	srand((unsigned) time(0));
	// Create map
	string algo = "";

//	cout << "algo_1 ; size_x_2 ; size_y_3 ; nb_obstacles_4 ; distance_start_goal_5 ; dist_global_6 ; run_time_7 ; nb_node_8 ; explored_cells_9 ; ";
//	cout << "algo_10 ; size_x_11 ; size_y_12 ; nb_obstacles_13 ; distance_start_goal_14 ; dist_global_15 ; run_time_16 ; nb_node_17 ; explored_cells_18 ; ";

	int size_x = 20;
	int size_y = 20;


	pair <int,int> start = make_pair(0,0);
	pair <int,int> goal  = make_pair(19,19);



	int nb_obstcales = 250;

	while (nb_obstcales <= 300){
		cerr << " OBSTACLE NBR  = " << nb_obstcales  << endl;;

		double algo_res_dist = 0;


		double dist_start_goal = Grid::get_dist(start,goal);
		double run_time = 0;
		int nb_node = size_x * size_y;
		int explored_cells = 0;


		srand((unsigned) time(0));

		int nb = 3000;
		while (nb--) {
			cerr << " OBSTACLE NBR  = " << nb_obstcales  << " TRN NBR  = " << nb << endl;
			Grid g = Grid(size_x, size_y);
			g.set_start(start);
			g.set_goal(goal);

			g.gen_map(nb_obstcales);

			//g.print_grid();

			Grid g1 = g;
			Grid g2 = g;

			list<pair<int, int>> lst_path;


			// D* Lite

			algo = "d_star_lite";
			lst_path.clear();
			explored_cells = 0;
			t1 = clock();
			D_Light_Star::find_path(start, goal, g2, lst_path,explored_cells);
			t2 = clock();
			algo_res_dist = Grid::get_length_path(lst_path);
			run_time = (double)(t2-t1)/CLOCKS_PER_SEC;
			cout << algo <<";"<< size_x <<";"<< size_y <<";"<< nb_obstcales <<";"<< dist_start_goal <<";"<< algo_res_dist <<";"<< run_time <<";"<< nb_node <<";"<< explored_cells << endl;

//			for (auto e : lst_path) {
//				cout << "(" << e.first << "," << e.second << ")" << endl;
//			}

			// H*
			algo = "h_star";
			lst_path.clear();
			explored_cells = 0;
			t1 = clock();
			HeadStar::find_path(start, goal, g1, lst_path,explored_cells);
			t2 = clock();
			algo_res_dist = Grid::get_length_path(lst_path);
			run_time = (double)(t2-t1)/CLOCKS_PER_SEC;
			cout << algo <<";"<< size_x <<";"<< size_y <<";"<< nb_obstcales <<";"<< dist_start_goal <<";"<< algo_res_dist <<";"<< run_time <<";"<< nb_node <<";"<< explored_cells<<";";



		}
		nb_obstcales = nb_obstcales + 10;
	}



	return 0;
}

