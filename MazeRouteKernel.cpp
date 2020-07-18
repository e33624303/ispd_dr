#define Mix_ILP_Itrack true
#define TrackAssignmentILP true
#define OutputIRouteOnly false

#define IAMIROUTE 9453

#include <limits.h>
#include <tuple>
#include <algorithm>
#include "MazeRouteKernel.h"
#include "iroute_generation.hpp"
#include "small_router.h"
#include "flute-3.1/flute.h"
#include "ilp.hpp"
#include "itrack.hpp"

bool NoTrackAssignment = false;

bool DetermineRaise(int x, int y, pair<Parser::Track, Parser::Track>  &upper_layer_track, bool dir){
	if (dir == false){
		int s_x = upper_layer_track.first.start_index;
		int step = upper_layer_track.first.Step;
		//printf("x : %d sx : %d step : %d\n", x, s_x, step);
		if (x < s_x) return false;
		if (x == s_x) return true;
		if (((x - s_x) % step) == 0 && ((x - s_x) / step < upper_layer_track.first.Do)){
			return true;
		}
		else return false;
	}
	else if (dir){
		int s_y = upper_layer_track.second.start_index;
		int step = upper_layer_track.second.Step;
		if (y < s_y) return false;
		if (y == s_y) return true;
		if (((y - s_y) % step) == 0 && ((y - s_y) / step < upper_layer_track.second.Do)){
			return true;
		}
		else return false;
	}
	return false;
}

bool DetermineFall(int x, int y, pair<Parser::Track, Parser::Track>  &lower_layer_track, bool dir){
	if (dir == false){
		int s_x = lower_layer_track.first.start_index;
		int step = lower_layer_track.first.Step;
		if (x < s_x) return false;
		if (x == s_x) return true;
		if (((x - s_x) % step) == 0 && ((x - s_x) / step < lower_layer_track.first.Do)){
			return true;
		}
		else return false;
	}
	else if (dir){
		int s_y = lower_layer_track.second.start_index;
		int step = lower_layer_track.second.Step;
		if (y < s_y) return false;
		if (y == s_y) return true;
		if (((y - s_y) % step) == 0 && ((y - s_y) / step < lower_layer_track.second.Do)){
			return true;
		}
		else return false;
	}
	return false;
}

bool DetermineInPlane(int x, int y, pair<Parser::Track, Parser::Track>  &current_layer_track){
	int s_x = current_layer_track.first.start_index;
	int xstep = current_layer_track.first.Step;
	int s_y = current_layer_track.second.start_index;
	int ystep = current_layer_track.second.Step;
	if (x < s_x) return false;
	if (y < s_y) return false;
	if (y == s_y && x == s_x) return true;
	if (((x - s_x) % xstep) == 0 && ((x - s_x) / xstep < current_layer_track.first.Do)){
		if (((y - s_y) % ystep) == 0 && ((y - s_y) / ystep < current_layer_track.second.Do)){
			return true;
		}
		else return false;
	}
	else return false;
	return false;
}

void ISPD_ConstructGridMap(vector < pair<Parser::Track, Parser::Track> > &Layer_track_list, int design_offset,
	int cur_layer, int highest_layer, bool cur_prefer_dir,
	ISPD_GridMap &grid_map, Real_Coor_Table &real_coor_table){

	pair<Parser::Track, Parser::Track>  lower_layer_track;
	pair<Parser::Track, Parser::Track>  current_layer_track;
	pair<Parser::Track, Parser::Track>  upper_layer_track;

	if (cur_layer != 1) lower_layer_track = Layer_track_list[(cur_layer - 1) - 1];
	current_layer_track = Layer_track_list[(cur_layer - 1)];
	if (cur_layer != highest_layer) upper_layer_track = Layer_track_list[(cur_layer - 1) + 1];

	vector <coor_t> x_axis;
	vector <coor_t> y_axis;

	map <coor_t, ISPD_NodeType /* Use for note node able to raise/fall/Steady */> X_Axis_Table;

	//printf("X(%d) : start_index : %d, Do %d , Step %d\n", cur_layer, current_layer_track.first.start_index , current_layer_track.first.Do, current_layer_track.first.Step);
	coor_t start2 = current_layer_track.first.start_index;
	ISPD_NodeType cur_nt(false, false, true);
	X_Axis_Table.emplace(start2, cur_nt);
	x_axis.push_back(start2);
	for (int i = 0; i < current_layer_track.first.Do - 1; i++){
		start2 += current_layer_track.first.Step;
		//printf("%f ", start2);
		X_Axis_Table.emplace(start2, cur_nt);
		x_axis.push_back(start2);
	}
	if (cur_layer != 1 && cur_prefer_dir == false){
		coor_t start1 = lower_layer_track.first.start_index;
		ISPD_NodeType fall_nt(false, true, false);
		auto iter = X_Axis_Table.find(start1);
		if (iter != X_Axis_Table.end()){
			//iter->second.log();
			iter->second.Fallable = true;
			/*auto iter2 = X_Axis_Table.find(start1);
			iter2->second.log();*/
		}
		else {
			X_Axis_Table.emplace(start1, fall_nt);
			x_axis.push_back(start1);
		}
		for (int i = 0; i < lower_layer_track.first.Do - 1; i++){
			start1 += lower_layer_track.first.Step;
			iter = X_Axis_Table.find(start1);
			if (iter != X_Axis_Table.end()){
				iter->second.Fallable = true;
			}
			else {
				X_Axis_Table.emplace(start1, fall_nt);
				x_axis.push_back(start1);
			}
		}
	}
	if (cur_layer != highest_layer && cur_prefer_dir == false){
		coor_t start3 = upper_layer_track.first.start_index;
		ISPD_NodeType raise_nt(true, false, false);
		auto iter = X_Axis_Table.find(start3);
		if (iter != X_Axis_Table.end()){
			iter->second.Raiseable = true;
		}
		else {
			X_Axis_Table.emplace(start3, raise_nt);
			x_axis.push_back(start3);
		}
		for (int i = 0; i < upper_layer_track.first.Do - 1; i++){
			start3 += upper_layer_track.first.Step;
			iter = X_Axis_Table.find(start3);
			if (iter != X_Axis_Table.end()){
				iter->second.Raiseable = true;
			}
			else {
				X_Axis_Table.emplace(start3, raise_nt);
				x_axis.push_back(start3);
			}
		}
	}

	stable_sort(x_axis.begin(), x_axis.end());

	map <coor_t, ISPD_NodeType > Y_Axis_Table; // Use for Filter repeat y point

	coor_t start5 = current_layer_track.second.start_index;
	Y_Axis_Table.emplace(start5, cur_nt);
	y_axis.push_back(start5);
	for (int i = 0; i < current_layer_track.second.Do - 1; i++){
		start5 += current_layer_track.second.Step;
		Y_Axis_Table.emplace(start5, cur_nt);
		y_axis.push_back(start5);
	}
	if (cur_layer != 1 && cur_prefer_dir == true){
		coor_t start4 = lower_layer_track.second.start_index;
		ISPD_NodeType fall_nt(false, true, false);
		auto iter = Y_Axis_Table.find(start4);
		if (iter != Y_Axis_Table.end()){
			iter->second.Fallable = true;
		}
		else {
			Y_Axis_Table.emplace(start4, fall_nt);
			y_axis.push_back(start4);
		}
		for (int i = 0; i < lower_layer_track.second.Do - 1; i++){
			start4 += lower_layer_track.second.Step;
			iter = Y_Axis_Table.find(start4);
			if (iter != Y_Axis_Table.end()){
				iter->second.Fallable = true;
			}
			else {
				Y_Axis_Table.emplace(start4, fall_nt);
				y_axis.push_back(start4);
			}
		}
	}
	if (cur_layer != highest_layer && cur_prefer_dir == true){
		coor_t start6 = upper_layer_track.second.start_index;
		ISPD_NodeType raise_nt(true, false, false);
		auto iter = Y_Axis_Table.find(start6);
		if (iter != Y_Axis_Table.end()){
			iter->second.Raiseable = true;
		}
		else {
			Y_Axis_Table.emplace(start6, raise_nt);
			y_axis.push_back(start6);
		}
		for (int i = 0; i < upper_layer_track.second.Do - 1; i++){
			start6 += upper_layer_track.second.Step;
			iter = Y_Axis_Table.find(start6);
			if (iter != Y_Axis_Table.end()){
				iter->second.Raiseable = true;
			}
			else {
				Y_Axis_Table.emplace(start6, raise_nt);
				y_axis.push_back(start6);
			}
		}
	}
		/*for (int i = 0; i < y_axis.size(); i++){
		printf("%d ", y_axis[i]);
	}*/
	stable_sort(y_axis.begin(), y_axis.end());

	real_coor_table.Coor_Table.push_back(make_pair(x_axis,y_axis));

	/*printf("Layer:%d:\n", cur_layer);
	for (int i = 0; i < y_axis.size(); i++){
	auto itery = Y_Axis_Table.find(y_axis[i]);
	printf("(%d,yR(%d),yF(%d))\n", y_axis[i],
	itery->second.Raiseable, itery->second.Fallable);
	}
	for (int i = 0; i < x_axis.size(); i++){
	auto iterx = X_Axis_Table.find(x_axis[i]);
	printf("(%d,xR(%d),xF(%d))\n", x_axis[i],
	iterx->second.Raiseable, iterx->second.Fallable);
	}*/

	/*vector <ISPD_NodeType> Y_Corressponding_Table;
	vector <ISPD_NodeType> X_Corressponding_Table;
	Y_Corressponding_Table.resize(y_axis.size());
	X_Corressponding_Table.resize(x_axis.size());
	for (int y = 0; y < Y_Corressponding_Table.size(); y++){
		auto itery = Y_Axis_Table.find(y_axis[y]);
		if (itery != Y_Axis_Table.end())
			Y_Corressponding_Table[y] = itery->second;

		//printf("Layer(%d) Y In Plane (%d)\n", cur_layer - 1, Y_Corressponding_Table[y].In_plane);
	}
	for (int x = 0; x < X_Corressponding_Table.size(); x++){
		auto iterx = X_Axis_Table.find(x_axis[x]);
		if (iterx != X_Axis_Table.end())
			X_Corressponding_Table[x] = iterx->second;

		//printf("Layer(%d) X In Plane (%d)\n", cur_layer - 1, X_Corressponding_Table[x].In_plane);
	}*/

	//printf("Y(%d) : start_index : %d, Do %d , Step %d\n", cur_layer, current_layer_track.second.start_index, current_layer_track.second.Do, current_layer_track.second.Step);

	for (int i = 0; i < y_axis.size(); i++)
	{
		vector < MazeNode > grid_row_map;
		//auto itery = Y_Axis_Table.find(y_axis[i]);
		for (int j = 0; j < x_axis.size(); j++){
			//auto iterx = X_Axis_Table.find(x_axis[j]);

			/*bool RaiseAble = (X_Corressponding_Table[j].Raiseable) | (Y_Corressponding_Table[i].Raiseable);
			bool FallAble = (X_Corressponding_Table[j].Fallable) | (Y_Corressponding_Table[i].Fallable);*/
			
			//bool InPlane = X_Corressponding_Table[j].In_plane & Y_Corressponding_Table[i].In_plane;
			bool RaiseAble = false;
			bool FallAble = false;
			if (cur_layer != highest_layer){
				RaiseAble = DetermineRaise(x_axis[j], y_axis[i], upper_layer_track, cur_prefer_dir);
			}
			if (cur_layer != 1)
				FallAble = DetermineFall(x_axis[j], y_axis[i], lower_layer_track, cur_prefer_dir);
			bool InPlane = DetermineInPlane(x_axis[j], y_axis[i], current_layer_track);
			MazeNode mn(InPlane, RaiseAble, FallAble);
			grid_row_map.push_back(mn);
		}
		grid_map.push_back(grid_row_map);
	}

	
	printf("Metal(%d): GridMap( %d x %d )\n", cur_layer, x_axis.size(), y_axis.size());
}

void ISPD_ViaBridge_connection(int cur_layer, FastCoorMap &fast_coor_map, vector<ISPD_GridMap> &GridMap_layout,
							   Real_Coor_Table &real_coor_table, RaisingFalling_Connection_Table &connection_table, bool same_distribution = false)
{

	map<coor_t, int> Y_Axis_Table = fast_coor_map.at(cur_layer - 1).second;
	map<coor_t, int> X_Axis_Table = fast_coor_map.at(cur_layer - 1).first;
	vector<int> Y_Corressponding_Table;
	vector<int> X_Corressponding_Table;
	Y_Corressponding_Table.resize(GridMap_layout[cur_layer].size());
	X_Corressponding_Table.resize(GridMap_layout[cur_layer][0].size());
	//printf("y : size (%d)\n", GridMap_layout[cur_layer].size());
	for (int y = 0; y < GridMap_layout[cur_layer].size(); y++)
	{
			auto itery = Y_Axis_Table.find(get<1>(real_coor_table.Index2Coor(0,y,cur_layer )));
			if (itery != Y_Axis_Table.end())
				Y_Corressponding_Table[y] = itery->second;
			else
				Y_Corressponding_Table[y] = -1;
	}
	//printf("x : size (%d)\n", GridMap_layout[cur_layer][0].size());
	for (int x = 0; x < GridMap_layout[cur_layer][0].size(); x++)
	{
			auto iterx = X_Axis_Table.find(get<0>(real_coor_table.Index2Coor(x,0,cur_layer )));
			if (iterx != X_Axis_Table.end())
				X_Corressponding_Table[x] = iterx->second;
			else
				X_Corressponding_Table[x] = -1;
	}
	//printf("y(%d)\n", cur_layer);
	for (int y = 0; y < GridMap_layout[cur_layer].size(); y++)
	{
		if(GridMap_layout[cur_layer][y][0].FallAble)
		{
			int map_y = Y_Corressponding_Table[y];
			connection_table.Raising_Table.at(cur_layer - 1).second.at(map_y) = y;
			connection_table.Falling_Table.at(cur_layer).second.at(y) = map_y;
		}
	}
	//printf("x(%d)\n", cur_layer);
	for (int x = 0; x < GridMap_layout[cur_layer][0].size(); x++)
	{
		if(GridMap_layout[cur_layer][0][x].FallAble)
		{
			int map_x = X_Corressponding_Table[x];
			connection_table.Raising_Table.at(cur_layer - 1).first.at(map_x) = x;
			connection_table.Falling_Table.at(cur_layer).first.at(x) = map_x;
		}
	}
}

void LayoutGridMapConstruction(Parser::Design &design, FastCoorMap &fast_coor_map, vector<ISPD_GridMap> &GridMap_layout,
							   Real_Coor_Table &real_coor_table, RaisingFalling_Connection_Table &connection_table)
{

	//clock_t GM_start = clock();

	printf("Construct GridMap\n");
	for (int layer = 0; layer < design.ispd_track_list.size(); layer++){
		bool Dir = design.Layer_list[layer * 2].metalLayer.DIRECTION;
		//printf("layer %d dir %d\n", layer + 1, Dir);
		ISPD_GridMap grid_map;
		ISPD_ConstructGridMap(design.ispd_track_list, design.ISPD_OFFSET, layer + 1, design.ispd_track_list.size(), Dir, grid_map, real_coor_table);
		GridMap_layout.push_back(grid_map);
	}

	int node_num = 0;
	int RCT_memory = 0;
	for (int layer = 0; layer < design.ispd_track_list.size(); layer++)
	{
		map<coor_t, int> X_Axis_Table;
		map<coor_t, int> Y_Axis_Table;

		for (int y = 0; y < real_coor_table.Coor_Table.at(layer).second.size(); y++)
		{
			Y_Axis_Table.emplace(real_coor_table.Coor_Table.at(layer).second.at(y), y);
		}
		for (int x = 0; x < real_coor_table.Coor_Table.at(layer).first.size(); x++)
		{
			X_Axis_Table.emplace(real_coor_table.Coor_Table.at(layer).first.at(x), x);
		}
		node_num += (X_Axis_Table.size() * Y_Axis_Table.size());
		RCT_memory += (X_Axis_Table.size() + Y_Axis_Table.size()) * 4;

		fast_coor_map.push_back(make_pair(X_Axis_Table, Y_Axis_Table));
	}

	printf("Memory Usage\n");
	printf("Node_Num : %d\n", node_num);
	printf("real_coor_table memory : %d bytes\n", RCT_memory);
	printf("connection_table memory : %d bytes\n", RCT_memory * 2);

	connection_table.SetTable(real_coor_table);

	printf("ViaBridge connection\n");
	for (int layer = 0; layer < design.ispd_track_list.size() - 1; layer++){
		ISPD_ViaBridge_connection(layer + 1, fast_coor_map, GridMap_layout, real_coor_table, connection_table,false);
	}

	//clock_t GM_end = clock();
	//double duration = (double)(GM_end - GM_start) / CLOCKS_PER_SEC;
	//printf("Grid Map Construct Time Cost : %.2lf sec\n", duration);
}


bool PinMapToGridmap(FastCoorMap &fast_coor_map, Parser::IPSD_Routing_PIN &Pin, vector <Node> &Node_list,bool &NoNodeNeedRefine){
	NoNodeNeedRefine = false;

	// Pin 
	if (false){

		
		int layer = Pin.Layer;
		auto map_x = fast_coor_map[layer - 1].first.find(Pin.IPin_index.first);
		auto map_y = fast_coor_map[layer - 1].second.find(Pin.IPin_index.second);
		if (map_x != fast_coor_map[layer - 1].first.end() || map_y != fast_coor_map[layer - 1].second.end()){
			//printf("PinMapToGridmap::Pin(%d,%d,%d)\n", map_x->second, map_y->second, layer - 1);
			Node node;
			node.x = map_x->second;
			node.y = map_y->second;
			node.z = layer - 1;
			Node_list.push_back(node);
		}
		else{
			printf("Error::PinMapToGridmap No Grid node for Pin(%d,%d,%d)\n", Pin.IPin_index.first, Pin.IPin_index.second, layer);
			exit(1);
		}

		return Pin.type;
	}
	// Shape
	else{

		for (int i = 0; i < Pin.IRect_list.size(); i++){
			int layer = -1;
			layer = Pin.IRect_list[i].Layer;
			//printf("Layer : %d\n", layer);
			//layer = 1;
			auto Lxp = fast_coor_map.at(layer - 1).first.upper_bound(Pin.IRect_list[i].LB.first);
			auto Rxp = fast_coor_map.at(layer - 1).first.lower_bound(Pin.IRect_list[i].RT.first);
			Rxp--;
			auto Byp = fast_coor_map.at(layer - 1).second.upper_bound(Pin.IRect_list[i].LB.second);
			auto Typ = fast_coor_map.at(layer - 1).second.lower_bound(Pin.IRect_list[i].RT.second);
			Typ--;

			/*printf("PinMapToGridmap::PinShape(%d):Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", i , layer - 1, Pin.IRect_list[i].LB.first,
				Pin.IRect_list[i].LB.second, Pin.IRect_list[i].RT.first, Pin.IRect_list[i].RT.second, 
				Lxp->first, Byp->first, Rxp->first, Typ->first,
				Lxp->second, Byp->second, Rxp->second, Typ->second);*/

			if (Lxp->second > Rxp->second || Byp->second > Typ->second){
				printf("WARNING:: Guide with no node inside\n");
			}
			else if (Lxp->first > Pin.IRect_list[i].RT.first || Rxp->first < Pin.IRect_list[i].LB.first || Byp->first > Pin.IRect_list[i].RT.second || Typ->first < Pin.IRect_list[i].LB.second){
				//printf("Error::PinMapToGridmap : No Grid Node ");
			/*	printf("PinMapToGridmap::PinShape(%d):Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", i, layer - 1, Pin.IRect_list[i].LB.first,
					Pin.IRect_list[i].LB.second, Pin.IRect_list[i].RT.first, Pin.IRect_list[i].RT.second,
					Lxp->first, Byp->first, Rxp->first, Typ->first,
					Lxp->second, Byp->second, Rxp->second, Typ->second);*/
				for (int y = Byp->second; y <= Typ->second; y++){
					for (int x = Lxp->second; x <= Rxp->second; x++){
						Node temp_node(x, y, layer - 1);
						Node_list.push_back(temp_node);
					}
				}
				NoNodeNeedRefine = true;
			}
			else{
				for (int y = Byp->second; y <= Typ->second; y++){
					for (int x = Lxp->second; x <= Rxp->second; x++){
						Node temp_node(x, y, layer - 1);
						Node_list.push_back(temp_node);
					}
				}
				NoNodeNeedRefine = false;
			}
		}
		return Pin.type;
	}

}

// I/O for ISPD
void GetRoutingNetsAndObstacles(
	vector<TwoPinRipUpNet> &ripup_nets,
	vector<Node> &obstacle_nodes
	)
{
	vector<TwoPinRipUpNet> tmp_ripup_nets;
	vector<Node> tmp_obstacle_nodes;
	// 參考 Grid_mapper::MapTwoPinNet() 匯入 TwoPinNet -> tmp_ripup_nets     再 push 進 ripup_nets
	// 參考 Grid_mapper::MapBlockage()  匯入 Blockage  -> tmp_obstacle_nodes 再 push 進 obstacle_nodes

	// 若 Net 含 2 Pin 以上 , 利用 MST 建立 Routing Tree 參考 MinimumSpanningTree 建立 MST for Net

	/*
	TwoPinNet  :
	TwoPinRipUpNet{
	int net_id;
	int two_pin_net_id;  // a Net contain multiple two pin nets => two_pin_net_id larger than or equal to net_id
	TwoPinNetConnection::Pin
	TwoPinNetConnection::Box
	}
	*/

	return;
}

void Construct_SpacingTable(Design &design, vector < vector < pair <int, int> > > &Spacing_Table, 
	vector <Parser::INT_EOL_Rule>  &ISPD_EndOfLine_Spacing, vector <int> &ISPD_Via_cut_Spacing){

	ISPD_EndOfLine_Spacing.resize(design.Metal_Layer_Num);
	Spacing_Table.resize(design.Metal_Layer_Num);
	for (int i = 0; i < design.Metal_Layer_Num; i++){
		ISPD_EndOfLine_Spacing[i].ENDOFLINE = (int)(design.Layer_list[2 * i].metalLayer.EOL.ENDOFLINE * design.ISPD_OFFSET);
		if(ISPD_EndOfLine_Spacing[i].ENDOFLINE == -1 * design.ISPD_OFFSET)
		{
			ISPD_EndOfLine_Spacing[i].ENDOFLINE = ISPD_EndOfLine_Spacing[i-1].ENDOFLINE ;
		}

		ISPD_EndOfLine_Spacing[i].SPACING = (int)(design.Layer_list[2 * i].metalLayer.EOL.SPACING * design.ISPD_OFFSET);
		if (ISPD_EndOfLine_Spacing[i].SPACING == -1 * design.ISPD_OFFSET)
		{
			ISPD_EndOfLine_Spacing[i].SPACING = ISPD_EndOfLine_Spacing[i - 1].SPACING;
		}

		ISPD_EndOfLine_Spacing[i].WITHIN = (int)(design.Layer_list[2 * i].metalLayer.EOL.WITHIN * design.ISPD_OFFSET);
		if (ISPD_EndOfLine_Spacing[i].WITHIN == -1 * design.ISPD_OFFSET)
		{
			ISPD_EndOfLine_Spacing[i].WITHIN = ISPD_EndOfLine_Spacing[i - 1].WITHIN;
		}

		for (int st = 0; st < design.Layer_list[2 * i].metalLayer.spacing_table.WIDTH.size(); st++)
			Spacing_Table[i].push_back(make_pair((int)(design.Layer_list[2 * i].metalLayer.spacing_table.WIDTH[st].first * design.ISPD_OFFSET),
			(int)(design.Layer_list[2 * i].metalLayer.spacing_table.WIDTH[st].second * design.ISPD_OFFSET)));
	}

	ISPD_Via_cut_Spacing.resize(design.Via_Layer_Num);
	for (int i = 0; i < design.Via_Layer_Num; i++){
		ISPD_Via_cut_Spacing[i] = (int)(design.Layer_list[2 * i + 1].viaLayer.CUT_SPACING  * design.ISPD_OFFSET);
	}
	

}

void ISPD_blockage_array_initialization(vector<ISPD_GridMap> &GridMap_layout,
	vector< vector < vector<int> > > &ISPD_original_blockage_array, 
	vector<vector< vector<int> > > &ISPD_original_EOL_array_,
	vector< vector< vector<int> > > &ISPD_original_iroute_array_,
	vector< vector< vector<bool> > > &ISPD_L2_OBS_array_)
	{
	//ISPD_blockage_array initialize
	ISPD_original_blockage_array.resize(GridMap_layout.size());
	ISPD_original_EOL_array_.resize(GridMap_layout.size());
	ISPD_L2_OBS_array_.resize(GridMap_layout.size());

	//ISPD_original_iroute_array_.resize(GridMap_layout.size());
	for (int i = 0; i < GridMap_layout.size(); i++)
	{
		ISPD_original_blockage_array[i].resize(GridMap_layout[i].size());
		ISPD_original_EOL_array_[i].resize(GridMap_layout[i].size());
		ISPD_L2_OBS_array_[i].resize(GridMap_layout[i].size());
		//ISPD_original_iroute_array_[i].resize(GridMap_layout[i].size());
		for (int j = 0; j < GridMap_layout[i].size(); j++)
		{
			ISPD_original_blockage_array[i][j].resize(GridMap_layout[i][j].size());
			ISPD_original_EOL_array_[i][j].resize(GridMap_layout[i][j].size());
			ISPD_L2_OBS_array_[i][j].resize(GridMap_layout[i][j].size());
			//ISPD_original_iroute_array_[i][j].resize(GridMap_layout[i][j].size());
		}
	}
	

	for (int i = 0; i < GridMap_layout.size(); i++)
	{
		for (int j = 0; j < GridMap_layout[i].size(); j++)
		{
			for (int k = 0; k < GridMap_layout[i][j].size(); k++)
			{
				ISPD_original_blockage_array[i][j][k] = NO_BLOCKAGE;
				ISPD_original_EOL_array_[i][j][k] = NO_BLOCKAGE;
				ISPD_L2_OBS_array_[i][j][k] = false;
				//ISPD_original_iroute_array_[i][j][k] = NO_BLOCKAGE;
			}
		}
	}
}

void SetRectRangeToBlockage(int Lx,int Rx,int By,int Ty,
	vector<vector< vector<int> > > &_blockage_array_,int net_id,int layer){
	for (int y = By; y <= Ty ; y++){
		for (int x = Lx; x <= Rx ; x++){
			if (_blockage_array_[layer][y][x] == NO_BLOCKAGE){
				if (net_id == -1){
					_blockage_array_[layer][y][x] = HARD_BLOCKAGE;
				}
				else{
					_blockage_array_[layer][y][x] = net_id;
				}
			}
			else if (_blockage_array_[layer][y][x] > 0){
				_blockage_array_[layer][y][x] = HARD_BLOCKAGE;
			}
		}
	}
}

void SetAllShapeToBlockage(Design &design, vector<vector< vector<int> > > &ISPD_original_blockage_array_, 
	vector<vector< vector<int> > > &ISPD_original_EOL_array_, vector < vector < pair <int, int> > > &Spacing_Table
	, vector <Parser::INT_EOL_Rule>  &ISPD_EndOfLine_Spacing, FastCoorMap &fast_coor_map){

	for (int i = 0; i < design.AllShapes.size(); i++){
		//printf("AllShapes LB(%d,%d) RT(%d,%d)\n", design.AllShapes[i].LB.first, design.AllShapes[i].LB.second, design.AllShapes[i].RT.first, design.AllShapes[i].RT.second);
		int width = min(design.AllShapes[i].RT.first - design.AllShapes[i].LB.first, 
			design.AllShapes[i].RT.second - design.AllShapes[i].LB.second);
		//printf("Width = %d\n", width) ;
		int layer = design.AllShapes[i].Layer - 1;
		int spacing = -1;
		//printf("layer = %d %d\n", layer, Spacing_Table[layer].size() );
		for (int st = 0; st < Spacing_Table[layer].size(); st++){
			if (width >= Spacing_Table[layer][st].first)
				spacing = Spacing_Table[layer][st].second;
			else{
				break;
			}
		}
		if (spacing == -1){
			printf("SetAllShapeToBlockage:: Error for spacing = -1\n");
			exit(-1);
		}
		spacing += (int)(0.5 * design.Layer_list[2 * layer].metalLayer.WIDTH * design.ISPD_OFFSET);
		// EOL Addition
		if(design.AllShapes[i].RT.first - design.AllShapes[i].LB.first 
		< ISPD_EndOfLine_Spacing[layer].ENDOFLINE){
			Parser::I_Rect Horizontal_EOL;
			Horizontal_EOL.LB.first = 
				design.AllShapes[i].LB.first - ISPD_EndOfLine_Spacing[layer].WITHIN;
			Horizontal_EOL.RT.first = 
				design.AllShapes[i].RT.first + ISPD_EndOfLine_Spacing[layer].WITHIN;
			// Bot EOL
			auto Lxp_Bot = fast_coor_map.at(layer).first.upper_bound(Horizontal_EOL.LB.first);
			auto Rxp_Bot = fast_coor_map.at(layer).first.lower_bound(Horizontal_EOL.RT.first);
			Rxp_Bot--;
			auto Byp_Bot = fast_coor_map.at(layer).second.upper_bound(design.AllShapes[i].LB.second - ISPD_EndOfLine_Spacing[layer].SPACING);
			auto Typ_Bot = fast_coor_map.at(layer).second.lower_bound(design.AllShapes[i].LB.second);
			Typ_Bot--;
			if (Lxp_Bot->first > (Horizontal_EOL.RT.first) 
			|| Rxp_Bot->first < (Horizontal_EOL.LB.first) 
			|| Byp_Bot->first >(design.AllShapes[i].LB.second) 
			|| Typ_Bot->first < (design.AllShapes[i].LB.second - ISPD_EndOfLine_Spacing[layer].SPACING)){
				;//printf("SetAllShapeToBlockage:: : No Grid Node ");
			}
			else{	
				SetRectRangeToBlockage(Lxp_Bot->second,Rxp_Bot->second,
				Byp_Bot->second, Typ_Bot->second,ISPD_original_blockage_array_,
				design.AllShapes_NetId[i],layer);
			}
			// Top EOL
			Byp_Bot = fast_coor_map.at(layer).second.upper_bound(design.AllShapes[i].RT.second);
			Typ_Bot = fast_coor_map.at(layer).second.lower_bound(design.AllShapes[i].RT.second + ISPD_EndOfLine_Spacing[layer].SPACING);
			Typ_Bot--;
			if (Lxp_Bot->first > (Horizontal_EOL.RT.first) 
			|| Rxp_Bot->first < (Horizontal_EOL.LB.first) 
			|| Byp_Bot->first >(design.AllShapes[i].RT.second + ISPD_EndOfLine_Spacing[layer].SPACING) 
			|| Typ_Bot->first < (design.AllShapes[i].RT.second)){
				;//printf("SetAllShapeToBlockage:: : No Grid Node ");
			}
			else{	
				SetRectRangeToBlockage(Lxp_Bot->second,Rxp_Bot->second,
				Byp_Bot->second, Typ_Bot->second,ISPD_original_blockage_array_,design.AllShapes_NetId[i],layer);
			}
		}
		if(design.AllShapes[i].RT.second - design.AllShapes[i].LB.second 
		< ISPD_EndOfLine_Spacing[layer].ENDOFLINE){
			Parser::I_Rect Vertical_EOL;
			Vertical_EOL.LB.second = 
				design.AllShapes[i].LB.second - ISPD_EndOfLine_Spacing[layer].WITHIN;
			Vertical_EOL.RT.second = 
				design.AllShapes[i].RT.second + ISPD_EndOfLine_Spacing[layer].WITHIN;
			// LEFT EOL
			auto Lxp_Bot = fast_coor_map.at(layer).first.upper_bound(design.AllShapes[i].LB.first - ISPD_EndOfLine_Spacing[layer].SPACING);
			auto Rxp_Bot = fast_coor_map.at(layer).first.lower_bound(design.AllShapes[i].RT.first);
			Rxp_Bot--;
			auto Byp_Bot = fast_coor_map.at(layer).second.upper_bound(Vertical_EOL.LB.second);
			auto Typ_Bot = fast_coor_map.at(layer).second.lower_bound(Vertical_EOL.RT.second);
			Typ_Bot--;
			if (Lxp_Bot->first > (design.AllShapes[i].RT.first)
			|| Rxp_Bot->first < (design.AllShapes[i].LB.first - ISPD_EndOfLine_Spacing[layer].SPACING) 
			|| Byp_Bot->first >(Vertical_EOL.RT.second) 
			|| Typ_Bot->first < (Vertical_EOL.LB.second)){
				;//printf("SetAllShapeToBlockage:: : No Grid Node ");
			}
			else{	
				SetRectRangeToBlockage(Lxp_Bot->second,Rxp_Bot->second,
				Byp_Bot->second, Typ_Bot->second,ISPD_original_blockage_array_,design.AllShapes_NetId[i],layer);
			}
			// Top EOL
			Lxp_Bot = fast_coor_map.at(layer).first.upper_bound(design.AllShapes[i].LB.first);
			Rxp_Bot = fast_coor_map.at(layer).first.lower_bound(design.AllShapes[i].RT.first + ISPD_EndOfLine_Spacing[layer].SPACING);
			Rxp_Bot--;

			if (Lxp_Bot->first > (design.AllShapes[i].RT.first + ISPD_EndOfLine_Spacing[layer].SPACING) 
			|| Rxp_Bot->first < (design.AllShapes[i].LB.first) 
			|| Byp_Bot->first >(Vertical_EOL.RT.second) 
			|| Typ_Bot->first < (Vertical_EOL.LB.second)){
				;//printf("SetAllShapeToBlockage:: : No Grid Node ");
			}
			else{	
				SetRectRangeToBlockage(Lxp_Bot->second,Rxp_Bot->second,
				Byp_Bot->second, Typ_Bot->second,ISPD_original_blockage_array_,design.AllShapes_NetId[i],layer);
			}
		}
		// Parallel Run Length Spacing
		if(design.AllShapes[i].RT.first - design.AllShapes[i].LB.first 
		>= design.AllShapes[i].RT.second - design.AllShapes[i].LB.second){
			auto Lxp_spacing = fast_coor_map.at(layer).first.upper_bound(design.AllShapes[i].LB.first);
			auto Rxp_spacing = fast_coor_map.at(layer).first.lower_bound(design.AllShapes[i].RT.first);
			Rxp_spacing--;
			auto Byp_spacing = fast_coor_map.at(layer).second.upper_bound(design.AllShapes[i].LB.second - spacing);
			auto Typ_spacing = fast_coor_map.at(layer).second.lower_bound(design.AllShapes[i].RT.second + spacing);
			Typ_spacing--;

			if (Lxp_spacing->first > (design.AllShapes[i].RT.first + spacing) || Rxp_spacing->first < (design.AllShapes[i].LB.first - spacing) 
			|| Byp_spacing->first >(design.AllShapes[i].RT.second + spacing) || Typ_spacing->first < (design.AllShapes[i].LB.second - spacing)){
			//printf("SetAllShapeToBlockage:: : No Grid Node ");
			}
			else{
				SetRectRangeToBlockage(Lxp_spacing->second,Rxp_spacing->second,
				Byp_spacing->second, Typ_spacing->second,ISPD_original_blockage_array_,
				design.AllShapes_NetId[i],layer);
			}
		}
		else{
			auto Lxp_spacing = fast_coor_map.at(layer).first.upper_bound(design.AllShapes[i].LB.first - spacing);
			auto Rxp_spacing = fast_coor_map.at(layer).first.lower_bound(design.AllShapes[i].RT.first + spacing);
			Rxp_spacing--;
			auto Byp_spacing = fast_coor_map.at(layer).second.upper_bound(design.AllShapes[i].LB.second);
			auto Typ_spacing = fast_coor_map.at(layer).second.lower_bound(design.AllShapes[i].RT.second);
			Typ_spacing--;
			if (Lxp_spacing->first > (design.AllShapes[i].RT.first + spacing) || Rxp_spacing->first < (design.AllShapes[i].LB.first - spacing) 
			|| Byp_spacing->first >(design.AllShapes[i].RT.second + spacing) || Typ_spacing->first < (design.AllShapes[i].LB.second - spacing)){
			//printf("SetAllShapeToBlockage:: : No Grid Node ");
			}
			else{
				SetRectRangeToBlockage(Lxp_spacing->second,Rxp_spacing->second,
				Byp_spacing->second, Typ_spacing->second,ISPD_original_blockage_array_,
				design.AllShapes_NetId[i],layer);
			}
		}

		int eol_spacing = ISPD_EndOfLine_Spacing[layer].SPACING ;
		
		auto Lxp_eol = fast_coor_map.at(layer).first.upper_bound(design.AllShapes[i].LB.first - eol_spacing);
		auto Rxp_eol = fast_coor_map.at(layer).first.lower_bound(design.AllShapes[i].RT.first + eol_spacing);
		Rxp_eol--;
		auto Byp_eol = fast_coor_map.at(layer).second.upper_bound(design.AllShapes[i].LB.second - eol_spacing);
		auto Typ_eol = fast_coor_map.at(layer).second.lower_bound(design.AllShapes[i].RT.second + eol_spacing);
		Typ_eol--;
		if (Lxp_eol->first > (design.AllShapes[i].RT.first + eol_spacing) || Rxp_eol->first < (design.AllShapes[i].LB.first - eol_spacing)
			|| Byp_eol->first >(design.AllShapes[i].RT.second + eol_spacing) || Typ_eol->first < (design.AllShapes[i].LB.second - eol_spacing)){
			//printf("SetAllShapeToBlockage:: : No Grid Node ");
		}
		else{
			for (int y = Byp_eol->second; y <= Typ_eol->second; y++){
				for (int x = Lxp_eol->second; x <= Rxp_eol->second; x++){
					if (ISPD_original_EOL_array_[layer][y][x] == NO_BLOCKAGE){
						if (design.AllShapes_NetId[i] == -1){
							ISPD_original_EOL_array_[layer][y][x] = HARD_BLOCKAGE;
						}
						else{
							ISPD_original_EOL_array_[layer][y][x] = design.AllShapes_NetId[i];
						}
					}
					else if (ISPD_original_EOL_array_[layer][y][x] > 0){
						ISPD_original_EOL_array_[layer][y][x] = HARD_BLOCKAGE;
					}
				}
			}
		}
		


	}
}

void SetIRouteAndObstaclesOnBlockage(Parser::Design &design,
						 vector<ISPD_GridMap> &GridMap_layout,
						 FastCoorMap &fast_coor_map,
						 TRACKASSIGNMENT::IRouteGenerator &iroute_generator,
						 vector<vector<vector<int>>> &ISPD_original_blockage_array,
						 vector<vector<vector<int>>> &ISPD_original_iroute_array,
						 vector<vector<vector<bool>>> &ISPD_L2_OBS_array_,
						 vector<SpaceEvaluationGraph> &SpaceEvaluationLayout)
{
	/// --------
	TRACKASSIGNMENT::Design_Information &Design_info = iroute_generator.design_information;

	for(int a = 0 ; a < Design_info.routing_net_information.size() ; a++)
	{
		int net_index = a;
		// all net
		TRACKASSIGNMENT::Routing_Net_Information &tempNet = Design_info.routing_net_information.at(a);
		for( int b = 0 ; b < tempNet.Layer_Array_for_Guide.size() ; b++) 
		{
			//layer guide in net
			for(int c = 0 ; c < tempNet.Layer_Array_for_Guide.at(b).size() ; c++)
			{
				TRACKASSIGNMENT::Guide_Component &thisGuide = tempNet.Layer_Array_for_Guide.at(b).at(c);
				for(int d = 0 ; d < thisGuide.Original_IRoute_list.size() ; d++)
				{
					
					TRACKASSIGNMENT::IRoute &thisIROUTE = thisGuide.Original_IRoute_list.at(d);
					int layer = thisIROUTE.Layer;
					//printf("Layer : %d\n", layer);
					//layer = 1;

					if (thisIROUTE.LB.first > thisIROUTE.RT.first)
					{
						int tempInt = thisIROUTE.LB.first;
						thisIROUTE.LB.first = thisIROUTE.RT.first;
						thisIROUTE.RT.first = tempInt;
					}
					if (thisIROUTE.LB.second > thisIROUTE.RT.second)
					{
						int tempInt = thisIROUTE.LB.second;
						thisIROUTE.LB.second = thisIROUTE.RT.second;
						thisIROUTE.RT.second = tempInt;
					}

					bool same = false; // 0:x 1:y
					if (thisIROUTE.LB.first == thisIROUTE.RT.first)
					{
						same = false; // x same, vertical
					}
					else
						same = true; // y same, horizontal

					if (!same)
					{
						Parser::RipUpSegment temp_ignore;
						Parser::I_Rect IRoute_Rect(thisIROUTE.LB.first, thisIROUTE.LB.second, thisIROUTE.RT.first, thisIROUTE.RT.second, layer - 1);
						SpaceEvaluationLayout.at(layer - 1).PushNetBlockageToTree(IRoute_Rect, net_index);//temp_ignore);
						auto Lxp = fast_coor_map.at(layer - 1).first.find(thisIROUTE.LB.first);
						auto Rxp = fast_coor_map.at(layer - 1).first.find(thisIROUTE.RT.first);
						auto Byp = fast_coor_map.at(layer - 1).second.upper_bound(thisIROUTE.LB.second - 1);
						auto Typ = fast_coor_map.at(layer - 1).second.lower_bound(thisIROUTE.RT.second + 1);
						Typ--;

						/*						
						printf("SetIRouteOnBlockage::Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", 
								layer - 1, thisIROUTE.LB.first,
								thisIROUTE.LB.second, thisIROUTE.RT.first, thisIROUTE.RT.second, 
								Lxp->second, Byp->second, Rxp->second, Typ->second);
						*/

						if (Byp->second > Typ->second)
						{
						}
						else
						{
							for (int y = Byp->second; y <= Typ->second; y++)
							{
								for (int x = Lxp->second; x <= Rxp->second; x++)
								{
									//printf("SetIRouteOnBlockage::Net(%d) IRoute blockage on(%d,%d,%d)\n", x, y, layer - 1);
									ISPD_original_blockage_array[layer - 1][y][x] = net_index;
									//ISPD_original_iroute_array[layer - 1][y][x] = net_index;
								}
							}
						} // else
					}
					else
					{
						Parser::RipUpSegment temp_ignore;
						Parser::I_Rect IRoute_Rect(thisIROUTE.LB.first, thisIROUTE.LB.second, thisIROUTE.RT.first, thisIROUTE.RT.second, layer - 1);
						SpaceEvaluationLayout.at(layer - 1).PushNetBlockageToTree(IRoute_Rect, net_index); //, temp_ignore);
						auto Lxp = fast_coor_map.at(layer - 1).first.upper_bound(thisIROUTE.LB.first - 1);
						auto Rxp = fast_coor_map.at(layer - 1).first.lower_bound(thisIROUTE.RT.first + 1);
						auto Byp = fast_coor_map.at(layer - 1).second.find(thisIROUTE.LB.second);
						auto Typ = fast_coor_map.at(layer - 1).second.find(thisIROUTE.RT.second);
						Rxp--;

						if (Lxp->second > Rxp->second)
						{
						}
						else
						{
							for (int y = Byp->second; y <= Typ->second; y++)
							{
								for (int x = Lxp->second; x <= Rxp->second; x++)
								{
									//printf("SetIRouteOnBlockage::Net(%d) IRoute blockage on(%d,%d,%d)\n", x, y, layer - 1);
									ISPD_original_blockage_array[layer - 1][y][x] = net_index;
									//ISPD_original_iroute_array[layer - 1][y][x] = net_index;
								}
							}
						} // else
					}
					
				}
			}

		} // for

	}

	for (int a = 0; a < design.AllOBSs.size(); a++)
	{
		Parser::RipUpSegment temp_ignore;

		int newLayer = design.AllOBSs.at(a).Layer;
		SpaceEvaluationLayout.at(newLayer - 1).PushNetBlockageToTree(design.AllOBSs.at(a), HARD_BLOCKAGE);//, temp_ignore);
		auto _Lxp = fast_coor_map.at(newLayer - 1).first.upper_bound(design.AllOBSs.at(a).LB.first - 1);
		auto _Rxp = fast_coor_map.at(newLayer - 1).first.lower_bound(design.AllOBSs.at(a).RT.first + 1);
		_Rxp--;
		auto _Byp = fast_coor_map.at(newLayer - 1).second.upper_bound(design.AllOBSs.at(a).LB.second - 1);
		auto _Typ = fast_coor_map.at(newLayer - 1).second.lower_bound(design.AllOBSs.at(a).RT.second + 1);
		_Typ--;
/*
		printf("SetOBS_OnBlockage:: Rect Layer(%d) LB(%d,%d) RT(%d,%d) \n",
			   design.AllOBSs.at(a).Layer,
			   design.AllOBSs.at(a).LB.first,
			   design.AllOBSs.at(a).LB.second,
			   design.AllOBSs.at(a).RT.first,
			   design.AllOBSs.at(a).RT.second);
*/
		if (_Lxp->second > _Rxp->second || _Byp->second > _Typ->second)
		{
			;
		}
		else
		{
			/*
			printf("SetOBS_OnLayout:: LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", 
				   design.AllOBSs.at(a).LB.first, design.AllOBSs.at(a).LB.second, design.AllOBSs.at(a).RT.first, design.AllOBSs.at(a).RT.second,
				   _Lxp->first, _Byp->first, _Rxp->first, _Typ->first,
				   _Lxp->second, _Byp->second, _Rxp->second, _Typ->second);
			*/
			for (int y = _Byp->second; y <= _Typ->second; y++)
			{
				for (int x = _Lxp->second; x <= _Rxp->second; x++)
				{
					ISPD_original_blockage_array[newLayer - 1][y][x] = HARD_BLOCKAGE;
					ISPD_L2_OBS_array_[newLayer - 1][y][x] = true;
					ISPD_original_blockage_array[newLayer - 1][y][x] = INT_MAX;
				}
			}
		}
	}
}

void reset_CongestionMap(vector<vector<vector<pair<int, int> >>> &CongestionMap, vector<ISPD_GridMap> &GridMap_layout)
{
	CongestionMap.resize(GridMap_layout.size());
	for (int i = 0; i < GridMap_layout.size(); i++){
		CongestionMap[i].resize(GridMap_layout[i].size());
		for (int j = 0; j < GridMap_layout[i].size(); j++){
			CongestionMap[i][j].resize(GridMap_layout[i][j].size());
		}
	}

	for (int i = 0; i < GridMap_layout.size(); i++)
	{
		for (int j = 0; j < GridMap_layout[i].size(); j++)
		{
			for (int k = 0; k < GridMap_layout[i][j].size(); k++){
				CongestionMap[i][j][k] = make_pair(-1,0);
			}
		}
	}
}

void SetAllPinShapeOnLayout(Parser::Design &design,
							vector<ISPD_GridMap> &GridMap_layout,
							FastCoorMap &fast_coor_map,
							TRACKASSIGNMENT::IRouteGenerator &iroute_generator,
							vector<vector<vector<int>>> &ISPD_original_blockage_array,
							vector<vector<vector<pair<int, int>>>> &CongestionMap,
							Real_Coor_Table &real_coor_table)
{
	int MaxLayerNum = design.Layer_list.size()/2;
	vector<bool> PreferDirList; // 0 :ver, 1 :hor
	PreferDirList.resize(design.Layer_list.size()/2);
	int layer_counter = 0;
	// build prefer direction list
	for( int index = 0 ; index < design.Layer_list.size() ; index++)
	{
		if(index%2 == 0)
		{
			PreferDirList.at(layer_counter) = design.Layer_list.at(index).metalLayer.DIRECTION;

			if (layer_counter > 0 && PreferDirList.at(layer_counter - 1) == PreferDirList.at(layer_counter))
			{
				cout << "ERROR Layer message" << endl;
				exit(-1);
			}
			layer_counter++;
		}
	}

	//X = Vertical, Y = Horizontal
	//					  ver, hor
	vector<pair<Parser::Track, Parser::Track> > LayerOfTrack;
	LayerOfTrack.resize(design.Track_list.size()/2 +1);
	for (int index = 0 ; index < design.Track_list.size() ; index++)
	{
		string tempLayer = design.Track_list.at(index).Layer;
		int intlayer = -1;

		if (tempLayer.size() >= 6)
		{
			tempLayer.erase(tempLayer.begin(), tempLayer.begin() + 5);
			intlayer = atoi(tempLayer.c_str());
		}
		else if(tempLayer.size() == 1)
		{
			intlayer = atoi(tempLayer.c_str());
		}
		else  
		{
			cout << "ERROR:: Wrong Layer info (" << tempLayer << ")\n" ;
			exit(-1);
		}

		if (design.Track_list.at(index).Dir.compare("X") == 0)
		{
			LayerOfTrack.at(intlayer).first = design.Track_list.at(index);
		}
		else 
		{
			LayerOfTrack.at(intlayer).second = design.Track_list.at(index);
		}

	}

	bool debug_mode = false;
	int debug_net_index = 3145;

	for (int index = 0; index < (int)design.SortedShapesList.size() ; index++)
	{
		int net_index = get<1>(design.SortedShapesList.at(index));
		int pin_index = get<2>(design.SortedShapesList.at(index));
		//printf("Net_index(%d) Pin_index(%d)\n", net_index, pin_index);
		IPSD_Routing_PIN &CurrentPin = design.ispd_routing_net.at(net_index).PIN_list.at(pin_index);
		vector<RipUpNode> &layoutNode = design.ispd_routing_net.at(net_index).PIN_list.at(pin_index).LayoutNode;
		vector<RipUpNode> &layoutNodeOriginal = design.ispd_routing_net.at(net_index).PIN_list.at(pin_index).LayoutNodeOriginal;
		vector<RipUpNode> &layoutNodeTRUE = design.ispd_routing_net.at(net_index).PIN_list.at(pin_index).LayoutNodeTRUE;

		if (net_index == debug_net_index && debug_mode)
		{
			printf("SetAllPinShapeOnLayout:: Macro name(%s)\n", CurrentPin.Original_MARCO.Name.c_str());
		}

		int success_assign_counter = 0;
		int out_of_bound_IRect = 0;
		int inside_four_grid = 0;
		IntPair inside_pin_center;
		IntPair inside_pin_bound_LB;
		IntPair inside_pin_bound_RT;

		for (int i = 0; i < CurrentPin.IRect_list.size(); i++)
		{
			if (net_index == debug_net_index && debug_mode)
			{
				
				printf("SetAllPinShapeOnLayout:: Rect Layer(%d) LB(%d,%d) RT(%d,%d) \n",
				   CurrentPin.IRect_list.at(i).Layer,
				   CurrentPin.IRect_list.at(i).LB.first,
				   CurrentPin.IRect_list.at(i).LB.second,
				   CurrentPin.IRect_list.at(i).RT.first,
				   CurrentPin.IRect_list.at(i).RT.second );

				if (CurrentPin.IRect_list.at(i).LB.first == 371430 && CurrentPin.IRect_list.at(i).LB.second == 319350)
				{
					printf("catch!\n");
					int psss;
					cin>>psss;
				}
			}
			bool no_pin_found = false ;

			int layer = CurrentPin.IRect_list.at(i).Layer;
			int newLayer = layer;
			if (newLayer == 1)
				newLayer++;

			auto _Lxp = fast_coor_map.at(newLayer-1).first.upper_bound(CurrentPin.IRect_list[i].LB.first-1);
			auto _Rxp = fast_coor_map.at(newLayer-1).first.lower_bound(CurrentPin.IRect_list[i].RT.first+1);
			_Rxp--;
			auto _Byp = fast_coor_map.at(newLayer-1).second.upper_bound(CurrentPin.IRect_list[i].LB.second-1);
			auto _Typ = fast_coor_map.at(newLayer-1).second.lower_bound(CurrentPin.IRect_list[i].RT.second+1);
			_Typ--;
			/*
			printf("SetAllPinShapeOnLayout:: Rect Layer(%d) LB(%d,%d) RT(%d,%d) \n",
				   CurrentPin.IRect_list.at(i).Layer,
				   CurrentPin.IRect_list.at(i).LB.first,
				   CurrentPin.IRect_list.at(i).LB.second,
				   CurrentPin.IRect_list.at(i).RT.first,
				   CurrentPin.IRect_list.at(i).RT.second);
			*/
			if (net_index == debug_net_index && debug_mode)
				printf("SetAllPinShapeOnLayout::PinShape(%d):Layer(%d) LB(%d,%d) RT(%d,%d) ==> LBp(%d,%d) RTp(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", i, newLayer, CurrentPin.IRect_list[i].LB.first,
					   CurrentPin.IRect_list[i].LB.second, CurrentPin.IRect_list[i].RT.first, CurrentPin.IRect_list[i].RT.second,
					   _Lxp->first, _Byp->first, _Rxp->first, _Typ->first,
					   _Lxp->second, _Byp->second, _Rxp->second, _Typ->second);

			int max_index_Y = GridMap_layout[newLayer - 1].size() - 1;
			int max_index_X = GridMap_layout[newLayer - 1][max_index_Y].size() - 1;

			tuple<coor_t, coor_t, int> temp_coor;
			temp_coor = real_coor_table.Index2Coor(0, 0, newLayer - 1);
			pair<int, int> NodeLB = make_pair(get<0>(temp_coor), get<1>(temp_coor));
			temp_coor = real_coor_table.Index2Coor(max_index_X, max_index_Y, newLayer - 1);
			pair<int, int> NodeRT = make_pair(get<0>(temp_coor), get<1>(temp_coor));
			//printf("SetAllPinShapeOnLayout::Layer(%d) NodeLB(%d,%d) NodeRT(%d,%d)\n", newLayer, NodeLB.first, NodeLB.second, NodeRT.first, NodeRT.second);
			pair<int, int> InsideBoundLB = make_pair(_Rxp->first, _Typ->first);
			pair<int, int> InsideBoundRT = make_pair(_Lxp->first, _Byp->first);
			
			if ((_Lxp->second - 1 == _Rxp->second && _Byp->second - 1 == _Typ->second) &&
				(InsideBoundLB.first < CurrentPin.IRect_list[i].LB.first && CurrentPin.IRect_list[i].LB.first < InsideBoundRT.first) &&
				(InsideBoundLB.second < CurrentPin.IRect_list[i].LB.second && CurrentPin.IRect_list[i].LB.second < InsideBoundRT.second) &&
				(InsideBoundLB.first < CurrentPin.IRect_list[i].RT.first && CurrentPin.IRect_list[i].RT.first < InsideBoundRT.first) &&
				(InsideBoundLB.second < CurrentPin.IRect_list[i].RT.second && CurrentPin.IRect_list[i].RT.second < InsideBoundRT.second))
			{
				/*
				printf("SetAllPinShapeOnLayout::Layer(%d) InsideBoundLB(%d,%d) InsideBoundRT(%d,%d) (%d,%d) (%d,%d)\n", newLayer, _Rxp->first, _Typ->first, _Lxp->first, _Byp->first, _Rxp->second, _Typ->second, _Lxp->second, _Byp->second);
				printf("SetAllPinShapeOnLayout:: Rect Layer(%d) LB(%d,%d) RT(%d,%d) \n",
					   CurrentPin.IRect_list.at(i).Layer,
					   CurrentPin.IRect_list.at(i).LB.first,
					   CurrentPin.IRect_list.at(i).LB.second,
					   CurrentPin.IRect_list.at(i).RT.first,
					   CurrentPin.IRect_list.at(i).RT.second);
				printf("SetAllPinShapeOnLayout:: Inside the nodes.\n");
				*/
				inside_four_grid++;
				inside_pin_center = make_pair((CurrentPin.IRect_list[i].LB.first + CurrentPin.IRect_list[i].RT.first) / 2,
											  (CurrentPin.IRect_list[i].LB.second + CurrentPin.IRect_list[i].RT.second) / 2);
				inside_pin_bound_RT = make_pair(_Lxp->first, _Byp->first);
				inside_pin_bound_LB = make_pair(_Rxp->first, _Typ->first);
			}

			if (TRACKASSIGNMENT::rec_overlap(NodeLB, NodeRT, CurrentPin.IRect_list[i].LB, CurrentPin.IRect_list[i].RT) == false)
			{
				//printf("SetAllPinShapeOnLayout:: OutSide.\n");
				out_of_bound_IRect++;
			}

			if (_Lxp->second > _Rxp->second || _Byp->second > _Typ->second)
			{
				//printf("Not found \n\n");
				no_pin_found = true;
			}
			else
			{
				/*
				printf("SetAllPinShapeOnLayout::PinShape(%d):Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", i, newLayer, CurrentPin.IRect_list[i].LB.first,
					   CurrentPin.IRect_list[i].LB.second, CurrentPin.IRect_list[i].RT.first, CurrentPin.IRect_list[i].RT.second,
					   _Lxp->first, _Byp->first, _Rxp->first, _Typ->first,
					   _Lxp->second, _Byp->second, _Rxp->second, _Typ->second);
				*/
				int pin_shape_range = abs(_Typ->second - _Byp->second) * (_Rxp->second - _Lxp->second);
				if (pin_shape_range == 0) pin_shape_range = 1;
				for (int y = _Byp->second; y <= _Typ->second; y++)
				{
					for (int x = _Lxp->second; x <= _Rxp->second; x++)
					{
						// it's okay to put on layout
						//ISPD_original_blockage_array[layer - 1][y][x] = net_index; // net_index
						//layoutNode.push_back(RipUpNode(x, y, layer));
						/*
						if(y == 1020)
						{
							int pause;
							cin>>pause;
						}
						*/
						
						layoutNodeOriginal.push_back(RipUpNode(x, y, newLayer));
						layoutNodeTRUE.push_back(RipUpNode(x, y, newLayer - 1));
						if (CongestionMap[newLayer - 1][y][x].first == -1){
							CongestionMap[newLayer - 1][y][x].first = net_index;
							CongestionMap[newLayer - 1][y][x].second += PINSHAPE_COVER_PENALTY / pin_shape_range;
						}
						success_assign_counter++;
					}
				}
			}

			if(no_pin_found == true)
			{
				int SameLowVerX = -1, SameLowHorY = -1, SameHighVerX = -1, SameHighHorY = -1;
				int UpLowVerX = -1, UpLowHorY = -1, UpHighVerX = -1, UpHighHorY = -1;
				int LowLowVerX = -1, LowLowHorY = -1, LowHighVerX = -1, LowHighHorY = -1;
				int Rleft = -1, Rright = -1, Rtop = -1, Rdown = -1;
				int layer = -1;
				layer = CurrentPin.IRect_list.at(i).Layer;
				//X = Vertical, Y = Horizontal
				// find the left and right track 
				//            up and down  track
				int start = -1, step = -1;

				// same ver
				// low
				start = LayerOfTrack.at(layer).first.start_index;
				step = LayerOfTrack.at(layer).first.Step;
				//printf("start(%d) step(%d)\n", start, step);
				int remain = (CurrentPin.IRect_list.at(i).LB.first - start) / step ;
				SameLowVerX = remain * step + start;
				// high
				remain = (CurrentPin.IRect_list.at(i).RT.first - start) / step ;
				SameHighVerX = remain * step + start + step;
				//printf("SameLowVerX(%d) SameHighVerX(%d)\n", SameLowVerX, SameHighVerX);

				// same hor
				// low
				start = LayerOfTrack.at(layer).second.start_index;
				step = LayerOfTrack.at(layer).second.Step;
				//printf("start(%d) step(%d)\n", start, step);
				remain = (CurrentPin.IRect_list.at(i).LB.second - start) / step;
				SameLowHorY = remain * step + start;
				// high
				remain = (CurrentPin.IRect_list.at(i).RT.second - start) / step;
				SameHighHorY = remain * step + start + step;
				//printf("SameLowHorY(%d) SameHighHorY(%d)\n", SameLowHorY, SameHighHorY);

				// upper prefer
				if (layer + 1 <= MaxLayerNum )
				{
					if(PreferDirList.at(layer + 1) == false) // ver
					{
						//printf("Upper layer Vertical(%d)\n", layer + 1);
						// low
						start = LayerOfTrack.at(layer + 1).first.start_index;
						step = LayerOfTrack.at(layer + 1).first.Step;
						//printf("start(%d) step(%d)\n", start, step);
						remain = (CurrentPin.IRect_list.at(i).LB.first - start) / step;
						UpLowVerX = remain * step + start;
						// high
						remain = (CurrentPin.IRect_list.at(i).RT.first - start) / step;
						UpHighVerX = remain * step + start + step;
						//printf("UpLowVerX(%d) UpHighVerX(%d)\n", UpLowVerX, UpHighVerX);
					}
					else   
					{
						//printf("Upper layer Horizontal(%d)\n", layer + 1); // hor
						// low
						start = LayerOfTrack.at(layer + 1).second.start_index;
						step = LayerOfTrack.at(layer + 1).second.Step;
						//printf("start(%d) step(%d)\n", start, step);
						remain = (CurrentPin.IRect_list.at(i).LB.second - start) / step;
						UpLowHorY = remain * step + start;
						// high
						remain = (CurrentPin.IRect_list.at(i).RT.second - start) / step;
						UpHighHorY = remain * step + start + step;
						//printf("UpLowHorY(%d) UpHighHorY(%d)\n", UpLowHorY, UpHighHorY);
					}
				}

				// lower prefer
				if (layer - 1 > 0)
				{
					if (PreferDirList.at(layer - 1) == false) // ver
					{
						//printf("Lower layer Vertical(%d)\n", layer - 1);
						// low
						start = LayerOfTrack.at(layer - 1).first.start_index;
						step = LayerOfTrack.at(layer - 1).first.Step;
						//printf("start(%d) step(%d)\n", start, step);
						remain = (CurrentPin.IRect_list.at(i).LB.first - start) / step;
						LowLowVerX = remain * step + start;
						// high
						remain = (CurrentPin.IRect_list.at(i).RT.first - start) / step;
						LowHighVerX = remain * step + start + step;
						//printf("LowLowVerX(%d) LowHighVerX(%d)\n", LowLowVerX, LowHighVerX);
					}
					else
					{
						//printf("Lower layer Horizontal(%d)\n", layer - 1); // hor
						// low
						start = LayerOfTrack.at(layer - 1).second.start_index;
						step = LayerOfTrack.at(layer - 1).second.Step;
						//printf("start(%d) step(%d)\n", start, step);
						remain = (CurrentPin.IRect_list.at(i).LB.second - start) / step;
						LowLowHorY = remain * step + start;
						// high
						remain = (CurrentPin.IRect_list.at(i).RT.second - start) / step;
						LowHighHorY = remain * step + start + step;
						//printf("LowLowHorY(%d) LowHighHorY(%d)\n", LowLowHorY, LowHighHorY);
					}
				}

				// LEFT
				//cout << "LEFT\n";
				if(UpLowVerX != -1 && LowLowVerX != -1)
				{
					Rleft = max(max(SameLowVerX, UpLowVerX), LowLowVerX);
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameLowVerX, UpLowVerX, LowLowVerX, Rleft);
				}
				else if (UpLowVerX != -1 && LowLowVerX == -1)
				{
					Rleft = max(SameLowVerX, UpLowVerX);
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameLowVerX, UpLowVerX, LowLowVerX, Rleft);
				}
				else if (UpLowVerX == -1 && LowLowVerX != -1)
				{
					Rleft = max(SameLowVerX, LowLowVerX);
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameLowVerX, UpLowVerX, LowLowVerX, Rleft);
				}
				else if (UpLowVerX == -1 && LowLowVerX == -1)
				{
					Rleft = SameLowVerX;
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameLowVerX, UpLowVerX, LowLowVerX, Rleft);
				}

				// DOWN
				//cout << "DOWN\n";
				if (UpLowHorY != -1 && LowLowHorY != -1)
				{
					Rdown = max(max(SameLowHorY, UpLowHorY), LowLowHorY);
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameLowHorY, UpLowHorY, LowLowHorY, Rdown);
				}
				else if (UpLowHorY != -1 && LowLowHorY == -1)
				{
					Rdown = max(SameLowHorY, UpLowHorY);
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameLowHorY, UpLowHorY, LowLowHorY, Rdown);
				}
				else if (UpLowHorY == -1 && LowLowHorY != -1)
				{
					Rdown = max(SameLowHorY, LowLowHorY);
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameLowHorY, UpLowHorY, LowLowHorY, Rdown);
				}
				else if (UpLowHorY == -1 && LowLowHorY == -1)
				{
					Rdown = SameLowHorY;
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameLowHorY, UpLowHorY, LowLowHorY, Rdown);
				}

				// RIGHT
				//cout << "RIGHT\n";
				if (UpHighVerX != -1 && LowHighVerX != -1)
				{
					Rright = min(min(SameHighVerX, UpHighVerX), LowHighVerX);
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameHighVerX, UpHighVerX, LowHighVerX, Rright);
				}
				else if (UpHighVerX != -1 && LowHighVerX == -1)
				{
					Rright = min(SameHighVerX, UpHighVerX);
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameHighVerX, UpHighVerX, LowHighVerX, Rright);
				}
				else if (UpHighVerX == -1 && LowHighVerX != -1)
				{
					Rright = min(SameHighVerX, LowHighVerX);
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameHighVerX, UpHighVerX, LowHighVerX, Rright);
				}
				else if (UpHighVerX == -1 && LowHighVerX == -1)
				{
					Rright = SameHighVerX;
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameHighVerX, UpHighVerX, LowHighVerX, Rright);
				}

				// TOP
				//cout << "TOP\n";
				if (UpHighHorY != -1 && LowHighHorY != -1)
				{
					Rtop = min(min(SameHighHorY, UpHighHorY), LowHighHorY);
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameHighHorY, UpHighHorY, LowHighHorY, Rtop);
				}
				else if (UpHighHorY != -1 && LowHighHorY == -1)
				{
					Rtop = min(SameHighHorY, UpHighHorY);
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameHighHorY, UpHighHorY, LowHighHorY, Rtop);
				}
				else if (UpHighHorY == -1 && LowHighHorY != -1)
				{
					Rtop = min(SameHighHorY, LowHighHorY);
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameHighHorY, UpHighHorY, LowHighHorY, Rtop);
				}
				else if (UpHighHorY == -1 && LowHighHorY == -1)
				{
					Rtop = SameHighHorY;
					//printf("Same(%d) Up(%d) Low(%d) --> (%d)\n", SameHighHorY, UpHighHorY, LowHighHorY, Rtop);
				}

				I_Rect newIRECT = I_Rect(Rleft, Rdown, Rright, Rtop, layer);
				int newLayer = layer;
				if(newLayer == 1)
					newLayer++;

				// coor to map
				auto Lxp = fast_coor_map.at(newLayer-1).first.upper_bound(newIRECT.LB.first-1);
				auto Rxp = fast_coor_map.at(newLayer-1).first.lower_bound(newIRECT.RT.first+1);
				Rxp--;
				auto Byp = fast_coor_map.at(newLayer-1).second.upper_bound(newIRECT.LB.second-1);
				auto Typ = fast_coor_map.at(newLayer-1).second.lower_bound(newIRECT.RT.second+1);
				Typ--;
				/*
				printf("SetAllPinShapeOnLayout::PinShape(%d):Layer(%d) LB(%d,%d) RT(%d,%d) ==> LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", i, newLayer, CurrentPin.IRect_list[i].LB.first,
					   CurrentPin.IRect_list[i].LB.second, CurrentPin.IRect_list[i].RT.first, CurrentPin.IRect_list[i].RT.second,
					   newIRECT.LB.first - 1, newIRECT.LB.second - 1, newIRECT.RT.first + 1, newIRECT.RT.second + 1,
					   Lxp->first, Byp->first, Rxp->first, Typ->first,
					   Lxp->second, Byp->second, Rxp->second, Typ->second);
				*/
				if (Lxp->second > Rxp->second || Byp->second > Typ->second)
				{
				}
				else
				{
					/*
					printf("SetAllPinShapeOnLayout::PinShape(%d):Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", i, newLayer, CurrentPin.IRect_list[i].LB.first,
						CurrentPin.IRect_list[i].LB.second, CurrentPin.IRect_list[i].RT.first, CurrentPin.IRect_list[i].RT.second,
						Lxp->first, Byp->first, Rxp->first, Typ->first,
						Lxp->second, Byp->second, Rxp->second, Typ->second);
					*/

					pair<int, int> skip_LB = make_pair(Lxp->second, Byp->second);
					pair<int, int> skip_RB = make_pair(Rxp->second, Byp->second);
					pair<int, int> skip_LT = make_pair(Lxp->second, Typ->second);
					pair<int, int> skip_RT = make_pair(Rxp->second, Typ->second);
					/*
					printf("SetAllPinShapeOnLayout:: Skip_LB(%d,%d), Skip_RB(%d,%d), Skip_LT(%d,%d), Skip_RT(%d,%d)\n",
						   skip_LB.first, skip_LB.second, skip_RB.first, skip_RB.second, skip_LT.first, skip_LT.second, skip_RT.first, skip_RT.second);
					*/
					int x_offset = skip_RT.first - skip_LB.first + 1;
					int y_offset = skip_RT.second - skip_LB.second + 1;
					bool keep = false;
					if(x_offset * y_offset <= 4)
					{
						/*
						printf("SetAllPinShapeOnLayout::PinShape(%d):Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", i, newLayer, CurrentPin.IRect_list[i].LB.first,
							   CurrentPin.IRect_list[i].LB.second, CurrentPin.IRect_list[i].RT.first, CurrentPin.IRect_list[i].RT.second,
							   Lxp->first, Byp->first, Rxp->first, Typ->first,
							   Lxp->second, Byp->second, Rxp->second, Typ->second);
						printf("SetAllPinShapeOnLayout::Net(%d) Pin(%d) PinShape only has less than four pins on layout, it need to add special metal\n",
								net_index, pin_index);
						*/
						keep = true;
						//exit(-1);
					}



					int pin_shape_range = abs(_Typ->second - _Byp->second) * (_Rxp->second - _Lxp->second);
					if (pin_shape_range == 0)
						pin_shape_range = 1;

					

					for (int y = Byp->second; y <= Typ->second; y++)
					{
						for (int x = Lxp->second; x <= Rxp->second; x++)
						{
							if (!keep && (
								(x == skip_LB.first && y == skip_LB.second) || (x == skip_RB.first && y == skip_RB.second) ||
								(x == skip_LT.first && y == skip_LT.second) || (x == skip_RT.first && y == skip_RT.second)) )
							{
								continue;
							}

							//ISPD_original_blockage_array[layer - 1][y][x] = net_index; // net_index
							
								//printf("(%d,%d) Success -> %d\n", x, y, CongestionMap[newLayer - 1][y][x].first); // clear
							success_assign_counter++;

							layoutNode.push_back(RipUpNode(x, y, newLayer - 1));
							layoutNodeOriginal.push_back(RipUpNode(x, y, newLayer));
							if (CongestionMap[newLayer - 1][y][x].first == -1)
							{
								CongestionMap[newLayer - 1][y][x].first = net_index;
								CongestionMap[newLayer - 1][y][x].second += PINSHAPE_COVER_PENALTY / pin_shape_range;
							}
						}
					}
				}
			} // if, no found
		} // for IRect

		if (success_assign_counter == 0)
		{
			printf("SetAllPinShapeOnLayout::Net(%d) Num Of Pins is %d\n",
				   net_index, success_assign_counter);
			exit(-1);
		}

		if (out_of_bound_IRect == CurrentPin.IRect_list.size())
		{
			// out of bound memory
			CurrentPin.OutSideMemory = true;
		}

		if (inside_four_grid == CurrentPin.IRect_list.size())
		{
			//printf("inside_four_grid = %d\n", inside_four_grid);
			CurrentPin.InsideGridNode = true;

			//IntPair inside_pin_bound_LB;
			//IntPair inside_pin_bound_RT;
			//IntPair inside_pin_center
			//printf("CurrentPin.Layer %d\n",CurrentPin.Layer);
			// LB
			CurrentPin.IsolatedPinMetalSegmentLB.first.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1 , make_pair(inside_pin_bound_LB.first, inside_pin_bound_LB.second), 
																make_pair(inside_pin_center.first, inside_pin_bound_LB.second), 
																(inside_pin_bound_LB.first == inside_pin_center.first)? false: true));

			CurrentPin.IsolatedPinMetalSegmentLB.first.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1 , make_pair(inside_pin_center.first, inside_pin_bound_LB.second), 
																make_pair(inside_pin_center.first, inside_pin_center.second), 
																(inside_pin_center.first == inside_pin_center.first)? false: true));

			CurrentPin.IsolatedPinMetalSegmentLB.second.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1 , make_pair(inside_pin_bound_LB.first, inside_pin_bound_LB.second), 
																make_pair(inside_pin_bound_LB.first, inside_pin_center.second), 
																(inside_pin_bound_LB.first == inside_pin_bound_LB.first)? false: true));

			CurrentPin.IsolatedPinMetalSegmentLB.second.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1, make_pair(inside_pin_bound_LB.first, inside_pin_center.second), 
																make_pair(inside_pin_center.first, inside_pin_center.second), 
																(inside_pin_bound_LB.first == inside_pin_center.first)? false: true));

			// LT
			CurrentPin.IsolatedPinMetalSegmentLT.first.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1 , make_pair(inside_pin_bound_LB.first, inside_pin_center.second), 
																make_pair(inside_pin_bound_LB.first, inside_pin_bound_RT.second), 
																(inside_pin_bound_LB.first == inside_pin_bound_LB.first)? false: true));

			CurrentPin.IsolatedPinMetalSegmentLT.first.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1, make_pair(inside_pin_bound_LB.first, inside_pin_center.second), 
																make_pair(inside_pin_center.first, inside_pin_center.second), 
																(inside_pin_bound_LB.first == inside_pin_center.first)? false: true));

			CurrentPin.IsolatedPinMetalSegmentLT.second.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1 , make_pair(inside_pin_bound_LB.first, inside_pin_bound_RT.second), 
																make_pair(inside_pin_center.first, inside_pin_bound_RT.second), 
																(inside_pin_bound_LB.first == inside_pin_center.first)? false: true));

			CurrentPin.IsolatedPinMetalSegmentLT.second.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1, make_pair(inside_pin_center.first, inside_pin_center.second), 
																make_pair(inside_pin_center.first, inside_pin_bound_RT.second), 
																(inside_pin_center.first == inside_pin_center.first)? false: true));
			// RB
			CurrentPin.IsolatedPinMetalSegmentRB.first.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1, make_pair(inside_pin_center.first, inside_pin_bound_LB.second), 
																make_pair(inside_pin_bound_RT.first, inside_pin_bound_LB.second), 
																(inside_pin_center.first == inside_pin_bound_RT.first)? false: true));

			CurrentPin.IsolatedPinMetalSegmentRB.first.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1, make_pair(inside_pin_center.first, inside_pin_bound_LB.second), 
																make_pair(inside_pin_center.first, inside_pin_center.second), 
																(inside_pin_center.first == inside_pin_center.first)? false: true));

			CurrentPin.IsolatedPinMetalSegmentRB.second.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1, make_pair(inside_pin_bound_RT.first, inside_pin_bound_LB.second), 
																make_pair(inside_pin_bound_RT.first, inside_pin_center.second), 
																(inside_pin_bound_RT.first == inside_pin_bound_RT.first)? false: true));

			CurrentPin.IsolatedPinMetalSegmentRB.second.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1, make_pair(inside_pin_center.first, inside_pin_center.second), 
																make_pair(inside_pin_bound_RT.first, inside_pin_center.second), 
																(inside_pin_center.first == inside_pin_bound_RT.first)? false: true));
			// RT
			CurrentPin.IsolatedPinMetalSegmentRT.first.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1, make_pair(inside_pin_center.first, inside_pin_center.second), 
																make_pair(inside_pin_center.first, inside_pin_bound_RT.second), 
																(inside_pin_center.first == inside_pin_center.first)? false: true));

			CurrentPin.IsolatedPinMetalSegmentRT.first.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1, make_pair(inside_pin_center.first, inside_pin_bound_RT.second), 
																make_pair(inside_pin_bound_RT.first, inside_pin_bound_RT.second), 
																(inside_pin_center.first == inside_pin_bound_RT.first)? false: true));

			CurrentPin.IsolatedPinMetalSegmentRT.second.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1, make_pair(inside_pin_center.first, inside_pin_center.second), 
																make_pair(inside_pin_bound_RT.first, inside_pin_center.second), 
																(inside_pin_center.first == inside_pin_bound_RT.first)? false: true));

			CurrentPin.IsolatedPinMetalSegmentRT.second.push_back(wire_path(0, CurrentPin.IRect_list.at(0).Layer - 1, make_pair(inside_pin_bound_RT.first, inside_pin_center.second), 
																make_pair(inside_pin_bound_RT.first, inside_pin_bound_RT.second), 
																(inside_pin_bound_RT.first == inside_pin_bound_RT.first)? false: true));

		}

		if (net_index == debug_net_index && debug_mode)
		{
			printf("pause\n");
			int pause;
			cin >> pause;
		}
	}

}

// flute function
// use for traverse steiner tree and generate 2 pin net
bool pairCompare(const IntPair &firstElem, const IntPair &secondElem)
{
	return firstElem.first < secondElem.first;
}

void FLUTE_TwoPinNetGenerate(vector<IPSD_Routing_PIN> &PIN_list, map<pair<int, int>, int> &Pin2index, vector<IntPair> &PinList, vector<ISPD_2PinNet> &TwoPinNetList, vector<vector<RipUpNode> *> &Merged_path,
							 vector<int> &serial, I_Rect &sourcebound, I_Rect &targetbound, int &Net_id, bool &iroute_exist)
{
	//cout << "PIN NUM  =  " << PinList.size()<<endl;
	/*
	PinList.push_back(make_pair(1, 5));
	PinList.push_back(make_pair(2, 1));
	PinList.push_back(make_pair(3, 7));
	PinList.push_back(make_pair(4, 4));
	PinList.push_back(make_pair(6, 8));
	PinList.push_back(make_pair(7, 2));
	PinList.push_back(make_pair(8, 6));
	*/
	Tree flutetree;
	int x_array[PinList.size()];
	int y_array[PinList.size()];

	for (int xx = 0; xx < PinList.size(); xx++)
	{
		x_array[xx] = PinList.at(xx).first;
		y_array[xx] = PinList.at(xx).second;
		//cout << "x = " << x_array[xx] << "  y = " << y_array[xx]<<endl;
	}

	flutetree = flute(PinList.size(), x_array, y_array, ACCURACY);
	int tree_index = 0;

	vector<IntPair> cost2index;

	for(int a = 0 ; a < PinList.size() ; a++)
	{
		//cout << "NOW x = " << PinList.at(a).first << "  y = " << PinList.at(a).second << endl;
		int Cost_pin2root = 0;
		for (int i = 0; i < 2 * flutetree.deg - 2; i++)
		{
			bool out = false;
			if (flutetree.branch[i].x == PinList.at(a).first && flutetree.branch[i].y == PinList.at(a).second)
			{
				//cout << 2*flutetree.deg -2 << endl;
				int last = -1;
				while (true)
				{
					int ori_x = flutetree.branch[i].x;
					int ori_y = flutetree.branch[i].y;
					int end_x = flutetree.branch[flutetree.branch[i].n].x;
					int end_y = flutetree.branch[flutetree.branch[i].n].y;
					int x_offset = abs((ori_x - end_x));
					int y_offset = abs((ori_y - end_y));
					Cost_pin2root += x_offset;
					Cost_pin2root += y_offset;

					//printf("%d %d\n%d %d\n\n", ori_x, ori_y, end_x, end_y);
					i = flutetree.branch[i].n;
					if(last == i)
					{
						out = true;
						break;
					}
					last = i;
				} // while
			}// if

			if(out)
				break;
		}

		cost2index.push_back(make_pair(Cost_pin2root, a));
	} // for

	stable_sort(cost2index.begin(), cost2index.end(), pairCompare);

	vector<int> tempPreRoutedNet;

	map<int, bool> PairExists;

	//cout << "build flute 2 pin net " << iroute_exist << endl;
	//cout << "number = " << cost2index.size() << endl;

	int before_size = TwoPinNetList.size();

	int counter = 0, temp = -1 ;
	for(auto &index : cost2index)
	{
		//Serial.push_back(index.second);
		auto trueIndex = Pin2index.find(PinList.at(index.second));

		if(counter == 0 && !iroute_exist)
		{
			//cout << "0 >>> " << trueIndex->second<<endl;
			PairExists.emplace(trueIndex->second, true);
		}

		auto Pairexists = PairExists.find(trueIndex->second);

		if (trueIndex->second != -1 && Pairexists == PairExists.end())
		{
			ISPD_2PinNet new2PinNet;
			new2PinNet.clear();
			new2PinNet.IRoute_first = false;
			new2PinNet.Net_id = Net_id;
			

			if (!iroute_exist)
			{
				new2PinNet.TwoPinIndex = make_pair(trueIndex->second, temp); // swap!
				new2PinNet.TargetIRouteIndex = temp;
				new2PinNet.A_TargetBound = I_Rect(PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].x
											  , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].y
											  , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal.size()-1].x
											  , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal.size()-1].y
											  , -1);
				//printf("TargetBound Pin(%d,%d) (%d,%d)", new2PinNet.A_TargetBound.LB.first, new2PinNet.A_TargetBound.LB.second, new2PinNet.A_TargetBound.RT.first, new2PinNet.A_TargetBound.RT.second);

				/*
				cout << ">> " << trueIndex->second << "  " << temp << " target : " << new2PinNet.TargetIRouteIndex << endl;
				printf("-> (%d,%d) -> (%d,%d) (%d,%d) (%d)\n", PIN_list.at(trueIndex->second).LayoutNodeOriginal[0].x
															   , PIN_list.at(trueIndex->second).LayoutNodeOriginal[0].y
															   , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].x
															   , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].y
															   , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal.size()-1].x
															   , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal.size()-1].y
															   , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].z);
				*/
			}
			else
			{
				//cout << "counter = " << counter << endl;
				new2PinNet.TwoPinIndex = make_pair(trueIndex->second, -1); // swap!
				new2PinNet.TargetIRouteIndex = PIN_list.at(trueIndex->second).target_index;

				new2PinNet.A_TargetBound = I_Rect(PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].x
											  , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].y
											  , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal.size()-1].x
											  , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal.size()-1].y
											  , -1);
				//printf("TargetBound IRoute2(%d,%d) (%d,%d)", new2PinNet.A_TargetBound.LB.first, new2PinNet.A_TargetBound.LB.second, new2PinNet.A_TargetBound.RT.first, new2PinNet.A_TargetBound.RT.second);

				//cout << ">> " << trueIndex->second << "  " << -1 << " target : " << new2PinNet.TargetIRouteIndex << endl;
				/*
				printf("-> (%d,%d) -> (%d,%d) (%d,%d) (%d)\n", PIN_list.at(trueIndex->second).LayoutNodeOriginal[0].x
															   , PIN_list.at(trueIndex->second).LayoutNodeOriginal[0].y
															   , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].x
															   , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].y
															   , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal.size()-1].x
															   , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal.size()-1].y
															   , PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].z);
				*/
			}

			PairExists.emplace(trueIndex->second, true);

			new2PinNet.SourceBound.push_back(sourcebound);
			//printf("SourceBound (%d,%d) (%d,%d)\n", sourcebound.LB.first, sourcebound.LB.second, sourcebound.RT.first, sourcebound.RT.second);
			new2PinNet.TargetBound.push_back(targetbound);
			//printf("targetbound (%d,%d) (%d,%d)\n", targetbound.LB.first, targetbound.LB.second, targetbound.RT.first, targetbound.RT.second);
			new2PinNet.PreRoutedNet.resize(tempPreRoutedNet.size());
			new2PinNet.subtree2subtree = false;
			for(int net_index = 0 ; net_index < tempPreRoutedNet.size() ; net_index++)
			{
				new2PinNet.PreRoutedNet.at(net_index) = tempPreRoutedNet.at(net_index);
				//cout << " >> " << new2PinNet.PreRoutedNet.at(net_index) <<endl;
			}
			TwoPinNetList.push_back(new2PinNet);
			tempPreRoutedNet.push_back(TwoPinNetList.size()-1) ;
			/*
			if(sourcebound.LB.first == 78000 && sourcebound.LB.second == 85500 && sourcebound.RT.first == 84000 && sourcebound.RT.second == 91200)
			{
				cout << "erere\n";
				for (int a = 0; a < PIN_list.at(trueIndex->second).IRect_list.size(); a++)
				{
					printf("src rect(%d,%d) (%d,%d)\n", PIN_list.at(trueIndex->second).IRect_list.at(a).LB.first, PIN_list.at(trueIndex->second).IRect_list.at(a).LB.second,
						   PIN_list.at(trueIndex->second).IRect_list.at(a).RT.first, PIN_list.at(trueIndex->second).IRect_list.at(a).RT.second);
				}
				for (int a = 0; a < PIN_list.at(temp).IRect_list.size(); a++)
				{
					printf("tar rect(%d,%d) (%d,%d)\n", PIN_list.at(temp).IRect_list.at(a).LB.first, PIN_list.at(temp).IRect_list.at(a).LB.second,
						   PIN_list.at(temp).IRect_list.at(a).RT.first, PIN_list.at(temp).IRect_list.at(a).RT.second);
				}
				//exit(9);
			}
			*/
			Merged_path.push_back(&new2PinNet.RipUp_path); // add reference pointer
		}

		
		if(temp != trueIndex->second && trueIndex->second != -1)
		{
			//tempPreRoutedNet.push_back(trueIndex->second);
			temp = trueIndex->second;
			//serial.push_back(trueIndex->second);
			counter++;
		}
		
		
	}

	int after_size = TwoPinNetList.size();
	
	/*
	cout <<"add " << after_size - before_size << endl;
	int top;
	cin>>top;
	*/
	
}

int distance_to_Line(IntPair line_start, IntPair line_end, IntPair point)
{
	bool horizontal = line_start.second == line_end.second;

	if (horizontal)
	{
		if (line_start.first <= point.first && line_end.first >= point.first)
		{
			return abs(line_start.second - point.second);
		}
		else{
			int x = min(abs(line_start.first - point.first), abs(point.first - line_end.first));
			int y = abs(line_start.second - point.second);
			return (x + y);
		}
	}
	else{  // vertical
		if (line_start.second <= point.second && line_end.second >= point.second)
		{
			return abs(line_start.first - point.first);
		}
		else
		{
			int x = abs(line_start.first - point.first);
			int y = min(abs(line_start.second - point.second), abs(point.second - line_end.second));
			return (x + y);
		}
	}
}

bool In_Subtree(set<int> &_subtree_,int id){
	auto find_subtree = _subtree_.find(id);
	return (find_subtree != _subtree_.end());
}

void PinShapeBox(vector <RipUpNode> &PinShapeNode,I_Rect &TargetBox){

	TargetBox.LB.first = INT_MAX;
	TargetBox.LB.second = INT_MAX;
	TargetBox.RT.first = -1;
	TargetBox.RT.second = -1;
	for (int i = 0; i < PinShapeNode.size(); i++)
	{
		int x = PinShapeNode[i].x;
		int y = PinShapeNode[i].y;

		if (x < TargetBox.LB.first){
			TargetBox.LB.first = x;
		}
		if (x > TargetBox.RT.first)
		{
			TargetBox.RT.first = x;
		}
		if (y < TargetBox.LB.second)
		{
			TargetBox.LB.second = y;
		}
		if (y > TargetBox.RT.second)
		{
			TargetBox.RT.second = y;
		}
	}
}

void Local_MST_construction(vector<IntPair> &SinglePinList, map<pair<int, int>, int> &Pin2index,
							vector<pair<I_Rect, int>> &IRouteSegList, int net_id, ISPD_Routing_Net &tempRNet)
{

	vector < int > _IRoute_MinDis_ID;
	vector < int > _Pin_MinDis_ID;
	vector <bool> _ConnectedObject; // true:iroute , false:pin
	set<int> _subtree_;

	_IRoute_MinDis_ID.resize(SinglePinList.size());
	_Pin_MinDis_ID.resize(SinglePinList.size());
	_ConnectedObject.resize(SinglePinList.size());

	vector<IntPair> TempSinglePinList;
	for (int i = 0; i < SinglePinList.size(); i++){
		bool same_coor = false;
		for(int j =0; j < TempSinglePinList.size();j++){
			if(SinglePinList[i].first == TempSinglePinList[j].first 
			&& SinglePinList[i].second == TempSinglePinList[j].second){
				same_coor = true;
				break;
			}	
		}
		if(!same_coor)
			TempSinglePinList.push_back(SinglePinList[i]);
	}
	SinglePinList.clear();
	for(int i = 0; i < TempSinglePinList.size(); i++){
		SinglePinList.push_back(TempSinglePinList[i]);
	}

	bool IRoute_Open = true;
	for (int i = 0; i < SinglePinList.size(); i++)
	{
		bool this_pin_in_subtree = In_Subtree(_subtree_,i);
		int dis = INT_MAX;
		for (int iroute = 0; iroute < IRouteSegList.size(); iroute++){
			int distance = distance_to_Line(IRouteSegList[iroute].first.LB, IRouteSegList[iroute].first.RT, SinglePinList[i]);
			if(distance < dis){
				_ConnectedObject[i] = true;
				_IRoute_MinDis_ID[i] = iroute;
				dis = distance;
			}
		}
		for (int pin = 0; pin < SinglePinList.size(); pin++)
		{
			if (i == pin)
				continue;
			int distance = abs(SinglePinList[i].first - SinglePinList[pin].first) + abs(SinglePinList[i].second - SinglePinList[pin].second);

			if (distance < dis && !(this_pin_in_subtree & In_Subtree(_subtree_,pin)))
			{
				_ConnectedObject[i] = false;
				_Pin_MinDis_ID[i] = pin;
				dis = distance;
			}
		}

		if (_ConnectedObject[i] == false){
			_subtree_.insert(i);
			_subtree_.insert(_Pin_MinDis_ID[i]);
		}
		else{
			IRoute_Open = false;
		}
	}

	vector<int> tempPreRoutedNet;
	tempPreRoutedNet.clear();
	set<int> current_subtree;
	// 1. connect to IRoute
	for (int i = 0; i < SinglePinList.size(); i++){
		if (_ConnectedObject[i]==true){
			ISPD_2PinNet new2PinNet;
			new2PinNet.clear();
			new2PinNet.IRoute_first = false;
			new2PinNet.subtree2subtree = false;

			// Net id
			new2PinNet.Net_id = net_id;
			// source target
			auto pin_map = Pin2index.find(make_pair(SinglePinList[i].first, SinglePinList[i].second));
			new2PinNet.TwoPinIndex = make_pair(pin_map->second, -1);// swap!

			// A* boundary
			new2PinNet.A_TargetBound = IRouteSegList[_IRoute_MinDis_ID[i]].first;

			// For R&R structure
			new2PinNet.PreRoutedNet.resize(tempPreRoutedNet.size());
			for (int net_index = 0; net_index < tempPreRoutedNet.size(); net_index++)
			{
				new2PinNet.PreRoutedNet.at(net_index) = tempPreRoutedNet.at(net_index);
			}
			tempPreRoutedNet = new2PinNet.PreRoutedNet;
			// push into net
			tempRNet.TwoPinNetList.push_back(new2PinNet); // push back new 2pin net

			current_subtree.insert(i);
		}
	}

	if (IRouteSegList.size() == 0){
		if (SinglePinList.size() < 2){
			printf("Only 1 pin shape\n");
			exit(1);
		}
		ISPD_2PinNet new2PinNet;
		new2PinNet.clear();
		new2PinNet.IRoute_first = false;
		new2PinNet.subtree2subtree = false;

		// Net id
		new2PinNet.Net_id = net_id;
		// source target
		auto pin_map1 = Pin2index.find(make_pair(SinglePinList[0].first, SinglePinList[0].second));
		auto pin_map2 = Pin2index.find(make_pair(SinglePinList[1].first, SinglePinList[1].second));
		new2PinNet.TwoPinIndex = make_pair(pin_map1->second, pin_map2->second); // swap!

		int Target_id = pin_map2->second;
		I_Rect Target_Box;
		PinShapeBox(tempRNet.PIN_list[Target_id].LayoutNodeOriginal, Target_Box);
		// A* boundary
		new2PinNet.A_TargetBound = Target_Box;

		// For R&R structure
		new2PinNet.PreRoutedNet.resize(tempPreRoutedNet.size());
		for (int net_index = 0; net_index < tempPreRoutedNet.size(); net_index++)
		{
			new2PinNet.PreRoutedNet.at(net_index) = tempPreRoutedNet.at(net_index);
		}
		tempPreRoutedNet = new2PinNet.PreRoutedNet;
		// push into net
		tempRNet.TwoPinNetList.push_back(new2PinNet); // push back new 2pin net
		current_subtree.insert(0);
		current_subtree.insert(1);
	}

	int cur_size = current_subtree.size();
	// BFS searching Node
	while (current_subtree.size() != SinglePinList.size())
	{
		cur_size = current_subtree.size();
		for (int i = 0; i < SinglePinList.size(); i++)
		{
			if(In_Subtree(current_subtree, i)) continue;
			int pin_id = _Pin_MinDis_ID[i];
			if (In_Subtree(current_subtree, pin_id)){
				ISPD_2PinNet new2PinNet;
				new2PinNet.clear();
				new2PinNet.IRoute_first = false;
				new2PinNet.subtree2subtree = false;

				// Net id
				new2PinNet.Net_id = net_id;
				// source target
				auto pin_map = Pin2index.find(make_pair(SinglePinList[i].first, SinglePinList[i].second));
				new2PinNet.TwoPinIndex = make_pair(pin_map->second, -1); // swap!
				printf("id(%d)-coor(%d,%d)-pair(%d,%d)\n", i ,SinglePinList[i].first, SinglePinList[i].second,pin_map->second, -1);
				int Target_id = -1;

				auto target_pin = Pin2index.find(make_pair(SinglePinList[pin_id].first, SinglePinList[pin_id].second));
				Target_id = target_pin->second;

				I_Rect Target_Box;
				PinShapeBox(tempRNet.PIN_list[Target_id].LayoutNodeOriginal, Target_Box);
				// A* boundary
				new2PinNet.A_TargetBound = Target_Box;

				// For R&R structure
				new2PinNet.PreRoutedNet.resize(tempPreRoutedNet.size());
				for (int net_index = 0; net_index < tempPreRoutedNet.size(); net_index++)
				{
					new2PinNet.PreRoutedNet.at(net_index) = tempPreRoutedNet.at(net_index);
				}
				tempPreRoutedNet = new2PinNet.PreRoutedNet;
				// push into net
				tempRNet.TwoPinNetList.push_back(new2PinNet); // push back new 2pin net
				current_subtree.insert(i);
			}
		}
		if (cur_size == current_subtree.size()){
			printf("Connectivity Error Net(%d)\n",net_id);
			if (IRoute_Open)
			{
				printf("IRoute Open IRouteSegList size(%d)\n", IRouteSegList.size());
			}
			exit(1);
		}
	}
	//printf("\n");
}

void NetConstruction(Parser::Design &design,
					 vector<ISPD_GridMap> &GridMap_layout,
					 Real_Coor_Table &real_coor_table,
					 FastCoorMap &fast_coor_map,
					 vector<TwoPinRipUpNet> &two_pin_rip_up_net,
					 TRACKASSIGNMENT::IRouteGenerator &iroute_generator,
					 vector<vector<vector<int>>> &ISPD_original_blockage_array)
{
	// flute read look up table
	readLUT();
	// make track layer to integer
	for(int i = 0 ; i < design.Track_list.size() ; i++)
	{
		string tempLayer = design.Track_list.at(i).Layer;
		tempLayer.erase(tempLayer.begin(),tempLayer.begin()+5);
		design.Track_list.at(i).i_layer = atoi(tempLayer.c_str());
		//printf("NetConstruction::Transfer Track layer into integer(%d)\n", design.Track_list.at(i).i_layer);
	}

	// extract pin shape into map pin
	for (int i = iroute_generator.design_information.routing_net_information.size()-1; i >= 0; i--)
	{
		int pin_counter = 0;
		vector<IntPair> SinglePinList;
		map<pair<int, int>, int> Pin2index;
		int current_net_index = i;
		ISPD_Routing_Net &tempRNet = design.ispd_routing_net.at(current_net_index);

		vector <int> tempIRouteList;
		// iroute first
		for (int j = 0; j < iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Guide.size(); j++)
		{
			//printf("Layer %d : \n\n", j);
			//printf("Guide:\n\n");
			
			for (int k = 0; k < iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Guide[j].size(); k++)
			{
				TRACKASSIGNMENT::Guide_Component &guide = iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Guide[j].at(k);

				if(!NoTrackAssignment)
				{
					vector<IntPair> tempIROUTE;
					for (int l = 0; l < guide.Original_IRoute_list.size(); l++)
					{
						TRACKASSIGNMENT::IRoute &thisIR = guide.Original_IRoute_list.at(l);
						// add to IPSD_Routing_PIN
						// LB
						IPSD_Routing_PIN tempISPD_Routing_PIN;
						tempISPD_Routing_PIN.IsIRoutePath = IAMIROUTE; // for output use
						tempISPD_Routing_PIN.Original_MARCO.Name = "IRoute";
						tempISPD_Routing_PIN.type = false;
						tempISPD_Routing_PIN.pseudo = false;
						tempISPD_Routing_PIN.IRect_list.push_back(I_Rect(thisIR.LB.first, thisIR.LB.second, thisIR.RT.first, thisIR.RT.second, guide.Layer));
						// tempISPD_Routing_PIN.SteinerTreePin = make_pair(thisIR.LB.first, thisIR.LB.second);

						if (thisIR.LB.first > thisIR.RT.first)
						{
							int tempInt = thisIR.LB.first;
							thisIR.LB.first = thisIR.RT.first;
							thisIR.RT.first = tempInt;
						}
						if (thisIR.LB.second > thisIR.RT.second)
						{
							int tempInt = thisIR.LB.second;
							thisIR.LB.second = thisIR.RT.second;
							thisIR.RT.second = tempInt;
						}

						bool same = false; // 0:x 1:y
						if (thisIR.LB.first == thisIR.RT.first)
						{
							same = false; // x same, vertical
						}
						else
							same = true; // y same, horizontal

						if (!same)
						{
							auto Lxp = fast_coor_map.at(guide.Layer - 1).first.find(thisIR.LB.first);
							auto Rxp = fast_coor_map.at(guide.Layer - 1).first.find(thisIR.RT.first);
							auto Byp = fast_coor_map.at(guide.Layer - 1).second.upper_bound(thisIR.LB.second - 1);
							auto Typ = fast_coor_map.at(guide.Layer - 1).second.lower_bound(thisIR.RT.second + 1);
							Typ--;
							/*
								printf("NetConstruction::Ver IRoute : Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", guide.Layer, thisIR.LB.first,
									thisIR.LB.second, thisIR.RT.first, thisIR.RT.second,
									Lxp->first, Byp->first, Rxp->first, Typ->first,
									Lxp->second, Byp->second, Rxp->second, Typ->second);
								*/

							if (Byp->second > Typ->second)
							{
								printf("NetConstruction::IRoute didn't map on layout\n");
								printf("NetConstruction::IRoute : Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", guide.Layer, thisIR.LB.first,
									   thisIR.LB.second, thisIR.RT.first, thisIR.RT.second,
									   Lxp->second, Byp->second, Rxp->second, Typ->second);
								exit(-1);
							}
							else
							{

								for (int y = Byp->second; y <= Typ->second; y++)
								{
									for (int x = Lxp->second; x <= Rxp->second; x++)
									{
										tempISPD_Routing_PIN.LayoutNodeOriginal.push_back(RipUpNode(x, y, guide.Layer));
									}
								}

								tempRNet.PIN_list.push_back(tempISPD_Routing_PIN);
								tempRNet.IRoute_PIN_list.push_back(tempRNet.PIN_list.size() - 1);
								thisIR.List_index = tempRNet.PIN_list.size() - 1;
								tempIRouteList.push_back(tempRNet.PIN_list.size() - 1);
								// RT
							} // else
						}
						else
						{
							auto Lxp = fast_coor_map.at(guide.Layer - 1).first.upper_bound(thisIR.LB.first - 1);
							auto Rxp = fast_coor_map.at(guide.Layer - 1).first.lower_bound(thisIR.RT.first + 1);
							auto Byp = fast_coor_map.at(guide.Layer - 1).second.find(thisIR.LB.second);
							auto Typ = fast_coor_map.at(guide.Layer - 1).second.find(thisIR.RT.second);
							Rxp--;
							/*
								printf("NetConstruction::Hor IRoute : Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", guide.Layer, thisIR.LB.first,
									thisIR.LB.second, thisIR.RT.first, thisIR.RT.second,
									Lxp->first, Byp->first, Rxp->first, Typ->first,
									Lxp->second, Byp->second, Rxp->second, Typ->second);
								*/

							
							if (Lxp->second > Rxp->second)
							{
								printf("NetConstruction::IRoute didn't map on layout\n");
								printf("NetConstruction::IRoute : Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", guide.Layer - 1, thisIR.LB.first,
									   thisIR.LB.second, thisIR.RT.first, thisIR.RT.second,
									   Lxp->second, Byp->second, Rxp->second, Typ->second);
								exit(-1);
							}
							else
							{

								for (int y = Byp->second; y <= Typ->second; y++)
								{
									for (int x = Lxp->second; x <= Rxp->second; x++)
									{
										tempISPD_Routing_PIN.LayoutNodeOriginal.push_back(RipUpNode(x, y, guide.Layer));
									}
								}

								tempRNet.PIN_list.push_back(tempISPD_Routing_PIN);
								tempRNet.IRoute_PIN_list.push_back(tempRNet.PIN_list.size() - 1);
								thisIR.List_index = tempRNet.PIN_list.size() - 1;
								tempIRouteList.push_back(tempRNet.PIN_list.size() - 1);
								// RT
							} // else
						}
					}
				} // if, track assignment
			}
		} // for
		// end, iroute first

		if(tempIRouteList.size() < 2)
		{
			tempRNet.IRoute_first = false;
			continue;
		}
		else
		{
			tempRNet.IRoute_first = true;
		}

		vector<int> iroute_net_list;
		int num_of_iroute = tempIRouteList.size();
		vector<bool> VisitedIRouteList;
		vector<vector<int>> distance_map;
		VisitedIRouteList.resize(num_of_iroute);
		distance_map.resize(num_of_iroute);
		for( int j = 0 ; j < tempIRouteList.size() ; j++)
		{
			VisitedIRouteList.at(j) = false;
			distance_map.at(j).resize(num_of_iroute);
			//TRACKASSIGNMENT::line_distance();
		}

		for( int j = 0 ; j < tempIRouteList.size() ; j++)
		{
			for( int k = 0 ; k < tempIRouteList.size() ; k++)
			{
				if(k < j)
				{
					distance_map[j][k] = distance_map[k][j];
					continue;
				}

				if(k == j)
				{
					distance_map[j][k] = 0;
				}

				int j_index = tempIRouteList.at(j);
				int k_index = tempIRouteList.at(k);

				int j_size = tempRNet.PIN_list.at(j_index).LayoutNodeOriginal.size();
				int k_size = tempRNet.PIN_list.at(k_index).LayoutNodeOriginal.size();

				I_Rect RectJ = I_Rect(tempRNet.PIN_list.at(j_index).LayoutNodeOriginal.at(0).x,
									  tempRNet.PIN_list.at(j_index).LayoutNodeOriginal.at(0).y,
									  tempRNet.PIN_list.at(j_index).LayoutNodeOriginal.at(j_size - 1).x,
									  tempRNet.PIN_list.at(j_index).LayoutNodeOriginal.at(j_size - 1).y, -1);
				I_Rect RectK = I_Rect(tempRNet.PIN_list.at(k_index).LayoutNodeOriginal.at(0).x,
									  tempRNet.PIN_list.at(k_index).LayoutNodeOriginal.at(0).y,
									  tempRNet.PIN_list.at(k_index).LayoutNodeOriginal.at(k_size - 1).x,
									  tempRNet.PIN_list.at(k_index).LayoutNodeOriginal.at(k_size - 1).y, -1);
				int distance_of_iroute = TRACKASSIGNMENT::line_distance(RectJ, RectK);
				distance_map[j][k] = distance_of_iroute;
			}
		}

		VisitedIRouteList.at(0) = true;
		iroute_net_list.push_back(tempIRouteList.at(0));
		vector<int> target_bound;

		int counter = 0;

		while(true)
		{
			int last_j = -1;
			int last_visit = -1;
			int min_dis = INT_MAX;
			if(counter == tempIRouteList.size()-1 || tempIRouteList.size() == 0)
				break;

			for (int j = 0; j < tempIRouteList.size(); j++)
			{
				if (!VisitedIRouteList.at(j))
				{
					continue;
				}
				else
				{
					for (int k = 0; k < tempIRouteList.size(); k++)
					{
						if (j != k && !VisitedIRouteList.at(k) && min_dis >= distance_map[j][k])
						{
							min_dis = distance_map[j][k];
							last_visit = k;
							last_j = j;
						}
					}
				}
			}

			if(counter == 0)
			{
				target_bound.push_back(last_visit);
			}
			else
			{
				target_bound.push_back(last_j);
			}

			iroute_net_list.push_back(tempIRouteList.at(last_visit));
			VisitedIRouteList.at(last_visit) = true;
			counter++;
		}

		for (int index_iroute = 0; index_iroute < VisitedIRouteList.size(); index_iroute++)
		{
			if (!VisitedIRouteList.at(index_iroute))
			{
				cout << "There are iroutes not visited." << endl;
				exit(1);
			}
		}

		vector<int> tempPreRoutedNet;
		// start build iroute 2-pin net
		for (int index_iroute = 1; index_iroute < iroute_net_list.size() ; index_iroute++)
		{
			ISPD_2PinNet new2PinNet;
			new2PinNet.clear();

			new2PinNet.TargetIRouteIndex = tempIRouteList.at( target_bound.at(index_iroute - 1) );
			new2PinNet.IRoute_first = true;
			new2PinNet.Net_id = i;
			new2PinNet.TwoPinIndex = make_pair(iroute_net_list.at(index_iroute), iroute_net_list.at(index_iroute - 1)); // swap!
			new2PinNet.A_TargetBound = I_Rect(tempRNet.PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].x
											  , tempRNet.PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].y
											  , tempRNet.PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[tempRNet.PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal.size()-1].x
											  , tempRNet.PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[tempRNet.PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal.size()-1].y
											  , -1);

			//printf("TargetBound IRoute(%d,%d) (%d,%d)\n", new2PinNet.A_TargetBound.LB.first, new2PinNet.A_TargetBound.LB.second, new2PinNet.A_TargetBound.RT.first, new2PinNet.A_TargetBound.RT.second);
			//cout << ">> " << iroute_net_list.at(index_iroute) << "  " <<  iroute_net_list.at(index_iroute - 1) << " Target : " <<new2PinNet.TargetIRouteIndex<<endl;
			new2PinNet.PreRoutedNet.resize(tempPreRoutedNet.size());
			new2PinNet.subtree2subtree = false;
			for (int net_index = 0; net_index < tempPreRoutedNet.size(); net_index++)
			{
				new2PinNet.PreRoutedNet.at(net_index) = tempPreRoutedNet.at(net_index);
			}
			tempRNet.TwoPinNetList.push_back(new2PinNet);
			tempPreRoutedNet.push_back(tempRNet.TwoPinNetList.size() - 1);

			tempRNet.Merged_path.push_back(&new2PinNet.RipUp_path); // add reference pointer
		}
		
	}

	for (int i = iroute_generator.design_information.routing_net_information.size() - 1; i >= 0; i--)
	{
		int pin_counter = 0;
		vector<IntPair> SinglePinList;
		map<pair<int, int>, int> Pin2index;
		int current_net_index = i;

		ISPD_Routing_Net &tempRNet = design.ispd_routing_net.at(current_net_index);

		vector<pair<I_Rect, int> > IRouteSegList;
		if (!NoTrackAssignment)
		{
			TRACKASSIGNMENT::Routing_Net_Information &tempRNetInfo = iroute_generator.design_information.routing_net_information.at(current_net_index);
			cout << "---------net:  " << design.ispd_routing_net[i].Original_Data.Net_name << "--------\n\n";

			// setting up guide table
			int numoflayer = tempRNetInfo.Layer_Array_for_Guide.size();
			int numofguide = -1;
			int numofguides = 0;
			for (int j = 0; j < tempRNetInfo.Layer_Array_for_Guide.size(); j++)
			{
				if (numofguide < (int)tempRNetInfo.Layer_Array_for_Guide.at(j).size())
				{
					numofguide = (int)tempRNetInfo.Layer_Array_for_Guide.at(j).size();
				}
				numofguides += (int)tempRNetInfo.Layer_Array_for_Guide.at(j).size();
			}
			printf("Number of guides : %d  %d\n", numofguides, numofguide);
			// ignore this case
			if (numofguides == 0)
				continue;

			vector<vector<bool>> visited_guide_table;
			visited_guide_table.resize(numoflayer);

			int guide_layer;
			int guide_index;
			bool firsttime = true;

			for (int j = 0; j < (int)tempRNetInfo.Layer_Array_for_Guide.size(); j++)
			{
				visited_guide_table.at(j).resize(numofguide);
				for (int k = 0; k < numofguide; k++)
				{
					visited_guide_table.at(j).at(k) = false;
				}

				if (firsttime && iroute_generator.design_information.routing_net_information[current_net_index].Layer_Array_for_Guide[j].size() != 0)
				{
					guide_layer = j;
					guide_index = 0;
					firsttime = false;
					cout << "j = " << j << " k = " << 0 << endl;
				}
			}

			printf("guide : (%d,%d)\n", guide_layer, guide_index);
			vector<IntPair> Traversal_queue;

			TRACKASSIGNMENT::Guide_Component &guide = iroute_generator.design_information.routing_net_information[current_net_index].Layer_Array_for_Guide[guide_layer].at(guide_index);

			visited_guide_table[guide_layer][guide_index] = true;

			guide.visited = true;

			for (auto &same_neighbor : guide.SameLayerList)
			{
				if (visited_guide_table[guide.Layer][same_neighbor] == false)
					Traversal_queue.push_back(make_pair(guide.Layer, same_neighbor));
				visited_guide_table[guide.Layer][same_neighbor] = true;
				//printf("add same guide : (%d,%d)\n", guide.Layer, same_neighbor);
			}
			for (auto &low_neighbor : guide.LowerLayerList)
			{
				Traversal_queue.push_back(make_pair(guide.Layer - 1, low_neighbor));
				visited_guide_table[guide.Layer - 1][low_neighbor] = true;
				//printf("add low guide : (%d,%d)\n", guide.Layer - 1, low_neighbor);
			}

			for (auto &up_neighbor : guide.UpperLayerList)
			{
				Traversal_queue.push_back(make_pair(guide.Layer + 1, up_neighbor));
				visited_guide_table[guide.Layer + 1][up_neighbor] = true;
				//printf("add up guide : (%d,%d)\n", guide.Layer + 1, up_neighbor);
			}

			// MAIN for
			for (int index = 0; index < Traversal_queue.size(); index++)
			{
				// find iroute
				int now_guide_layer = Traversal_queue.at(index).first;
				int now_guide_index = Traversal_queue.at(index).second;
				TRACKASSIGNMENT::Guide_Component &now_guide = tempRNetInfo.Layer_Array_for_Guide.at(now_guide_layer).at(now_guide_index);

				//printf("Guide(%f,%f) (%f,%f) Layer(%d)\n", now_guide.LB.first, now_guide.LB.second, now_guide.RT.first, now_guide.RT.second, now_guide.Layer);
				for (int iroute_index = 0; iroute_index < now_guide.Original_IRoute_list.size(); iroute_index++)
				{
					int target_iroute = now_guide.Original_IRoute_list.at(iroute_index).List_index;

					// ===================
					IRouteSegList.push_back(make_pair(tempRNet.PIN_list.at(target_iroute).IRect_list.at(0), target_iroute)); // LB RT : line segment
				}

				for (auto &same_neighbor : now_guide.SameLayerList)
				{
					if (visited_guide_table[now_guide.Layer][same_neighbor] == false)
					{
						Traversal_queue.push_back(make_pair(now_guide.Layer, same_neighbor));
						visited_guide_table[now_guide.Layer][same_neighbor] = true;
						//printf("add same guide : (%d,%d)\n", now_guide.Layer, same_neighbor);
					}
				}
				// add next guide
				for (auto &low_neighbor : now_guide.LowerLayerList)
				{
					//printf(">> guide : (%d,%d)\n", now_guide.Layer - 1, low_neighbor);
					//cout << "BOOL = " << visited_guide_table[now_guide.Layer - 1][low_neighbor]<<endl;
					if (visited_guide_table[now_guide.Layer - 1][low_neighbor] == false)
					{
						Traversal_queue.push_back(make_pair(now_guide.Layer - 1, low_neighbor));
						visited_guide_table[now_guide.Layer - 1][low_neighbor] = true;
						//printf("add low guide : (%d,%d)\n", now_guide.Layer - 1, low_neighbor);
					}
				}

				for (auto &up_neighbor : now_guide.UpperLayerList)
				{
					//printf(">> guide : (%d,%d)\n", now_guide.Layer + 1, up_neighbor, visited_guide_table[now_guide.Layer + 1][up_neighbor]);
					//cout << "BOOL = " << visited_guide_table[now_guide.Layer + 1][up_neighbor] << endl;
					if (visited_guide_table[now_guide.Layer + 1][up_neighbor] == false)
					{
						Traversal_queue.push_back(make_pair(now_guide.Layer + 1, up_neighbor));
						visited_guide_table[now_guide.Layer + 1][up_neighbor] = true;
						//printf("add up guide : (%d,%d)\n", now_guide.Layer + 1, up_neighbor);
					}
				}
			} // for
		}

		//cout << "---------net:  " << design.ispd_routing_net[i].Original_Data.Net_name << "--------\n\n";
		for (int j = 0; j < iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Guide.size(); j++)
		{
			//printf("Layer %d : \n\n", j);
			//printf("Guide:\n\n");
			
			for (int k = 0; k < iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Guide[j].size(); k++)
			{
				TRACKASSIGNMENT::Guide_Component &guide = iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Guide[j].at(k);

				/*
				printf("NetConstruction::Guide LB (%f, %f), RT (%f, %f) Layer = %d, Guide id = %d\n",
					   guide.LB.first, guide.LB.second, guide.RT.first, guide.RT.second, guide.Layer, guide.guide_index);
				printf("size = %d\n", iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Pin[j].size());
				*/
				// collect pin in this guide
				for (int l = 0; l < iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Pin[j].size(); l++)
				{					
					TRACKASSIGNMENT::Big_Pin &BPin = iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Pin[j].at(l);
					
					if(TRACKASSIGNMENT::rec_overlap(guide.LB, guide.RT, BPin.LB, BPin.RT))
					{
						int current_pin_index = BPin.pin_index;
						//printf("NetConstruction::Pin index(%d)\n", current_pin_index);

						// if overlap, extract a pin to Pin single-pin list
						vector<IntPair> tempPinList;

						int centX = 0, centY = 0, counter = 0;
						IPSD_Routing_PIN &Pin = tempRNet.PIN_list.at(current_pin_index);

						//Pin.target_index = target_iroute;

						for (int m = 0; m < Pin.LayoutNodeOriginal.size(); m++)
						{
							int __x = Pin.LayoutNodeOriginal.at(m).x;
							int __y = Pin.LayoutNodeOriginal.at(m).y;
							int __z = Pin.LayoutNodeOriginal.at(m).z;
							// all node in this pin
							centX += __x;
							centY += __y;
							counter++;
							tempPinList.push_back(make_pair(__x, __y));

						} // for, LayoutNodeOriginal

						// extract a target pin from tempPinList
						//printf("%d / %d\n", centX, counter);
						//printf("%d / %d\n", centY, counter);
						if(counter != 0)
						{
							centX = centX / counter;
							centY = centY / counter;
						}

						int MinOffset = INT_MAX;
						int FinalX = -1, FinalY = -1;
						for(auto &NodePair : tempPinList)
						{							
							int tempX = NodePair.first - centX;
							int tempY = NodePair.second - centY;
							int tempOffset = abs(tempX) + abs(tempY);
							if(tempOffset < MinOffset)
							{
								FinalX = NodePair.first;
								FinalY = NodePair.second;
								MinOffset = tempOffset;
							}
						}
						//printf("NetConstruction:: #Nodes(%d):MedianX(%d):MedianY(%d)::CandidateNode(%d,%d)\n", counter, centX, centY, FinalX, FinalY);

						SinglePinList.push_back(make_pair(FinalX, FinalY)); // single pin in this guide
						Pin2index.emplace(make_pair(FinalX, FinalY), current_pin_index);
						//cout << ">> " << FinalX <<"  " << FinalY << "  " << current_pin_index << endl;
						// debug
						//Pin.SteinerTreePin = make_pair(FinalX, FinalY);
						pin_counter++;
					} // if overlap
				} // for
				// collect IRoute pseudo pin

				SinglePinList;
				Pin2index;

				//int distance = TRACKASSIGNMENT::line_distance(I_Rect r1, I_Rect r2);

				IRouteSegList;
				guide.LB;
				guide.RT;

			} // guide in layer
		} // layer

		if(pin_counter == 0)
			continue;

		I_Rect sourcebound = I_Rect(-1, -1, -1, -1, -1);
		I_Rect targetbound = I_Rect(-1, -1, -1, -1, -1);

		ISPD_Subnet subnet;
		subnet.GuideIndex = make_pair(-1, -1); // layer, index

		/*
		FLUTE_TwoPinNetGenerate(tempRNet.PIN_list, Pin2index, SinglePinList, tempRNet.TwoPinNetList, tempRNet.Merged_path, subnet.PinIndexSerial, 
								sourcebound, targetbound, current_net_index, tempRNet.IRoute_first);
		*/

		Local_MST_construction(SinglePinList, Pin2index, IRouteSegList, current_net_index , tempRNet);

		SinglePinList.clear();
		Pin2index.clear();
	} // for, build local guide 2PinNet
	/*
	cout << "phase two\n";
	int pause;
	cin>>pause;
	*/

	int num_of_2PinNet = 0;
	for (int i = design.ispd_routing_net.size() - 1; i >= 0; i--)
	{
		num_of_2PinNet += design.ispd_routing_net.at(i).TwoPinNetList.size();
	}

	two_pin_rip_up_net.resize(num_of_2PinNet);

	int current_two_pin_net_id = 0;
	//printf("design.ispd_routing_net.size() : %d\n", design.ispd_routing_net.size());
	bool second_rnd = false;
	for (int i = design.ispd_routing_net.size() - 1; i >= 0; i--)
	{

		printf("NetName(%s)\n", design.ispd_routing_net.at(i).Original_Data.Net_name.c_str());

		for (int TwoPinNetIndex = 0; TwoPinNetIndex < design.ispd_routing_net.at(i).TwoPinNetList.size(); TwoPinNetIndex++)
		{
			
			ISPD_2PinNet &tempTwoPinNetData = design.ispd_routing_net.at(i).TwoPinNetList.at(TwoPinNetIndex);
			//printf("Pair(%d,%d)\n", tempTwoPinNetData.TwoPinIndex.first, tempTwoPinNetData.TwoPinIndex.second);
			/*if (second_rnd == false && tempTwoPinNetData.IRoute_first == false)
				continue;
			if (second_rnd == true && tempTwoPinNetData.IRoute_first == true)
				continue;*/
			/*
			if(tempTwoPinNetData.TwoPinIndex.first >= 0)
				printf("Net(%d) Pin1(%d:%s) ", TwoPinNetIndex,
				   tempTwoPinNetData.TwoPinIndex.first,
				   design.ispd_routing_net.at(i).PIN_list.at(tempTwoPinNetData.TwoPinIndex.first).Original_MARCO.Name.c_str());
			if(tempTwoPinNetData.TwoPinIndex.second >= 0)
				printf(" Pin2(%d:%s)", tempTwoPinNetData.TwoPinIndex.second,
				   design.ispd_routing_net.at(i).PIN_list.at(tempTwoPinNetData.TwoPinIndex.second).Original_MARCO.Name.c_str());

			cout << endl;
			*/
			TwoPinRipUpNet two_pin_net;
			two_pin_net.netname = design.ispd_routing_net.at(i).Original_Data.Net_name.c_str();
			two_pin_net.TwoPinNetData = &(design.ispd_routing_net.at(i).TwoPinNetList.at(TwoPinNetIndex));
			two_pin_net.TwoPinNetList = &(design.ispd_routing_net.at(i).TwoPinNetList);

			two_pin_net.TargetBound = tempTwoPinNetData.A_TargetBound;

			two_pin_net.DesignPinList = &design.ispd_routing_net.at(i).PIN_list;
			two_pin_net.net_id = i;
			two_pin_net.two_pin_net_id = current_two_pin_net_id++;
			if (design.ispd_routing_net[i].GUIDE_list.size() == 0)
			{
				printf("Error:: Net(%d) : Global guide size is 0\n", i);
				exit(1);
			}
			two_pin_net.Global_guide = design.ispd_routing_net[i].GUIDE_list;

			//cout << tempTwoPinNetData.subtree2subtree << "  " <<  tempTwoPinNetData.subtree_is_pin << endl;

			if (tempTwoPinNetData.subtree2subtree == false && tempTwoPinNetData.subtree_is_pin == false)
			{
				// pin 2 pin
				Parser::IPSD_Routing_PIN SrcPin = design.ispd_routing_net[i].PIN_list[tempTwoPinNetData.TwoPinIndex.first];

				vector<Node> Nodelist_src;

				//printf("PinMapToGridmap Net(%d) SRC: \n", i);
				//printf("Rect Num : %d , IRect Num : %d\n", SrcPin.Rect_list.size(), SrcPin.IRect_list.size());

				for(int LayoutNodeIndex = 0 ; LayoutNodeIndex < SrcPin.LayoutNodeOriginal.size() ; LayoutNodeIndex++)
				{
					// put nodes to Nodelist
					int __x = SrcPin.LayoutNodeOriginal.at(LayoutNodeIndex).x;
					int __y = SrcPin.LayoutNodeOriginal.at(LayoutNodeIndex).y;
					int __z = SrcPin.LayoutNodeOriginal.at(LayoutNodeIndex).z;

					//printf("Net(%d)	SRCPin(%d,%d,%d)\n", i, __x, __y, __z);
					Node temp_node(__x, __y, __z-1);
					Nodelist_src.push_back(temp_node);
				}

				//layoutNodeOriginal
				for (int LayoutNodeIndex = 0; LayoutNodeIndex < SrcPin.LayoutNode.size(); LayoutNodeIndex++)
				{
					// put nodes to Nodelist
					int __x = SrcPin.LayoutNode.at(LayoutNodeIndex).x;
					int __y = SrcPin.LayoutNode.at(LayoutNodeIndex).y;
					int __z = SrcPin.LayoutNode.at(LayoutNodeIndex).z;
					Pin p;
					//printf("Net(%d)	SRCPin(%d)	Node(%d):: (%d,%d,%d)\n", i, , node, __x, __y, __z);
					p.SetPinPoint(kPinPoint, __x, __y, __z);
					two_pin_net.two_pin_net_connection.pseudo_Source_pin_path_list.push_back(p);
				}

				//LayoutNodeTRUE
				for (int LayoutNodeIndex = 0; LayoutNodeIndex < SrcPin.LayoutNodeTRUE.size(); LayoutNodeIndex++)
				{
					// put nodes to Nodelist
					int __x = SrcPin.LayoutNodeTRUE.at(LayoutNodeIndex).x;
					int __y = SrcPin.LayoutNodeTRUE.at(LayoutNodeIndex).y;
					int __z = SrcPin.LayoutNodeTRUE.at(LayoutNodeIndex).z;
					Pin p;
					//printf("Net(%d)	SRCPin(%d)	Node(%d):: (%d,%d,%d)\n", i, , node, __x, __y, __z);
					p.SetPinPoint(kPinPoint, __x, __y, __z);
					two_pin_net.two_pin_net_connection.real_Source_pin_path_list.push_back(p);
				}

				if (false)
				{ // Pin
					two_pin_net.two_pin_net_connection.source.SetPinPoint(kPinPoint, Nodelist_src[0].x, Nodelist_src[0].y, Nodelist_src[0].z);
					two_pin_net.two_pin_net_connection.src_type = 0;
				}
				else
				{ // shape
					two_pin_net.two_pin_net_connection.src_type = 1;
					for (int node = 0; node < Nodelist_src.size(); node++)
					{
						Pin p;
						//printf("Net(%d)	SRCPin(%d)	Node(%d):: (%d,%d,%d)\n", i, pin, node, Nodelist_src[node].x, Nodelist_src[node].y, Nodelist_src[node].z);
						p.SetPinPoint(kPinPoint, Nodelist_src[node].x, Nodelist_src[node].y, Nodelist_src[node].z);
						two_pin_net.two_pin_net_connection.source_pin_list.push_back(p);
						tempTwoPinNetData.SrcNodes.push_back(RipUpNode(Nodelist_src[node].x, Nodelist_src[node].y, Nodelist_src[node].z));
					}
				}
				//================================================================================================
				//printf("PinMapToGridmap Net(%d) TAR: \n",i);
				//printf("Rect Num : %d , IRect Num : %d\n", TarPin.Rect_list.size(), TarPin.IRect_list.size());
				//if(tempTwoPinNetData.TwoPinIndex.second != -1)
				if (true)
				{
					if(tempTwoPinNetData.TwoPinIndex.second == -1)
					{
						tempTwoPinNetData.TwoPinIndex.second = 0;
					}
					Parser::IPSD_Routing_PIN TarPin = design.ispd_routing_net[i].PIN_list[tempTwoPinNetData.TwoPinIndex.second];

					vector<Node> Nodelist_tar;

					for (int LayoutNodeIndex = 0; LayoutNodeIndex < TarPin.LayoutNodeOriginal.size(); LayoutNodeIndex++)
					{
						// put nodes to Nodelist
						int __x = TarPin.LayoutNodeOriginal.at(LayoutNodeIndex).x;
						int __y = TarPin.LayoutNodeOriginal.at(LayoutNodeIndex).y;
						int __z = TarPin.LayoutNodeOriginal.at(LayoutNodeIndex).z;
						//printf("TarPin:: (%d,%d,%d)\n", __x, __y, __z);
						
						Node temp_node(__x, __y, __z-1);
						Nodelist_tar.push_back(temp_node);
					}

					//layoutNodeOriginal
					for (int LayoutNodeIndex = 0; LayoutNodeIndex < TarPin.LayoutNode.size(); LayoutNodeIndex++)
					{
						// put nodes to Nodelist
						int __x = TarPin.LayoutNode.at(LayoutNodeIndex).x;
						int __y = TarPin.LayoutNode.at(LayoutNodeIndex).y;
						int __z = TarPin.LayoutNode.at(LayoutNodeIndex).z;
						Pin p;
						//printf("TarPin:: (%d,%d,%d)\n", __x, __y, __z);
						p.SetPinPoint(kPinPoint, __x, __y, __z);
						two_pin_net.two_pin_net_connection.pseudo_Target_pin_path_list.push_back(p);
					}

					//LayoutNodeTRUE
					for (int LayoutNodeIndex = 0; LayoutNodeIndex < TarPin.LayoutNodeTRUE.size(); LayoutNodeIndex++)
					{
						// put nodes to Nodelist
						int __x = TarPin.LayoutNodeTRUE.at(LayoutNodeIndex).x;
						int __y = TarPin.LayoutNodeTRUE.at(LayoutNodeIndex).y;
						int __z = TarPin.LayoutNodeTRUE.at(LayoutNodeIndex).z;
						Pin p;
						//printf("TarPin:: (%d,%d,%d)\n", __x, __y, __z);
						p.SetPinPoint(kPinPoint, __x, __y, __z);
						two_pin_net.two_pin_net_connection.real_Target_pin_path_list.push_back(p);
					}

					//printf("Net(%d)	TARPin(%d)	Node(%d):: (%d,%d,%d)\n", i, pin, node, Nodelist_tar[node].x, Nodelist_tar[node].y, Nodelist_tar[node].z);
					//printf("PinMapToGridmap END\n");
					if (false)
					{ // Pin
						two_pin_net.two_pin_net_connection.target.SetPinPoint(kPinPoint, Nodelist_tar[0].x, Nodelist_tar[0].y, Nodelist_tar[0].z);
						two_pin_net.two_pin_net_connection.tar_type = 0;
					}
					else
					{ // shape
						two_pin_net.two_pin_net_connection.tar_type = 1;
						for (int node = 0; node < Nodelist_tar.size(); node++)
						{
							Pin p;
							//printf("TARPin list:: (%d,%d,%d)\n", Nodelist_tar[node].x, Nodelist_tar[node].y, Nodelist_tar[node].z);
							p.SetPinPoint(kPinPoint, Nodelist_tar[node].x, Nodelist_tar[node].y, Nodelist_tar[node].z);
							two_pin_net.two_pin_net_connection.target_pin_list.push_back(p);
							tempTwoPinNetData.TarNodes.push_back(RipUpNode(Nodelist_tar[node].x, Nodelist_tar[node].y, Nodelist_tar[node].z));
						}
					}
				} // not -1
			}
			else if(tempTwoPinNetData.subtree_is_pin == true)
			{
				
				// subtree 2 pin
				tempTwoPinNetData.TwoPinIndex.first = tempTwoPinNetData.SubtreePinIndex;
				Parser::IPSD_Routing_PIN SrcPin = design.ispd_routing_net[i].PIN_list[tempTwoPinNetData.SubtreePinIndex];
				vector<Node> Nodelist_src;
				//printf("SubtreePinIndex->SrcPin(%d) MacroName(%s) \n", i, SrcPin.Original_MARCO.Name.c_str());
				//printf("Rect Num : %d , IRect Num : %d\n", SrcPin.Rect_list.size(), SrcPin.IRect_list.size());

				for (int LayoutNodeIndex = 0; LayoutNodeIndex < SrcPin.LayoutNodeOriginal.size(); LayoutNodeIndex++)
				{
					// put nodes to Nodelist
					int __x = SrcPin.LayoutNodeOriginal.at(LayoutNodeIndex).x;
					int __y = SrcPin.LayoutNodeOriginal.at(LayoutNodeIndex).y;
					int __z = SrcPin.LayoutNodeOriginal.at(LayoutNodeIndex).z;
					
					Node temp_node(__x, __y, __z-1);
					Nodelist_src.push_back(temp_node);
				}
				//layoutNodeOriginal
				for (int LayoutNodeIndex = 0; LayoutNodeIndex < SrcPin.LayoutNode.size(); LayoutNodeIndex++)
				{
					// put nodes to Nodelist
					int __x = SrcPin.LayoutNode.at(LayoutNodeIndex).x;
					int __y = SrcPin.LayoutNode.at(LayoutNodeIndex).y;
					int __z = SrcPin.LayoutNode.at(LayoutNodeIndex).z;
					Pin p;
					//printf("Net(%d)	SRCPin(%d)	Node(%d):: (%d,%d,%d)\n", i, , node, __x, __y, __z);
					p.SetPinPoint(kPinPoint, __x, __y, __z);
					two_pin_net.two_pin_net_connection.pseudo_Source_pin_path_list.push_back(p);
				}
				//LayoutNodeTRUE
				for (int LayoutNodeIndex = 0; LayoutNodeIndex < SrcPin.LayoutNodeTRUE.size(); LayoutNodeIndex++)
				{
					// put nodes to Nodelist
					int __x = SrcPin.LayoutNodeTRUE.at(LayoutNodeIndex).x;
					int __y = SrcPin.LayoutNodeTRUE.at(LayoutNodeIndex).y;
					int __z = SrcPin.LayoutNodeTRUE.at(LayoutNodeIndex).z;
					Pin p;
					//printf("Net(%d)	SRCPin(%d)	Node(%d):: (%d,%d,%d)\n", i, , node, __x, __y, __z);
					p.SetPinPoint(kPinPoint, __x, __y, __z);
					two_pin_net.two_pin_net_connection.real_Source_pin_path_list.push_back(p);
				}

				if (false)
				{ // Pin
					two_pin_net.two_pin_net_connection.source.SetPinPoint(kPinPoint, Nodelist_src[0].x, Nodelist_src[0].y, Nodelist_src[0].z);
					two_pin_net.two_pin_net_connection.src_type = 0;
				}
				else
				{ // shape
					two_pin_net.two_pin_net_connection.src_type = 1;
					if(Nodelist_src.size() == 0)
					{
						exit(-1);
					}

					for (int node = 0; node < Nodelist_src.size(); node++)
					{
						Pin p;
						//printf("Net(%d)	SRCPin(%d)	Node(%d):: (%d,%d,%d)\n", i, , node, Nodelist_src[node].x, Nodelist_src[node].y, Nodelist_src[node].z);
						p.SetPinPoint(kPinPoint, Nodelist_src[node].x, Nodelist_src[node].y, Nodelist_src[node].z);
						two_pin_net.two_pin_net_connection.source_pin_list.push_back(p);
						tempTwoPinNetData.SrcNodes.push_back(RipUpNode(Nodelist_src[node].x, Nodelist_src[node].y, Nodelist_src[node].z));
					}
				}
				// target is subtree

				two_pin_net.two_pin_net_connection.tar_type = 2;
			}
			else 
			{
				// subtree 2 subtree
				two_pin_net.two_pin_net_connection.src_type = 2;
				two_pin_net.two_pin_net_connection.tar_type = 2;
			}

			two_pin_rip_up_net.at(current_two_pin_net_id-1) = two_pin_net;
			/*
			if (two_pin_rip_up_net.size() > 0)
				cout << "-=-=-=>>>  " << two_pin_rip_up_net.at(two_pin_rip_up_net.size() - 1).TwoPinNetData << endl;
			two_pin_rip_up_net.at(two_pin_rip_up_net.size() - 1).TwoPinNetData = &(design.ispd_routing_net.at(i).TwoPinNetList.at(TwoPinNetIndex));
			two_pin_rip_up_net.at(two_pin_rip_up_net.size() - 1).TwoPinNetList = &(design.ispd_routing_net.at(i).TwoPinNetList);
			if (two_pin_rip_up_net.size() > 0)
				cout << "oop-=-=-=>>>  " << two_pin_rip_up_net.at(two_pin_rip_up_net.size() - 1).TwoPinNetData << endl;
			*/

		}

		//if(two_pin_rip_up_net.size() > 0)
		//	cout << &two_pin_rip_up_net.at(0) << "  " << two_pin_rip_up_net.at(0).TwoPinNetData << endl;
		int oe;
		//cin>>oe;

		if( i == 0 && !second_rnd)
		{
			//i = design.ispd_routing_net.size();
			second_rnd = true;
		}
	}
	
}

void FLUTE_NetConstruction(Parser::Design &design,
					 vector<ISPD_GridMap> &GridMap_layout,
					 Real_Coor_Table &real_coor_table,
					 FastCoorMap &fast_coor_map,
					 vector<TwoPinRipUpNet> &two_pin_rip_up_net,
					 TRACKASSIGNMENT::IRouteGenerator &iroute_generator,
					 vector<vector<vector<int>>> &ISPD_original_blockage_array)
{
	// flute read look up table
	readLUT();
	// make track layer to integer
	for (int i = 0; i < design.Track_list.size(); i++)
	{
		string tempLayer = design.Track_list.at(i).Layer;
		tempLayer.erase(tempLayer.begin(), tempLayer.begin() + 5);
		design.Track_list.at(i).i_layer = atoi(tempLayer.c_str());
		//printf("NetConstruction::Transfer Track layer into integer(%d)\n", design.Track_list.at(i).i_layer);
	}

	// extract pin shape into map pin
	for (int i = iroute_generator.design_information.routing_net_information.size() - 1; i >= 0; i--)
	{
		int pin_counter = 0;
		vector<IntPair> SinglePinList;
		map<pair<int, int>, int> Pin2index;
		int current_net_index = i;
		ISPD_Routing_Net &tempRNet = design.ispd_routing_net.at(current_net_index);

		vector<int> tempIRouteList;
		// iroute first
		for (int j = 0; j < iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Guide.size(); j++)
		{
			//printf("Layer %d : \n\n", j);
			//printf("Guide:\n\n");

			for (int k = 0; k < iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Guide[j].size(); k++)
			{
				TRACKASSIGNMENT::Guide_Component &guide = iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Guide[j].at(k);

				if (!NoTrackAssignment)
				{
					vector<IntPair> tempIROUTE;
					for (int l = 0; l < guide.Original_IRoute_list.size(); l++)
					{
						TRACKASSIGNMENT::IRoute &thisIR = guide.Original_IRoute_list.at(l);
						// add to IPSD_Routing_PIN
						// LB
						IPSD_Routing_PIN tempISPD_Routing_PIN;
						tempISPD_Routing_PIN.IsIRoutePath = IAMIROUTE; // for output use
						tempISPD_Routing_PIN.Original_MARCO.Name = "IRoute";
						tempISPD_Routing_PIN.type = false;
						tempISPD_Routing_PIN.pseudo = false;
						tempISPD_Routing_PIN.IRect_list.push_back(I_Rect(thisIR.LB.first, thisIR.LB.second, thisIR.RT.first, thisIR.RT.second, guide.Layer));
						// tempISPD_Routing_PIN.SteinerTreePin = make_pair(thisIR.LB.first, thisIR.LB.second);

						if (thisIR.LB.first > thisIR.RT.first)
						{
							int tempInt = thisIR.LB.first;
							thisIR.LB.first = thisIR.RT.first;
							thisIR.RT.first = tempInt;
						}
						if (thisIR.LB.second > thisIR.RT.second)
						{
							int tempInt = thisIR.LB.second;
							thisIR.LB.second = thisIR.RT.second;
							thisIR.RT.second = tempInt;
						}

						bool same = false; // 0:x 1:y
						if (thisIR.LB.first == thisIR.RT.first)
						{
							same = false; // x same, vertical
						}
						else
							same = true; // y same, horizontal

						if (!same)
						{
							auto Lxp = fast_coor_map.at(guide.Layer - 1).first.find(thisIR.LB.first);
							auto Rxp = fast_coor_map.at(guide.Layer - 1).first.find(thisIR.RT.first);
							auto Byp = fast_coor_map.at(guide.Layer - 1).second.upper_bound(thisIR.LB.second - 1);
							auto Typ = fast_coor_map.at(guide.Layer - 1).second.lower_bound(thisIR.RT.second + 1);
							Typ--;
							/*
								printf("NetConstruction::Ver IRoute : Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", guide.Layer, thisIR.LB.first,
									thisIR.LB.second, thisIR.RT.first, thisIR.RT.second,
									Lxp->first, Byp->first, Rxp->first, Typ->first,
									Lxp->second, Byp->second, Rxp->second, Typ->second);
								*/

							if (Byp->second > Typ->second)
							{
								printf("NetConstruction::IRoute didn't map on layout\n");
								printf("NetConstruction::IRoute : Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", guide.Layer, thisIR.LB.first,
									   thisIR.LB.second, thisIR.RT.first, thisIR.RT.second,
									   Lxp->second, Byp->second, Rxp->second, Typ->second);
								exit(-1);
							}
							else
							{

								for (int y = Byp->second; y <= Typ->second; y++)
								{
									for (int x = Lxp->second; x <= Rxp->second; x++)
									{
										tempISPD_Routing_PIN.LayoutNodeOriginal.push_back(RipUpNode(x, y, guide.Layer));
									}
								}

								tempRNet.PIN_list.push_back(tempISPD_Routing_PIN);
								tempRNet.IRoute_PIN_list.push_back(tempRNet.PIN_list.size() - 1);
								thisIR.List_index = tempRNet.PIN_list.size() - 1;
								tempIRouteList.push_back(tempRNet.PIN_list.size() - 1);
								// RT
							} // else
						}
						else
						{
							auto Lxp = fast_coor_map.at(guide.Layer - 1).first.upper_bound(thisIR.LB.first - 1);
							auto Rxp = fast_coor_map.at(guide.Layer - 1).first.lower_bound(thisIR.RT.first + 1);
							auto Byp = fast_coor_map.at(guide.Layer - 1).second.find(thisIR.LB.second);
							auto Typ = fast_coor_map.at(guide.Layer - 1).second.find(thisIR.RT.second);
							Rxp--;
							/*
								printf("NetConstruction::Hor IRoute : Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", guide.Layer, thisIR.LB.first,
									thisIR.LB.second, thisIR.RT.first, thisIR.RT.second,
									Lxp->first, Byp->first, Rxp->first, Typ->first,
									Lxp->second, Byp->second, Rxp->second, Typ->second);
								*/

							if (Lxp->second > Rxp->second)
							{
								printf("NetConstruction::IRoute didn't map on layout\n");
								printf("NetConstruction::IRoute : Layer(%d) LB(%d,%d) RT(%d,%d) -> LBp(%d,%d) RTp(%d,%d)\n", guide.Layer - 1, thisIR.LB.first,
									   thisIR.LB.second, thisIR.RT.first, thisIR.RT.second,
									   Lxp->second, Byp->second, Rxp->second, Typ->second);
								exit(-1);
							}
							else
							{

								for (int y = Byp->second; y <= Typ->second; y++)
								{
									for (int x = Lxp->second; x <= Rxp->second; x++)
									{
										tempISPD_Routing_PIN.LayoutNodeOriginal.push_back(RipUpNode(x, y, guide.Layer));
									}
								}

								tempRNet.PIN_list.push_back(tempISPD_Routing_PIN);
								tempRNet.IRoute_PIN_list.push_back(tempRNet.PIN_list.size() - 1);
								thisIR.List_index = tempRNet.PIN_list.size() - 1;
								tempIRouteList.push_back(tempRNet.PIN_list.size() - 1);
								// RT
							} // else
						}
					}
				} // if, track assignment
			}
		} // for
		// end, iroute first

		if (tempIRouteList.size() < 2)
		{
			tempRNet.IRoute_first = false;
			continue;
		}
		else
		{
			tempRNet.IRoute_first = true;
		}

		vector<int> iroute_net_list;
		int num_of_iroute = tempIRouteList.size();
		vector<bool> VisitedIRouteList;
		vector<vector<int>> distance_map;
		VisitedIRouteList.resize(num_of_iroute);
		distance_map.resize(num_of_iroute);
		for (int j = 0; j < tempIRouteList.size(); j++)
		{
			VisitedIRouteList.at(j) = false;
			distance_map.at(j).resize(num_of_iroute);
			//TRACKASSIGNMENT::line_distance();
		}

		for (int j = 0; j < tempIRouteList.size(); j++)
		{
			for (int k = 0; k < tempIRouteList.size(); k++)
			{
				if (k < j)
				{
					distance_map[j][k] = distance_map[k][j];
					continue;
				}

				if (k == j)
				{
					distance_map[j][k] = 0;
				}

				int j_index = tempIRouteList.at(j);
				int k_index = tempIRouteList.at(k);

				int j_size = tempRNet.PIN_list.at(j_index).LayoutNodeOriginal.size();
				int k_size = tempRNet.PIN_list.at(k_index).LayoutNodeOriginal.size();

				I_Rect RectJ = I_Rect(tempRNet.PIN_list.at(j_index).LayoutNodeOriginal.at(0).x,
									  tempRNet.PIN_list.at(j_index).LayoutNodeOriginal.at(0).y,
									  tempRNet.PIN_list.at(j_index).LayoutNodeOriginal.at(j_size - 1).x,
									  tempRNet.PIN_list.at(j_index).LayoutNodeOriginal.at(j_size - 1).y, -1);
				I_Rect RectK = I_Rect(tempRNet.PIN_list.at(k_index).LayoutNodeOriginal.at(0).x,
									  tempRNet.PIN_list.at(k_index).LayoutNodeOriginal.at(0).y,
									  tempRNet.PIN_list.at(k_index).LayoutNodeOriginal.at(k_size - 1).x,
									  tempRNet.PIN_list.at(k_index).LayoutNodeOriginal.at(k_size - 1).y, -1);
				int distance_of_iroute = TRACKASSIGNMENT::line_distance(RectJ, RectK);
				distance_map[j][k] = distance_of_iroute;
			}
		}

		VisitedIRouteList.at(0) = true;
		iroute_net_list.push_back(tempIRouteList.at(0));
		vector<int> target_bound;

		int counter = 0;

		while (true)
		{
			int last_j = -1;
			int last_visit = -1;
			int min_dis = INT_MAX;
			if (counter == tempIRouteList.size() - 1 || tempIRouteList.size() == 0)
				break;

			for (int j = 0; j < tempIRouteList.size(); j++)
			{
				if (!VisitedIRouteList.at(j))
				{
					continue;
				}
				else
				{
					for (int k = 0; k < tempIRouteList.size(); k++)
					{
						if (j != k && !VisitedIRouteList.at(k) && min_dis >= distance_map[j][k])
						{
							min_dis = distance_map[j][k];
							last_visit = k;
							last_j = j;
						}
					}
				}
			}

			if (counter == 0)
			{
				target_bound.push_back(last_visit);
			}
			else
			{
				target_bound.push_back(last_j);
			}

			iroute_net_list.push_back(tempIRouteList.at(last_visit));
			VisitedIRouteList.at(last_visit) = true;
			counter++;
		}

		for (int index_iroute = 0; index_iroute < VisitedIRouteList.size(); index_iroute++)
		{
			if (!VisitedIRouteList.at(index_iroute))
			{
				cout << "There are iroutes not visited." << endl;
				exit(1);
			}
		}

		vector<int> tempPreRoutedNet;
		// start build iroute 2-pin net
		for (int index_iroute = 1; index_iroute < iroute_net_list.size(); index_iroute++)
		{
			ISPD_2PinNet new2PinNet;
			new2PinNet.clear();

			new2PinNet.TargetIRouteIndex = tempIRouteList.at(target_bound.at(index_iroute - 1));
			new2PinNet.IRoute_first = true;
			new2PinNet.Net_id = i;
			new2PinNet.TwoPinIndex = make_pair(iroute_net_list.at(index_iroute), iroute_net_list.at(index_iroute - 1)); // swap!
			new2PinNet.A_TargetBound = I_Rect(tempRNet.PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].x, tempRNet.PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[0].y, tempRNet.PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[tempRNet.PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal.size() - 1].x, tempRNet.PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal[tempRNet.PIN_list.at(new2PinNet.TargetIRouteIndex).LayoutNodeOriginal.size() - 1].y, -1);

			//printf("TargetBound IRoute(%d,%d) (%d,%d)\n", new2PinNet.A_TargetBound.LB.first, new2PinNet.A_TargetBound.LB.second, new2PinNet.A_TargetBound.RT.first, new2PinNet.A_TargetBound.RT.second);
			//cout << ">> " << iroute_net_list.at(index_iroute) << "  " <<  iroute_net_list.at(index_iroute - 1) << " Target : " <<new2PinNet.TargetIRouteIndex<<endl;
			new2PinNet.PreRoutedNet.resize(tempPreRoutedNet.size());
			new2PinNet.subtree2subtree = false;
			for (int net_index = 0; net_index < tempPreRoutedNet.size(); net_index++)
			{
				new2PinNet.PreRoutedNet.at(net_index) = tempPreRoutedNet.at(net_index);
			}
			tempRNet.TwoPinNetList.push_back(new2PinNet);
			tempPreRoutedNet.push_back(tempRNet.TwoPinNetList.size() - 1);

			tempRNet.Merged_path.push_back(&new2PinNet.RipUp_path); // add reference pointer
		}
	}

	for (int i = iroute_generator.design_information.routing_net_information.size() - 1; i >= 0; i--)
	{
		int pin_counter = 0;
		vector<IntPair> SinglePinList;
		map<pair<int, int>, int> Pin2index;
		int current_net_index = i;
		ISPD_Routing_Net &tempRNet = design.ispd_routing_net.at(current_net_index);

		//cout << "---------net:  " << design.ispd_routing_net[i].Original_Data.Net_name << "--------\n\n";
		for (int j = 0; j < iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Guide.size(); j++)
		{
			//printf("Layer %d : \n\n", j);
			//printf("Guide:\n\n");

			for (int k = 0; k < iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Guide[j].size(); k++)
			{
				bool no_iroute = true;

				TRACKASSIGNMENT::Guide_Component &guide = iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Guide[j].at(k);

				guide.visited = false;

				int target_iroute = -1;

				if (!NoTrackAssignment)
				{
					//cout << "j = " << j<<endl;

					vector<IntPair> tempIROUTE;
					for (int l = 0; l < guide.Original_IRoute_list.size(); l++)
					{
						TRACKASSIGNMENT::IRoute &thisIR = guide.Original_IRoute_list.at(l);

						int IR_index = thisIR.List_index;
						target_iroute = IR_index;

						for (int node_index = 0; node_index < tempRNet.PIN_list.at(IR_index).LayoutNodeOriginal.size(); node_index++)
						{
							int x = tempRNet.PIN_list.at(IR_index).LayoutNodeOriginal.at(node_index).x;
							int y = tempRNet.PIN_list.at(IR_index).LayoutNodeOriginal.at(node_index).y;

							SinglePinList.push_back(make_pair(x, y));
							Pin2index.emplace(make_pair(x, y), -1);
						}

						no_iroute = false;
					}

					if (guide.Original_IRoute_list.size() == 0)
					{
						TRACKASSIGNMENT::Routing_Net_Information &tempRNetInfo = iroute_generator.design_information.routing_net_information.at(current_net_index);
						//cout << "---------net:  " << design.ispd_routing_net[i].Original_Data.Net_name << "--------\n\n";

						// setting up guide table
						int numoflayer = tempRNetInfo.Layer_Array_for_Guide.size();
						int numofguide = -1;
						int numofguides = 0;
						for (int j = 0; j < tempRNetInfo.Layer_Array_for_Guide.size(); j++)
						{
							if (numofguide < (int)tempRNetInfo.Layer_Array_for_Guide.at(j).size())
							{
								numofguide = (int)tempRNetInfo.Layer_Array_for_Guide.at(j).size();
							}
							numofguides += (int)tempRNetInfo.Layer_Array_for_Guide.at(j).size();
						}
						//printf("Number of guides : %d  %d\n", numofguides, numofguide);
						// ignore this case
						if (numofguides == 0)
							continue;

						vector<vector<bool>> visited_guide_table;
						visited_guide_table.resize(numoflayer);
						for (int j = 0; j < (int)tempRNetInfo.Layer_Array_for_Guide.size(); j++)
						{
							visited_guide_table.at(j).resize(numofguide);
							for (int k = 0; k < numofguide; k++)
							{
								visited_guide_table.at(j).at(k) = false;
							}
						}

						vector<IntPair> Traversal_queue;
						int guide_layer = j;
						int guide_index = k;

						visited_guide_table[guide_layer][guide_index] = true;
						//printf("guide : (%d,%d)\n", guide_layer, guide_index);

						guide.visited = true;

						for (auto &same_neighbor : guide.SameLayerList)
						{
							if (visited_guide_table[guide.Layer][same_neighbor] == false)
								Traversal_queue.push_back(make_pair(guide.Layer, same_neighbor));
							visited_guide_table[guide.Layer][same_neighbor] = true;
							//printf("add same guide : (%d,%d)\n", guide.Layer, same_neighbor);
						}
						for (auto &low_neighbor : guide.LowerLayerList)
						{
							Traversal_queue.push_back(make_pair(guide.Layer - 1, low_neighbor));
							visited_guide_table[guide.Layer - 1][low_neighbor] = true;
							//printf("add low guide : (%d,%d)\n", guide.Layer - 1, low_neighbor);
						}

						for (auto &up_neighbor : guide.UpperLayerList)
						{
							Traversal_queue.push_back(make_pair(guide.Layer + 1, up_neighbor));
							visited_guide_table[guide.Layer + 1][up_neighbor] = true;
							//printf("add up guide : (%d,%d)\n", guide.Layer + 1, up_neighbor);
						}

						// MAIN for
						for (int index = 0; index < Traversal_queue.size(); index++)
						{
							// find iroute
							int now_guide_layer = Traversal_queue.at(index).first;
							int now_guide_index = Traversal_queue.at(index).second;
							TRACKASSIGNMENT::Guide_Component &now_guide = tempRNetInfo.Layer_Array_for_Guide.at(now_guide_layer).at(now_guide_index);

							if (now_guide.Layer == 1)
								continue;

							//printf("Guide(%f,%f) (%f,%f) Layer(%d)\n", now_guide.LB.first, now_guide.LB.second, now_guide.RT.first, now_guide.RT.second, now_guide.Layer);
							bool Found = false;
							for (int iroute_index = 0; iroute_index < now_guide.Original_IRoute_list.size(); iroute_index++)
							{
								target_iroute = now_guide.Original_IRoute_list.at(iroute_index).List_index;
								Found = true;
							}
							if (Found)
							{
								break;
							}

							for (auto &same_neighbor : now_guide.SameLayerList)
							{
								if (visited_guide_table[now_guide.Layer][same_neighbor] == false)
								{
									Traversal_queue.push_back(make_pair(now_guide.Layer, same_neighbor));
									visited_guide_table[now_guide.Layer][same_neighbor] = true;
									//printf("add same guide : (%d,%d)\n", now_guide.Layer, same_neighbor);
								}
							}
							// add next guide
							for (auto &low_neighbor : now_guide.LowerLayerList)
							{
								//printf(">> guide : (%d,%d)\n", now_guide.Layer - 1, low_neighbor);
								//cout << "BOOL = " << visited_guide_table[now_guide.Layer - 1][low_neighbor]<<endl;
								if (visited_guide_table[now_guide.Layer - 1][low_neighbor] == false)
								{
									Traversal_queue.push_back(make_pair(now_guide.Layer - 1, low_neighbor));
									visited_guide_table[now_guide.Layer - 1][low_neighbor] = true;
									//printf("add low guide : (%d,%d)\n", now_guide.Layer - 1, low_neighbor);
								}
							}

							for (auto &up_neighbor : now_guide.UpperLayerList)
							{
								//printf(">> guide : (%d,%d)\n", now_guide.Layer + 1, up_neighbor, visited_guide_table[now_guide.Layer + 1][up_neighbor]);
								//cout << "BOOL = " << visited_guide_table[now_guide.Layer + 1][up_neighbor] << endl;
								if (visited_guide_table[now_guide.Layer + 1][up_neighbor] == false)
								{
									Traversal_queue.push_back(make_pair(now_guide.Layer + 1, up_neighbor));
									visited_guide_table[now_guide.Layer + 1][up_neighbor] = true;
									//printf("add up guide : (%d,%d)\n", now_guide.Layer + 1, up_neighbor);
								}
							}
						} // for
					}
				}

				if (target_iroute == -1 && tempRNet.IRoute_first == true)
				{
					cout << "ERROR target_iroute = -1" << endl;
					exit(-1);
				}

				/*
				int wwwww;
				cin>>wwwww;*/
				/*
				printf("NetConstruction::Guide LB (%f, %f), RT (%f, %f) Layer = %d, Guide id = %d\n",
					   guide.LB.first, guide.LB.second, guide.RT.first, guide.RT.second, guide.Layer, guide.guide_index);
				printf("size = %d\n", iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Pin[j].size());
				*/
				// collect pin in this guide
				for (int l = 0; l < iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Pin[j].size(); l++)
				{
					TRACKASSIGNMENT::Big_Pin &BPin = iroute_generator.design_information.routing_net_information[i].Layer_Array_for_Pin[j].at(l);

					if (TRACKASSIGNMENT::rec_overlap(guide.LB, guide.RT, BPin.LB, BPin.RT))
					{
						int current_pin_index = BPin.pin_index;
						//printf("NetConstruction::Pin index(%d)\n", current_pin_index);

						// if overlap, extract a pin to Pin single-pin list
						vector<IntPair> tempPinList;

						int centX = 0, centY = 0, counter = 0;
						IPSD_Routing_PIN &Pin = tempRNet.PIN_list.at(current_pin_index);

						Pin.target_index = target_iroute;

						for (int m = 0; m < Pin.LayoutNodeOriginal.size(); m++)
						{
							int __x = Pin.LayoutNodeOriginal.at(m).x;
							int __y = Pin.LayoutNodeOriginal.at(m).y;
							int __z = Pin.LayoutNodeOriginal.at(m).z;
							// all node in this pin
							centX += __x;
							centY += __y;
							counter++;
							tempPinList.push_back(make_pair(__x, __y));

						} // for, LayoutNodeOriginal

						// extract a target pin from tempPinList
						//printf("%d / %d\n", centX, counter);
						//printf("%d / %d\n", centY, counter);
						if (counter != 0)
						{
							centX = centX / counter;
							centY = centY / counter;
						}

						int MinOffset = INT_MAX;
						int FinalX = -1, FinalY = -1;
						for (auto &NodePair : tempPinList)
						{
							int tempX = NodePair.first - centX;
							int tempY = NodePair.second - centY;
							int tempOffset = abs(tempX) + abs(tempY);
							if (tempOffset < MinOffset)
							{
								FinalX = NodePair.first;
								FinalY = NodePair.second;
								MinOffset = tempOffset;
							}
						}
						//printf("NetConstruction:: #Nodes(%d):MedianX(%d):MedianY(%d)::CandidateNode(%d,%d)\n", counter, centX, centY, FinalX, FinalY);

						SinglePinList.push_back(make_pair(FinalX, FinalY)); // single pin in this guide
						Pin2index.emplace(make_pair(FinalX, FinalY), current_pin_index);
						//cout << ">> " << FinalX <<"  " << FinalY << "  " << current_pin_index << endl;
						// debug
						//Pin.SteinerTreePin = make_pair(FinalX, FinalY);
						pin_counter++;
					} // if overlap
				}	 // for
					  // collect IRoute pseudo pin

			} // guide in layer
		}	 // layer

		if (pin_counter == 0)
			continue;

		I_Rect sourcebound = I_Rect(-1, -1, -1, -1, -1);
		I_Rect targetbound = I_Rect(-1, -1, -1, -1, -1);

		ISPD_Subnet subnet;
		subnet.GuideIndex = make_pair(-1, -1); // layer, index

		FLUTE_TwoPinNetGenerate(tempRNet.PIN_list, Pin2index, SinglePinList, tempRNet.TwoPinNetList, tempRNet.Merged_path, subnet.PinIndexSerial,
								sourcebound, targetbound, current_net_index, tempRNet.IRoute_first);

		SinglePinList.clear();
		Pin2index.clear();
	} // for, build local guide 2PinNet
	/*
	cout << "phase two\n";
	int pause;
	cin>>pause;
	*/

	int num_of_2PinNet = 0;
	for (int i = design.ispd_routing_net.size() - 1; i >= 0; i--)
	{
		num_of_2PinNet += design.ispd_routing_net.at(i).TwoPinNetList.size();
	}

	two_pin_rip_up_net.resize(num_of_2PinNet);

	int current_two_pin_net_id = 0;
	//printf("design.ispd_routing_net.size() : %d\n", design.ispd_routing_net.size());
	bool second_rnd = false;
	for (int i = design.ispd_routing_net.size() - 1; i >= 0; i--)
	{

		//printf("NetName(%s)\n", design.ispd_routing_net.at(i).Original_Data.Net_name.c_str());

		for (int TwoPinNetIndex = 0; TwoPinNetIndex < design.ispd_routing_net.at(i).TwoPinNetList.size(); TwoPinNetIndex++)
		{

			ISPD_2PinNet &tempTwoPinNetData = design.ispd_routing_net.at(i).TwoPinNetList.at(TwoPinNetIndex);

			/*if (second_rnd == false && tempTwoPinNetData.IRoute_first == false)
				continue;
			if (second_rnd == true && tempTwoPinNetData.IRoute_first == true)
				continue;*/
			/*
			if(tempTwoPinNetData.TwoPinIndex.first >= 0)
				printf("Net(%d) Pin1(%d:%s) ", TwoPinNetIndex,
				   tempTwoPinNetData.TwoPinIndex.first,
				   design.ispd_routing_net.at(i).PIN_list.at(tempTwoPinNetData.TwoPinIndex.first).Original_MARCO.Name.c_str());
			if(tempTwoPinNetData.TwoPinIndex.second >= 0)
				printf(" Pin2(%d:%s)", tempTwoPinNetData.TwoPinIndex.second,
				   design.ispd_routing_net.at(i).PIN_list.at(tempTwoPinNetData.TwoPinIndex.second).Original_MARCO.Name.c_str());

			cout << endl;
			*/
			TwoPinRipUpNet two_pin_net;
			two_pin_net.netname = design.ispd_routing_net.at(i).Original_Data.Net_name.c_str();
			two_pin_net.TwoPinNetData = &(design.ispd_routing_net.at(i).TwoPinNetList.at(TwoPinNetIndex));
			two_pin_net.TwoPinNetList = &(design.ispd_routing_net.at(i).TwoPinNetList);

			two_pin_net.TargetBound = tempTwoPinNetData.A_TargetBound;

			two_pin_net.DesignPinList = &design.ispd_routing_net.at(i).PIN_list;
			two_pin_net.net_id = i;
			two_pin_net.two_pin_net_id = current_two_pin_net_id++;
			if (design.ispd_routing_net[i].GUIDE_list.size() == 0)
			{
				printf("Error:: Net(%d) : Global guide size is 0\n", i);
				exit(1);
			}
			two_pin_net.Global_guide = design.ispd_routing_net[i].GUIDE_list;

			//cout << tempTwoPinNetData.subtree2subtree << "  " <<  tempTwoPinNetData.subtree_is_pin << endl;

			if (tempTwoPinNetData.subtree2subtree == false && tempTwoPinNetData.subtree_is_pin == false)
			{
				// pin 2 pin
				Parser::IPSD_Routing_PIN SrcPin = design.ispd_routing_net[i].PIN_list[tempTwoPinNetData.TwoPinIndex.first];

				vector<Node> Nodelist_src;

				//printf("PinMapToGridmap Net(%d) SRC: \n", i);
				//printf("Rect Num : %d , IRect Num : %d\n", SrcPin.Rect_list.size(), SrcPin.IRect_list.size());

				for (int LayoutNodeIndex = 0; LayoutNodeIndex < SrcPin.LayoutNodeOriginal.size(); LayoutNodeIndex++)
				{
					// put nodes to Nodelist
					int __x = SrcPin.LayoutNodeOriginal.at(LayoutNodeIndex).x;
					int __y = SrcPin.LayoutNodeOriginal.at(LayoutNodeIndex).y;
					int __z = SrcPin.LayoutNodeOriginal.at(LayoutNodeIndex).z;

					Node temp_node(__x, __y, __z - 1);
					Nodelist_src.push_back(temp_node);
				}

				//layoutNodeOriginal
				for (int LayoutNodeIndex = 0; LayoutNodeIndex < SrcPin.LayoutNode.size(); LayoutNodeIndex++)
				{
					// put nodes to Nodelist
					int __x = SrcPin.LayoutNode.at(LayoutNodeIndex).x;
					int __y = SrcPin.LayoutNode.at(LayoutNodeIndex).y;
					int __z = SrcPin.LayoutNode.at(LayoutNodeIndex).z;
					Pin p;
					//printf("Net(%d)	SRCPin(%d)	Node(%d):: (%d,%d,%d)\n", i, , node, __x, __y, __z);
					p.SetPinPoint(kPinPoint, __x, __y, __z);
					two_pin_net.two_pin_net_connection.pseudo_Source_pin_path_list.push_back(p);
				}

				//LayoutNodeTRUE
				for (int LayoutNodeIndex = 0; LayoutNodeIndex < SrcPin.LayoutNodeTRUE.size(); LayoutNodeIndex++)
				{
					// put nodes to Nodelist
					int __x = SrcPin.LayoutNodeTRUE.at(LayoutNodeIndex).x;
					int __y = SrcPin.LayoutNodeTRUE.at(LayoutNodeIndex).y;
					int __z = SrcPin.LayoutNodeTRUE.at(LayoutNodeIndex).z;
					Pin p;
					//printf("Net(%d)	SRCPin(%d)	Node(%d):: (%d,%d,%d)\n", i, , node, __x, __y, __z);
					p.SetPinPoint(kPinPoint, __x, __y, __z);
					two_pin_net.two_pin_net_connection.real_Source_pin_path_list.push_back(p);
				}

				if (false)
				{ // Pin
					two_pin_net.two_pin_net_connection.source.SetPinPoint(kPinPoint, Nodelist_src[0].x, Nodelist_src[0].y, Nodelist_src[0].z);
					two_pin_net.two_pin_net_connection.src_type = 0;
				}
				else
				{ // shape
					two_pin_net.two_pin_net_connection.src_type = 1;
					for (int node = 0; node < Nodelist_src.size(); node++)
					{
						Pin p;
						//printf("Net(%d)	SRCPin(%d)	Node(%d):: (%d,%d,%d)\n", i, pin, node, Nodelist_src[node].x, Nodelist_src[node].y, Nodelist_src[node].z);
						p.SetPinPoint(kPinPoint, Nodelist_src[node].x, Nodelist_src[node].y, Nodelist_src[node].z);
						two_pin_net.two_pin_net_connection.source_pin_list.push_back(p);
						tempTwoPinNetData.SrcNodes.push_back(RipUpNode(Nodelist_src[node].x, Nodelist_src[node].y, Nodelist_src[node].z));
					}
				}
				//================================================================================================
				//printf("PinMapToGridmap Net(%d) TAR: \n",i);
				//printf("Rect Num : %d , IRect Num : %d\n", TarPin.Rect_list.size(), TarPin.IRect_list.size());
				//if(tempTwoPinNetData.TwoPinIndex.second != -1)
				if (true)
				{
					if (tempTwoPinNetData.TwoPinIndex.second == -1)
					{
						tempTwoPinNetData.TwoPinIndex.second = 0;
					}
					Parser::IPSD_Routing_PIN TarPin = design.ispd_routing_net[i].PIN_list[tempTwoPinNetData.TwoPinIndex.second];

					vector<Node> Nodelist_tar;

					for (int LayoutNodeIndex = 0; LayoutNodeIndex < TarPin.LayoutNodeOriginal.size(); LayoutNodeIndex++)
					{
						// put nodes to Nodelist
						int __x = TarPin.LayoutNodeOriginal.at(LayoutNodeIndex).x;
						int __y = TarPin.LayoutNodeOriginal.at(LayoutNodeIndex).y;
						int __z = TarPin.LayoutNodeOriginal.at(LayoutNodeIndex).z;
						//printf("TarPin:: (%d,%d,%d)\n", __x, __y, __z);

						Node temp_node(__x, __y, __z - 1);
						Nodelist_tar.push_back(temp_node);
					}

					//layoutNodeOriginal
					for (int LayoutNodeIndex = 0; LayoutNodeIndex < TarPin.LayoutNode.size(); LayoutNodeIndex++)
					{
						// put nodes to Nodelist
						int __x = TarPin.LayoutNode.at(LayoutNodeIndex).x;
						int __y = TarPin.LayoutNode.at(LayoutNodeIndex).y;
						int __z = TarPin.LayoutNode.at(LayoutNodeIndex).z;
						Pin p;
						//printf("TarPin:: (%d,%d,%d)\n", __x, __y, __z);
						p.SetPinPoint(kPinPoint, __x, __y, __z);
						two_pin_net.two_pin_net_connection.pseudo_Target_pin_path_list.push_back(p);
					}

					//LayoutNodeTRUE
					for (int LayoutNodeIndex = 0; LayoutNodeIndex < TarPin.LayoutNodeTRUE.size(); LayoutNodeIndex++)
					{
						// put nodes to Nodelist
						int __x = TarPin.LayoutNodeTRUE.at(LayoutNodeIndex).x;
						int __y = TarPin.LayoutNodeTRUE.at(LayoutNodeIndex).y;
						int __z = TarPin.LayoutNodeTRUE.at(LayoutNodeIndex).z;
						Pin p;
						//printf("TarPin:: (%d,%d,%d)\n", __x, __y, __z);
						p.SetPinPoint(kPinPoint, __x, __y, __z);
						two_pin_net.two_pin_net_connection.real_Target_pin_path_list.push_back(p);
					}

					//printf("Net(%d)	TARPin(%d)	Node(%d):: (%d,%d,%d)\n", i, pin, node, Nodelist_tar[node].x, Nodelist_tar[node].y, Nodelist_tar[node].z);
					//printf("PinMapToGridmap END\n");
					if (false)
					{ // Pin
						two_pin_net.two_pin_net_connection.target.SetPinPoint(kPinPoint, Nodelist_tar[0].x, Nodelist_tar[0].y, Nodelist_tar[0].z);
						two_pin_net.two_pin_net_connection.tar_type = 0;
					}
					else
					{ // shape
						two_pin_net.two_pin_net_connection.tar_type = 1;
						for (int node = 0; node < Nodelist_tar.size(); node++)
						{
							Pin p;
							//printf("TARPin list:: (%d,%d,%d)\n", Nodelist_tar[node].x, Nodelist_tar[node].y, Nodelist_tar[node].z);
							p.SetPinPoint(kPinPoint, Nodelist_tar[node].x, Nodelist_tar[node].y, Nodelist_tar[node].z);
							two_pin_net.two_pin_net_connection.target_pin_list.push_back(p);
							tempTwoPinNetData.TarNodes.push_back(RipUpNode(Nodelist_tar[node].x, Nodelist_tar[node].y, Nodelist_tar[node].z));
						}
					}
				} // not -1
			}
			else if (tempTwoPinNetData.subtree_is_pin == true)
			{

				// subtree 2 pin
				tempTwoPinNetData.TwoPinIndex.first = tempTwoPinNetData.SubtreePinIndex;
				Parser::IPSD_Routing_PIN SrcPin = design.ispd_routing_net[i].PIN_list[tempTwoPinNetData.SubtreePinIndex];
				vector<Node> Nodelist_src;
				//printf("SubtreePinIndex->SrcPin(%d) MacroName(%s) \n", i, SrcPin.Original_MARCO.Name.c_str());
				//printf("Rect Num : %d , IRect Num : %d\n", SrcPin.Rect_list.size(), SrcPin.IRect_list.size());

				for (int LayoutNodeIndex = 0; LayoutNodeIndex < SrcPin.LayoutNodeOriginal.size(); LayoutNodeIndex++)
				{
					// put nodes to Nodelist
					int __x = SrcPin.LayoutNodeOriginal.at(LayoutNodeIndex).x;
					int __y = SrcPin.LayoutNodeOriginal.at(LayoutNodeIndex).y;
					int __z = SrcPin.LayoutNodeOriginal.at(LayoutNodeIndex).z;

					Node temp_node(__x, __y, __z - 1);
					Nodelist_src.push_back(temp_node);
				}
				//layoutNodeOriginal
				for (int LayoutNodeIndex = 0; LayoutNodeIndex < SrcPin.LayoutNode.size(); LayoutNodeIndex++)
				{
					// put nodes to Nodelist
					int __x = SrcPin.LayoutNode.at(LayoutNodeIndex).x;
					int __y = SrcPin.LayoutNode.at(LayoutNodeIndex).y;
					int __z = SrcPin.LayoutNode.at(LayoutNodeIndex).z;
					Pin p;
					//printf("Net(%d)	SRCPin(%d)	Node(%d):: (%d,%d,%d)\n", i, , node, __x, __y, __z);
					p.SetPinPoint(kPinPoint, __x, __y, __z);
					two_pin_net.two_pin_net_connection.pseudo_Source_pin_path_list.push_back(p);
				}
				//LayoutNodeTRUE
				for (int LayoutNodeIndex = 0; LayoutNodeIndex < SrcPin.LayoutNodeTRUE.size(); LayoutNodeIndex++)
				{
					// put nodes to Nodelist
					int __x = SrcPin.LayoutNodeTRUE.at(LayoutNodeIndex).x;
					int __y = SrcPin.LayoutNodeTRUE.at(LayoutNodeIndex).y;
					int __z = SrcPin.LayoutNodeTRUE.at(LayoutNodeIndex).z;
					Pin p;
					//printf("Net(%d)	SRCPin(%d)	Node(%d):: (%d,%d,%d)\n", i, , node, __x, __y, __z);
					p.SetPinPoint(kPinPoint, __x, __y, __z);
					two_pin_net.two_pin_net_connection.real_Source_pin_path_list.push_back(p);
				}

				if (false)
				{ // Pin
					two_pin_net.two_pin_net_connection.source.SetPinPoint(kPinPoint, Nodelist_src[0].x, Nodelist_src[0].y, Nodelist_src[0].z);
					two_pin_net.two_pin_net_connection.src_type = 0;
				}
				else
				{ // shape
					two_pin_net.two_pin_net_connection.src_type = 1;
					if (Nodelist_src.size() == 0)
					{
						exit(-1);
					}

					for (int node = 0; node < Nodelist_src.size(); node++)
					{
						Pin p;
						//printf("Net(%d)	SRCPin(%d)	Node(%d):: (%d,%d,%d)\n", i, , node, Nodelist_src[node].x, Nodelist_src[node].y, Nodelist_src[node].z);
						p.SetPinPoint(kPinPoint, Nodelist_src[node].x, Nodelist_src[node].y, Nodelist_src[node].z);
						two_pin_net.two_pin_net_connection.source_pin_list.push_back(p);
						tempTwoPinNetData.SrcNodes.push_back(RipUpNode(Nodelist_src[node].x, Nodelist_src[node].y, Nodelist_src[node].z));
					}
				}
				// target is subtree

				two_pin_net.two_pin_net_connection.tar_type = 2;
			}
			else
			{
				// subtree 2 subtree
				two_pin_net.two_pin_net_connection.src_type = 2;
				two_pin_net.two_pin_net_connection.tar_type = 2;
			}

			two_pin_rip_up_net.at(current_two_pin_net_id - 1) = two_pin_net;
			/*
			if (two_pin_rip_up_net.size() > 0)
				cout << "-=-=-=>>>  " << two_pin_rip_up_net.at(two_pin_rip_up_net.size() - 1).TwoPinNetData << endl;
			two_pin_rip_up_net.at(two_pin_rip_up_net.size() - 1).TwoPinNetData = &(design.ispd_routing_net.at(i).TwoPinNetList.at(TwoPinNetIndex));
			two_pin_rip_up_net.at(two_pin_rip_up_net.size() - 1).TwoPinNetList = &(design.ispd_routing_net.at(i).TwoPinNetList);
			if (two_pin_rip_up_net.size() > 0)
				cout << "oop-=-=-=>>>  " << two_pin_rip_up_net.at(two_pin_rip_up_net.size() - 1).TwoPinNetData << endl;
			*/
		}

		//if(two_pin_rip_up_net.size() > 0)
		//	cout << &two_pin_rip_up_net.at(0) << "  " << two_pin_rip_up_net.at(0).TwoPinNetData << endl;
		//int oe;
		//cin>>oe;

		if (i == 0 && !second_rnd)
		{
			//i = design.ispd_routing_net.size();
			second_rnd = true;
		}
	}
}

void FloatRectangle2Integer(Rect &frect,I_Rect &irect, int offset){
	irect.Layer = -1;
	irect.LB.first = (int)(frect.LB.first * offset);
	irect.LB.second = (int)(frect.LB.second * offset);
	irect.RT.first = (int)(frect.RT.first * offset);
	irect.RT.second = (int)(frect.RT.second * offset);
}


void Evaluate_Two_Rect(vector<Parser::INT_EOL_Rule> &ISPD_EndOfLine_Spacing, vector < vector < pair <int, int> > > &Spacing_Table,
			Parser::Enc_Relation &enc_relation,Parser::I_Rect &via1_rec , Parser::I_Rect &via2_rec,int layer){

		int eol_width = ISPD_EndOfLine_Spacing.at(layer).ENDOFLINE; 
		int eol_spacing = ISPD_EndOfLine_Spacing.at(layer).SPACING;


		// run length spacing 
		int via1_width = min(via1_rec.RT.first - via1_rec.LB.first, via1_rec.RT.second - via1_rec.LB.second);
		int via1_PRL_spacing = -1;
		for (int st = 0; st < Spacing_Table.at(layer).size(); st++){
			if (via1_width >= Spacing_Table.at(layer)[st].first)
				via1_PRL_spacing = Spacing_Table.at(layer)[st].second;
			else{
				break;
			}
		}

		int via2_width = min(via2_rec.RT.first - via2_rec.LB.first, via2_rec.RT.second - via2_rec.LB.second);
		int via2_PRL_spacing = -1;
		for (int st = 0; st < Spacing_Table.at(layer).size(); st++){
			if (via2_width >= Spacing_Table.at(layer)[st].first)
				via2_PRL_spacing = Spacing_Table.at(layer)[st].second;
			else{
				break;
			}
		}

		// eol checking
		bool via1_horizontal_eol = false;
		bool via1_vertical_eol = false;

		if((via1_rec.RT.first - via1_rec.LB.first) < eol_width){
			// vertical eol
			via1_vertical_eol = true;
		}
		if((via1_rec.RT.second - via1_rec.LB.second) < eol_width){
			// horizontal eol
			via1_horizontal_eol = true;
		}

		bool via2_horizontal_eol = false;
		bool via2_vertical_eol = false;

		if((via2_rec.RT.first - via2_rec.LB.first) < eol_width){
			// vertical eol
			via2_vertical_eol = true;
		}
		if((via2_rec.RT.second - via2_rec.LB.second) < eol_width){
			// horizontal eol
			via2_horizontal_eol = true;
		}

		int via1_half_width = (via1_rec.RT.first - via1_rec.LB.first) / 2; // horizontal
		int via1_half_height = (via1_rec.RT.second - via1_rec.LB.second) / 2; // vertical

		int via2_half_width = (via2_rec.RT.first - via2_rec.LB.first) / 2; // horizontal
		int via2_half_height = (via2_rec.RT.second - via2_rec.LB.second) / 2; // vertical
		
		// Evaluate the spacing

		if(via1_horizontal_eol || via2_horizontal_eol){
			enc_relation.same_horizontal_spacing = eol_spacing + via1_half_width + via2_half_width;
		}
		else{
			enc_relation.same_horizontal_spacing = max(via1_PRL_spacing,via2_PRL_spacing) + via1_half_width + via2_half_width;
		}

		if(via1_vertical_eol || via2_vertical_eol){
			enc_relation.same_vertical_spacing = eol_spacing + via1_half_height + via2_half_height;
		}
		else{
			enc_relation.same_vertical_spacing = max(via1_PRL_spacing,via2_PRL_spacing) + via1_half_height + via2_half_height;
		}

		printf("VIA 1 ENC LB(%d,%d) RT(%d,%d)\n",via1_rec.LB.first,via1_rec.LB.second,via1_rec.RT.first,via1_rec.RT.second);
		printf("VIA 2 ENC LB(%d,%d) RT(%d,%d)\n",via2_rec.LB.first,via2_rec.LB.second,via2_rec.RT.first,via2_rec.RT.second);
		printf("EOL WIDTH(%d) SPACING(%d)\n",eol_width,eol_spacing);
		printf("PRL SPACING 1(%d) PRL SPACING 2(%d)\n",via1_PRL_spacing,via2_PRL_spacing);
		printf("same_horizontal_spacing(%d) , same_vertical_spacing(%d)\n",enc_relation.same_horizontal_spacing,enc_relation.same_vertical_spacing);

}


void ViaTypeList_Construction(Parser::Design &design, vector < vector <Parser::Via> > &via_type_list,vector<Parser::INT_EOL_Rule> &ISPD_EndOfLine_Spacing, vector < vector < pair <int, int> > > &Spacing_Table,
vector < vector < vector < Parser::Enc_Relation > > >  &Enc_Relation_Pair_Table ){

	via_type_list.resize(design.Metal_Layer_Num - 1);

	for (int i = 0; i < design.Via_list.size(); i++){
		sscanf(design.Via_list[i].BotLayerName.c_str(), "Metal%d", &design.Via_list[i].BotLayer);
		sscanf(design.Via_list[i].ViaLayerName.c_str(), "Via%d", &design.Via_list[i].ViaLayer);
		sscanf(design.Via_list[i].TopLayerName.c_str(), "Metal%d", &design.Via_list[i].TopLayer);

		FloatRectangle2Integer(design.Via_list[i].BotLayerRect, design.Via_list[i].BotLayerIRect, design.ISPD_OFFSET);
		FloatRectangle2Integer(design.Via_list[i].ViaLayerRect, design.Via_list[i].ViaLayerIRect, design.ISPD_OFFSET);
		FloatRectangle2Integer(design.Via_list[i].TopLayerRect, design.Via_list[i].TopLayerIRect, design.ISPD_OFFSET);

		via_type_list[design.Via_list[i].BotLayer - 1].push_back(design.Via_list[i]);
	}

	// first raising, second, falling
	
	Enc_Relation_Pair_Table.resize(design.Metal_Layer_Num);
	for(int layer = 0; layer < design.Metal_Layer_Num; layer++){
		vector <Parser::I_Rect> Enc_list;

		// falling
		if( layer >= 1){
			for(int enc = 0; enc < via_type_list[layer - 1].size(); enc++){
				Enc_list.push_back(via_type_list[layer - 1][enc].TopLayerIRect);
			}
		}
		
		// raising
		if( layer < via_type_list.size()){
			for(int enc = 0; enc < via_type_list[layer].size(); enc++){
				Enc_list.push_back(via_type_list[layer][enc].BotLayerIRect);
			}
		}
		
		
		// allocate second dim
		Enc_Relation_Pair_Table.at(layer).resize(Enc_list.size());
		for(int i = 0; i < Enc_list.size() ; i++){
			// allocate third dim
			Enc_Relation_Pair_Table.at(layer).at(i).resize(Enc_list.size());
			// i : middle via , j : neighbor via
			for(int j = 0; j < Enc_list.size(); j++){
				Parser::Enc_Relation temp_enc_relation;
				printf("Evaluate:: Layer(%d) VIA(%d) - VIA(%d)\n",layer,i,j);
				Evaluate_Two_Rect(ISPD_EndOfLine_Spacing,Spacing_Table,temp_enc_relation,Enc_list.at(i),Enc_list.at(j),layer);
				printf("======================================\n");
				Enc_Relation_Pair_Table.at(layer).at(i).at(j) = temp_enc_relation;		
			}	
		}
	}

	/*
	for (int i = 0; i < design.Via_Layer_Num; i++){
		for (int via = 0; via < via_type_list[i].size(); via++){
			printf("Layer(%d) : %s - BOT[LB (%d,%d) , RT(%d,%d)] / Top[LB (%d,%d) , RT(%d,%d)]\n", i, 
				via_type_list[i][via].Name.c_str(), via_type_list[i][via].BotLayerIRect.LB.first, via_type_list[i][via].BotLayerIRect.LB.second
				, via_type_list[i][via].BotLayerIRect.RT.first , via_type_list[i][via].BotLayerIRect.RT.second, 
				via_type_list[i][via].TopLayerIRect.LB.first, via_type_list[i][via].TopLayerIRect.LB.second,
				via_type_list[i][via].TopLayerIRect.RT.first,via_type_list[i][via].TopLayerIRect.RT.second );
		}
	}
	*/
}

void Construct_SpacingList(Parser::Design &design,vector <int> &ISPD_Via_cut_Spacing, vector <int> &ISPD_Metal_Spacing,
	vector <Parser::INT_EOL_Rule> &ISPD_EndOfLine_Spacing){
	ISPD_EndOfLine_Spacing.resize(design.Metal_Layer_Num);
	ISPD_Metal_Spacing.resize(design.Metal_Layer_Num);
	for (int i = 0; i < design.Metal_Layer_Num; i++){
		ISPD_EndOfLine_Spacing[i].ENDOFLINE = (int)(design.Layer_list[2 * i].metalLayer.EOL.ENDOFLINE * design.ISPD_OFFSET);
		ISPD_EndOfLine_Spacing[i].SPACING = (int)(design.Layer_list[2 * i].metalLayer.EOL.SPACING * design.ISPD_OFFSET);
		ISPD_EndOfLine_Spacing[i].WITHIN = (int)(design.Layer_list[2 * i].metalLayer.EOL.WITHIN * design.ISPD_OFFSET);
		ISPD_Metal_Spacing[i] = (int)(design.Layer_list[2 * i].metalLayer.SPACING * design.ISPD_OFFSET);
	}
	ISPD_Via_cut_Spacing.resize(design.Via_Layer_Num);
	for (int i = 0; i < design.Via_Layer_Num; i++){
		ISPD_Via_cut_Spacing[i] = (int)(design.Layer_list[2 * i + 1].viaLayer.CUT_SPACING  * design.ISPD_OFFSET);
	}
}

void SpaceEvaluationGraph_Construction(Parser::Design &design, vector<SpaceEvaluationGraph> &SpaceEvaluationLayout,
									   vector<ISPD_GridMap> &GridMap_layout, Real_Coor_Table &real_coor_table,
									   FastCoorMap &fast_coor_map, vector<Parser::INT_EOL_Rule> &ISPD_EndOfLine_Spacing,
									   vector<vector<pair<int, int>>> &Spacing_Table, vector<vector<Parser::Via>> &via_type_list)
{

	SpaceEvaluationLayout.resize(GridMap_layout.size());

	for (int i = 0; i < GridMap_layout.size(); i++){
		int width = -1;
		if (i == GridMap_layout.size() - 1){
			width = min(via_type_list[i - 1].at(0).BotLayerIRect.RT.first - via_type_list[i - 1].at(0).BotLayerIRect.LB.first,
				via_type_list[i - 1].at(0).BotLayerIRect.RT.second - via_type_list[i - 1].at(0).BotLayerIRect.LB.second);
		}
		else{
			width = min(via_type_list[i].at(0).BotLayerIRect.RT.first - via_type_list[i].at(0).BotLayerIRect.LB.first,
				via_type_list[i].at(0).BotLayerIRect.RT.second - via_type_list[i].at(0).BotLayerIRect.LB.second);
		}
		
		SpaceEvaluationLayout[i].tree = new RTree <int, int, 2, float, 8, 4>();
		SpaceEvaluationLayout[i].EOL_tree = new RTree<int, int, 2, float, 8, 4>();
		//cout << SpaceEvaluationLayout[i].tree << endl;
		SpaceEvaluationLayout[i]
			.SetGraph(GridMap_layout[i], fast_coor_map, real_coor_table,
					  Spacing_Table[i], ISPD_EndOfLine_Spacing[i], width, design.AllShapes, design.AllShapes_NetId, i);
	}
	
	for (int i = 0; i < design.AllShapes.size(); i++){
		int layer = design.AllShapes.at(i).Layer - 1;
		//SpaceEvaluationLayout[layer].PushBlockage(design.AllShapes.at(i));
		Parser::RipUpSegment temp_ignore;
		SpaceEvaluationLayout[layer].PushNetBlockageToTree(design.AllShapes.at(i), design.AllShapes_NetId.at(i)); //, temp_ignore);
	}
}

void LayerMinWidth_list(Parser::Design &design, vector <int> &MinWidth_list){

	for (int i = 0; i < design.Metal_Layer_Num; i++){
		MinWidth_list.push_back((int)(design.Layer_list[2 * i].metalLayer.WIDTH * design.ISPD_OFFSET));
	}
}



int main(int argc, char **argv){
	printf("Team number: 19\n");
	printf("Member: Shih-Ting Lin, Ming-Jie Fong, Ching-Hsi Chen, Wei-Ren Lai, He-Cheng Tsai\n");
	printf("Affiliation: National Chiao Tung University\n\n\n\n");
	printf("<< ISPD Routing Program >>\n");
	printf("Start Parsing LEF/DEF...\n");
    clock_t total_t_start, total_t_stop;
    total_t_start = clock();
	Parser::Design design;
	MainParser(argc, argv, design);
	//design.ISPD_OFFSET = 2000;
	//MainWriter(argc, argv, design);

	int gcellnum = ((design.Die_Area.RT.first - design.Die_Area.LB.first) / design.GCell_Width) * ((design.Die_Area.RT.second - design.Die_Area.LB.second) / design.GCell_Height);
	if(gcellnum < 100)
	{
		NoTrackAssignment = true;
	}
	else 
		NoTrackAssignment = false;
	/* 
	Routing Schedule : 
	1. grid map construction
	2. Pin/shape map to node
	3. IRoute extraction
	4. ILP 
	5. Mazeroute
	*/
	//NoTrackAssignment = true;
	//iroute should generate here
	TRACKASSIGNMENT::IRouteGenerator iroute_generator;
	printf("IRoute_Generator::Build_Whole_Design\n");
	iroute_generator.build_whole_design(design);

	iroute_generator.BuildGuideConnection();
	bool L3_disable = false;
	if(design.AllOBSs.size() > 0)
	{
		//L3_disable = true;
	}
	//cout << "L3_disable : " << L3_disable << endl;

	iroute_generator.InitialPanelList(design, L3_disable);
    double max_runtime = -1;
    int max_iroute_cnt = 0, min_iroute_cnt = 1e9, total_cnt = 0;
    bool panel_flag_v = 0, panel_flag_h = 0;
    clock_t t_start, t_stop, i_start, i_stop;
    ofstream ssout;
    ssout.open("statistic.txt");
    t_start = clock();
    if(NoTrackAssignment == false)
    {
        if (Mix_ILP_Itrack) {
            printf("start to solve track assignments by Mixing Method(ilp and itrack)\n");
            int penalty=0;
            for(int i=2 ; i<iroute_generator.Hor_Panel_list.size() ; i++){
                if (design.Layer_list[(i-1)*2].metalLayer.DIRECTION == true) {
                    int track_width=0;
                    for(int j=0 ; j<iroute_generator.Hor_Panel_list[i].size() ; j++){
                        if(iroute_generator.Hor_Panel_list[i][j].track.size()>1){
                            track_width=2*(iroute_generator.Hor_Panel_list[i][j].track[1]-iroute_generator.Hor_Panel_list[i][j].track[0]);
                            break;
                        }
                    }
                    for(int j=0 ; j<iroute_generator.Hor_Panel_list[i].size() ; j++){
                        //printf("track width:%d\n",track_width);
                        printf("now is compute hortical layer %d , panel %d\n",i,j);
                        printf("iroute_list_size = %d\n", iroute_generator.Hor_Panel_list[i][j].iroute_list.size());
                        if (panel_flag_h == 0) {
                            ssout << "Hor_Panel Height = " << iroute_generator.Hor_Panel_list[i][j].RT.second - iroute_generator.Hor_Panel_list[i][j].LB.second << endl;
                            ssout << "Hor_Panel Width = " << iroute_generator.Hor_Panel_list[i][j].RT.first - iroute_generator.Hor_Panel_list[i][j].LB.first << endl;
                            panel_flag_h = 1;
                        }
                        if (iroute_generator.Hor_Panel_list[i][j].iroute_list.size() > max_iroute_cnt) max_iroute_cnt = iroute_generator.Hor_Panel_list[i][j].iroute_list.size();
                        if (iroute_generator.Hor_Panel_list[i][j].iroute_list.size() != 0 && iroute_generator.Hor_Panel_list[i][j].iroute_list.size() < min_iroute_cnt) min_iroute_cnt = iroute_generator.Hor_Panel_list[i][j].iroute_list.size();
                        total_cnt += iroute_generator.Hor_Panel_list[i][j].iroute_list.size();
                        i_start = clock();
                        if (iroute_generator.Hor_Panel_list[i][j].iroute_list.size() < 300)
                            solve_problem_horizontal(iroute_generator.Hor_Panel_list[i][j],iroute_generator.design_information,track_width,i,iroute_generator.Hor_Panel_list );
                        else
                            Trackassignment_one::TA TA1( iroute_generator.Hor_Panel_list[i][j], iroute_generator);
                        i_stop = clock();
                        double tmp_time =  double(i_stop - i_start) / CLOCKS_PER_SEC;
                        if (tmp_time > max_runtime) max_runtime = tmp_time;
                    }
                }
                else {
                    int track_width=0;
                    for(int j=0 ; j<iroute_generator.Ver_Panel_list[i].size() ; j++){
                        if(iroute_generator.Ver_Panel_list[i][j].track.size()>1){
                            track_width=2*(iroute_generator.Ver_Panel_list[i][j].track[1]-iroute_generator.Ver_Panel_list[i][j].track[0]);
                            break;
                        }
                    }
                    for(int j=0 ; j<iroute_generator.Ver_Panel_list[i].size() ; j++){
                        //printf("track width:%d\n",track_width);
                        printf("now is compute vertical layer %d , panel %d\n",i,j);
                        printf("iroute_list_size = %d\n", iroute_generator.Ver_Panel_list[i][j].iroute_list.size());
                        if (panel_flag_v == 0) {
                            ssout << "Ver_Panel Height = " << iroute_generator.Ver_Panel_list[i][j].RT.second - iroute_generator.Ver_Panel_list[i][j].LB.second << endl;
                            ssout << "Ver_Panel Width = " << iroute_generator.Ver_Panel_list[i][j].RT.first - iroute_generator.Ver_Panel_list[i][j].LB.first << endl;
                            panel_flag_v = 1;
                        }
                        if (iroute_generator.Ver_Panel_list[i][j].iroute_list.size() > max_iroute_cnt) max_iroute_cnt = iroute_generator.Ver_Panel_list[i][j].iroute_list.size();
                        if (iroute_generator.Ver_Panel_list[i][j].iroute_list.size() != 0 && iroute_generator.Ver_Panel_list[i][j].iroute_list.size() < min_iroute_cnt) min_iroute_cnt = iroute_generator.Ver_Panel_list[i][j].iroute_list.size();
                        total_cnt += iroute_generator.Ver_Panel_list[i][j].iroute_list.size();
                        i_start = clock();
                        if (iroute_generator.Ver_Panel_list[i][j].iroute_list.size() < 300)
                            solve_problem_vertical(iroute_generator.Ver_Panel_list[i][j],iroute_generator.design_information,track_width,i,iroute_generator.Ver_Panel_list);
                        else
                            Trackassignment_one::TA TA1( iroute_generator.Ver_Panel_list[i][j], iroute_generator);
                        i_stop = clock();
                        double tmp_time =  double(i_stop - i_start) / CLOCKS_PER_SEC;
                        if (tmp_time > max_runtime) max_runtime = tmp_time;
                    }
                }
            }
            printf("end of solving track assignments by by Mixing Method(ilp and itrack)\n");
        }
        else {
            if(TrackAssignmentILP)
            {
                printf("start to solve track assignments by ilp\n");
                int size=(iroute_generator.Hor_Panel_list.size()>iroute_generator.Ver_Panel_list.size())?iroute_generator.Hor_Panel_list.size():iroute_generator.Ver_Panel_list.size();
                for(int i=2 ; i<size ; i++){
                    if (design.Layer_list[(i-1)*2].metalLayer.DIRECTION == true) {
                        int track_width=0;
                        for(int j=0 ; j<iroute_generator.Hor_Panel_list[i].size() ; j++){
                            if(iroute_generator.Hor_Panel_list[i][j].track.size()>1){
                                track_width=2*(iroute_generator.Hor_Panel_list[i][j].track[1]-iroute_generator.Hor_Panel_list[i][j].track[0]);
                                break;
                            }
                        }
                        for(int j=0 ; j<iroute_generator.Hor_Panel_list[i].size() ; j++){
                            //printf("track width:%d\n",track_width);
                            //printf("now is compute hortical layer %d , panel %d\n",i,j);
                            //printf("iroute_list_size = %d\n", iroute_generator.Hor_Panel_list[i][j].iroute_list.size());
                            //if(iroute_generator.Hor_Panel_list[i][j].iroute_list.size()<800)
                                solve_problem_horizontal(iroute_generator.Hor_Panel_list[i][j],iroute_generator.design_information,track_width,i,iroute_generator.Hor_Panel_list );
                        }
                    }
                    else {
                        //for(int i=2 ; i<iroute_generator.Ver_Panel_list.size() ; i++){
                        int track_width=0;
                        for(int j=0 ; j<iroute_generator.Ver_Panel_list[i].size() ; j++){
                            if(iroute_generator.Ver_Panel_list[i][j].track.size()>1){
                                track_width=2*(iroute_generator.Ver_Panel_list[i][j].track[1]-iroute_generator.Ver_Panel_list[i][j].track[0]);
                                break;
                            }
                        }
                        for(int j=0 ; j<iroute_generator.Ver_Panel_list[i].size() ; j++){
                            //printf("track width:%d\n",track_width);
                            //printf("now is compute vertical layer %d , panel %d\n",i,j);
                            //printf("iroute_list_size = %d\n", iroute_generator.Ver_Panel_list[i][j].iroute_list.size());
                            //if(iroute_generator.Ver_Panel_list[i][j].iroute_list.size()<800)
                                solve_problem_vertical(iroute_generator.Ver_Panel_list[i][j],iroute_generator.design_information,track_width,i,iroute_generator.Ver_Panel_list);
                            
                        }
                        //}
                    }
                }
                printf("end of solving track assignments by ilp\n");
            }
            /*else
            {
                printf("start to solve track assignments by itrack\n");
                for (int i = 2; i < iroute_generator.Hor_Panel_list.size(); i++) {
                    for (int j = 0; j < iroute_generator.Hor_Panel_list[i].size(); j++) {
                        Trackassignment_one::TA TA1( iroute_generator.Hor_Panel_list[i][j], iroute_generator);
                    }
                }
                for (int i = 2; i < iroute_generator.Ver_Panel_list.size(); i++) {
                    for (int j = 0; j < iroute_generator.Ver_Panel_list[i].size(); j++) {
                        Trackassignment_one::TA TA1( iroute_generator.Ver_Panel_list[i][j], iroute_generator);
                    }
                }
                printf("end of solving track assignments by itrack\n");
                //TA1.main_func(iroute_generator);
            }*/
        }
    }
    t_stop = clock();
    ssout << "Total Track Assignment Time = " <<  double(t_stop - t_start) / CLOCKS_PER_SEC << endl;
    ssout << "Max_runtime = " << max_runtime << endl;
    ssout << "max_iroute_cnt = " << max_iroute_cnt << ", min_iroute_cnt = " << min_iroute_cnt << endl;
    ssout << "total_cnt = " << total_cnt << endl;
    /*for(int i=2 ; i<iroute_generator.Hor_Panel_list.size() ; i++) {
        for(int j=0 ; j<iroute_generator.Hor_Panel_list[i].size() ; j++) {
            for (int k = 0; k < iroute_generator.Hor_Panel_list[i][j].iroute_list.size(); k++) {
                if (iroute_generator.Hor_Panel_list[i][j].iroute_list[k]->LB.first <= 0 || iroute_generator.Hor_Panel_list[i][j].iroute_list[k]->LB.first >= 1715200) {
                    cout << "hout of bound, LB.first = " << iroute_generator.Hor_Panel_list[i][j].iroute_list[k]->LB.first << endl;
                    break;
                }
                if (iroute_generator.Hor_Panel_list[i][j].iroute_list[k]->RT.first <= 0 || iroute_generator.Hor_Panel_list[i][j].iroute_list[k]->RT.first >= 1715200) {
                    cout << "hout of bound, RT.first = " << iroute_generator.Hor_Panel_list[i][j].iroute_list[k]->RT.first << endl;
                    break;
                }
                if (iroute_generator.Hor_Panel_list[i][j].iroute_list[k]->LB.second <= 0 || iroute_generator.Hor_Panel_list[i][j].iroute_list[k]->LB.second >= 1062400) {
                    cout << "hout of bound, LB.second = " << iroute_generator.Hor_Panel_list[i][j].iroute_list[k]->LB.second << endl;
                    break;
                }
                if (iroute_generator.Hor_Panel_list[i][j].iroute_list[k]->RT.second <= 0 || iroute_generator.Hor_Panel_list[i][j].iroute_list[k]->RT.second >= 1062400) {
                    cout << "hout of bound, RT.second = " << iroute_generator.Hor_Panel_list[i][j].iroute_list[k]->RT.second << endl;
                    break;
                }
            }
        }
    }
    for(int i=2 ; i<iroute_generator.Ver_Panel_list.size() ; i++) {
        for(int j=0 ; j<iroute_generator.Ver_Panel_list[i].size() ; j++) {
            for (int k = 0; k < iroute_generator.Ver_Panel_list[i][j].iroute_list.size(); k++) {
                if (iroute_generator.Ver_Panel_list[i][j].iroute_list[k]->LB.first <= 0 || iroute_generator.Ver_Panel_list[i][j].iroute_list[k]->LB.first >= 1715200) {
                    cout << "vout of bound, LB.first = " << iroute_generator.Ver_Panel_list[i][j].iroute_list[k]->LB.first << endl;
                    break;
                }
                if (iroute_generator.Ver_Panel_list[i][j].iroute_list[k]->RT.first <= 0 || iroute_generator.Ver_Panel_list[i][j].iroute_list[k]->RT.first >= 1715200) {
                    cout << "vout of bound, RT.first = " << iroute_generator.Ver_Panel_list[i][j].iroute_list[k]->RT.first << endl;
                    break;
                }
                if (iroute_generator.Ver_Panel_list[i][j].iroute_list[k]->LB.second <= 0 || iroute_generator.Ver_Panel_list[i][j].iroute_list[k]->LB.second >= 1062400) {
                    cout << "vout of bound, LB.second = " << iroute_generator.Ver_Panel_list[i][j].iroute_list[k]->LB.second << endl;
                    break;
                }
                if (iroute_generator.Ver_Panel_list[i][j].iroute_list[k]->RT.second <= 0 || iroute_generator.Ver_Panel_list[i][j].iroute_list[k]->RT.second >= 1062400) {
                    cout << "vout of bound, RT.second = " << iroute_generator.Ver_Panel_list[i][j].iroute_list[k]->RT.second << endl;
                    break;
                }
            }
        }
    }*/
    //return 0;
    /*
	SmallRouter(
	const int layout_width,
	const int layout_height,
	const int layout_layer,
	const vector<TwoPinRipUpNet> &two_pin_rip_up_net,
	const vector<Node> &obstacle_nodes,
	const bool kLineEndModeRouting,
	const bool kEndEndSpacingMode
	);
	*/
	vector<ISPD_GridMap> GridMap_layout;
	FastCoorMap fast_coor_map;
	Real_Coor_Table real_coor_table;
	RaisingFalling_Connection_Table connection_table;
	LayoutGridMapConstruction(design, fast_coor_map, GridMap_layout, real_coor_table, connection_table);
	//real_coor_table.Table_Status_Log();

	
	vector<Parser::INT_EOL_Rule>
		ISPD_EndOfLine_Spacing;
	vector < vector < pair <int, int> > > Spacing_Table;
	vector <int> ISPD_Via_cut_Spacing;

	printf("Construct Spacing Table\n");
	Construct_SpacingTable(design, Spacing_Table, ISPD_EndOfLine_Spacing, ISPD_Via_cut_Spacing);
	
	vector<TwoPinRipUpNet> two_pin_rip_up_net;
	TwoPinRipUpNet two_pin_net;

	vector< vector< vector<int> > > ISPD_original_iroute_array_;
	vector< vector< vector<int> > > ISPD_original_blockage_array_;
	vector< vector< vector<int> > > ISPD_original_EOL_array_;
	vector< vector< vector<bool> > > ISPD_L2_OBS_array_;	
	vector<vector<vector<pair<int, int> > > > CongestionMap;
	vector < vector < vector < Parser::Enc_Relation> > >  Enc_Relation_Pair_Table;// first dim: layer , second & third dim: via pair 
	vector<vector<Parser::Via>> via_type_list;
	printf("ViaTypeList Construction\n");
	ViaTypeList_Construction(design, via_type_list,ISPD_EndOfLine_Spacing,Spacing_Table,Enc_Relation_Pair_Table);
	
	EncId_HashTable _EncId_HashTable_(via_type_list,design.Metal_Layer_Num);
	vector<SpaceEvaluationGraph> SpaceEvaluationLayout;
	printf("SpaceEvaluationGraph Construction\n");
	SpaceEvaluationGraph_Construction(design, SpaceEvaluationLayout, GridMap_layout, real_coor_table,
									  fast_coor_map, ISPD_EndOfLine_Spacing, Spacing_Table, via_type_list);
	printf("Blockage_array Initialization\n");
	ISPD_blockage_array_initialization(GridMap_layout, ISPD_original_blockage_array_, ISPD_original_EOL_array_, ISPD_original_iroute_array_, ISPD_L2_OBS_array_);
	printf("SetAllPinShapeOnLayout\n");
	reset_CongestionMap(CongestionMap, GridMap_layout);
	SetAllPinShapeOnLayout(design, GridMap_layout, fast_coor_map, iroute_generator, ISPD_original_blockage_array_, CongestionMap, real_coor_table);
	printf("Set AllShape To Blockage\n");
	SetAllShapeToBlockage(design, ISPD_original_blockage_array_, ISPD_original_EOL_array_, Spacing_Table, 
		ISPD_EndOfLine_Spacing, fast_coor_map);
	printf("Set IRoute To Blockage\n");
	SetIRouteAndObstaclesOnBlockage(design, GridMap_layout, fast_coor_map, iroute_generator, ISPD_original_blockage_array_, ISPD_original_iroute_array_, ISPD_L2_OBS_array_, SpaceEvaluationLayout);
	printf("Net Construction\n");
	FLUTE_NetConstruction(design, GridMap_layout, real_coor_table, fast_coor_map, two_pin_rip_up_net, iroute_generator, ISPD_original_blockage_array_);
	//NetConstruction(design, GridMap_layout, real_coor_table, fast_coor_map, two_pin_rip_up_net, iroute_generator, ISPD_original_blockage_array_);
	
	vector<Node> obstacle_nodes;
	coor_t layout_width = design.Die_Area.RT.first  - design.Die_Area.LB.first;
	coor_t layout_height= design.Die_Area.RT.second - design.Die_Area.LB.second;
	printf("***********Routing Information*************\n");
	printf("Layout Width : %d Layout Height : %d \n", layout_width, layout_height);
	printf("Metal Layer Num : %d\n", design.Metal_Layer_Num);
	
	vector <int> MinWidth_list;
	LayerMinWidth_list(design, MinWidth_list);

	vector <int> ISPD_Metal_Spacing;
	
	/*printf("Construct SpacingList\n");
	Construct_SpacingList(design, ISPD_Via_cut_Spacing, ISPD_Metal_Spacing, ISPD_EndOfLine_Spacing);*/
	/*int pause;
	cin >> pause;*/
	//real_coor_table.Table_Status_Log();
	clock_t start = clock();
	SmallRouter router(layout_width, layout_height, design.Metal_Layer_Num, two_pin_rip_up_net, obstacle_nodes,
					   GridMap_layout, real_coor_table, connection_table,fast_coor_map,
					   design.ispd_track_list, ISPD_Via_cut_Spacing,
					   Spacing_Table, ISPD_EndOfLine_Spacing, via_type_list,
					   ISPD_original_blockage_array_, ISPD_original_EOL_array_, SpaceEvaluationLayout,Enc_Relation_Pair_Table,_EncId_HashTable_ ,MinWidth_list, CongestionMap,
					   ISPD_original_iroute_array_, ISPD_L2_OBS_array_,
					   design.Layer_list[0].metalLayer.DIRECTION, false, false);
	router.Net_Num = design.ispd_routing_net.size();

	/*while(1)
		;
	return 0;*/
	//printf("Routing End");

	if(OutputIRouteOnly)
	{
		for (int i = 0; i < design.ispd_routing_net.size(); i++)
		{
			design.ispd_routing_net.at(i).WireList.clear();
		}
 	}
	else 
	{
		
		router.ISPD_StartRouting();

		if (router.Net_wirePath.size() != design.ispd_routing_net.size())
		{
			printf("ERROR::Net Number Error \n");
			exit(1);
		}
		for (int i = 0; i < router.Net_wirePath.size(); i++)
		{
			design.ispd_routing_net[i].WireList = router.Net_wirePath[i];
			/*
			printf("============= Net(%d) ===============\n", i);
			for (int path = 0; path < design.ispd_routing_net[i].WireList.size(); path++)
			{
				printf("Src(%d,%d) - Tar(%d,%d) - Metal%d \n", design.ispd_routing_net[i].WireList[path].Src_Pin.first, design.ispd_routing_net[i].WireList[path].Src_Pin.second,
					   design.ispd_routing_net[i].WireList[path].Tar_Pin.first, design.ispd_routing_net[i].WireList[path].Tar_Pin.second, design.ispd_routing_net[i].WireList[path].Layer);
			}
			*/
		}
		
	}
	// IRoute Output
	if(OutputIRouteOnly == false)
	{
		for (int i = 0; i < design.ispd_routing_net.size(); i++)
		{
			for (int j = 0; j < design.ispd_routing_net.at(i).IRoute_PIN_list.size(); j++)
			{
				int pin_index = design.ispd_routing_net.at(i).IRoute_PIN_list.at(j);
				if (design.ispd_routing_net.at(i).PIN_list.at(pin_index).IsIRoutePath == IAMIROUTE)
				{
					int _size = (int)design.ispd_routing_net.at(i).PIN_list.at(pin_index).SynthesisSegList.size();

					for(int k = 0 ; k < _size ; k++)
					{
						int _length = (int)design.ispd_routing_net.at(i).PIN_list.at(pin_index).SynthesisSegList.at(k).size();

						int _x = design.ispd_routing_net.at(i).PIN_list.at(pin_index).SynthesisSegList.at(k).at(0).x;
						int _y = design.ispd_routing_net.at(i).PIN_list.at(pin_index).SynthesisSegList.at(k).at(0).y;
						int _z = design.ispd_routing_net.at(i).PIN_list.at(pin_index).SynthesisSegList.at(k).at(0).z;

						int __x = design.ispd_routing_net.at(i).PIN_list.at(pin_index).SynthesisSegList.at(k).at(_length-1).x;
						int __y = design.ispd_routing_net.at(i).PIN_list.at(pin_index).SynthesisSegList.at(k).at(_length-1).y;
						int __z = design.ispd_routing_net.at(i).PIN_list.at(pin_index).SynthesisSegList.at(k).at(_length-1).z;

						//printf("(%d,%d,%d) (%d,%d,%d)\n", _x, _y, _z, __x, __y, __z);
						bool _dir = false;
						if (_x == __x)
						{
							// vertical
							_dir = false;
						}
						else if (_y == __y)
						{
							_dir = true;
						}
						else
						{
							// error
							cout << "ERROR::IROUTE OUTPUT" << endl;
						}

						wire_path path;
						path.path_type = 0;
						tuple<coor_t, coor_t, int> temp_coor;
						temp_coor = real_coor_table.Index2Coor(_x,_y,_z-1);
						path.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
						path.Layer = _z - 1;
						temp_coor = real_coor_table.Index2Coor(__x, __y, __z - 1);
						path.Tar_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
						path.dir = _dir;

						design.ispd_routing_net[i].WireList.push_back(path);
						//printf("ADD IROUTE Size(%d) (%d) Layer(%d) SRC(%d,%d) TAR(%d,%d)\n", _size, pin_index, path.Layer, path.Src_Pin.first, path.Src_Pin.second, path.Tar_Pin.first, path.Tar_Pin.second);
					}
				}
			}
		}
	} // if 
	else
	{
		for (int i = 0; i < design.ispd_routing_net.size(); i++)
		{
			for (int j = 0; j < design.ispd_routing_net.at(i).IRoute_PIN_list.size(); j++)
			{
				int pin_index = design.ispd_routing_net.at(i).IRoute_PIN_list.at(j);

				if (design.ispd_routing_net.at(i).PIN_list.at(pin_index).IsIRoutePath == IAMIROUTE)
				{
					int _length = (int)design.ispd_routing_net.at(i).PIN_list.at(pin_index).LayoutNodeOriginal.size();

					int _x = design.ispd_routing_net.at(i).PIN_list.at(pin_index).LayoutNodeOriginal.at(0).x;
					int _y = design.ispd_routing_net.at(i).PIN_list.at(pin_index).LayoutNodeOriginal.at(0).y;
					int _z = design.ispd_routing_net.at(i).PIN_list.at(pin_index).LayoutNodeOriginal.at(0).z;

					int __x = design.ispd_routing_net.at(i).PIN_list.at(pin_index).LayoutNodeOriginal.at(_length - 1).x;
					int __y = design.ispd_routing_net.at(i).PIN_list.at(pin_index).LayoutNodeOriginal.at(_length - 1).y;
					int __z = design.ispd_routing_net.at(i).PIN_list.at(pin_index).LayoutNodeOriginal.at(_length - 1).z;

					//printf("(%d,%d,%d) (%d,%d,%d)\n", _x, _y, _z, __x, __y, __z);
					bool _dir = false;
					if (_x == __x)
					{
						// vertical
						_dir = false;
					}
					else if (_y == __y)
					{
						_dir = true;
					}
					else
					{
						// error
						cout << "ERROR::IROUTE OUTPUT" << endl;
					}

					wire_path path;
					path.path_type = 0;
					tuple<coor_t, coor_t, int> temp_coor;
					temp_coor = real_coor_table.Index2Coor(_x, _y, _z - 1);
					path.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
					path.Layer = _z - 1;
					temp_coor = real_coor_table.Index2Coor(__x, __y, __z - 1);
					path.Tar_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
					path.dir = _dir;

					design.ispd_routing_net[i].WireList.push_back(path);

					//printf("ADD IROUTE Layer(%d) SRC(%d,%d) TAR(%d,%d)\n", path.Layer, path.Src_Pin.first, path.Src_Pin.second, path.Tar_Pin.first, path.Tar_Pin.second);
				}
			}
		}
	} // else, only iroute

	clock_t end = clock();

	double duration = (double)(end - start) / CLOCKS_PER_SEC;
	printf("Routing Time Cost : %.2lf sec\n", duration);

	/*for(int i = 0; i < design.ispd_routing_net.at(11557).WireList.size(); i++){
		printf("Net(11557) (%d)path Layer(%d)\n",i,design.ispd_routing_net.at(11557).WireList[i].Layer);
	}*/

	MainWriter(argc, argv, design);

	for (int i = 0; i < SpaceEvaluationLayout.size(); i++)
	{
		SpaceEvaluationLayout.at(i).clear();
		SpaceEvaluationLayout.at(i).clearTree();
	}
	SpaceEvaluationLayout.clear();

	router.clear();
    total_t_stop = clock();
    ssout << "Total Runtime = " <<  double(total_t_stop - total_t_start) / CLOCKS_PER_SEC << endl;
    cout << "Total Runtime = " <<  double(total_t_stop - total_t_start) / CLOCKS_PER_SEC << endl;
	printf("Program End\n");

	return 0;
}
