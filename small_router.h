#ifndef SMALL_ROUTER_H
#define SMALL_ROUTER_H

// #define PrintDetailedResult

#include <cstdio>
using std::printf;
using std::sscanf;
#include <climits>
#include <cfloat>
// Check math.h for more. An extract from the "old" math.h (in 2009):
/* Define _USE_MATH_DEFINES before including math.h to expose these macro
** definitions for common math constants.  These are placed under an #ifdef
** since these commonly-defined names are not part of the C/C++ standards.
**/
#define _USE_MATH_DEFINES
#include <cmath>
using std::exp;

#include <cstring>
using std::strcmp;

#include <fstream>
using std::ifstream;
using std::ofstream;
#include <iostream>
using std::cout;
using std::endl;
#include <iomanip>
#include <queue>
using std::queue;
#include <vector>
using std::vector;
#include <map>
using std::map;
#include <unordered_set>
using std::unordered_set;
#include <set>
using std::set;
#include <utility>
using std::pair;
#include <algorithm>
using std::sort;
using std::push_heap;
using std::pop_heap;
using std::make_heap;
#include <tuple>
#include <limits.h>

#include <unordered_map>
using std::unordered_map;

#include "graph_struct.h"
#include "history_congestion_graph.h"
#include "line_end_graph.h"
#include "lef_def_parser/Structure.h"
#include "Definition.h"
#include "MazeRouteKernel.h"
#include "SpaceEvaluationGraph.hpp"
//#include "output_layout.h"



// track_capacity_ usage
extern const int kNoBlockage;
extern const int kBlockage;

// routing algorithm usage
const int kNoDirection = -1; // source
const int kInitDirection = 0; // no arrow
const int kMoveStep = 1;


// moving direction
extern const int kRight;
extern const int kLeft;
extern const int kTop;
extern const int kBottom;
extern const int kUp;
extern const int kDown;

// line-end recognition for pin
extern const int kNone;
extern const int kPinLocation;
extern const int kPinBlockage;
extern const int kNormalBlockage;
extern const int kPotentialBlockage;

typedef list<int> NetList;
typedef NetList::iterator NetListIterator;
typedef unordered_map< ULL, NetList > LayoutMap;


class RectMapFunction{
	vector < pair < vector <coor_t>, vector <coor_t> > > fast_coor_map;
	public:
		RectMapFunction(){}
		void SetGraph(vector < pair < map<coor_t, int>, map<coor_t, int > > > &origin_map){
			fast_coor_map.resize(origin_map.size());
			for (int i = 0; i < origin_map.size(); i++)
			{
				for (auto x = origin_map[i].first.begin(); x != origin_map[i].first.end(); x++)
				{
					fast_coor_map.at(i).first.push_back(x->first);
				}

				for (auto y = origin_map[i].second.begin(); y != origin_map[i].second.end(); y++)
				{
					fast_coor_map.at(i).second.push_back(y->first);
				}
			}
		}
		// if match value then return value
		// if in the nearest interval then return the smaller one
		int LowerPrefer_binary_search(vector<coor_t> &Map, int target, int k, int min_range, int max_range)
		{
			//printf("%d size:%d\n", k, Map.size());
			if (target == Map[k])
				return k;
			if( k > 0 ){
				if(target == Map[k - 1]) return k - 1;
				if (target < Map[k] && target > Map[k - 1])
				{
					return k -1;
				}
			}
			if( k + 1 < Map.size() ){
				if(target == Map[k + 1]) return k + 1;
				if (target > Map[k] && target < Map[k + 1])
				{
					return k;
				}
			}

			if(k == 0 || k == 1) return -1;
			if(k == Map.size() - 1 || k == Map.size() - 2) return -1;
			
			if(target > Map[k])
				return LowerPrefer_binary_search(Map, target, round((k + max_range) / 2), k, max_range);
			else
				return LowerPrefer_binary_search(Map, target, round((k + min_range) / 2), min_range, k);
		}
		// if match value then return value
		// if in the nearest interval then return the larger one
		int UpperPrefer_binary_search(vector<coor_t> &Map, int target, int k ,int min_range ,int max_range)
		{
			//printf("target: %d, map[%d]:%d size:%d\n", target, k, Map[k], Map.size());
			if(target == Map[k]) return k;
			if( k > 0 ){
				if(target == Map[k - 1]) return k - 1;
				if (target < Map[k] && target > Map[k - 1])
				{
					return k ;
				}
			}
			if( k + 1 < Map.size() ){
				if(target == Map[k + 1]) return k + 1;
				if (target > Map[k] && target < Map[k + 1])
				{
					return k + 1;
				}
			}

			if(k == 0 || k == 1) return -1;
			if(k == Map.size() - 1 || k == Map.size() - 2) return -1;
			
			if(target > Map.at(k))
				return UpperPrefer_binary_search(Map,target, round( (k + max_range) / 2 ), k , max_range);
			else
				return UpperPrefer_binary_search(Map, target, round((k + min_range) / 2), min_range, k);
		}


		// axis is true => x, axis is false => y
		// layer start from 0
		// return false if point out of range
		bool FindLowerTarget(int map_point,int layer ,bool axis ,int &index){
			
			if (axis == true)
			{
				int k = (0 + fast_coor_map.at(layer).first.size()) / 2;
				index = LowerPrefer_binary_search(fast_coor_map.at(layer).first, map_point, k, 0, fast_coor_map.at(layer).first.size());
				return index != -1;
			}
			else if(axis == false){
				int k = (0 + fast_coor_map.at(layer).second.size()) / 2;
				index = LowerPrefer_binary_search(fast_coor_map.at(layer).second, map_point, k, 0,fast_coor_map.at(layer).second.size());
				return index != -1;
			}
		}

		// axis is true => x, axis is false => y
		// layer start from 0
		// return false if point out of range
		bool FindUpperTarget(int map_point,int layer ,bool axis ,int &index){
			if(axis == true){
				int k = (0 + fast_coor_map.at(layer).first.size()) / 2;
				index = UpperPrefer_binary_search(fast_coor_map.at(layer).first, map_point, k, 0, fast_coor_map.at(layer).first.size());
				bool re = index != -1;
				return re;
			}
			else if(axis == false){
				//printf("map_point : %d\n", map_point);
				//printf("layer : %d\n",layer);
				int k = (0 + fast_coor_map.at(layer).second.size()) / 2;
				//printf("k : %d\n",k);
				index = UpperPrefer_binary_search(fast_coor_map.at(layer).second, map_point, k, 0, fast_coor_map.at(layer).second.size());
				return (index != -1);
			}
		}
		
		// Map the shrinked(lower bound) range of the rectangle in layout
		void MapRect_ShrinkedRange(Parser::I_Rect &Input_Rect, Parser::I_Rect &Output_Index_Range){
			int layer = Input_Rect.Layer;
			//printf("MapRect_ShrinkedRange\n");
			//printf("MapRect_ShrinkedRange::LB(%d,%d) RT(%d,%d)\n",Input_Rect.LB.first,Input_Rect.LB.second,Input_Rect.RT.first,Input_Rect.RT.second);
			// LB X
			int LB_index = 0;
			if (FindUpperTarget(Input_Rect.LB.first, layer, true, LB_index))
			{
				Output_Index_Range.LB.first =  LB_index;
			}
			else{
				if(Input_Rect.LB.first <= fast_coor_map.at(layer).first.at(0)){
					Output_Index_Range.LB.first = 0;
				}
				else if(Input_Rect.LB.first >= fast_coor_map.at(layer).first.at(fast_coor_map.at(layer).first.size() - 1))
				{
					Output_Index_Range.LB.first = fast_coor_map.at(layer).first.size() - 1;
				}
				else{
					printf("LBX Mapping Error LB X = (%d) , Layout x range %d ~ %d\n", Input_Rect.LB.first,fast_coor_map.at(layer).first.at(0),fast_coor_map.at(layer).first.at(fast_coor_map.at(layer).first.size() - 1));
					exit(1);
				}
			}
			//printf("LBY\n");
			// LB Y
			int LB_index2 = 0;
			if (FindUpperTarget(Input_Rect.LB.second, layer, false, LB_index2))
			{
				Output_Index_Range.LB.second = LB_index2;
			}
			else{
				if(Input_Rect.LB.second <= fast_coor_map.at(layer).second.at(0)){
					Output_Index_Range.LB.second = 0;
				}
				else if(Input_Rect.LB.second >= fast_coor_map.at(layer).second.at(fast_coor_map.at(layer).second.size() - 1))
				{
					Output_Index_Range.LB.second = fast_coor_map.at(layer).second.size() - 1;
				}
				else{
					printf("LBY Mapping Error LB Y = (%d) , Layout x range %d ~ %d\n", Input_Rect.LB.second,fast_coor_map.at(layer).second.at(0),fast_coor_map.at(layer).second.at(fast_coor_map.at(layer).second.size() - 1));
					exit(1);
				}
			}
			// RT X
			if(FindLowerTarget(Input_Rect.RT.first,layer,true,LB_index)){
				Output_Index_Range.RT.first =  LB_index;
			}
			else{
				if(Input_Rect.RT.first <= fast_coor_map.at(layer).first.at(0)){
					Output_Index_Range.RT.first = 0;
				}
				else if(Input_Rect.RT.first >= fast_coor_map.at(layer).first.at(fast_coor_map.at(layer).first.size() - 1))
				{
					Output_Index_Range.RT.first = fast_coor_map.at(layer).first.size() - 1;
				}
				else{
					printf("RTX Mapping Error RT X = (%d) , Layout x range %d ~ %d\n", Input_Rect.RT.first,fast_coor_map.at(layer).first.at(0),fast_coor_map.at(layer).first.at(fast_coor_map.at(layer).first.size() - 1));
					exit(1);
				}
			}

			// RT Y
			if(FindLowerTarget(Input_Rect.RT.second,layer,false,LB_index)){
				Output_Index_Range.RT.second =  LB_index;
			}
			else{
				if(Input_Rect.RT.second <= fast_coor_map.at(layer).second.at(0)){
					Output_Index_Range.RT.second = 0;
				}
				else if(Input_Rect.RT.second >= fast_coor_map.at(layer).second.at(fast_coor_map.at(layer).second.size() - 1))
				{
					Output_Index_Range.RT.second = fast_coor_map.at(layer).second.size() - 1;
				}
				else{
					printf("RTY Mapping Error RT Y = (%d) , Layout x range %d ~ %d\n", Input_Rect.RT.second,fast_coor_map.at(layer).second.at(0),fast_coor_map.at(layer).second.at(fast_coor_map.at(layer).second.size() - 1));
					exit(1);
				}
			}
			//printf("block LB(%d,%d) RT(%d,%d)\n", Output_Index_Range.LB.first, Output_Index_Range.LB.second, Output_Index_Range.RT.first, Output_Index_Range.RT.second);

			//printf("End MapRect_ShrinkedRange \n");
		}

		// Map the inflated(upper bound) range of the rectangle in layout
		void MapRect_InflatedRange(Parser::I_Rect &Input_Rect,Parser::I_Rect &Output_Index_Range){
;
		}
};

struct Solution {
	int short_num = INT_MAX;
	int open_num = INT_MAX;
	int overlap_violation_num = INT_MAX;
	int line_end_violation_num = INT_MAX;
	vector<TwoPinRipUpNet> nets;

	void SetSolution(
		const int short_val = INT_MAX, 
		const int open_val = INT_MAX, 
		const int overlap_val = INT_MAX, 
		const int line_end_val = INT_MAX
	);
	void SetNets(vector<TwoPinRipUpNet> &nets_val);
	void operator = (const Solution &sol);
	void PrintSolution();
	friend bool IsBetterSolution(const Solution &a, const Solution &b);

};

// Temporary net info for rip-up-and-reroute
struct TwoPinRerouteNet 
{
	int index;
	int two_pin_net_id;
	int rip_up_times;
	int violation_num;
	int wire_length;
	float net_score;
	float straight_ratio;
	bool is_straight;
	vector<Node> history_node;
	//****** Line-end
	int line_end_violation_num;
	vector<TurnNode> turn_node;
	vector<Node> via_enc_vio_nodes;

	void clear() 
	{
		turn_node.clear();
		via_enc_vio_nodes.clear();
	}
};


inline bool LessWireLength(
		const TwoPinRerouteNet &lhs, const TwoPinRerouteNet &rhs)
{ return (lhs.wire_length < rhs.wire_length); }


inline bool LessNetScore(
		const TwoPinRerouteNet &lhs, const TwoPinRerouteNet &rhs)
{ return (lhs.net_score < rhs.net_score); }

inline bool LessLineEndVioaltion(
		const TwoPinRerouteNet &lhs, const TwoPinRerouteNet &rhs)
{ return (lhs.line_end_violation_num < rhs.line_end_violation_num); }

inline bool MoreStraight(
		const TwoPinRerouteNet &lhs, const TwoPinRerouteNet &rhs)
{
	return (lhs.straight_ratio > rhs.straight_ratio);
}

inline bool NoBend(
		const TwoPinRerouteNet &lhs, const TwoPinRerouteNet &rhs)
{
	if(lhs.is_straight && !rhs.is_straight)
		return true;
	else if(!lhs.is_straight && rhs.is_straight)
		return false;
	else
		return (lhs.wire_length < rhs.wire_length);
}


class SmallRouter {
 public:

    // Ctor
	SmallRouter();
	SmallRouter(
		const int layout_width,
		const int layout_height,
		const int layout_layer,
		vector<TwoPinRipUpNet> &two_pin_rip_up_net,
		vector<Node> &obstacle_nodes,
		vector<vector<vector<MazeNode>>> &GridMap_layout,
		Real_Coor_Table &real_coor_table,
		RaisingFalling_Connection_Table &connection_table,
		vector<pair<map<coor_t, int>, map<coor_t, int>>> &fast_coor_map,
		vector<pair<Parser::Track, Parser::Track>> &Layer_track_list,
		vector<int> &ISPD_Via_cut_Spacing,
		vector<vector<pair<int, int>>> &Spacing_Table,
		vector<Parser::INT_EOL_Rule> &ISPD_EndOfLine_Spacing,
		vector<vector<Parser::Via>> &ISPD_via_type_list,
		vector<vector<vector<int>>> &Blocage_Map,
		vector<vector<vector<int>>> &ISPD_EOL_Blocage_Map,
		vector<SpaceEvaluationGraph> &_SpaceEvaluationLayout_,
		vector < vector < vector < Parser::Enc_Relation > > >  &Enc_Relation_Pair_Table,
		Parser::EncId_HashTable &_EncId_HashTable_,
		vector<int> MinWidth_list,
		vector<vector<vector<pair<int, int>>>> &Pin_Cover_Map,
		vector<vector<vector<int>>> &ISPD_original_iroute_array,
		vector<vector<vector<bool>>> &ISPD_L2_OBS_array_,
		const bool Metal1Dir,
		const bool kLineEndModeRouting,
		const bool kEndEndSpacingMode);

	// Dtor
   // ~SmallRouter();
	void clear()
	{
		for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
			for (int j = 0; j < (*ISPD_GridMap_layout).at(i).size(); j++)
				(*ISPD_GridMap_layout).at(i).at(j).clear();
		(*ISPD_GridMap_layout).clear();

		for (int i = 0; i < (*ISPD_fast_coor_map).size(); i++)
		{
			(*ISPD_fast_coor_map).at(i).first.clear();
			(*ISPD_fast_coor_map).at(i).second.clear();
		}
		(*ISPD_fast_coor_map).clear();

		ISPD_Layer_track_list.clear();
		prefer_dir.clear();
		ISPD_Via_cut_Spacing.clear();

		for (int i = 0; i < ISPD_Spacing_Table.size(); i++)
			ISPD_Spacing_Table.at(i).clear();
		ISPD_Spacing_Table.clear();

		ISPD_EndOfLine_Spacing.clear();

		for (int i = 0; i < ISPD_via_type_list.size(); i++)
			ISPD_via_type_list.at(i).clear();
		ISPD_via_type_list.clear();

		for (int i = 0; i < ISPD_Viablockage_array_.size(); i++)
			for (int j = 0; j < ISPD_Viablockage_array_.at(i).size(); j++)
				ISPD_Viablockage_array_.at(i).at(j).clear();
		ISPD_Viablockage_array_.clear();

		for (int i = 0; i < (*ISPD_blockage_array_).size(); i++)
			for (int j = 0; j < (*ISPD_blockage_array_).at(i).size(); j++)
				(*ISPD_blockage_array_).at(i).at(j).clear();
		(*ISPD_blockage_array_).clear();

		for (int i = 0; i < (*ISPD_EOL_Blocage_Map).size(); i++)
			for (int j = 0; j < (*ISPD_EOL_Blocage_Map).at(i).size(); j++)
				(*ISPD_EOL_Blocage_Map).at(i).at(j).clear();
		(*ISPD_EOL_Blocage_Map).clear();

		track_capacity_ = NULL;
		cost_array_ = NULL;
		back_track_array_ = NULL;

		for (int i = 0; i < ISPD_track_capacity_.size(); i++)
			for (int j = 0; j < ISPD_track_capacity_.at(i).size(); j++)
				ISPD_track_capacity_.at(i).at(j).clear();
		ISPD_track_capacity_.clear();

		for (int i = 0; i < ISPD_cost_array_.size(); i++)
			for (int j = 0; j < ISPD_cost_array_.at(i).size(); j++)
				ISPD_cost_array_.at(i).at(j).clear();
		ISPD_cost_array_.clear();

		for (int i = 0; i < ISPD_back_track_array_.size(); i++)
			for (int j = 0; j < ISPD_back_track_array_.at(i).size(); j++)
				ISPD_back_track_array_.at(i).at(j).clear();
		ISPD_back_track_array_.clear();

		for (int i = 0; i < ISPD_back_track_Step_array_.size(); i++)
			for (int j = 0; j < ISPD_back_track_Step_array_.at(i).size(); j++)
				ISPD_back_track_Step_array_.at(i).at(j).clear();
		ISPD_back_track_Step_array_.clear();

		for (int i = 0; i < ISPD_Viatype_array_.size(); i++)
			for (int j = 0; j < ISPD_Viatype_array_.at(i).size(); j++)
				ISPD_Viatype_array_.at(i).at(j).clear();
		ISPD_Viatype_array_.clear();

		for (int i = 0; i < SpaceEvaluationLayout.size(); i++)
		{
			SpaceEvaluationLayout.at(i).clear();
			SpaceEvaluationLayout.at(i).tree = NULL;
			SpaceEvaluationLayout.at(i).EOL_tree = NULL;
		}
		SpaceEvaluationLayout.clear();
		ISPD_MinWidth_list.clear();
		obstacle_.clear();

		CachePinTable.clear();
		Cache_pin_path_list.clear();

		for (int i = 0; i < rip_up_two_pin_nets_.size(); i++)
			rip_up_two_pin_nets_.at(i).clear();
		rip_up_two_pin_nets_.clear();

		for (int i = 0; i < reroute_two_pin_net_.size(); i++)
			reroute_two_pin_net_.at(i).clear();
		reroute_two_pin_net_.clear();

	}
	
    RectMapFunction RectangleMapping;
	RaisingFalling_Connection_Table ISPD_connection_table;
	// Set the preferred direction
	void
	SetPreferredDirection(
		const bool m1_is_verical,
		const bool m2_is_verical);
	// Set the required line-end spacing.
	void SetLineEndSpacing(
			RoutingLayerInfos &routing_layer_infos, 
			const int low_layer,
			const int high_layer
			);
	// Set the via enclosure expansion.
	void SetViaEncExpansion(
			RoutingLayerInfos &routing_layer_infos,
			const int low_layer,
			const int high_layer
			);
	
	// Import file for testing and delete the vector
	void ImportFile(const char* input_file_name);
	
	// Routing main function
	void StartRouting(
			RoutingLayerInfos &routing_layer_infos,
			const int low_layer,
			const int high_layer
			);
	void ISPD_StartRouting();
	
	// Output file
	// void OutputGDSFile(const char *output_file_name);
	// void OutputPinGDT(const char *output_file);
	// float ConvertToDBU(const int coord, const int DBU);


	inline void get_two_pin_rip_up_net(
			vector<TwoPinRipUpNet> &ripup_nets) const 
	{ 
		ripup_nets.assign(
				rip_up_two_pin_nets_.begin(), 
				rip_up_two_pin_nets_.end());
	}
 private:
 
	void set_track_capacity();
	void set_cost_array();
	void set_back_track_array();
	void set_pin_array();
	void set_id_table();
	void ISPD_set_cost_array();
	void ISPD_set_track_capacity();
	void ISPD_set_back_track_array();
	void ISPD_set_pin_array();
	void ISPD_set_id_table();
	inline void reset_cost_array(const float value = FLT_MAX);
	inline void reset_back_track_array();

	// Functions for initial settings
	// fix the pin location to track_capacity_
	void SetNetPinsToTrack();
	// set the blockage to track_capacity_
	void SetObstacleToTrack();
	// Init line-end related info
	void InitPinLocationLineEndGraph();

 
	// Main Flow
	void FirstPhaseRouting();
	void SecondPhaseRouting();
	void ISPD_FirstPhaseRouting();
 
	// Routing Algorithm Usage
	// Set the bounding box to constrain the search area.
	inline void SetBoundingBox(
			BoundingBox &bounding_box, 
			const TwoPinNetConnection &connection, 
			const int rip_up_times
			);
	// Set to be full a bounding box.
	inline void SetBoundingBox(
			BoundingBox &bounding_box
			);
	// Set the scores of nets for sorting net sequence.
	void SetNetScore(
			const int iteration_times, 
			const int max_iteration_times
			);
	// Search a path of a 2-pin net.
	void MazeRouting3D(
			const int two_pin_net_id, 
			const int two_pin_net_index,
			const Node &source, 
			const Node &target,
			const BoundingBox &bounding_box
			);
	void MazeRouting3D(
			const int two_pin_net_id, 
			const int two_pin_net_index,
			const TwoPinNetConnection &connection,
			const BoundingBox &bounding_box,
			Node &target_point
			);

	void ISPD_Setting_Switching_Node(const vector<Parser::I_Rect> &GlobalGuide,
		vector < vector <Node> > &switching_node_Raise, vector < vector <Node> > &switching_node_Fall, int range = 1);
	bool NodeCanGo(int x, int y, int z, bool UpperSwitching, bool LowerSwitching);
	Node ToNextNode(Node current_node, int direction, vector < vector <Parser::I_Rect> > &Layer_Global_guide);
	void ISPD_Congestion_Map_Estimation_Based_Routing();
	int ISPD_MazeRouting3D(
		const int two_pin_net_id,
		const int two_pin_net_index,
		const TwoPinNetConnection &connection,
		const vector<Parser::I_Rect> &GlobalGuide,
		const Parser::I_Rect &BoundingBox,
		Node &target_point,
		map<tuple<int, int, int>, Parser::PathAndEnc> SourceRefinmentMap,
		map<Node, Parser::PathAndEnc> &TargetRefinmentMap,
		map<tuple<int, int, int>, Parser::PathAndEnc> &RefinmentMap,
		bool SrcIsIRoute,
		vector <Pin> &ValidPinList,
		Parser::I_Rect &Target_Box);
	bool LiteBackTrack(
		const int two_pin_net_index,
		const Node &back_target_point,
		TwoPinNetConnection &connection,
		const int net_id
		);
	int ISPD_MazeRouting3D_Allow_Short(
		const int two_pin_net_id,
		const int two_pin_net_index,
		const TwoPinNetConnection &connection,
		const vector<Parser::I_Rect> &GlobalGuide,
		const Parser::I_Rect &BoundingBox,
		Node &target_point,
		map<tuple<int, int, int>, Parser::PathAndEnc> &SourceRefinmentMap,
		map<Node, Parser::PathAndEnc> &TargetRefinmentMap, bool isIroute, Parser::I_Rect &Target_Box);
	float ISPD_UpdateCost_Allow_Short(const Node &current_node, const Node &next_node, const int direction, const bool is_via_direction,
									  const int two_pin_net_id, const int two_pin_net_index, vector<vector<Parser::I_Rect>> &Layer_Global_guide, map<Node, int> &TargetTable, map<tuple<int, int, int>, Parser::PathAndEnc> &Refinementmap, Parser::I_Rect &TargetBox);
	void ISPD_MazeRouting3D_P2P(
		const int two_pin_net_id,
		const int two_pin_net_index,
		const Node &source,
		const Node &target,
		vector <Parser::I_Rect> &Global_guide
		);
	void SetSourcePin_Allow_Short(const TwoPinNetConnection &connection,
								  map<Node, Parser::PathAndEnc> &PseudoSource_PathTable,
								  vector<Node> &maze_queue, int two_pin_net_id, bool isIroute );
	bool MemBlockRefine(vector<Parser::I_Rect> &PinShape, Node &node, Parser::wire_path &RefinePath);
	int BestVia(Parser::I_Rect PinShape, Node &src);
	void PathConstruction(list<Node> &detail_grid_path, int two_pin_net_index,
						  vector<Parser::wire_path> &wire_list, vector<Parser::I_Rect> &Patch, vector<Parser::wire_path> &This_wire_list);
	void Push_path_into_SEG(vector<Parser::wire_path> This_wire_list, int two_pin_net_id);
	bool OffGridViaPrediction_Allow_Short(Node &pseudo_target, const vector<Parser::I_Rect> PinShape, vector<Parser::wire_path> &OffGridPath, Parser::I_Rect &Enc, pair<int, int> &via_type_pair, int net_id);

	// Check whether the next node are blocked.
	inline bool IsNodeBlocked(
		const Node &next_node,
		const BoundingBox &bounding_box,
		const int two_pin_net_id,
		const int direction);
	// Return the current cost of the grid.
	/*inline float UpdateOverlappedCost(
			const MazeNode &maze_node,
			const int two_pin_net_id, 
			const int two_pin_net_index
			);
	inline float UpdateViaEncOverlappedCost(
			const Node &node,
			const int two_pin_net_id, 
			const int two_pin_net_index
			);
	inline float UpdateCost(
			const MazeNode &current_node,
			const MazeNode &next_node,
			const int direction,
			const bool is_via_direction,
			const int two_pin_net_id,
			const int two_pin_net_index
			);*/
	float ISPD_UpdateCost(const Node &current_node, const Node &next_node, const int direction, const bool is_via_direction,
						  const int two_pin_net_id, const int two_pin_net_index, vector<vector<Parser::I_Rect>> &Layer_Global_guide, map<Node, int> &TargetTable, map<tuple<int, int, int>, Parser::PathAndEnc> &Refinementmap, Parser::I_Rect &TargetBox);
	/*inline void GetViaEncNodes(
			const int direction,
			const MazeNode &current_node,
			const MazeNode &next_node,
			vector<Node> &enc_nodes
			);*/
	void PseusoNodeDetermin(TwoPinNetConnection &connection,
							Node &source, Node &Target, int net_id,
							Parser::I_Rect &SrcEnc, Parser::I_Rect &TarEnc, map<Node, Parser::PathAndEnc > &SourceRefinmentMap,
							map<Node, Parser::PathAndEnc> &TargetRefinmentMap);

	// Back Track After Searching
	bool BackTrack(
			const int two_pin_net_index,
			const int from_x, const int from_y, const int from_z, 
			const int to_x, const int to_y, const int to_z
			); 
	bool BackTrack(
			const int two_pin_net_index,
			const Node &source,
			const Node &target
			);
	bool BackTrack(
		const int two_pin_net_index,
		const Node &back_target_point,
		TwoPinNetConnection &connection,
		const int net_id, Parser::I_Rect &SrcEnc, Parser::I_Rect &TarEnc,
		Node &source , Node &target
		);
	void SettingBoundBoxGuide(const vector<vector<Parser::I_Rect>> &Layer_Global_guide, vector<Node> maze_queue, vector<Parser::I_Rect> &BB); 
	bool NodeInBoundingBox(const vector<Parser::I_Rect> &BoundingBox, Node &next);


	// Find the Turns of a Path after Backtracing.
	void FindPathTurns(const int two_pin_net_index);
	inline void GetTurnNodes(
			const bool is_rbegin_node,
			const Node &node, 
			const Node &pre_node, 
			const Node &post_node,
			vector<TurnNode> &turn_nodes
			);


	int Net_Open_Num;
	// Rip-Up-and-Reroute Scheme
	// 1. used in the 1st phase
	void InitialRerouteTwoPinNet();
	// 2. used in the following phases
	void SetRerouteTwoPinNet();
	// Reroute part.
	void SetTwoPinNetPathToTrack(
			const int two_pin_net_index, 
			const int two_pin_net_id
			);
	void GetRightGridAfterReroute(
			const Node &path_node,
			const int two_pin_net_id
			);
	// Rip-Up part.
	void RipUpTwoPinNetFromTrack(
			const int two_pin_net_index, 
			const int two_pin_net_id
			);
	int GetRightGridNumberAfterRipUp(
			const ULL grid_index, 
			const int grid_number, 
			const int two_pin_net_id
			);
	// Check history node.
	void ReCheckAndUpdateHistoryGraph(
			const vector<Node>& history_node
			);

	// Misc usage.
	inline int CountMultiNetNumber();
	inline int CountLayoutViolationNumber();
	inline int CountViolationNumber(const int two_pin_net_index); 
	inline int CountWireLength(const int two_pin_net_index); 
	void UpdateRipUpTwoPinNetFromRerouteTwoPinNet(); 
	
	// Track Index Transform Usage
	inline ULL Transform3dArrayTo1dIndex(
			const int x, 
			const int y, 
			const int z
			);
	inline ULL Transform3dArrayTo1dIndex(const Node &node);
	//inline ULL Transform3dArrayTo1dIndex(const MazeNode &maze_node);
	inline ULL Transform3dArrayTo1dIndex(const TurnNode &turn_node);
	
	inline void Transform1dTo3dIndex(ULL index);
	
	// Report the Result 
	void AnalyzeTheResult(
			int &short_net_num, 
			int &open_net_num, 
			int &overlap_violation_num
			);
 
 
	// Display Net Info
	void ShowRerouteNetInfo(const int rip_up_times);

	// Only For Debug
	void PrintLayoutMap();
	void PrintTrackGraph();
	void PrintTrackGraphViolation();
	void PrintTrackNetIndex();
	void PrintCostArray();
	void PrintBackTrackArray();
	void PrintNetPins();
	
	// Preferred direction
	int kOddLayer = 0;
	int kEvenLayer = 1;
	
	// Mode selection
	const bool kLineEndModeRouting_;
	const bool kEndEndSpacingMode_;
	const bool kViaEncCostMode_ = true;
	vector<bool> kViaEncExpansion_;
 
	// Tuning Parameters
	int total_rip_up_times;
	const int kBlockageTimes_ = 3;
	const int kSecondPhaseRipUpIteration_ = 1000;
	
	const float kBaseCost_ = 1.0;
	const float kViaCost_ = 3.0;
	const float kOverlapBeta_ = 2.0;
	const float kOverlapSlope_ = -1.0;
	const float kOverlapOffset_ = -1.0;
	const float kViaEncOverlapBeta_ = 2.0;
	// const float kViaEncOverlapBeta_ = 3.0;
	const float kViaEncOverlapSlope_ = -1.0;
	const float kViaEncOverlapOffset_ = -1.0;
	const int kIgnoreHistoryInterval = 9;

	const float kNetScoreAlpha_ = 1.75; // Wire-Length
	const float kNetScoreBeta_ = 2.0; // Violation
	const float kNetScoreGama_ = 1.0; // Via Count

	const double kBBoxAlpha_ = 9;
	const double kBBoxBeta_ = 5.0;
 
    // 3D layout info
	int layout_width_;
	int layout_height_;
	int layout_layer_;
	vector< vector < vector < MazeNode > > > *ISPD_GridMap_layout;
	Real_Coor_Table ISPD_real_coor_table;
	vector < pair < map<coor_t, int>, map<coor_t, int > > > *ISPD_fast_coor_map;
	vector < pair<Parser::Track, Parser::Track> > ISPD_Layer_track_list;
	vector <bool> prefer_dir; // 0 : horzontal , 1 : vertical
	vector <int> ISPD_Via_cut_Spacing;
	vector < vector < pair <int, int> > > ISPD_Spacing_Table;
	vector <Parser::INT_EOL_Rule> ISPD_EndOfLine_Spacing;
	vector < vector <Parser::Via> > ISPD_via_type_list;
	vector<vector<vector<bool>>> ISPD_Target_Map;
	bool ***ISPD_Subtree_Map;
	vector < vector < vector <pair<int, int> > > > *ISPD_Congestion_Map;
	vector < vector < vector <int > > > ISPD_iroute_array;
	vector < vector < vector <bool > > > ISPD_L2_OBS_array;
	Parser::EncId_HashTable ISPD_enc_id_HashTable;
	vector < vector < vector < Parser::Enc_Relation > > >  ISPD_Enc_Relation_Pair_Table;
	vector < vector < vector < ViaTypeBit > > > ISPD_Via_Type_Map;
	bool VIA_BIT_MAP_CHECKING(const Node &node, pair <int,int> type , int net_id, I_Rect &Enclosure,bool up_down/* up : true , down : false*/);
	void ISPD_set_BitViaType_array_();
	// ISPD Blockage
	vector<vector<vector< int > > > ISPD_origin_blockage_array_; // 3D vector to note blockage
	vector<vector<vector< int > > > ISPD_origin_EOL_Blocage_Map;
	vector< vector < vector < ViaBlock > > >  ISPD_Viablockage_array_; // 3D vector to note via blockage
	vector< vector < vector <int> > > *ISPD_blockage_array_; // 3D vector to note blockage
	vector < vector < vector <int> > > *ISPD_EOL_Blocage_Map;
	void Push_path_into_CongestionMap(list<Node> &detail_grid_path, int id);
	void Unmark_Blockage(const vector <Parser::I_Rect> &PinShape);
	void Remark_Blockage(const vector <Parser::I_Rect> &PinShape);
	void ExtractViaFromBackTrack(int back_from_z, int back_from_y, int back_from_x, Parser::I_Rect &Enc, bool src_tar);
	bool Stack_Via_Estimation(const Node &cur_node);
	void LongerStackVia(Node &cur, wire_path & addition);
	// Main algorithm
	int *track_capacity_;
	float *cost_array_;
	int *back_track_array_;
	// For ISPD
#ifdef FIB_HEAP
	vector<vector<vector<node<Node> *> > > ISPD_Fib_Node_Location;
	void ISPD_set_Fib_Node_Location_array_()
	{
		ISPD_Fib_Node_Location.resize((*ISPD_GridMap_layout).size());
		ISPD_Node_In_Heap.resize((*ISPD_GridMap_layout).size());

		for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
		{
			ISPD_Fib_Node_Location[i].resize((*ISPD_GridMap_layout)[i].size());

			for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
			{
				ISPD_Fib_Node_Location[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
				
			}
		}
		for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
		{
			for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
			{
				for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++){
					ISPD_Fib_Node_Location[i][j][k] = NULL;
				}
			}
		}
	}
#endif
#ifdef NCTU_GR_HEAP
	vector<vector<vector< pheap_el<Node> *> > > ISPD_Fib_Node_Location;
	vector<vector<vector<bool> > > ISPD_Node_In_Heap;
	void ISPD_set_Fib_Node_Location_array_()
	{
		ISPD_Fib_Node_Location.resize((*ISPD_GridMap_layout).size());
		ISPD_Node_In_Heap.resize((*ISPD_GridMap_layout).size());

		for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
		{
			ISPD_Fib_Node_Location[i].resize((*ISPD_GridMap_layout)[i].size());

			ISPD_Node_In_Heap[i].resize((*ISPD_GridMap_layout)[i].size());

			for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
			{
				ISPD_Fib_Node_Location[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
				ISPD_Node_In_Heap[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
			}
		}
		for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
		{
			for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
			{
				for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++)
				{

					ISPD_Fib_Node_Location[i][j][k] = NULL;
					ISPD_Node_In_Heap[i][j][k] = false;
				}
			}
		}
	}
#endif														  // FIB_HEAP
	vector<vector<vector<int>>> ISPD_track_capacity_;		  // 3D vector to note
	vector< vector < vector <float> > > ISPD_cost_array_;     // 3D vector to note node cost
	vector<vector<vector<int8_t> > > ISPD_back_track_array_;	// 3D vector to note back track
	vector<vector<vector<int8_t> > > ISPD_back_track_Step_array_; // 3D vector to note back track step
	bool ***ISPD_ViaHit_array_;
	bool ***ISPD_PathHit_array_;
	vector<tuple<int, int, int>> ISPD_modify_list;
	vector<tuple<int, int, int>> ISPD_TargetMap_modify_list;
	vector<tuple<int, int, int>> ISPD_ViaHit_modify_list;
	vector<tuple<int, int, int>> ISPD_PathHit_modify_list;
	vector<tuple<int, int, int>> ISPD_Subtree_modify_list;
	void ResetFunction();
	void ISPD_Net_Reset();
	void ISPD_set_ViaHit_array_()
	{
		ISPD_ViaHit_array_ = new bool**[(*ISPD_GridMap_layout).size()];
		for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
		{
			ISPD_ViaHit_array_[i] = new bool *[(*ISPD_GridMap_layout)[i].size()];
			for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
			{
				ISPD_ViaHit_array_[i][j] = new bool[(*ISPD_GridMap_layout)[i][j].size()];
			}
		}
		for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
		{
			for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
			{
				std::memset(this->ISPD_ViaHit_array_[i][j], 0, sizeof(bool) * (*ISPD_GridMap_layout)[i][j].size());
			}
		}
	}
	void ISPD_reset_ViaHit_array_(){
		for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
		{
			for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
			{
				std::memset(this->ISPD_ViaHit_array_[i][j], 0, sizeof(bool) * (*ISPD_GridMap_layout)[i][j].size());
			}
		}
	}
	void ISPD_set_PathHit_array_()
	{
		ISPD_PathHit_array_ = new bool **[(*ISPD_GridMap_layout).size()];
		for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
		{
			ISPD_PathHit_array_[i] = new bool *[(*ISPD_GridMap_layout)[i].size()];
			for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
			{
				ISPD_PathHit_array_[i][j] = new bool[(*ISPD_GridMap_layout)[i][j].size()];
			}
		}
		for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
		{
			for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
			{
				std::memset(this->ISPD_PathHit_array_[i][j], 0, sizeof(bool) * (*ISPD_GridMap_layout)[i][j].size());
			}
		}
	}
	void ISPD_reset_PathHit_array_()
	{
		for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
		{
			for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
			{
				std::memset(this->ISPD_PathHit_array_[i][j], 0, sizeof(bool) * (*ISPD_GridMap_layout)[i][j].size());
			}
		}
	}
	vector<vector<vector<ViaTypeNote> > > ISPD_Viatype_array_; // 3D vector to note via type when back track
	void SetViaEnclosureBlockage(Node back_from, Node new_node, int dir, pair <int, int> via_id,int net_id);
	void OffGridViaRefinment(Node &pseudo_target, vector <Parser::I_Rect> &PinShape, wire_path &offGrid_Refine/*Must be type 1*/,int net_id);
	bool OffGridViaPrediction(Node &pseudo_target, const vector<Parser::I_Rect> PinShape, vector<Parser::wire_path> &OffGridPath, Parser::I_Rect &Enc, pair <int,int> &via_type_pair,int net_id);
	bool OffGridViaEnclosureChecking(int _x_coor_, int _y_coor_, int cur_layer, int next_layer, pair<int, int> type, int net_id);
	void HitPatchMetal(vector<Parser::I_Rect> &sourcePin, vector<Parser::I_Rect> &targetPin,
					   vector<Parser::wire_path> &wirePath);
	void ISPD_set_Viatype_array_();
	bool TargetTableFind(Node &node, bool IsSrcIRoute, int two_pin_net_id, I_Rect &Target_Box);
	void ISPD_reset_Viatype_array_();
	void ISPD_set_Target_array_();
	void ISPD_reset_Target_array_();
	void ISPD_Insert_Target_array_(vector<Pin> &element);
	void ISPD_set_Subtree_array_();
	void ISPD_reset_Subtree_array_();

	void ISPD_reset_Subtree_ViaHit_PathHit_array_();
	void ISPD_Free_Subtree_ViaHit_PathHit_array_();
	void ISPD_Insert_Subtree_array_(vector<Pin> &element);
	void ISPD_Insert_ViaHit_array_(int &x, int &y, int z);
	void ISPD_Insert_PathHit_array_(int &x, int &y, int z);
	//bool ***ISPD_True_Target_Map_;   // 3D vector to note 
	void ISPD_reset_TrueTarget_array_();
	void PseusoNodeDetermin_FromNetTable_Short(TwoPinNetConnection &connection, Node &source,
															Node &Target, int net_id, Parser::I_Rect &SrcEnc, Parser::I_Rect &TarEnc,
															map<tuple<int, int, int>, Parser::PathAndEnc> &RefinmentMap,
															map<tuple<int, int, int>, Parser::PathAndEnc> &SourceRefinmentMap);
	void SetSourcePin_Allow_Short_v2(const TwoPinNetConnection &connection,
												  map<tuple<int, int, int>, Parser::PathAndEnc> &Source_PathTable,
												  vector<Node> &maze_queue, int two_pin_net_id, bool isIroute);
	void ISPD_set_Congestion_map_(vector<vector<vector<pair<int, int>>>> &Pin_Cover_Map);
	void ISPD_reset_Congestion_map_();
	void ISPD_Add_Congestion_map_(int x, int y, int z, int AddNum);
	void ISPD_Restore_Origin_Blockage_array(vector<vector<vector<int>>> &Blocage_Map,
											vector<vector<vector<int>>> &EOL_Blocage_Map);
	void PseusoNodeDetermin_FromNetTable(TwoPinNetConnection &connection, Node &source,
													  Node &Target, int net_id, Parser::I_Rect &SrcEnc, Parser::I_Rect &TarEnc,
													  map<tuple<int, int, int>, Parser::PathAndEnc> &RefinmentMap);
	void ISPD_SetNetPinRefinement(
		TwoPinRipUpNet &ripup_net,
		map<tuple<int, int, int>, Parser::PathAndEnc> &Net_Pin_Corresponding_Refinement,
		int net_id);
	void FindFirstValidTarget(TwoPinRipUpNet &ripup_net,
										   map<tuple<int, int, int>, Parser::PathAndEnc> &Net_Pin_Corresponding_Refinement,
										   vector<Pin> &Valid_Target,
										   int Pin_index);
	void ISPD_IRouteRevise(TwoPinRipUpNet &ripup_net, int &PinIndex);
	void ISPD_set_blockage_array_(vector<vector<vector<int> > > &Blocage_Map);
	void ISPD_set_eol_blockage_array_(vector < vector < vector <int> > > &EOL_Blocage_Map);
	bool ViaEnclosureChecking(const Node &cur_node, const Node &next_node, pair <int, int> type, int net_id);
	vector <SpaceEvaluationGraph> SpaceEvaluationLayout;
	void ISPD_HitRectPatch(Parser::I_Rect &Enclosure, vector <Parser::I_Rect> &PinShape, vector <Parser::I_Rect> &AdditionPatch,int net_id);
	vector <int> ISPD_MinWidth_list;
	void ISPD_set_Viablockage_array_();
	bool ISPD_IsNodeBlocked(
		const Node &cur_node,
		const Node &next_node,
		const int two_pin_net_id,
		const int direction,
		const int via_type,
		vector <Parser::I_Rect> &NetShape
		);
	void SetValidSourcePin(
		const TwoPinNetConnection &connection,
		map<tuple<int, int, int>, Parser::PathAndEnc> &RefinementTable,
		vector<Node> &maze_queue, int two_pin_net_id, bool SrcIsIRoute);
	void SetValidTargetPin(
		const TwoPinNetConnection &connection,
		map<Node, Parser::PathAndEnc> &PseudoTarget_PathTable,
		int two_pin_net_id);
	void DetailPath2RipUpPath(list<Node> &detail_grid_path, TwoPinRipUpNet &ripup_net, int net_index);
	void Push_path_into_blockage(list<Node> &detail_grid_path, int id);
	void ISPD_set_back_track_Step_array_();
	void ISPD_reset_back_track_Step_array_(int value);
	// For Routing Algorithm
	HistoryCongestionGraph history_congestion_graph_;
	void ISPD_set_history_congestion_graph_();
	void PushRectIntoBlockage(Parser::I_Rect &block, int net_id, int layer);
	void ExtractLineEndBlock(Parser::I_Rect &block,int net_id);
	LayoutMap layout_map_;
	NetIdTable net_id_table_;
	// line-end map
	LineEndGraph line_end_graph_;

	// Nets To Be Rip Up and Reroute
	// Origin two-pin net from window.
	vector<TwoPinRipUpNet> rip_up_two_pin_nets_;
	// The two-pin net to be rerouted.
	vector<TwoPinRerouteNet> reroute_two_pin_net_; 
	vector<Node> obstacle_;

	//speed up cache
	map<tuple<int, int, int>, bool> CachePinTable;
	vector<Pin> Cache_pin_path_list;
	// for marking target on bitwise
	map<tuple<int, int, int>, bool> TempTargetTable;
	vector<Pin> Temp_target_path_list;
	// for rip-up & reroute
	vector<vector<vector<int>>> ISPD_NetID_array; // 3D vector to note path net id
	void ISPD_set_NetID_array_();
	void ISPD_reset_NetID_array_();
	void ISPD_reset_2PinNetStatus();
	void ISPD_RipUp2PinNet(int RipUpNet_index);

  public:
	vector < vector <Parser::wire_path> > Net_wirePath;
	int Net_Num;
	// Record the Best Solution
	Solution current_solution_;
	Solution best_solution_;
};


// The two terminals of a 2-pin net
/*struct TwoPinNetConnection {

	// for OA identification
	//oaBox source_box;
	//oaBox target_box;

	// replace oaBox
	Box source_box;
	Box target_box;

	// mapping pin position
	Pin source;
	Pin target;

	// a table that recognize the target nodes
	unordered_set<ULL> target_table;

	TwoPinNetConnection();
	TwoPinNetConnection(
		TwoPinNetConnection &connection
		);

	void SetTargetTable(
		const int layout_width,
		const int layout_height,
		const int layout_layer
		);
	void EraseTargetKey(const ULL target_key);
	int GetEstimatedWireLength() const;

	bool DoesSourceOverlapTarget() const;
	bool IsSourceEqualToTarget() const;
	bool IsStraight() const;
	float GetStraightRatio() const;

	void PrintTargetTable();
	inline bool IsTargetReached(ULL index_key) const
	{
		const bool is_target_reached =
			(this->target_table.find(index_key) !=
			this->target_table.end());
		return is_target_reached;
	}

};


// The detail path info of a 2-pin net
struct TwoPinNetDetailedPath {
	list<Node> detail_grid_path;
	vector<TurnNode> path_turn_nodes;
};


// The list of nets which occupied a grid node
struct NetInfo {
	list<int> net_list;
};


// The nets needed to be rerouted and its basic info imported by file 
struct TwoPinRipUpNet {
	// for OA identification
	//oaNet *p_net;
	Net *p_net;
	// for router identification
	int net_id;
	int two_pin_net_id;

	// cost evaluation
	int rip_up_times;
	int violation_num;
	int wire_length;

	TwoPinNetConnection two_pin_net_connection;
	TwoPinNetDetailedPath two_pin_net_detailed_path;
	bool path_is_found;
	bool both_pins_are_overlapped;
	bool has_only_one_pin;

	TwoPinRipUpNet();
	TwoPinRipUpNet(const TwoPinRipUpNet &net);

	inline TwoPinNetConnection& GetConnection()
	{
		return this->two_pin_net_connection;
	}
	inline Pin& GetSource()
	{
		return this->GetConnection().source;
	}
	inline Pin& GetTarget()
	{
		return this->GetConnection().target;
	}
	inline list<Node>& GetDetailedGridPath()
	{
		return this->two_pin_net_detailed_path.detail_grid_path;
	}
	inline vector<TurnNode>& GetPathTurnNodes()
	{
		return this->two_pin_net_detailed_path.path_turn_nodes;
	}

	bool operator < (const TwoPinRipUpNet &a) const;
	void PrintTargetTable()
	{
		cout << "net id = " << net_id << " ";
		cout << "2-pin net id = " << two_pin_net_id << endl;
		two_pin_net_connection.PrintTargetTable();
	}
};
*/


// Used to indentify the net id of a 2-pin net and ite 2-pin net id 
/*struct NetIdPair {
	int net_id;
	int two_pin_net_id;
	int two_pin_net_index;

	NetIdPair(
		const int net_id_val = -1,
		const int two_pin_net_id_val = -1,
		const int two_pin_net_index_val = -1
		)
	{
		net_id = net_id_val;
		two_pin_net_id = two_pin_net_id_val;
		two_pin_net_index = two_pin_net_index_val;
	}
};
typedef map<int, NetIdPair> NetIdTable;
typedef NetIdTable::iterator NetIdTableIterator;


// track_capacity_ usage
const int kNoBlockage = 0;
const int kBlockage = -1;


// moving direction
const int kRight = 1;
const int kLeft = 2;
const int kTop = 3;
const int kBottom = 4;
const int kUp = 5;
const int kDown = 6;

const int kNone = 0;
const int kPinPseudoLocation = 20;
const int kPinLocation = 21;
const int kPinLocationRectangle = 22;
const int kPinBlockage = 31;
const int kPotentialBlockage = 32;
const int kNormalBlockage = 36;*/



#endif


