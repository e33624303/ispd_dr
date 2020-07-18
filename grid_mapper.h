
#ifndef GRID_MAPPER_H
#define GRID_MAPPER_H

/*#include "oaDesignDB.h"
using namespace oa;*/

#include <iostream>
using std::cout;
using std::endl;
#include <map>
using std::map;
#include <queue>
using std::queue;
#include <vector>
using std::vector;
#include <algorithm>
using std::swap;
#include <utility>
using std::pair;

#include "routing_info.h"
//#include "window_cutter.h"
#include "mst_template.h"
#include "graph_struct.h"

class Block{

};

// Record the nets to be rerouted.
// The nets may be originally broken in the window.
struct PseudoNet
{
	struct Pin
	{
		//oaFig *pin_fig;  // 可以記錄 PIN 的型態 - for ISPD
		//oaBox pin_box;
		Box pin_box;
		//oaLayerNum pin_layer;
		UInt pin_layer;
		Pin(Box &box, UInt layer);
		Pin(Pin &p_pin);
		friend float GetEdgeCost(const Pin &pin1, const Pin &pin2);
	};

	Net *net;
	vector<Pin> pins;

	PseudoNet();
	PseudoNet(const PseudoNet &p_net);
};
typedef vector<PseudoNet> PseudoNets;


class GridMapper
{
 public:
	GridMapper(
			RoutingLayerInfos *p_infos, Block *top_block,
			const int h_track_start, const int v_track_start, 
			const int h_track_spacing, const int v_track_spacing,
			const int h_track_num, const int v_track_num,
			const int low_layer, const int high_layer
			);

	// Mapping Layout to Grid Graph.
	void MapTwoPinNet(
			const PseudoNet &pseudo_net, 
			const IndexPairs &pairs, 
			vector<TwoPinRipUpNet> &ripup_net
			);
	void MapBlockage(
			const PseudoNet &pseudo_net, 
			vector<Node> &blockage_nodes
			);

	void OutputGDTFile(
			const char *input_file, 
			vector<TwoPinRipUpNet> &ripup_net
			);

	// Generate Net Path to Layout.
	void GenerateNetPathToLayout(
			vector<TwoPinRipUpNet> &two_pin_rip_up_net
			);

	inline int get_grid_width() 
		{ return grid_width_; }
	inline int get_grid_height() 
		{ return grid_height_; }
	inline int get_grid_layer() 
		{ return grid_layer_; }
	inline bool is_first_layer_vertical() 
		{ return is_first_layer_vertical_; }

 private:

	// Mapping Layout to Grid Graph.
	void MapToGrid(
			const PseudoNet::Pin &pin, 
			int &left, 
			int &bottom, 
			int &right, 
			int &top
			);
	PinType CheckPinType(
			const oaFig *p_fig,
			const int &left, 
			const int &bottom, 
			const int &right, 
			const int &top
			);

	// Generate Net Path to Layout.
	oaPathSeg* CreatePathSeg(
			const Node &from_node, 
			const Node &to_node
			);
	oaPathSeg* CreatePathSeg(
			const oaBox &source_box,
			const oaBox &target_box,
			const int grid_layer
			);
	oaVia* CreateVia(
			const Node &from_node, 
			const Node &to_node
			);
	void CreateVia(
			const Node &from_node,
			const Node &to_node,
			vector<oaVia*> &p_vias
			);
	void EntendPathSegment(
			oaPathSeg *path_seg,
			oaBox &pin_box
			);
	void GetGridPathSegments(
			list<Node> &grid_path,
			vector< pair<Node*, Node*> > &segments
			);
	void MergePathSegments(
			vector<oaPathSeg*> path_segs
			);
	void MergeOverlappedPathSegments(
			vector<oaPathSeg*> overlapped_segs
			);
	void PruneRedundantVias(
			vector<oaVia*> vias
			);


	int h_track_start_;
	int v_track_start_;
	int h_track_spacing_;
	int v_track_spacing_;
	int h_track_num_;
	int v_track_num_;
	vector<bool> kViaEncExpansion_;

	int low_layer_;
	int high_layer_;
	map< int, int > layer_map_;
	map< int, int > reverse_layer_map_;

	int grid_width_;
	int grid_height_;
	int grid_layer_;
	bool is_first_layer_vertical_;
	RoutingLayerInfos *p_infos_;
	map< int, int > infos_layer_map_;
	map< int, int > reverse_infos_layer_map_;

	int net_id_ = 1;

	oaBlock *top_block_;
};

#endif

