#ifndef LINE_END_GRAGH_H
#define LINE_END_GRAGH_H

#include <cstdio>
using std::printf;
#include <climits>
#include <cmath>
using std::sqrt;

#include <iostream>
using std::cout;
using std::endl;
#include <fstream>
using std::ofstream;
#include <vector>
using std::vector;
// using namespace std;


#include "graph_struct.h"
#include "routing_info.h"


// track_capacity_ usage
extern const int kNoBlockage;
extern const int kBlockage;

// line-end recognition for pin
extern const int kNone;
extern const int kPinLocation;
extern const int kPinLocationRectangle;
extern const int kPinBlockage;
extern const int kNormalBlockage;
extern const int kPotentialBlockage;

const int kSegRightEnd = 41;
const int kSegLeftEnd = 42;
const int kSegTopEnd = 43;
const int kSegBottomEnd = 44;


class LineEndGraph {
 public:
 
	LineEndGraph();
	~LineEndGraph();

	void SetPreferredDirection(
			const int odd_layer, 
			const int even_layer
			);
	void SetLineEndSpacing(
			RoutingLayerInfos &routing_layer_infos, 
			const int low_layer,
			const int high_layer
			);
	void SetMaskNum(
			const int grid_layer,
			const int mask_num
			);
	void SetViaEncCostMode(const bool is_via_end_mode);
	void SetUpGraph(
			const int layout_width, 
			const int layout_height, 
			const int layout_layer,
			int* const track_capacity, 
			NetIdTable* const net_id_table
			);
	
	// Blockage Insertion
	void InitPartialPinLocation(
			const PinType pin_type, 
			const vector<Node> &pin_nodes
			);
	void InsertBlockageForPin(const vector<Node> &pin_nodes);


	void InsertBlockage(const vector<TurnNode> &turn_nodes);
	void InsertBlockage(const TurnNode &turn_node);
	void CleanBlockage(const vector<TurnNode> &turn_nodes);
	void CleanBlockage(const TurnNode &turn_node);

	void CleanBlockageWholeGraph();

	inline bool HasBlockage(const MazeNode &maze_node);
	inline bool HasNormalBlockage(const MazeNode &maze_node);
	inline float GetLineEndStatus(
			const int x, 
			const int y, 
			const int z, 
			const int direction
			);
	inline int GetLineEndBlockageStatus(
			const int x, 
			const int y, 
			const int z
			);
	
	// Use the function to check whether a net has line-end violation.
	bool VerifyLineEndViolation(
			const vector<TurnNode> &turn_nodes,
			vector<TurnNode> &vio_turn_nodes
			);
	// bool VerifyLineEndViolation(const TurnNode &turn_node);
	bool VerifyLineEndViolationVer2(const TurnNode &turn_node);

	int AnalyzeLineEndViolation();
	
	
	// debug
	void PrintLineEndGraph();

 private:
	
	// Veriry the line-end violation caused by pins and Insert Pseudo Blcokage.
	void InsertBlockageForPin(const Node &pin);
	
	// Check if line-end violation around the node(Horizontal)
	bool VerifyHorizontalLineEndVioaltion(const TurnNode &turn_node);

	// Check if line-end violation around the node(Vertical)
	bool VerifyVerticalLineEndVioaltion(const TurnNode &turn_node);

	// Check If the node has line-end pair. 
	// Return the line-end positions.
	bool HasLineEndPair(
			const TurnNode &turn_node, 
			int &line_end_head, 
			int &line_end_tail
			);
	inline bool SetLineEndCheckPosition(
			const int check_id,
			const TurnNode &turn_node, 
			const int line_end_head,
			const int line_end_tail,
			int &parity_track, 
			int &start, 
			int &end
			);
	
	// Find if the input line-end has parity line-end violation.
	// Only find violaton on the next higher track.
	int AnalyzeParityLineEndViolation(
			const int start, 
			const int end,
			const int track,
			const int layer
			);

	// Computing index from 3D to 1D
	inline ULL Transform3dArrayTo1dIndex(
			const int x, 
			const int y, 
			const int z
			);
	inline ULL Transform3dArrayTo1dIndex(const Node &node);
	inline ULL Transform3dArrayTo1dIndex(const TurnNode &turn_node);
	inline ULL Transform3dArrayTo1dIndex(const MazeNode &maze_node);

	// inline bool HasLineEndRule(const TurnNode &turn_node);
	inline bool IsPinOnGrid(const TurnNode &turn_node);
	inline bool IsPinRectOnGrid(const TurnNode &turn_node);
	inline bool IsBlockageOnGrid(const TurnNode &turn_node);
	inline bool IsPinBlockageOnGrid(const TurnNode &turn_node);
 
	vector<int> kLineEndSpacing_;
	vector<int> kSafeSpacing_;
	vector<int> kMaskNum_;

	const int kCheckNextTrackPre = 2;
	const int kCheckNextTrackPost = 1;
	const int kCheckPreviousTrackPre = 4;
	const int kCheckPreviousTrackPost = 3;

	bool kViaEncCostMode_ = false;

	// preferred direction
	int kOddLayer = 1;
	int kEvenLayer = 0;

	int layout_width_ = INT_MIN;
	int layout_height_ = INT_MIN;
	int layout_layer_ = INT_MIN;
	
	int *p_track_capacity_ = NULL;
	NetIdTable *p_net_id_table_ = NULL;
	
	int *line_end_graph_ = NULL;
	int *line_end_history_graph_ = NULL;
	bool graph_is_setup_ = false;
};


inline bool LineEndGraph::HasBlockage(const MazeNode &maze_node)
{
	bool has_blockage = false;
	
	const ULL node_index = Transform3dArrayTo1dIndex(maze_node);
	const int map_status = line_end_graph_[node_index];
	if(
			kPinBlockage == map_status ||
			kNormalBlockage == map_status
		)
		has_blockage = true;
	return has_blockage;
}

inline bool LineEndGraph::HasNormalBlockage(const MazeNode &maze_node)
{
	bool has_blockage = false;
	
	const ULL node_index = Transform3dArrayTo1dIndex(maze_node);
	const int map_status = line_end_graph_[node_index];
	if(
			kNormalBlockage == map_status
		)
		has_blockage = true;
	return has_blockage;
}

// Compute the line-end cost of the grid.
inline float LineEndGraph::GetLineEndStatus(
		const int x, 
		const int y, 
		const int z, 
		const int direction)
{
	// float line_end_cost = 3.0; 
	float line_end_cost = 0.0; 
	
	const ULL index = Transform3dArrayTo1dIndex(x, y, z);
	const int line_end_state = this->line_end_graph_[index];
	const float line_end_history_cost = this->line_end_history_graph_[index];
	// const float line_end_history_cost = 0.0;

	if(line_end_state == kPinBlockage || 
		line_end_state == kNormalBlockage || 
		line_end_state == kPotentialBlockage)
	{
		// line_end_cost += 5.0;
		line_end_cost += 100.0;
	}

	line_end_cost += sqrt(line_end_history_cost) * 1.65;
	// line_end_cost *= 1.5; 
	return line_end_cost;
}


inline int LineEndGraph::GetLineEndBlockageStatus(
		const int x, 
		const int y, 
		const int z)
{
	const ULL node_index = Transform3dArrayTo1dIndex(x, y ,z);
	const int blockage_status = this->line_end_graph_[node_index];
	return blockage_status;
}


// Computing index from 3D to 1D
inline ULL LineEndGraph::Transform3dArrayTo1dIndex(
		const int x, 
		const int y, 
		const int z)
{
	return (ULL) \
		((z) * layout_width_ * layout_height_ + layout_width_ * (y) + (x) );
}

// Computing index from 3D to 1D
inline ULL LineEndGraph::Transform3dArrayTo1dIndex(const Node &node)
{
	const int x = node.x;
	const int y = node.y;
	const int z = node.z;
	return (ULL) \
		((z) * layout_width_ * layout_height_ + layout_width_ * (y) + (x) );
}

inline ULL LineEndGraph::Transform3dArrayTo1dIndex(const TurnNode &turn_node)
{
	const int x = turn_node.x;
	const int y = turn_node.y;
	const int z = turn_node.z;
	return (ULL) \
		((z) * layout_width_ * layout_height_ + layout_width_ * (y) + (x) );
}

inline ULL LineEndGraph::Transform3dArrayTo1dIndex(const MazeNode &maze_node)
{
	/*const int x = maze_node.x;
	const int y = maze_node.y;
	const int z = maze_node.z;
	return (ULL) 
		((z) * layout_width_ * layout_height_ + layout_width_ * (y) + (x) );*/
}

inline bool LineEndGraph::IsPinOnGrid(const TurnNode &turn_node)
{
	const ULL index = Transform3dArrayTo1dIndex(turn_node);
	return (
			(line_end_graph_[index] == kPinLocation) ||
			(line_end_graph_[index] == kPinLocationRectangle));
}

inline bool LineEndGraph::IsPinRectOnGrid(const TurnNode &turn_node)
{
	const ULL index = Transform3dArrayTo1dIndex(turn_node);
	return (line_end_graph_[index] == kPinLocationRectangle);
}

inline bool LineEndGraph::IsBlockageOnGrid(const TurnNode &turn_node)
{
	const ULL index = Transform3dArrayTo1dIndex(turn_node);
	return (
			(line_end_graph_[index] == kPinBlockage) ||
			(line_end_graph_[index] == kNormalBlockage));
}

inline bool LineEndGraph::IsPinBlockageOnGrid(const TurnNode &turn_node)
{
	const ULL index = Transform3dArrayTo1dIndex(turn_node);
	return (line_end_graph_[index] == kPinBlockage);
}
#endif


