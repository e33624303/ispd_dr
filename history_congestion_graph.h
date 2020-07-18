#ifndef HISTORY_CONGESTION_GRAPH_H
#define HISTORY_CONGESTION_GRAPH_H

#include <cstdio>
using std::printf;
#include <climits>
#include <cmath>
using std::sqrt;
using std::log;
#include <vector>
using std::vector;

#include "graph_struct.h"

// using namespace std;

class HistoryCongestionGraph {
 public:
	
	HistoryCongestionGraph();
	~HistoryCongestionGraph();
	
	/*void SetUpGraph(
			const int layout_width, 
			const int layout_height, 
			const int layout_layer,
			const bool kLineEndRoutingMode
			);*/
	inline void UpdateHistoryCongestion(
			const vector<Node>& history_node 
			);
	inline float GetHistoryCongestion(
			const int x, 
			const int y, 
			const int z
			);

	// Only for debug
	void PrintHistoryCongestionGraph();

	vector< vector < vector <float> > > ISPD_history_congestion_graph_;

 private:
	
	// Computing index from 3D to 1D
	inline ULL Transform3dArrayTo1dIndex(
			const int x, 
			const int y, 
			const int z
			);
	
	
	int layout_width_ = INT_MIN;
	int layout_height_ = INT_MIN;
	int layout_layer_ = INT_MIN;
	
	float* history_congestion_graph_ = 0;
	
	bool kLineEndRoutingMode_ = true;
};


inline void HistoryCongestionGraph::UpdateHistoryCongestion(
		const vector<Node>& history_node 
		)
{
	for(vector<Node>::const_iterator node_iter = history_node.begin();
		node_iter != history_node.end();
		node_iter++)
	{
		const int grid_x = node_iter->x;
		const int grid_y = node_iter->y;
		const int grid_z = node_iter->z;

		//const ULL node_index = Transform3dArrayTo1dIndex(grid_x, grid_y, grid_z);
		
		if (ISPD_history_congestion_graph_[grid_z][grid_y][grid_x] != 1.0)
		{
			int overlap_number = 
				(int)((-1 + sqrt(1 + 8 * ISPD_history_congestion_graph_[grid_z][grid_y][grid_x])) / 2);
			overlap_number++;
			float normalized_number = log(overlap_number);
			ISPD_history_congestion_graph_[grid_z][grid_y][grid_x] += normalized_number;
		}
		else
		{
			ISPD_history_congestion_graph_[grid_z][grid_y][grid_x] += 1.0;
		}
		
	}
	// PrintHistoryCongestionGraph();
}

inline float HistoryCongestionGraph::GetHistoryCongestion(
		const int x, 
		const int y, 
		const int z
		)
{
	/*const ULL node_index = 
		Transform3dArrayTo1dIndex(x, y, z);*/
	float normalized_history = 0.0;
	if(this->kLineEndRoutingMode_)
		normalized_history = 
		2 * sqrt(ISPD_history_congestion_graph_[z][y][x]) * 0.6950;
	else
		normalized_history = 
		5 * sqrt(ISPD_history_congestion_graph_[z][y][x]) * 0.6950;
	
	return normalized_history;
}


inline ULL HistoryCongestionGraph::Transform3dArrayTo1dIndex(
		const int x, 
		const int y, 
		const int z
		)
{
	return (ULL)((z) * layout_width_ * layout_height_ + layout_width_ * (y) + (x) );
}
#endif

