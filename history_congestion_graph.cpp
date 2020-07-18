#include "history_congestion_graph.h"


// Constructor
HistoryCongestionGraph::HistoryCongestionGraph() 
{
}


HistoryCongestionGraph::~HistoryCongestionGraph()
{
	if(history_congestion_graph_ != NULL)
		delete [] history_congestion_graph_;
}


/*void HistoryCongestionGraph::SetUpGraph(
		const int layout_width, 
		const int layout_height, 
		const int layout_layer,
		const bool kLineEndRoutingMode
		)
{
	this->layout_width_ = layout_width;
	this->layout_height_ = layout_height;
	this->layout_layer_ = layout_layer;
	
	
	const ULL array_size = layout_width_ * layout_height_ * layout_layer_;
	history_congestion_graph_ = new float[array_size];
	
	for(ULL index = 0; index < array_size; index++)
		history_congestion_graph_[index] = 1.0;
	
	this->kLineEndRoutingMode_ = kLineEndRoutingMode;
}*/


void HistoryCongestionGraph::PrintHistoryCongestionGraph()
{
	printf("\n\nHistory Congestion Graph\n");
	for(int z = 0; z < layout_layer_; z++)
	{
		printf("LAYER %d\n", z);
		for(int y = layout_height_ - 1; y >= 0; y--)
		{
			for(int x = 0; x < layout_width_; x++)
			{
				int grid_index = Transform3dArrayTo1dIndex(x, y, z);
				printf("%2f ", history_congestion_graph_[grid_index]);
			}
			printf("\n");
		}
		printf("\n");
	}
}

