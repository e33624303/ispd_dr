#include "mst_template.h"


template <class NodeType>
MinimumSpanningTree<NodeType>::MinimumSpanningTree(int max_node_num)
{
	if(max_node_num == 0 || max_node_num == 1)
	{
		cout << "MinimumSpanningTree: Invalid max node number = " << max_node_num << endl;
		exit(1);
	}

	this->max_node_num_ = max_node_num;
	this->near_node_ = new int[max_node_num_];
	this->distance_ = new float[max_node_num_];
	this->cost_matrix_ = new float[max_node_num_ * max_node_num_];
}

template <class NodeType>
MinimumSpanningTree<NodeType>::~MinimumSpanningTree()
{
	if(near_node_)
		delete [] near_node_;
	if(distance_)
		delete [] distance_;
	if(cost_matrix_)
		delete [] cost_matrix_;
}

template <class NodeType>
void MinimumSpanningTree<NodeType>::StartAlgorithm(
		const vector<NodeType> nodes, IndexPairs &index_pairs
		)
{
	// Check the number of nodes
	const int node_num = (int)nodes.size();
	if(node_num == 0)
	{
		#ifdef PrintDetailedInfoMST
		cout << "MinimumSpanningTree: There is no node." << endl;
		#endif
	}
	else if(node_num == 1)
	{
		#ifdef PrintDetailedInfoMST
		cout << "MinimumSpanningTree: Only 1 node. No need to run." << endl;
		#endif
	}
	else if(node_num == 2)
	{
		index_pairs.resize(1);
		index_pairs[0].first = 0;
		index_pairs[0].second = 1;
	}
	else
	{
		// Set cost matrix
		EvaluateCost(nodes);
		// Start the Prim MST algorithm
		PrimMST(node_num, index_pairs);
	}
	return;
}


// Assume there is a friend function "GetEdgeCost()" that
// can get the edge cost of any two nodes.
template <class NodeType>
void MinimumSpanningTree<NodeType>::EvaluateCost(const vector<NodeType> nodes)
{
	const unsigned int node_num = nodes.size();
	for(unsigned int row = 0; row < node_num; row++)
	{
		for(unsigned int column = row; column < node_num; column++)
		{
			if(row != column)
			{
				const NodeType &node1 = nodes[row];
				const NodeType &node2 = nodes[column];
				cost_matrix_[row * node_num + column] = 
					GetEdgeCost(node1, node2);
				cost_matrix_[column * node_num + row] = 
					cost_matrix_[row * node_num + column];
			}
			else
			{
				cost_matrix_[row * node_num + column] = FLT_MAX;
			}
		}
	}

	return;
}

template <class NodeType>
void MinimumSpanningTree<NodeType>::PrimMST(
		const int node_num, 
		IndexPairs &index_pairs
		)
{
	// Index pairs init
	// The edge num of mst = node num - 1
	index_pairs.resize(node_num - 1);

	// MST Init
	for(int index = 0; index < node_num; index++)
	{
		near_node_[index] = 0;
		distance_[index] = cost_matrix_[0 * node_num + index];
	}

	// Think of node 0 as the root of MST
	for(int edge_index = 0; edge_index < (node_num - 1); edge_index++)
	{
		// Find the node resulting in least cost added to the tree
		float min_distance = FLT_MAX;
		int min_node_index = -1;
		for(int node_index = 1; node_index < node_num; node_index++)
		{
			if(0 < distance_[node_index] && 
				distance_[node_index] < min_distance)
			{
				min_distance = distance_[node_index];
				min_node_index = node_index;
			}
		}
		
		// Added the minimum edge(2-pin net) to the MST(multi-pin net).
		int source_index = near_node_[min_node_index];
		int target_index = min_node_index;
		index_pairs[edge_index].first = source_index;
		index_pairs[edge_index].second = target_index;

		// Disable the node when it has been added to MST.
		this->distance_[min_node_index] = -1;
		
		// Update the disable and nearnode
		for(int node_index = 1; node_index < node_num; node_index++)
		{
			float cost_with_current_min_node = 
				cost_matrix_[min_node_index * node_num + node_index];
			if(0 < distance_[node_index] && 
				distance_[node_index] > cost_with_current_min_node)
			{
				near_node_[node_index] = min_node_index;
				distance_[node_index] = cost_with_current_min_node;
			}
		}
		
	}


	return;
}




