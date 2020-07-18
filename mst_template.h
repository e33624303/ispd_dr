#ifndef MST_TEMPLATE_H
#define MST_TEMPLATE_H

// #define PrintDetailedInfoMST

#include <cfloat>
#include <utility>
using std::pair;
#include <vector>
using std::vector;
#include <iostream>
#include <iomanip> // for setw()
#include "Definition.h"
using namespace std;

typedef pair< coor_t , coor_t > IndexPair;
typedef vector<IndexPair> IndexPairs;


const int Max_Weight = 1000; // 初始化key[]時需要infinity, 以Max_Weight代替

class GraphMST
{
  private:
	int num_vertex;
	std::vector< std::vector<int> > AdjMatrix;

  public:
	GraphMST() : num_vertex(0){};
	GraphMST(int n) : num_vertex(n)
	{
		AdjMatrix.resize(num_vertex);
		for (int i = 0; i < num_vertex; i++)
		{
			AdjMatrix[i].resize(num_vertex);
		}
	}
	void AssignMatrix(vector<std::vector<int> > &Matrix)
	{
		for (int i = 0; i < Matrix.size(); i++)
		{
			for (int j = 0; j < Matrix[i].size(); j++)
			{
				AdjMatrix[i][j] = Matrix[i][j];
			}
		}
	}
	void AddEdge(int from, int to, int weight);

	void PrimMST(int Start = 0);
	friend int MinKeyExtract(int *key, bool *visited, int size);
};

int MinKeyExtract(int *key, bool *visited, int size)
{

	int min = Max_Weight, min_idx = 0;
	for (int i = 0; i < size; i++)
	{
		if (visited[i] == false && key[i] < min)
		{
			min = key[i];
			min_idx = i;
		}
	}
	return min_idx;
}
void GraphMST::PrimMST(int Start)
{

	int key[num_vertex], predecessor[num_vertex];
	bool visited[num_vertex];

	for (int i = 0; i < num_vertex; i++)
	{
		key[i] = Max_Weight;
		predecessor[i] = -1;
		visited[i] = false; 
	}

	key[Start] = 0;
	for (int i = 0; i < num_vertex; i++)
	{
		int u = MinKeyExtract(key, visited, num_vertex);
		visited[u] = true;
		for (int i = 0; i < num_vertex; i++)
		{
			if (visited[i] == false && AdjMatrix[u][i] != 0 && AdjMatrix[u][i] < key[i])
			{
				predecessor[i] = u;
				key[i] = AdjMatrix[u][i];
			}
		}
	}
}
void GraphMST::AddEdge(int from, int to, int weight)
{
	AdjMatrix[from][to] = weight;
}



template <class NodeType>
class MinimumSpanningTree
{
 public:
	MinimumSpanningTree(int max_node_num = 100);
	~MinimumSpanningTree();

	// Input the nodes and return the edges of the MST.
	void StartAlgorithm(
			const vector<NodeType> nodes, 
			IndexPairs &index_pairs
			);

 private:
	
	void EvaluateCost(const vector<NodeType> nodes);
	void PrimMST(const int node_num, IndexPairs &index_pairs);
	
	int max_node_num_;
	int *near_node_;
	float *distance_;
	float *cost_matrix_;

};

//#include "mst_template.cpp"

// The DS is for testing the template.
/*struct Node
{
	int x, y;
	Node(int x_val = 0, int y_val = 0)
	{
		x = x_val;
		y = y_val;
	}

	Node(const Node &node)
	{
		x = node.x;
		y = node.y;
	}

	friend float GetEdgeCost(const Node &a, const Node &b)
	{
		const float edge_cost = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
		return edge_cost;
	}
};*/

#endif


