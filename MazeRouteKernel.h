#ifndef MAZEROUTINGKERNEL_H
#define MAZEROUTINGKERNEL_H
#include <iostream>
#include <fstream>
#include <ctime>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <set>
#include "graph_struct.h"
#include "lef_def_parser/MainParser.h"
#include "Definition.h"
#include "RTree.h"
#include "SpaceEvaluationGraph.hpp"

using namespace std;

class ISPD_NodeType{
public:
	bool Raiseable;
	bool Fallable;
	bool In_plane;
	ISPD_NodeType(){
		Raiseable = false;
		Fallable = false;
		In_plane = false;
	}
	ISPD_NodeType(bool r, bool f, bool i){
		Raiseable = r;
		Fallable = f;
		In_plane = i;
	}
	void log(){
		printf("R(%d),F(%d),I(%d)\n", Raiseable, Fallable, In_plane);
	}
};

typedef unordered_map <string, int> Umap_S2I;
typedef unordered_map <string, int > Umap_S2I_inst;
typedef vector < vector < MazeNode > > ISPD_GridMap;  // Use for note Grid map
typedef vector < vector < Node > > ISPD_GridMap_coor; // Use for note coordinate of each node in grid map
typedef vector < pair < map<coor_t, int>, map<coor_t, int > > > FastCoorMap;


#endif
