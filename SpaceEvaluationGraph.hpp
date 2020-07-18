#ifndef SPACE_EVALUATION_H
#define SPACE_EVALUATION_H
#include <iostream>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include "graph_struct.h"
#include "Definition.h"
#include "RTree.h"
#include "lef_def_parser/Structure.h"
#include <limits.h>

class DirSpace{
public:
	bool block;
	int width;
	int shortest_metal; // invalid(bigger)  for the width , the max space you can have
	int longest_metal;  // valid  (smaller) for the width , the max space you can have
	DirSpace(){
		block = false;
		width = -1;  // -1 means no width rule
		shortest_metal = INT_MAX;
		longest_metal = INT_MAX;
	}
};

class SpaceEvaluationNode{
public:
	// Use for Paralle Run length spacing
	bool block;
	coor_t x; coor_t y;
	// Use for Eol spacing
	DirSpace UpSpace;
	DirSpace DownSpace;
	DirSpace LeftSpace;
	DirSpace RightSpace;
	int net_id;
	SpaceEvaluationNode()
	{
		block = false;
		net_id = -1;
	}
	void SetCoor(coor_t coor_x, coor_t coor_y){
		x = coor_x;
		y = coor_y;
	}

	void Evaluation(Parser::I_Rect &Enclosure, Parser::I_Rect &EOL1, Parser::I_Rect &EOL2, int dir,
		 Parser::I_Rect &original_enclosure,int eol_spacing)
	{

		if (x >= EOL1.LB.first && x <= EOL1.RT.first)
		{
			if (y >= EOL1.LB.second && y <= EOL1.RT.second)
			{
				block = true;
				return;
			}
		}
		if (x >= EOL2.LB.first && x <= EOL2.RT.first)
		{
			if (y >= EOL2.LB.second && y <= EOL2.RT.second)
			{
				block = true;
				return;
			}
		}

		// In Normal Blockage
		if (x >= Enclosure.LB.first && x <= Enclosure.RT.first)
		{
			if (y >= Enclosure.LB.second && y <= Enclosure.RT.second)
			{
				block = true;
				return;
			}
			else if (y < Enclosure.LB.second)
			{
				// Bot of the Enc
				int new_space = original_enclosure.LB.second - y;
				if (dir == 2 && new_space <= eol_spacing)
				{
					UpSpace.block = true;
					return;
				}
				else
				{
					if (dir == 1)
					{
						// For vertical enc case , Horizontal path need to consider
						if (new_space <= eol_spacing){
							LeftSpace.block = true;
							RightSpace.block = true;
						}
						else{
							if (LeftSpace.width == -1 || ((new_space - eol_spacing) * 2) < LeftSpace.width)
								LeftSpace.width = (new_space - eol_spacing) * 2;
							if (RightSpace.width == -1 || ((new_space - eol_spacing) * 2) < RightSpace.width)
								RightSpace.width = (new_space - eol_spacing) * 2;
						}
					}
					// Pass then evaluate the available space
					// Upcase will form a EOL spacing
					new_space -= eol_spacing;
					if (new_space < UpSpace.longest_metal)
					{
						UpSpace.longest_metal = new_space;
					}
				}
			}
			else if (y > Enclosure.RT.second)
			{
				// Top of the enc
				int new_space = y - original_enclosure.RT.second;
				if (dir == 2 && new_space < eol_spacing)
				{
					DownSpace.block = true;
					return;
				}
				else
				{
					if (dir == 1)
					{
						// For vertical enc case , Horizontal path need to consider
						if (new_space <= eol_spacing)
						{
							LeftSpace.block = true;
							RightSpace.block = true;
						}
						else
						{
							if (LeftSpace.width == -1 || ((new_space - eol_spacing) * 2) < LeftSpace.width)
								LeftSpace.width = (new_space - eol_spacing) * 2;
							if (RightSpace.width == -1 || ((new_space - eol_spacing) * 2) < RightSpace.width)
								RightSpace.width = (new_space - eol_spacing) * 2;
						}
					}
					new_space -= eol_spacing;
					if (new_space < DownSpace.longest_metal)
					{
						DownSpace.longest_metal = new_space;
						return;
					}
				}
			}
		}
		else if (y >= Enclosure.LB.second && y <= Enclosure.RT.second)
		{
			if (x < Enclosure.LB.first)
			{
				// Left od the enc
				int new_space = original_enclosure.LB.first - x;
				if (dir == 1 && new_space <= eol_spacing)
				{
					RightSpace.block = true;
					return;
				}
				else
				{
					if (dir == 2)
					{
						// For Horizontal enc case , vertical path need to consider 
						if (new_space <= eol_spacing)
						{
							UpSpace.block = true;
							DownSpace.block = true;
						}
						else
						{
							if (UpSpace.width == -1 || ((new_space - eol_spacing) * 2) < UpSpace.width)
								UpSpace.width = (new_space - eol_spacing) * 2;
							if (DownSpace.width == -1 || ((new_space - eol_spacing) * 2) < DownSpace.width)
								DownSpace.width = (new_space - eol_spacing) * 2;
						}
					}
					new_space -= eol_spacing;
					if (new_space < RightSpace.longest_metal)
					{
						RightSpace.longest_metal = new_space;
						return;
					}
				}
			}
			else if (x > Enclosure.RT.first)
			{
				int new_space = x - original_enclosure.RT.first;
				if (dir == 1 && new_space < eol_spacing)
				{
					LeftSpace.block = true;
					return;
				}
				else
				{
					if (dir == 2)
					{
						// For horizontal enc case , vertical path need to consider
						if (new_space <= eol_spacing)
						{
							UpSpace.block = true;
							DownSpace.block = true;
						}
						else
						{
							if (UpSpace.width == -1 || ((new_space - eol_spacing) * 2) < UpSpace.width)
								UpSpace.width = (new_space - eol_spacing) * 2;
							if (DownSpace.width == -1 || ((new_space - eol_spacing) * 2) < DownSpace.width)
								DownSpace.width = (new_space - eol_spacing) * 2;
						}
					}
					new_space -= eol_spacing;
					if (new_space < LeftSpace.longest_metal)
					{
						LeftSpace.longest_metal = new_space;
						return;
					}
				}
			}
		}
		else{
			 // Not in the cross line need to consider EOL

			if (dir == 1)
			{
				// vertical EOL

				// Upper Right
				if (x > Enclosure.RT.first && y > Enclosure.RT.second)
				{
					if (LeftSpace.width == -1 || ((y - EOL2.RT.second) * 2) < LeftSpace.width)
						LeftSpace.width = (y - EOL2.RT.second) * 2;
					if (DownSpace.width == -1 || ((x - EOL2.RT.first) * 2) < DownSpace.width)
						DownSpace.width = (x - EOL2.RT.first) * 2;
					if (LeftSpace.width <= 0 || DownSpace.width <= 0)
					{
						this->block = true;
					}
				}
				// Lower Right
				else if (x > Enclosure.RT.first && y < Enclosure.LB.second)
				{
					if (LeftSpace.width == -1 || ((EOL1.LB.second - y) * 2) < LeftSpace.width)
						LeftSpace.width = (EOL1.LB.second - y) * 2;
					if (UpSpace.width == -1 || ((x - EOL1.RT.first) * 2) < UpSpace.width)
						UpSpace.width = (x - EOL1.RT.first) * 2;
					if (LeftSpace.width <= 0 || UpSpace.width <= 0)
					{
						this->block = true;
					}
				}
				// Upper Left
				else if (x < Enclosure.LB.first && y > Enclosure.RT.second)
				{
					if (RightSpace.width == -1 || ((y - EOL2.RT.second) * 2) < RightSpace.width)
						RightSpace.width = (y - EOL2.RT.second) * 2;
					if (DownSpace.width == -1 || ((EOL2.LB.first - x) * 2) < DownSpace.width)
						DownSpace.width = (EOL2.LB.first - x) * 2;
					if (RightSpace.width <= 0 || DownSpace.width <= 0)
					{
						this->block = true;
					}
				}
				// Lower Left
				else if (x < Enclosure.LB.first && y < Enclosure.LB.second)
				{
					if (RightSpace.width == -1 || ((EOL1.LB.second - y) * 2) < RightSpace.width)
						RightSpace.width = (EOL1.LB.second - y) * 2;
					if (UpSpace.width == -1 || ((EOL1.LB.first - x) * 2) < UpSpace.width)
						UpSpace.width = (EOL1.LB.first - x) * 2;
					if (RightSpace.width <= 0 || UpSpace.width <= 0)
					{
						this->block = true;
					}
				}
			}
			else if (dir == 2)
			{
				// horizontal EOL

				// Upper Right
				if (x > Enclosure.RT.first && y >  Enclosure.RT.second)
				{
					if (LeftSpace.width == -1 || ((y - EOL2.RT.second) * 2) < LeftSpace.width)
						LeftSpace.width = (y - EOL2.RT.second) * 2;
					if (DownSpace.width == -1 || ((x - EOL2.RT.first) * 2) < DownSpace.width)
						DownSpace.width = (x - EOL2.RT.first) * 2;
					if (LeftSpace.width <= 0 || DownSpace.width <= 0){
						this->block = true;
					}
				}
				// Lower Right
				else if (x > Enclosure.RT.first && y < Enclosure.LB.second){

					if (LeftSpace.width == -1 || ((EOL2.LB.second - y) * 2) < LeftSpace.width)
						LeftSpace.width = (EOL2.LB.second - y) * 2;
					if (UpSpace.width == -1 || ((x - EOL2.RT.first) * 2) < UpSpace.width)
						UpSpace.width = (x - EOL2.RT.first) * 2;
					if (LeftSpace.width <= 0 || UpSpace.width <= 0)
					{
						this->block = true;
					}
				}
				// Upper Left
				else if (x < Enclosure.LB.first && y > Enclosure.RT.second){

					if (RightSpace.width == -1 || ((y - EOL1.RT.second) * 2) < RightSpace.width)
						RightSpace.width = (y - EOL1.RT.second) * 2;
					if (DownSpace.width == -1 || ((EOL1.LB.first - x) * 2) < DownSpace.width)
						DownSpace.width = (EOL1.LB.first - x) * 2;
					if (RightSpace.width <= 0 || DownSpace.width <= 0)
					{
						this->block = true;
					}
				}
				// Lower Left
				else if (x < Enclosure.LB.first && y < Enclosure.LB.second)
				{
					if (RightSpace.width == -1 || ((EOL1.LB.second - y) * 2) < RightSpace.width)
						RightSpace.width = (EOL1.LB.second - y) * 2;
					if (UpSpace.width == -1 || ((EOL1.LB.first - x) * 2) < UpSpace.width)
						UpSpace.width = (EOL1.LB.first - x) * 2;
					if (RightSpace.width <= 0 || UpSpace.width <= 0)
					{
						this->block = true;
					}
				}
			}
		}
			
	}

};


class SpaceEvaluationGraph{
public:
	vector < pair <int, int> > spacing_table;
	Parser::INT_EOL_Rule eol_rule;
	int via_MetalWidth;
	Parser::Real_Coor_Table *SEG_real_coor_table;
	//vector < pair < map<coor_t, int>, map<coor_t, int > > > _fast_coor_map_;
	RTree<int, int, 2, float, 8, 4> *tree;		// normal tree for real obstacle (with inflation)
	RTree <int, int, 2, float, 8, 4> *EOL_tree; // store original blockage to detect EOL spacing
	vector<Parser::I_Rect> ALL_Shape;  // note the original blockage ( without inflation )
	vector <int> ALL_Shape_Net_list;
	int range;
	int Layer;
	SpaceEvaluationGraph()
	{
		;
	}

	void clear()
	{
		spacing_table.clear();
		ALL_Shape.clear();
		SEG_real_coor_table = NULL;
		ALL_Shape_Net_list.clear();
	}

	void clearTree()
	{
		tree->RemoveAll();
		EOL_tree->RemoveAll();
		delete EOL_tree;
		delete tree;
	}

	void SetGraph(vector<vector<MazeNode>> &grid_map, vector<pair<map<coor_t, int>, map<coor_t, int>>> &fast_coor_map, Parser::Real_Coor_Table &real_coor_table,
				  vector<pair<int, int>> &Spacing_table, Parser::INT_EOL_Rule Eol_Rule, int ViaMetalWidth, vector<Parser::I_Rect> &aLL_Shape, vector<int> &net_list, int ThisLayer)
	{
		SEG_real_coor_table = &real_coor_table;
		//_fast_coor_map_.assign(fast_coor_map.begin(), fast_coor_map.end());
		via_MetalWidth = ViaMetalWidth;
		eol_rule = Eol_Rule;
		spacing_table.assign(Spacing_table.begin(), Spacing_table.end());
		range = 1;

		Layer = ThisLayer;
		/*for (int i = 0; i < aLL_Shape.size(); i++)
		{
			if (aLL_Shape[i].Layer == Layer)
			{
				ALL_Shape.push_back(aLL_Shape[i]);
				ALL_Shape_Net_list.push_back(net_list[i]);
			}
		}*/
	}
	int SpacingOf2Shape(Parser::I_Rect &Enclosure, Parser::I_Rect &Shape){
		int width1 = min(Enclosure.RT.first - Enclosure.LB.first,
			Enclosure.RT.second - Enclosure.LB.second);
		int PRL_spacing1 = -1;
		for (int st = 0; st < spacing_table.size(); st++){
			if (width1 >= spacing_table[st].first)
				PRL_spacing1 = spacing_table[st].second;
			else{
				break;
			}
		}

		int width2 = min(Shape.RT.first - Shape.LB.first,
			Shape.RT.second - Shape.LB.second);
		int PRL_spacing2 = -1;
		for (int st = 0; st < spacing_table.size(); st++){
			if (width2 >= spacing_table[st].first)
				PRL_spacing2 = spacing_table[st].second;
			else{
				break;
			}
		}

		return max(PRL_spacing1, PRL_spacing2);
	}
	void InflatedBlockage(Parser::I_Rect &blockage){

		//int width_spacing = (via_MetalWidth / 2);
		int width = min(blockage.RT.first - blockage.LB.first,
			blockage.RT.second - blockage.LB.second);
		int PRL_spacing = -1;
		for (int st = 0; st < spacing_table.size(); st++){
			if (width >= spacing_table[st].first)
				PRL_spacing = spacing_table[st].second;
			else{
				break;
			}
		}

		int hor_spacing = PRL_spacing;
		int ver_spacing = PRL_spacing;
		if ((blockage.RT.first - blockage.LB.first) < eol_rule.ENDOFLINE){  // up down + eol 
			ver_spacing = max(PRL_spacing,eol_rule.SPACING);
		}
		if ((blockage.RT.second - blockage.LB.second) < eol_rule.ENDOFLINE){  // left right + eol
			hor_spacing = max(PRL_spacing,eol_rule.SPACING);
		}

		blockage.LB.first -= hor_spacing;
		blockage.RT.first += hor_spacing;
		blockage.LB.second -= ver_spacing;
		blockage.RT.second += ver_spacing;
	}

	void SmallInflatedBlockage(Parser::I_Rect &blockage){

		int width_spacing = (via_MetalWidth / 2);
		int width = min(blockage.RT.first - blockage.LB.first,
			blockage.RT.second - blockage.LB.second);
		int PRL_spacing = -1;
		for (int st = 0; st < spacing_table.size(); st++){
			if (width >= spacing_table[st].first)
				PRL_spacing = spacing_table[st].second;
			else{
				break;
			}
		}
		if (PRL_spacing > eol_rule.SPACING){
			int hor_spacing = PRL_spacing - eol_rule.SPACING;
			int ver_spacing = PRL_spacing - eol_rule.SPACING;
			blockage.LB.first -= hor_spacing;
			blockage.RT.first += hor_spacing;
			blockage.LB.second -= ver_spacing;
			blockage.RT.second += ver_spacing;
		}

	}
	void OverlappedPatchEstimation(Parser::I_Rect Rect, vector<Parser::I_Rect> &pinShape, vector<Parser::I_Rect> &Patch, int net_id);
	void OffGridVia(Node &pseudo_target, vector <Parser::I_Rect> &PinShape, Parser::wire_path &offGrid_Refine);

	void ONLYInflatedEnclosure(Parser::I_Rect &blockage)
	{

		int width = min(blockage.RT.first - blockage.LB.first,
						blockage.RT.second - blockage.LB.second);
		int PRL_spacing = -1;

#ifndef _EOL_CHECKING_
		PRL_spacing = spacing_table[spacing_table.size() - 1].second;
#else
		if (Layer == 0)
			PRL_spacing = max(eol_rule.SPACING, spacing_table[1].second);
		else
			PRL_spacing = eol_rule.SPACING;
#endif

		int hor_spacing = PRL_spacing;
		int ver_spacing = PRL_spacing;
		blockage.LB.first -= hor_spacing;
		blockage.RT.first += hor_spacing;
		blockage.LB.second -= ver_spacing;
		blockage.RT.second += ver_spacing;
	}

	void InflatedANDLocateEnclosure(Parser::I_Rect &blockage, Parser::I_Rect &ori_blockage, int offset, int &x, int &y)
	{

		int hor_spacing = 0;
		int ver_spacing = 0;
		if ((blockage.RT.first - blockage.LB.first) < eol_rule.ENDOFLINE)
		{ // up down + eol
			ver_spacing += eol_rule.SPACING;
			hor_spacing += eol_rule.WITHIN;
		}
		if ((blockage.RT.second - blockage.LB.second) < eol_rule.ENDOFLINE)
		{ // left right + eol
			hor_spacing += eol_rule.SPACING;
			ver_spacing += eol_rule.WITHIN;
		}
		
		blockage.LB.first -= hor_spacing;
		blockage.RT.first += hor_spacing;
		blockage.LB.second -= ver_spacing;
		blockage.RT.second += ver_spacing;
		blockage.LB.first -= offset;
		blockage.RT.first += offset;
		blockage.LB.second -= offset;
		blockage.RT.second += offset;
		// locate

		tuple<coor_t, coor_t, int> temp_coor;
		//SEG_real_coor_table->Table_Status_Log();
		temp_coor = SEG_real_coor_table->Index2Coor(x, y, Layer);
		blockage.LB.first += get<0>(temp_coor);
		blockage.RT.first += get<0>(temp_coor);
		blockage.LB.second += get<1>(temp_coor);
		blockage.RT.second += get<1>(temp_coor);

		ori_blockage.LB.first += get<0>(temp_coor);
		ori_blockage.RT.first += get<0>(temp_coor);
		ori_blockage.LB.second += get<1>(temp_coor);
		ori_blockage.RT.second += get<1>(temp_coor);
	}

	inline void PushBlockageToTree(Parser::I_Rect &Inflated_blockage, int &id){
		//SmallInflatedBlockage(Inflated_blockage);
		const int min[] = { Inflated_blockage.LB.first, Inflated_blockage.LB.second };
		const int max[] = { Inflated_blockage.RT.first, Inflated_blockage.RT.second };
		tree->Insert(min, max, id);
		// Note, all values including zero are fine in this version
	}
	inline void PushNetBlockageToTree(Parser::I_Rect &blockage,const int net_id){
		if(blockage.LB.first >= blockage.RT.first){
			int temp = blockage.RT.first;
			blockage.RT.first = blockage.LB.first;
			blockage.LB.first = temp;
		}
		if(blockage.LB.second >= blockage.RT.second){
			int temp = blockage.RT.second;
			blockage.RT.second = blockage.LB.second;
			blockage.LB.second = temp;
		}
		
		Parser::I_Rect Inflated_blockage = blockage;

		InflatedBlockage(Inflated_blockage);
		ALL_Shape.push_back(blockage);
		ALL_Shape_Net_list.push_back(net_id);

		const int min[] = { Inflated_blockage.LB.first, Inflated_blockage.LB.second };
		const int max[] = { Inflated_blockage.RT.first, Inflated_blockage.RT.second };
		tree->Insert(min, max, ALL_Shape.size() - 1);

		const int eol_min[] = {blockage.LB.first, blockage.LB.second};
		const int eol_max[] = {blockage.RT.first, blockage.RT.second};
		EOL_tree->Insert(eol_min, eol_max, ALL_Shape.size() - 1);
		// Note, all values including zero are fine in this version
	}
	void PatchEstimation(Parser::I_Rect rect, vector <Parser::I_Rect> &PinShape, vector <Parser::I_Rect> &Patch, int net_id);

	inline bool FineNetShape(Parser::I_Rect& find_rect, vector <Parser::I_Rect> &NetShape){
		for (int i = 0; i < NetShape.size(); i++){
			if (find_rect.SameRect(NetShape[i])){
				return true;
			}
		}
		return false;
	}

	bool ValidOfTwoShape(Parser::I_Rect &enclosure, Parser::I_Rect &Shape)
	{
		int PRL_spacing = SpacingOf2Shape(enclosure, Shape);
		int x1 = enclosure.RT.first;
		int x1b = enclosure.LB.first;
		int y1 = enclosure.RT.second;
		int y1b = enclosure.LB.second;
		int x2 = Shape.RT.first;
		int x2b = Shape.LB.first;
		int y2 = Shape.RT.second;
		int y2b = Shape.LB.second;
		bool left = x2 < x1b;
		bool right = x1 < x2b;
		bool bottom = y2 < y1b;
		bool top = y1 < y2b;
		if(top & left){
			// need to check EOL within
			if (y1 - y1b < eol_rule.ENDOFLINE || y2 - y2b < eol_rule.ENDOFLINE)
			{
				if (abs(y2b - y1) < eol_rule.WITHIN && abs(x1b - x2) < eol_rule.SPACING)
					return false;
			}
			if (x1 - x1b < eol_rule.ENDOFLINE || x2 - x2b < eol_rule.ENDOFLINE)
			{
				if (abs(x1b - x2) < eol_rule.WITHIN && abs(y2b - y1) < eol_rule.SPACING)
					return false;
			}
			return true;
		}
		else if (left & bottom){
			if (y1 - y1b < eol_rule.ENDOFLINE || y2 - y2b < eol_rule.ENDOFLINE)
			{
				if (abs(y2b - y1) < eol_rule.WITHIN && abs(x1b - x2) < eol_rule.SPACING)
					return false;
			}
			if (x1 - x1b < eol_rule.ENDOFLINE || x2 - x2b < eol_rule.ENDOFLINE)
			{
				if (abs(x1b - x2) < eol_rule.WITHIN && abs(y2b - y1) < eol_rule.SPACING)
					return false;
			}
			return true;
		}
		else if (bottom & right )
		{
			if (y1 - y1b < eol_rule.ENDOFLINE || y2 - y2b < eol_rule.ENDOFLINE)
			{
				if (abs(y2b - y1) < eol_rule.WITHIN && abs(x1b - x2) < eol_rule.SPACING)
					return false;
			}
			if (x1 - x1b < eol_rule.ENDOFLINE || x2 - x2b < eol_rule.ENDOFLINE)
			{
				if (abs(x1b - x2) < eol_rule.WITHIN && abs(y2b - y1) < eol_rule.SPACING)
					return false;
			}
			return true;
		}
		else if (right & top)
		{
			if (y1 - y1b < eol_rule.ENDOFLINE || y2 - y2b < eol_rule.ENDOFLINE)
			{
				if (abs(y2b - y1) < eol_rule.WITHIN && abs(x1b - x2) < eol_rule.SPACING)
					return false;
			}
			if (x1 - x1b < eol_rule.ENDOFLINE || x2 - x2b < eol_rule.ENDOFLINE)
			{
				if (abs(x1b - x2) < eol_rule.WITHIN && abs(y2b - y1) < eol_rule.SPACING)
					return false;
			}
			return true;
		}
		else if(left){ 
			int dis = x1b - x2;
			/*printf("via: LB(%d,%d) RT(%d,%d)\n", enclosure.LB.first, enclosure.LB.second, enclosure.RT.first, enclosure.RT.second);
			printf("Shape: LB(%d,%d) RT(%d,%d)\n", Shape.LB.first, Shape.LB.second, Shape.RT.first, Shape.RT.second);
			printf("EOL EOL(%d) , Spaceing(%d)\n", eol_rule.ENDOFLINE, eol_rule.SPACING);
			printf("dis (%d) PRL(%d)\n", dis, PRL_spacing);
			int pause;
			cin >> pause;*/
			if (dis < PRL_spacing)
			{
				return false;
			}
			else{
				// valid for prl spacing
				// check EOL spacing
				int width_enc = y1 - y1b;
				int width_shp = y2 - y2b;
				/*if (width_enc == 140){
					printf("via: LB(%d,%d) RT(%d,%d)\n", enclosure.LB.first, enclosure.LB.second, enclosure.RT.first, enclosure.RT.second);
					printf("Shape: LB(%d,%d) RT(%d,%d)\n", Shape.LB.first, Shape.LB.second, Shape.RT.first, Shape.RT.second);
					printf("EOL EOL(%d) , Spaceing(%d)\n", eol_rule.ENDOFLINE, eol_rule.SPACING);
					int pause;
					cin >> pause;
				}*/
				if (width_enc < eol_rule.ENDOFLINE || width_shp < eol_rule.ENDOFLINE)
				{
					if (dis < eol_rule.SPACING)
						return false;
				}
				return true;
			}
		}
		else if (right){
			int dis = x2b - x1;
			if (dis < PRL_spacing)
			{
				return false;
			}
			else
			{
				// valid for prl spacing
				// check EOL spacing
				int width_enc = y1 - y1b;
				int width_shp = y2 - y2b;
				if (width_enc < eol_rule.ENDOFLINE || width_shp < eol_rule.ENDOFLINE)
				{
					if (dis < eol_rule.SPACING)
						return false;
				}
				return true;
			}
		}
		else if (bottom){
			int dis = y1b - y2;
			if (dis < PRL_spacing)
			{
				return false;
			}
			else
			{
				// valid for prl spacing
				// check EOL spacing
				int width_enc = x1 - x1b;
				int width_shp = x2 - x2b;
				if (width_enc < eol_rule.ENDOFLINE || width_shp < eol_rule.ENDOFLINE)
				{
					if (dis < eol_rule.SPACING)
						return false;
				}
				return true;
			}
		}
		else if (top){
			int dis = y2b - y1;
			if (dis < PRL_spacing)
			{
				return false;
			}
			else
			{
				// valid for prl spacing
				// check EOL spacing
				int width_enc = x1 - x1b;
				int width_shp = x2 - x2b;
				if (width_enc < eol_rule.ENDOFLINE || width_shp < eol_rule.ENDOFLINE)
				{
					if (dis < eol_rule.SPACING)
						return false;
				}
				return true;
			}
		}
		else{
			return false;
		}
	}

	bool NodeEvluationByTree(Parser::I_Rect &enclosure, vector <Parser::I_Rect> &NetShape, int x, int y, int net_id);

	bool GeneralViaEvaluation(Parser::I_Rect &enclosure /* for real coordinate */, int net_id);
	void PushBlockage(Parser::I_Rect &Inflated_blockage)
	{
		;
	}

	void InflateViaEnclosure(Parser::I_Rect &Enclosure)
	{
		int width = min(Enclosure.RT.first - Enclosure.LB.first,
						Enclosure.RT.second - Enclosure.LB.second);
		int PRL_spacing = -1;
		for (int st = 0; st < spacing_table.size(); st++)
		{
			if (width >= spacing_table[st].first)
				PRL_spacing = spacing_table[st].second;
			else
			{
				break;
			}
		}
		if (Enclosure.RT.first - Enclosure.LB.first < Enclosure.RT.second - Enclosure.LB.second)
		{
			Enclosure.RT.first += PRL_spacing;
			Enclosure.LB.first -= PRL_spacing;
		}
		else{
			Enclosure.RT.second += PRL_spacing;
			Enclosure.LB.second -= PRL_spacing;
		}
	}
	void Find2_EOL_block(Parser::I_Rect &Enclosure, Parser::I_Rect &EOL1, Parser::I_Rect &EOL2)
	{
		if (Enclosure.RT.first - Enclosure.LB.first < Enclosure.RT.second - Enclosure.LB.second)
		{
			// Bot EOL
			EOL1.LB.first = Enclosure.LB.first - eol_rule.WITHIN;
			EOL1.RT.first = Enclosure.RT.first + eol_rule.WITHIN;
			EOL1.LB.second = Enclosure.LB.second - eol_rule.SPACING;
			EOL1.RT.second = Enclosure.LB.second;
			// Top EOL
			EOL2.LB.first = Enclosure.LB.first - eol_rule.WITHIN;
			EOL2.RT.first = Enclosure.RT.first + eol_rule.WITHIN;
			EOL2.LB.second = Enclosure.RT.second ;
			EOL2.RT.second = Enclosure.RT.second + eol_rule.SPACING;
		}
		else
		{
			// Left EOL
			EOL1.LB.first = Enclosure.LB.first - eol_rule.SPACING;
			EOL1.RT.first = Enclosure.LB.first;
			EOL1.LB.second = Enclosure.LB.second - eol_rule.WITHIN;
			EOL1.RT.second = Enclosure.RT.second + eol_rule.WITHIN;
			// Right EOL
			EOL2.LB.first = Enclosure.RT.first ;
			EOL2.RT.first = Enclosure.RT.first + eol_rule.SPACING;
			EOL2.LB.second = Enclosure.LB.second - eol_rule.WITHIN;
			EOL2.RT.second = Enclosure.RT.second + eol_rule.WITHIN;
		}
	}

	/*void FindingPossibleEvaluationRange(Parser::I_Rect &Enclosure , Parser::I_Rect &Range)
	{
		int low_y = 0; int high_y = 0;
		for (int i = 0; i < _space_eval_graph_.size(); i++)
		{
			if (_space_eval_graph_[i][0].y < Enclosure.LB.second){
				low_y = i;
			}
			if (_space_eval_graph_[i][0].y > Enclosure.RT.second){
				high_y = i;
				break;
			}
		}

		int left_x = 0; int right_x = 0;
		for (int i = 0; i < _space_eval_graph_[0].size(); i++)
		{
			if (_space_eval_graph_[0][i].x < Enclosure.LB.first)
			{
				left_x = i;
			}
			if (_space_eval_graph_[0][i].x > Enclosure.RT.first)
			{
				right_x = i;
				break;
			}
		}

		low_y--;
		high_y++;
		left_x--;
		right_x++;
		Range.LB.first = left_x;
		Range.RT.first = right_x;
		Range.LB.second = low_y;
		Range.RT.second = high_y;
	}*/

	/*bool CrossLineGraph_CheckViaEnclosure(Parser::I_Rect &Enclosure, int x , int y, int net_id)
	{
		int type = 0;
		if (Enclosure.RT.first - Enclosure.LB.first < Enclosure.RT.second - Enclosure.LB.second)
			type = 1;
		else
			type = 2;
		
		if(type == 1){
			// vertical
			int width = Enclosure.RT.first - Enclosure.LB.first;
			int length = (Enclosure.RT.second - Enclosure.LB.second)/2;
			if (_space_eval_graph_[y][x].block == true && _space_eval_graph_[y][x].net_id != net_id)
			{
				return false;
			}

			if (_space_eval_graph_[y][x].UpSpace.block == true || _space_eval_graph_[y][x].DownSpace.block == true)
			{
				return false;
			}
			else if (((_space_eval_graph_[y][x].UpSpace.width != -1) & (_space_eval_graph_[y][x].UpSpace.width < width)) ||
					 ((_space_eval_graph_[y][x].DownSpace.width != -1) & (_space_eval_graph_[y][x].DownSpace.width < width)))
			{
				return false;
			}
			else if (_space_eval_graph_[y][x].UpSpace.longest_metal < length || _space_eval_graph_[y][x].DownSpace.longest_metal < length)
			{
				return false;
			}
			return true;
		}
		else if (type == 2){
			// horizontal
			int width = Enclosure.RT.second - Enclosure.LB.second;
			int length = (Enclosure.RT.first - Enclosure.LB.first) / 2;
			if (_space_eval_graph_[y][x].LeftSpace.block == true || _space_eval_graph_[y][x].RightSpace.block == true)
			{
				return false;
			}
			else if (((_space_eval_graph_[y][x].LeftSpace.width != -1) & (_space_eval_graph_[y][x].LeftSpace.width < width)) ||
					 ((_space_eval_graph_[y][x].RightSpace.width != -1) & (_space_eval_graph_[y][x].RightSpace.width < width)))
			{
				return false;
			}
			else if (_space_eval_graph_[y][x].LeftSpace.longest_metal < length || _space_eval_graph_[y][x].RightSpace.longest_metal < length)
			{
				return false;
			}
			return true;
		}
	}

	void SpaceEvaluationOfViaEnclosure(Parser::I_Rect &Enclosure, Parser::I_Rect &EOL1, Parser::I_Rect &EOL2, 
		int dir, Parser::I_Rect &Original_Enc)
	{
		// vertical
		if (dir == 1)
		{

			Parser::I_Rect Range;
			FindingPossibleEvaluationRange(Enclosure, Range);
			for (int y = Range.LB.second; y <= Range.RT.second ; y++){
				if ( y < _space_eval_graph_.size() && y >= 0){
					for(int x = Range.LB.first ; x <= Range.RT.first ; x++){
						if (x >= 0 && x < _space_eval_graph_[y].size()){

							_space_eval_graph_[y][x].Evaluation(Enclosure, EOL1, EOL2, dir, Original_Enc,eol_rule.SPACING);
						}
					}
				}
			}
		}
		// horizontal
		else if (dir == 2){

		}
		else{
			printf("SpaceEvaluationOfViaEnclosure :: Error dir\n");
			exit(1);
		}

	}

	void HitPath(int sx,int sy,int tx ,int ty ,int path_width , int net_id){

		if(sy == ty){
			for (int i = sx; i <= tx; i++)
			{
				_space_eval_graph_[sy][i].block = true;
				_space_eval_graph_[sy][i].net_id = net_id;

				int y = sy;
				// lower the Path & Contain in the path metal
				while (y - 1 > 0 && _space_eval_graph_[sy][i].y - path_width < _space_eval_graph_[y-1][i].y)
				{
					_space_eval_graph_[y - 1][i].block = true;
					_space_eval_graph_[y - 1][i].net_id = net_id;

					y--;
				}
				// Lower the Path Metal 
				if(y - 1 > 0){
					;
				}
			}
		}
		else if(sx == tx){
			for (int i = sy; i <= ty; i++)
			{
				_space_eval_graph_[i][sx].block = true;
				_space_eval_graph_[i][sx].net_id = net_id;
			}
		}
		else{
			printf("Hit Path Error s(%d,%d) t(%d,%d)\n",sx,sy,tx,ty);
			exit(1);
		}
	}

	void HitViaEnclosure(int x, int y, Parser::I_Rect &Enclosure,int net_id)
	{
		int type = 0;
		Parser::I_Rect Original_Enc;
		Original_Enc.LB.first = Enclosure.LB.first;
		Original_Enc.RT.first = Enclosure.RT.first;
		Original_Enc.LB.second = Enclosure.LB.second;
		Original_Enc.RT.second = Enclosure.RT.second;
		if (Enclosure.RT.first - Enclosure.LB.first < Enclosure.RT.second - Enclosure.LB.second)
			type = 1;
		else type = 2;
		Parser::I_Rect EOL1 ; Parser::I_Rect EOL2;
		Find2_EOL_block(Enclosure, EOL1,EOL2);
		int addition ; 
		InflateViaEnclosure(Enclosure);
		if (type == 1)
		{
			SpaceEvaluationOfViaEnclosure(Enclosure, EOL1, EOL2, 1, Original_Enc);
		}
		else if (type == 2)
		{
			SpaceEvaluationOfViaEnclosure(Enclosure, EOL1, EOL2, 2, Original_Enc);
		}
	}*/
};






#endif