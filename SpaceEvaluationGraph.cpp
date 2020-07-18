#include "SpaceEvaluationGraph.hpp"

vector <int> ISPD_SEG_overlap_rec;
bool ISPD_SEG_SearchCallback(int id, long int arg)
{
	//printf("Hit data rect %d\n", id);
	ISPD_SEG_overlap_rec.push_back(id);
	return true; // keep going
}

bool doOverlap(Parser::I_Rect &blockage, Parser::I_Rect &enclosure)
{
	// If one rectangle is on left side of other
	if (blockage.LB.first > enclosure.RT.first || enclosure.LB.first > blockage.RT.first)
		return false;

	// If one rectangle is above other
	if (blockage.LB.second < enclosure.RT.second || enclosure.LB.second < blockage.RT.second)
		return false;

	return true;
}

bool TwoShapeOverlapped(Parser::I_Rect &blockage, Parser::I_Rect &enclosure)
{
	bool b1 = (blockage.LB.first < enclosure.RT.first) & (blockage.LB.first > enclosure.LB.first) & (blockage.LB.second < enclosure.RT.second) & (blockage.LB.second > enclosure.LB.second);
	bool b2 = (blockage.RT.first < enclosure.RT.first) & (blockage.RT.first > enclosure.LB.first) & (blockage.RT.second < enclosure.RT.second) & (blockage.RT.second > enclosure.LB.second);
	bool e1 = (enclosure.LB.first < blockage.RT.first) & (enclosure.LB.first > blockage.LB.first) & (enclosure.LB.second < blockage.RT.second) & (enclosure.LB.second > blockage.LB.second);
	bool e2 = (enclosure.RT.first < blockage.RT.first) & (enclosure.RT.first > blockage.LB.first) & (enclosure.RT.second < blockage.RT.second) & (enclosure.RT.second > blockage.LB.second);

	return b1 | b2 | e1 | e2;
}

bool SpaceEvaluationGraph::NodeEvluationByTree(Parser::I_Rect &enclosure, vector <Parser::I_Rect> &NetShape, int x, int y,int net_id){
	ISPD_SEG_overlap_rec.clear();
	vector <int> id_list;
	Parser::I_Rect ori_enclosure = enclosure;
	//printf("original enclosure : LB(%d,%d) RT(%d,%d)\n", enclosure.LB.first, enclosure.LB.second, enclosure.RT.first, enclosure.RT.second);

	InflatedANDLocateEnclosure(enclosure, ori_enclosure, ENCLOSURE_OFFSET, x, y);
	//printf("Inflated enclosure : LB(%d,%d) RT(%d,%d)\n", enclosure.LB.first, enclosure.LB.second, enclosure.RT.first, enclosure.RT.second);
	
	// normal tree search , using original enc to find
	const int search_min[] = {ori_enclosure.LB.first, ori_enclosure.LB.second};
	const int search_max[] = {ori_enclosure.RT.first, ori_enclosure.RT.second};
	int nhits = tree->Search(search_min, search_max, ISPD_SEG_SearchCallback, NULL);
	id_list = ISPD_SEG_overlap_rec; // assignment the overlapped shape id to id_list
	ISPD_SEG_overlap_rec.clear();
	/*if (_space_eval_graph_->at(y)[x].x == 199400 && _space_eval_graph_->at(y)[x].y == 227430 && (Layer == 1 || Layer == 0))
	{
		printf("normal\n");
		printf("Layer (%d)\n",Layer);
		printf("original enclosure : LB(%d,%d) RT(%d,%d)\n", ori_enclosure.LB.first, ori_enclosure.LB.second, ori_enclosure.RT.first, ori_enclosure.RT.second);
		printf("Inflated enclosure : LB(%d,%d) RT(%d,%d)\n", enclosure.LB.first, enclosure.LB.second, enclosure.RT.first, enclosure.RT.second);

		for (int i = 0; i < id_list.size(); i++)
		{
			Parser::I_Rect shape = ALL_Shape[id_list[i]];
			InflatedBlockage(shape);
			printf("PinShape : LB(%d,%d) RT(%d,%d)\n", shape.LB.first, shape.LB.second, shape.RT.first, shape.RT.second);
		}
		int pause;
		cin >> pause;
	}*/
	for (int i = 0; i < id_list.size(); i++)
	{
		if (ALL_Shape_Net_list[id_list[i]] == net_id)
		{
			if (Layer == 0) continue;

			if (doOverlap(ALL_Shape[id_list[i]], ori_enclosure))
				continue;
			else
				return false;
		}
		else
			return false;
	}
	
	// using inflated enc to find eol tree, check eol spacing
	const int eol_search_min[] = {enclosure.LB.first, enclosure.LB.second};
	const int eol_search_max[] = {enclosure.RT.first, enclosure.RT.second};
	nhits = EOL_tree->Search(eol_search_min, eol_search_max, ISPD_SEG_SearchCallback, NULL);
	id_list = ISPD_SEG_overlap_rec; // assignment the overlapped shape id to id_list
	ISPD_SEG_overlap_rec.clear();
	/*if (_space_eval_graph_->at(y)[x].x == 199400 && _space_eval_graph_->at(y)[x].y == 227430 && (Layer == 1 || Layer == 0))
	{
		printf("eol\n");
		printf("Layer (%d)\n", Layer);
		printf("original enclosure : LB(%d,%d) RT(%d,%d)\n", ori_enclosure.LB.first, ori_enclosure.LB.second, ori_enclosure.RT.first, ori_enclosure.RT.second);
		printf("Inflated enclosure : LB(%d,%d) RT(%d,%d)\n", enclosure.LB.first, enclosure.LB.second, enclosure.RT.first, enclosure.RT.second);

		for (int i = 0; i < id_list.size(); i++)
		{
			printf("PinShape : LB(%d,%d) RT(%d,%d)\n", ALL_Shape[id_list[i]].LB.first, ALL_Shape[id_list[i]].LB.second, ALL_Shape[id_list[i]].RT.first, ALL_Shape[id_list[i]].RT.second);
		}
		int pause;
		cin >> pause;
	}*/
	for (int i = 0; i < id_list.size(); i++)
	{
		if (ALL_Shape_Net_list[id_list[i]] == net_id)
		{
			if (Layer == 0)
				continue;

			if (doOverlap(ALL_Shape[id_list[i]], ori_enclosure))
				continue;
			else
				return false;
		}
		else
			return false;
	}

	/*if (_space_eval_graph_->at(y)[x].x == 199400 && _space_eval_graph_->at(y)[x].y == 227430 && (Layer == 1||Layer == 0))
	{
		printf("original enclosure : LB(%d,%d) RT(%d,%d)\n", ori_enclosure.LB.first, ori_enclosure.LB.second, ori_enclosure.RT.first, ori_enclosure.RT.second);
		printf("Inflated enclosure : LB(%d,%d) RT(%d,%d)\n", enclosure.LB.first, enclosure.LB.second, enclosure.RT.first, enclosure.RT.second);

		for (int i = 0; i < id_list.size(); i++)
		{
			printf("PinShape : LB(%d,%d) RT(%d,%d)\n", ALL_Shape[id_list[i]].LB.first, ALL_Shape[id_list[i]].LB.second, ALL_Shape[id_list[i]].RT.first, ALL_Shape[id_list[i]].RT.second);
		}
		int pause;
		cin >> pause;
	}
*/
	return true;
}

bool SpaceEvaluationGraph::GeneralViaEvaluation(Parser::I_Rect &enclosure /* for real coordinate */, int net_id)
{
	ISPD_SEG_overlap_rec.clear();
	vector<int> id_list;
	//printf("======= Map Coor (%d,%d) - (%d,%d) =======\n", x, y, _space_eval_graph_[y][x].x, _space_eval_graph_[y][x].y);
	//printf("Original enclosure : LB(%d,%d) RT(%d,%d)\n\n\n", enclosure.LB.first, enclosure.LB.second, enclosure.RT.first, enclosure.RT.second);
	Parser::I_Rect ori_enclosure = enclosure;
	ONLYInflatedEnclosure(enclosure);
	const int search_min[] = {ori_enclosure.LB.first, ori_enclosure.LB.second};
	const int search_max[] = {ori_enclosure.RT.first, ori_enclosure.RT.second};
	int nhits = tree->Search(search_min, search_max, ISPD_SEG_SearchCallback, NULL);
	id_list = ISPD_SEG_overlap_rec; // assignment the overlapped shape id to id_list
	ISPD_SEG_overlap_rec.clear();
	for (int i = 0; i < id_list.size(); i++)
	{
		if (ALL_Shape_Net_list[id_list[i]] == net_id)
		{
			if (Layer == 0)
				continue;

			if (doOverlap(ALL_Shape[id_list[i]], ori_enclosure))
				continue;
			else
				return false;
		}
		else
			return false;
	}

	// using inflated enc to find eol tree, check eol spacing
	const int eol_search_min[] = {enclosure.LB.first, enclosure.LB.second};
	const int eol_search_max[] = {enclosure.RT.first, enclosure.RT.second};
	nhits = EOL_tree->Search(eol_search_min, eol_search_max, ISPD_SEG_SearchCallback, NULL);
	id_list = ISPD_SEG_overlap_rec; // assignment the overlapped shape id to id_list
	ISPD_SEG_overlap_rec.clear();
	for (int i = 0; i < id_list.size(); i++)
	{
		if (ALL_Shape_Net_list[id_list[i]] == net_id)
		{
			continue;
		}
		else
			return false;
	}

	return true;
}

bool LineOverlap(int By1,int Ty1,int By2,int Ty2){

	if (By1 >= By2 && By1 >= Ty2){
		return false;
	}
	if (Ty1 <= By2 && Ty1 <= Ty2){
		return false;
	}

	return true;
}

void SpaceEvaluationGraph::OverlappedPatchEstimation(Parser::I_Rect Rect, vector<Parser::I_Rect> &pinShape, vector<Parser::I_Rect> &Patch, int net_id){
	ISPD_SEG_overlap_rec.clear();
	vector<int> id_list;

	// Enclosure coor checking
	if (Rect.LB.second >= Rect.RT.second){
		printf("WARNING::OverlappedPatchEstimation::Enclosure Coor Error LB(%d,%d) RT(%d,%d)\n", Rect.LB.first, Rect.LB.second, Rect.RT.first, Rect.RT.second);
		int temp = Rect.LB.second;
		Rect.LB.second = Rect.RT.second;
		Rect.RT.second = temp;
	}
	if (Rect.LB.first >= Rect.RT.first)
	{
		printf("WARNING::OverlappedPatchEstimation::Enclosure Coor Error LB(%d,%d) RT(%d,%d)\n", Rect.LB.first, Rect.LB.second, Rect.RT.first, Rect.RT.second);
		int temp = Rect.LB.first;
		Rect.LB.first = Rect.RT.first;
		Rect.RT.first = temp;
	}
	// Inflate searching range , using R-tree to search neighbor shape
	Parser::I_Rect rect = Rect;
	ONLYInflatedEnclosure(Rect);
	const int search_min[] = {Rect.LB.first, Rect.LB.second};
	const int search_max[] = {Rect.RT.first, Rect.RT.second};
	int nhits = EOL_tree->Search(search_min, search_max, ISPD_SEG_SearchCallback, NULL);
	id_list = ISPD_SEG_overlap_rec;
	ISPD_SEG_overlap_rec.clear();
	// Collect same net shape
	vector<Parser::I_Rect> PinShape;
	for (int i = 0; i < id_list.size(); i++)
	{
		// same net and without overlapping with each other
		if (ALL_Shape_Net_list[id_list[i]] == net_id && !(doOverlap(rect, ALL_Shape[id_list[i]])))
			PinShape.push_back(ALL_Shape[id_list[i]]);
	}


	for (int i = 0; i < PinShape.size(); i++)
	{
		if (true)
		{
			//printf("Pin : Layer(%d) LB(%d,%d) , RT(%d,%d)\n", PinShape[i].Layer, PinShape[i].LB.first, PinShape[i].LB.second, PinShape[i].RT.first, PinShape[i].RT.second);
			int Spacing = max(eol_rule.SPACING, spacing_table[1].second);
			//printf("Spacing : %d\n", Spacing);
			if (PinShape[i].LB.first > rect.RT.first)
			{
				// right of enc
				if (LineOverlap(PinShape[i].LB.second, PinShape[i].RT.second, rect.LB.second, rect.RT.second))
				{
					if (PinShape[i].LB.first < rect.RT.first + Spacing)
					{
						if (PinShape[i].LB.second <= rect.RT.second && PinShape[i].LB.second >= rect.LB.second)
						{
							Parser::I_Rect pch;
							//pch.LB.second = PinShape[i].LB.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							//pch.RT.second = rect.RT.second;
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = rect.RT.first;
							pch.RT.first = PinShape[i].LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].RT.second <= rect.RT.second && PinShape[i].RT.second >= rect.LB.second)
						{
							Parser::I_Rect pch;
							//pch.LB.second = rect.LB.second;
							//pch.RT.second = PinShape[i].RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = rect.RT.first;
							pch.RT.first = PinShape[i].LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.second <= rect.RT.second && PinShape[i].RT.second >= rect.RT.second)
						{
							Parser::I_Rect pch;
							//pch.LB.second = rect.LB.second;
							//pch.RT.second = rect.RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = rect.RT.first;
							pch.RT.first = PinShape[i].LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.second >= rect.RT.second && PinShape[i].RT.second <= rect.RT.second)
						{
							Parser::I_Rect pch;
							//pch.LB.second = PinShape[i].LB.second;
							//pch.RT.second = PinShape[i].RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = rect.RT.first;
							pch.RT.first = PinShape[i].LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
					}
				}
			}
			else if (PinShape[i].RT.first < rect.LB.first)
			{
				// left of enc
				if (LineOverlap(PinShape[i].LB.second, PinShape[i].RT.second, rect.LB.second, rect.RT.second))
				{
					if (PinShape[i].RT.first + Spacing > rect.LB.first)
					{
						if (PinShape[i].LB.second <= rect.RT.second && PinShape[i].LB.second >= rect.LB.second)
						{
							Parser::I_Rect pch;
							//pch.LB.second = PinShape[i].LB.second;
							//pch.RT.second = rect.RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = PinShape[i].LB.first;
							pch.RT.first = rect.RT.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].RT.second <= rect.RT.second && PinShape[i].RT.second >= rect.LB.second)
						{
							Parser::I_Rect pch;
							//pch.LB.second = rect.LB.second;
							//pch.RT.second = PinShape[i].RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = PinShape[i].RT.first;
							pch.RT.first = rect.LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.second <= rect.RT.second && PinShape[i].RT.second >= rect.RT.second)
						{
							Parser::I_Rect pch;
							//pch.LB.second = rect.LB.second;
							//pch.RT.second = rect.RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = PinShape[i].RT.first;
							pch.RT.first = rect.LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.second >= rect.RT.second && PinShape[i].RT.second <= rect.RT.second)
						{
							Parser::I_Rect pch;
							//pch.LB.second = PinShape[i].LB.second;
							//pch.RT.second = PinShape[i].RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = PinShape[i].RT.first;
							pch.RT.first = rect.LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
					}
				}
			}
			else if (PinShape[i].RT.second < rect.LB.second)
			{
				// lower Enc
				if (LineOverlap(PinShape[i].LB.first, PinShape[i].RT.first, rect.LB.first, rect.RT.first))
				{
					if (PinShape[i].RT.second + Spacing > rect.LB.second)
					{
						if (PinShape[i].LB.first <= rect.RT.first && PinShape[i].LB.first >= rect.LB.first)
						{
							Parser::I_Rect pch;
							pch.LB.second = PinShape[i].RT.second;
							pch.RT.second = rect.LB.second;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = rect.RT.first;
							//pch.RT.first = PinShape[i].RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].RT.first <= rect.RT.first && PinShape[i].RT.first >= rect.LB.first)
						{
							Parser::I_Rect pch;
							pch.LB.second = PinShape[i].RT.second;
							pch.RT.second = rect.LB.second;
							//pch.LB.first = rect.LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = PinShape[i].RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.first <= rect.RT.first && PinShape[i].RT.first >= rect.RT.first)
						{
							Parser::I_Rect pch;
							pch.LB.second = PinShape[i].RT.second;
							pch.RT.second = rect.LB.second;
							//pch.LB.first = rect.LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = rect.RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.first >= rect.RT.first && PinShape[i].RT.first <= rect.RT.first)
						{
							Parser::I_Rect pch;
							pch.LB.second = PinShape[i].RT.second;
							pch.RT.second = rect.LB.second;
							//pch.LB.first = PinShape[i].LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = PinShape[i].RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
					}
				}
			}
			else if (PinShape[i].LB.second > rect.RT.second)
			{
				// Upper Enc
				if (LineOverlap(PinShape[i].LB.first, PinShape[i].RT.first, rect.LB.first, rect.RT.first))
				{
					if (PinShape[i].LB.second < rect.RT.second + Spacing)
					{
						if (PinShape[i].LB.first <= rect.RT.first && PinShape[i].LB.first >= rect.LB.first)
						{
							Parser::I_Rect pch;
							pch.LB.second = rect.RT.second;
							pch.RT.second = PinShape[i].LB.second;
							//pch.LB.first = PinShape[i].LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = rect.RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].RT.first <= rect.RT.first && PinShape[i].RT.first >= rect.LB.first)
						{
							Parser::I_Rect pch;
							pch.LB.second = rect.RT.second;
							pch.RT.second = PinShape[i].LB.second;
							//pch.LB.first = rect.LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = PinShape[i].RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.first <= rect.RT.first && PinShape[i].RT.first >= rect.RT.first)
						{
							Parser::I_Rect pch;
							pch.LB.second = rect.RT.second;
							pch.RT.second = PinShape[i].LB.second;
							//pch.LB.first = rect.LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = rect.RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.first >= rect.RT.first && PinShape[i].RT.first <= rect.RT.first)
						{
							Parser::I_Rect pch;
							pch.LB.second = rect.RT.second;
							pch.RT.second = PinShape[i].LB.second;
							//pch.LB.first = PinShape[i].LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = PinShape[i].RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
					}
				}
			}
		}
	}
}

void SpaceEvaluationGraph::PatchEstimation(Parser::I_Rect Rect, vector<Parser::I_Rect> &pinShape, vector<Parser::I_Rect> &Patch, int net_id)
{
	/*RTree <int, int, 2, float, 8, 4> LocalTree;
	vector <int> id_list;
	for (int i = 0; i < PinShape.size(); i++){
		const int min[] = { PinShape[i].LB.first, PinShape[i].LB.second };
		const int max[] = { PinShape[i].RT.first, PinShape[i].RT.second };
		LocalTree.Insert(min, max, i);
	}
	const int search_min[] = { rect.LB.first, rect.LB.second };
	const int search_max[] = { rect.RT.first, rect.RT.second };
	ISPD_SEG_overlap_rec.clear();
	int nhits = LocalTree.Search(search_min, search_max, ISPD_SEG_SearchCallback, NULL);*/
	ISPD_SEG_overlap_rec.clear();
	vector<int> id_list;
	//printf("======= Map Coor (%d,%d) - (%d,%d) =======\n", x, y, _space_eval_graph_[y][x].x, _space_eval_graph_[y][x].y);
	//printf("Original enclosure : LB(%d,%d) RT(%d,%d)\n\n\n", enclosure.LB.first, enclosure.LB.second, enclosure.RT.first, enclosure.RT.second);
	Parser::I_Rect rect = Rect;
	ONLYInflatedEnclosure(Rect);
	const int search_min[] = {Rect.LB.first, Rect.LB.second};
	const int search_max[] = {Rect.RT.first, Rect.RT.second};
	int nhits = EOL_tree->Search(search_min, search_max, ISPD_SEG_SearchCallback, NULL);

	id_list = ISPD_SEG_overlap_rec;
	ISPD_SEG_overlap_rec.clear();
	vector<Parser::I_Rect> PinShape;
	for (int i = 0; i < id_list.size(); i++){
		if (ALL_Shape_Net_list[id_list[i]] == net_id)
			PinShape.push_back(ALL_Shape[id_list[i]]);
	}
	
	//printf("Enc : Layer(%d) LB(%d,%d) , RT(%d,%d)\n", rect.Layer, rect.LB.first, rect.LB.second, rect.RT.first, rect.RT.second);
	for (int i = 0; i < PinShape.size(); i++){
		if (true){
			//printf("Pin : Layer(%d) LB(%d,%d) , RT(%d,%d)\n", PinShape[i].Layer, PinShape[i].LB.first, PinShape[i].LB.second, PinShape[i].RT.first, PinShape[i].RT.second);
			int Spacing = max(eol_rule.SPACING,spacing_table[1].second);
			//printf("Spacing : %d\n", Spacing);
			if (PinShape[i].LB.first > rect.RT.first){
				// right of enc
				if (LineOverlap(PinShape[i].LB.second, PinShape[i].RT.second, rect.LB.second, rect.RT.second)){
					if (PinShape[i].LB.first < rect.RT.first + Spacing){
						if (PinShape[i].LB.second <= rect.RT.second && PinShape[i].LB.second >= rect.LB.second){
							Parser::I_Rect pch;
							//pch.LB.second = PinShape[i].LB.second;
							pch.LB.second = max(PinShape[i].LB.second,rect.LB.second);
							//pch.RT.second = rect.RT.second;
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = rect.RT.first;
							pch.RT.first = PinShape[i].LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].RT.second <= rect.RT.second && PinShape[i].RT.second >= rect.LB.second){
							Parser::I_Rect pch;
							//pch.LB.second = rect.LB.second;
							//pch.RT.second = PinShape[i].RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = rect.RT.first;
							pch.RT.first = PinShape[i].LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.second <= rect.RT.second && PinShape[i].RT.second >= rect.RT.second){
							Parser::I_Rect pch;
							//pch.LB.second = rect.LB.second;
							//pch.RT.second = rect.RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = rect.RT.first;
							pch.RT.first = PinShape[i].LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.second >= rect.RT.second && PinShape[i].RT.second <= rect.RT.second){
							Parser::I_Rect pch;
							//pch.LB.second = PinShape[i].LB.second;
							//pch.RT.second = PinShape[i].RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = rect.RT.first;
							pch.RT.first = PinShape[i].LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
					}
				}
			}
			else if (PinShape[i].RT.first <  rect.LB.first){
				// left of enc
				if (LineOverlap(PinShape[i].LB.second, PinShape[i].RT.second, rect.LB.second, rect.RT.second)){
					if (PinShape[i].RT.first + Spacing > rect.LB.first){
						if (PinShape[i].LB.second <= rect.RT.second && PinShape[i].LB.second >= rect.LB.second){
							Parser::I_Rect pch;
							//pch.LB.second = PinShape[i].LB.second;
							//pch.RT.second = rect.RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = PinShape[i].LB.first;
							pch.RT.first = rect.RT.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].RT.second <= rect.RT.second && PinShape[i].RT.second >= rect.LB.second){
							Parser::I_Rect pch;
							//pch.LB.second = rect.LB.second;
							//pch.RT.second = PinShape[i].RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = PinShape[i].RT.first;
							pch.RT.first = rect.LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.second <= rect.RT.second && PinShape[i].RT.second >= rect.RT.second){
							Parser::I_Rect pch;
							//pch.LB.second = rect.LB.second;
							//pch.RT.second = rect.RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = PinShape[i].RT.first;
							pch.RT.first = rect.LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.second >= rect.RT.second && PinShape[i].RT.second <= rect.RT.second){
							Parser::I_Rect pch;
							//pch.LB.second = PinShape[i].LB.second;
							//pch.RT.second = PinShape[i].RT.second;
							pch.LB.second = max(PinShape[i].LB.second, rect.LB.second);
							pch.RT.second = min(PinShape[i].RT.second, rect.RT.second);
							pch.LB.first = PinShape[i].RT.first;
							pch.RT.first = rect.LB.first;
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
					}
				}
			}
			else if (PinShape[i].RT.second <  rect.LB.second){
				// lower Enc
				if (LineOverlap(PinShape[i].LB.first, PinShape[i].RT.first, rect.LB.first, rect.RT.first)){
					if (PinShape[i].RT.second + Spacing > rect.LB.second){
						if (PinShape[i].LB.first <= rect.RT.first && PinShape[i].LB.first >= rect.LB.first){
							Parser::I_Rect pch;
							pch.LB.second = PinShape[i].RT.second;
							pch.RT.second = rect.LB.second;
							pch.LB.first = max(PinShape[i].LB.first,rect.LB.first);
							//pch.RT.first = rect.RT.first;
							//pch.RT.first = PinShape[i].RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].RT.first <= rect.RT.first && PinShape[i].RT.first >= rect.LB.first){
							Parser::I_Rect pch;
							pch.LB.second = PinShape[i].RT.second;
							pch.RT.second = rect.LB.second;
							//pch.LB.first = rect.LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = PinShape[i].RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.first <= rect.RT.first && PinShape[i].RT.first >= rect.RT.first){
							Parser::I_Rect pch;
							pch.LB.second = PinShape[i].RT.second;
							pch.RT.second = rect.LB.second;
							//pch.LB.first = rect.LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = rect.RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.first >= rect.RT.first && PinShape[i].RT.first <= rect.RT.first){
							Parser::I_Rect pch;
							pch.LB.second = PinShape[i].RT.second;
							pch.RT.second = rect.LB.second;
							//pch.LB.first = PinShape[i].LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = PinShape[i].RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
					}
					
				}
			}
			else if (PinShape[i].LB.second >  rect.RT.second){
				// Upper Enc
				if (LineOverlap(PinShape[i].LB.first, PinShape[i].RT.first, rect.LB.first, rect.RT.first)){
					if (PinShape[i].LB.second  < rect.RT.second + Spacing){
						if (PinShape[i].LB.first <= rect.RT.first && PinShape[i].LB.first >= rect.LB.first){
							Parser::I_Rect pch;
							pch.LB.second = rect.RT.second;
							pch.RT.second = PinShape[i].LB.second;
							//pch.LB.first = PinShape[i].LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = rect.RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].RT.first <= rect.RT.first && PinShape[i].RT.first >= rect.LB.first){
							Parser::I_Rect pch;
							pch.LB.second = rect.RT.second;
							pch.RT.second = PinShape[i].LB.second;
							//pch.LB.first = rect.LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = PinShape[i].RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.first <= rect.RT.first && PinShape[i].RT.first >= rect.RT.first){
							Parser::I_Rect pch;
							pch.LB.second = rect.RT.second;
							pch.RT.second = PinShape[i].LB.second;
							//pch.LB.first = rect.LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = rect.RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
						else if (PinShape[i].LB.first >= rect.RT.first && PinShape[i].RT.first <= rect.RT.first){
							Parser::I_Rect pch;
							pch.LB.second = rect.RT.second;
							pch.RT.second = PinShape[i].LB.second;
							//pch.LB.first = PinShape[i].LB.first;
							pch.LB.first = max(PinShape[i].LB.first, rect.LB.first);
							//pch.RT.first = PinShape[i].RT.first;
							pch.RT.first = min(PinShape[i].RT.first, rect.RT.first);
							pch.Layer = rect.Layer;
							Patch.push_back(pch);
						}
					}

				}

			}


		}
	}
}
