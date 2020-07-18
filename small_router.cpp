#include "small_router.h"
void Solution::SetSolution(
		const int short_val, 
		const int open_val, 
		const int overlap_val, 
		const int line_end_val
	)
{
	this->short_num = short_val;
	this->open_num = open_val;
	this->overlap_violation_num = overlap_val;
	this->line_end_violation_num = line_end_val;
}

void Solution::SetNets(vector<TwoPinRipUpNet> &nets_val)
{
	nets.assign(nets_val.begin(), nets_val.end());
}

void Solution::operator = (const Solution &sol)
{
	short_num = sol.short_num;
	open_num = sol.open_num;
	overlap_violation_num = sol.overlap_violation_num;
	line_end_violation_num = sol.line_end_violation_num;
	nets.assign(sol.nets.begin(), sol.nets.end());
}

void Solution::PrintSolution()
{
	cout << "Short net num ";
	cout << this->short_num << endl;
	cout << "Open net num ";
	cout << this->open_num << endl;
	cout << "Overlap num "; 
	cout << this->overlap_violation_num << endl;
	cout << "Line-end num ";
	cout << this->line_end_violation_num << endl;
}

// Is Solution a better than Solution b?
bool IsBetterSolution(const Solution &a, const Solution &b)
{
	bool is_better = false;
	const bool short_num_better = 
		(a.short_num <= b.short_num);
	const bool open_num_better = 
		(a.open_num <= b.open_num);
	const bool overlap_better = 
		(a.overlap_violation_num <= b.overlap_violation_num);
	const bool line_end_better = 
		(a.line_end_violation_num <= b.line_end_violation_num);

	is_better = (
		short_num_better & 
		open_num_better & 
		overlap_better
		);

	if(is_better && a.short_num == 0 && 
			a.open_num == 0 && a.overlap_violation_num == 0)
	{
		is_better = line_end_better;
	}

	return is_better;
}



SmallRouter::SmallRouter() : 
	total_rip_up_times(0), 
	layout_width_(INT_MIN), 
	layout_height_(INT_MIN), 
	layout_layer_(INT_MIN),
	track_capacity_(0), 
	cost_array_(0), 
	back_track_array_(0),
	kLineEndModeRouting_(false),
	kEndEndSpacingMode_(false)
{
}

SmallRouter::SmallRouter(
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
	vector<vector<vector<int>>> &EOL_Blocage_Map,
	vector<SpaceEvaluationGraph> &_SpaceEvaluationLayout_,
	vector < vector < vector < Parser::Enc_Relation > > >  &Enc_Relation_Pair_Table,
	Parser::EncId_HashTable &_EncId_HashTable_,
	vector<int> MinWidth_list,
	vector<vector<vector<pair<int, int>>>> &Pin_Cover_Map,
	vector<vector<vector<int>>> &ISPD_original_iroute_array,
	vector<vector<vector<bool>>> &ISPD_L2_OBS_array_,
	const bool Metal1Dir,
	const bool kLineEndModeRouting,
	const bool kEndEndSpacingMode) : total_rip_up_times(0),
									 layout_width_(layout_width),
									 layout_height_(layout_height),
									 layout_layer_(layout_layer),
									 ISPD_Layer_track_list(Layer_track_list),
									 track_capacity_(NULL),
									 cost_array_(NULL),
									 back_track_array_(NULL),
									 ISPD_Via_cut_Spacing(ISPD_Via_cut_Spacing),
									 ISPD_Spacing_Table(Spacing_Table),
									 ISPD_EndOfLine_Spacing(ISPD_EndOfLine_Spacing),
									 ISPD_via_type_list(ISPD_via_type_list),
									 ISPD_MinWidth_list(MinWidth_list),
									 kLineEndModeRouting_(kLineEndModeRouting),
									 kEndEndSpacingMode_(kEndEndSpacingMode)
{

	cout << endl << "========== Start Routing Info Initialization ==========" << endl << endl;

	if(kLineEndModeRouting_ && kEndEndSpacingMode_)
	{
		cout << "SmallRouter: Mode setting error in routing." << endl;
		exit(1);
	}
	else
	{
		cout << "Line-end Routing Mode = " << std::boolalpha << kLineEndModeRouting_ << endl;
		cout << "nX Routing Mode = " << std::boolalpha << kEndEndSpacingMode_ << endl;
	}

	this->ISPD_fast_coor_map = &fast_coor_map;
	this->RectangleMapping.SetGraph(fast_coor_map);
	//
	ISPD_Enc_Relation_Pair_Table = Enc_Relation_Pair_Table;
	ISPD_enc_id_HashTable = _EncId_HashTable_;
	this->ISPD_GridMap_layout = &GridMap_layout;
	this->ISPD_connection_table = connection_table;
	this->ISPD_real_coor_table = real_coor_table;

	/*int pause;
	cin >> pause;*/
	printf("SmallRouter::Two_pin_nets_ Assignment - Two Pin Net Num %d (unmodified)\n", two_pin_rip_up_net.size());
	// Init nets.
	rip_up_two_pin_nets_.resize(two_pin_rip_up_net.size());
	for(int NetIndex = 0 ; NetIndex < two_pin_rip_up_net.size() ; NetIndex++)
	{
		rip_up_two_pin_nets_.at(NetIndex).netname = two_pin_rip_up_net.at(NetIndex).netname;
		rip_up_two_pin_nets_.at(NetIndex).TwoPinNetData = two_pin_rip_up_net.at(NetIndex).TwoPinNetData;
		rip_up_two_pin_nets_.at(NetIndex).TwoPinNetList = two_pin_rip_up_net.at(NetIndex).TwoPinNetList;
		rip_up_two_pin_nets_.at(NetIndex).DesignPinList = two_pin_rip_up_net.at(NetIndex).DesignPinList;
		rip_up_two_pin_nets_.at(NetIndex).TargetBound = two_pin_rip_up_net.at(NetIndex).TargetBound;
		rip_up_two_pin_nets_.at(NetIndex).net_id = two_pin_rip_up_net.at(NetIndex).net_id;
		rip_up_two_pin_nets_.at(NetIndex).two_pin_net_id = two_pin_rip_up_net.at(NetIndex).two_pin_net_id;
		rip_up_two_pin_nets_.at(NetIndex).rip_up_times = two_pin_rip_up_net.at(NetIndex).rip_up_times;
		rip_up_two_pin_nets_.at(NetIndex).Global_guide = two_pin_rip_up_net.at(NetIndex).Global_guide;
		rip_up_two_pin_nets_.at(NetIndex).violation_num = two_pin_rip_up_net.at(NetIndex).violation_num;
		rip_up_two_pin_nets_.at(NetIndex).wire_length = two_pin_rip_up_net.at(NetIndex).wire_length;
		//source
		rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.src_type = two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.src_type;
		int SourcePinSize = (int)two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.source_pin_list.size();
		rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.source_pin_list.resize(SourcePinSize);
		for (int ConnectionIndex = 0; ConnectionIndex < SourcePinSize; ConnectionIndex++)
		{
			int x, y, z;
			two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.source_pin_list.at(ConnectionIndex).GetPinPointPosition(x, y, z);
			rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.source_pin_list.at(ConnectionIndex).SetPinPoint(kPinPoint, x, y, z);
		}

		two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.source_pin_list.clear();
		two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.source_pin_list.resize(0);

		//target
		rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.tar_type = two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.tar_type;
		int TargetPinSize = (int)two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.target_pin_list.size();
		rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.target_pin_list.resize(TargetPinSize);
		for (int ConnectionIndex = 0; ConnectionIndex < TargetPinSize; ConnectionIndex++)
		{
			int x, y, z;
			two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.target_pin_list.at(ConnectionIndex).GetPinPointPosition(x, y, z);
			rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.target_pin_list.at(ConnectionIndex).SetPinPoint(kPinPoint, x, y, z);
		}

		two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.target_pin_list.clear();
		two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.target_pin_list.resize(0);

		// pseudo source
		int PseudoSourcePinSize = (int)two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.pseudo_Source_pin_path_list.size();
		rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.pseudo_Source_pin_path_list.resize(PseudoSourcePinSize);
		for (int ConnectionIndex = 0; ConnectionIndex < PseudoSourcePinSize; ConnectionIndex++)
		{
			int x, y, z;
			two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.pseudo_Source_pin_path_list.at(ConnectionIndex).GetPinPointPosition(x, y, z);
			rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.pseudo_Source_pin_path_list.at(ConnectionIndex).SetPinPoint(kPinPoint, x, y, z);
		}

		two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.pseudo_Source_pin_path_list.clear();
		two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.pseudo_Source_pin_path_list.resize(0);

		// pseudo target
		int PseudoTargetPinSize = (int)two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.pseudo_Target_pin_path_list.size();
		rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.pseudo_Target_pin_path_list.resize(PseudoTargetPinSize);
		for (int ConnectionIndex = 0; ConnectionIndex < PseudoTargetPinSize; ConnectionIndex++)
		{
			int x, y, z;
			two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.pseudo_Target_pin_path_list.at(ConnectionIndex).GetPinPointPosition(x, y, z);
			rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.pseudo_Target_pin_path_list.at(ConnectionIndex).SetPinPoint(kPinPoint, x, y, z);
		}

		two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.pseudo_Target_pin_path_list.clear();
		two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.pseudo_Target_pin_path_list.resize(0);

		// real source
		int RealSourcePinSize = (int)two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.real_Source_pin_path_list.size();
		rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.real_Source_pin_path_list.resize(RealSourcePinSize);
		for (int ConnectionIndex = 0; ConnectionIndex < RealSourcePinSize; ConnectionIndex++)
		{
			int x, y, z;
			two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.real_Source_pin_path_list.at(ConnectionIndex).GetPinPointPosition(x, y, z);
			rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.real_Source_pin_path_list.at(ConnectionIndex).SetPinPoint(kPinPoint, x, y, z);
		}

		two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.real_Source_pin_path_list.clear();
		two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.real_Source_pin_path_list.resize(0);

		// real target
		int RealTargetPinSize = (int)two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.real_Target_pin_path_list.size();
		rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.real_Target_pin_path_list.resize(RealTargetPinSize);
		for (int ConnectionIndex = 0; ConnectionIndex < RealTargetPinSize; ConnectionIndex++)
		{
			int x, y, z;
			two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.real_Target_pin_path_list.at(ConnectionIndex).GetPinPointPosition(x, y, z);
			rip_up_two_pin_nets_.at(NetIndex).two_pin_net_connection.real_Target_pin_path_list.at(ConnectionIndex).SetPinPoint(kPinPoint, x, y, z);
		}

		two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.real_Target_pin_path_list.clear();
		two_pin_rip_up_net.at(NetIndex).two_pin_net_connection.real_Target_pin_path_list.resize(0);

		rip_up_two_pin_nets_.at(NetIndex).two_pin_net_detailed_path = two_pin_rip_up_net.at(NetIndex).two_pin_net_detailed_path;
		rip_up_two_pin_nets_.at(NetIndex).path_is_found = two_pin_rip_up_net.at(NetIndex).path_is_found;
		rip_up_two_pin_nets_.at(NetIndex).both_pins_are_overlapped = two_pin_rip_up_net.at(NetIndex).both_pins_are_overlapped;
		rip_up_two_pin_nets_.at(NetIndex).has_only_one_pin = two_pin_rip_up_net.at(NetIndex).has_only_one_pin;
	}

	/*
	rip_up_two_pin_nets_.assign(
			two_pin_rip_up_net.begin(), two_pin_rip_up_net.end());
	*/
	/*for(TwoPinRipUpNet &net : rip_up_two_pin_nets_)
	{
		const bool pins_are_overlapped = 
			net.two_pin_net_connection.DoesSourceOverlapTarget();
		const bool only_one_pin = 
			net.two_pin_net_connection.IsSourceEqualToTarget();
		if(pins_are_overlapped)
			net.both_pins_are_overlapped = true; 
		else if(only_one_pin)
			net.has_only_one_pin = true;
		else
			net.two_pin_net_connection.SetTargetTable( 
					layout_width, layout_height, layout_layer);

	}*/
	printf("SmallRouter::Init Obstacles (unmodified)\n");
	// Init Obstacles
	//this->obstacle_.assign(obstacle_nodes.begin(), obstacle_nodes.end());

	printf("SmallRouter::Init via enclosure expansion (unmodified)\n");
	// Init via enclosure expansion.
	kViaEncExpansion_.resize(layout_layer_);
	for(size_t idx = 0; idx < kViaEncExpansion_.size(); idx++)
		kViaEncExpansion_[idx] = false;

	printf("SmallRouter::Init Prefer direction \n");
	this->prefer_dir.push_back(Metal1Dir);
	for (int i = 1; i < layout_layer_; i++){
		if (this->prefer_dir[i - 1] == true){
			this->prefer_dir[i] = false;
		}
		else{
			this->prefer_dir[i] = true;
		}
	}

	printf("SmallRouter::ISPD_L2_OBS_array\n");
	this->ISPD_L2_OBS_array = ISPD_L2_OBS_array_;
	printf("SmallRouter::ISPD_set_iroute_array\n");
	this->ISPD_iroute_array = ISPD_original_iroute_array;

	printf("SmallRouter::ISPD_set_Target_array\n");
	ISPD_set_Target_array_();
	printf("SmallRouter::ISPD_set_Subtree_array\n");
	//ISPD_set_Subtree_array_();

	//cin >> pause;
	//this->SpaceEvaluationLayout.assign(_SpaceEvaluationLayout_.begin(), _SpaceEvaluationLayout_.end());

	this->SpaceEvaluationLayout.resize(_SpaceEvaluationLayout_.size());
	for (int i = 0; i < _SpaceEvaluationLayout_.size(); i++)
	{
		this->SpaceEvaluationLayout.at(i).spacing_table.resize(_SpaceEvaluationLayout_.at(i).spacing_table.size());
		for (int j = 0; j < _SpaceEvaluationLayout_.at(i).spacing_table.size(); j++)
		{
			this->SpaceEvaluationLayout.at(i).spacing_table.at(j) = make_pair(_SpaceEvaluationLayout_.at(i).spacing_table.at(j).first, _SpaceEvaluationLayout_.at(i).spacing_table.at(j).second);
		} // for

		_SpaceEvaluationLayout_.at(i).spacing_table.clear();

		this->SpaceEvaluationLayout.at(i).eol_rule = _SpaceEvaluationLayout_.at(i).eol_rule;
		this->SpaceEvaluationLayout.at(i).via_MetalWidth = _SpaceEvaluationLayout_.at(i).via_MetalWidth;

		this->SpaceEvaluationLayout.at(i).SEG_real_coor_table = _SpaceEvaluationLayout_.at(i).SEG_real_coor_table; //_SpaceEvaluationLayout_.at(i)._space_eval_graph_;
		
		/*for (int j = 0; j < _SpaceEvaluationLayout_.at(i)._space_eval_graph_.size(); j++)
		{
			this->SpaceEvaluationLayout.at(i)._space_eval_graph_.at(j) = _SpaceEvaluationLayout_.at(i)._space_eval_graph_.at(j);
		} // for*/

		/*this->SpaceEvaluationLayout.at(i)._fast_coor_map_.resize(_SpaceEvaluationLayout_.at(i)._fast_coor_map_.size());
		for (int j = 0; j < _SpaceEvaluationLayout_.at(i)._fast_coor_map_.size(); j++)
		{
			this->SpaceEvaluationLayout.at(i)._fast_coor_map_.at(j) = make_pair(_SpaceEvaluationLayout_.at(i)._fast_coor_map_.at(j).first, _SpaceEvaluationLayout_.at(i)._fast_coor_map_.at(j).second);
		} // for*/

		this->SpaceEvaluationLayout.at(i).ALL_Shape.resize(_SpaceEvaluationLayout_.at(i).ALL_Shape.size());
		for (int j = 0; j < _SpaceEvaluationLayout_.at(i).ALL_Shape.size(); j++)
		{
			this->SpaceEvaluationLayout.at(i).ALL_Shape.at(j) = _SpaceEvaluationLayout_.at(i).ALL_Shape.at(j);
		} // for

		_SpaceEvaluationLayout_.at(i).ALL_Shape.clear();

		this->SpaceEvaluationLayout.at(i).ALL_Shape_Net_list.resize(_SpaceEvaluationLayout_.at(i).ALL_Shape_Net_list.size());
		for (int j = 0; j < _SpaceEvaluationLayout_.at(i).ALL_Shape_Net_list.size(); j++)
		{
			this->SpaceEvaluationLayout.at(i).ALL_Shape_Net_list.at(j) = _SpaceEvaluationLayout_.at(i).ALL_Shape_Net_list.at(j);
		} // for

		_SpaceEvaluationLayout_.at(i).ALL_Shape_Net_list.clear();

		// important rtree
		this->SpaceEvaluationLayout.at(i).tree = _SpaceEvaluationLayout_.at(i).tree;
		this->SpaceEvaluationLayout.at(i).EOL_tree = _SpaceEvaluationLayout_.at(i).EOL_tree;
		this->SpaceEvaluationLayout.at(i).Layer = _SpaceEvaluationLayout_.at(i).Layer;
		//cout << this->SpaceEvaluationLayout.at(i).tree << endl;

		//_SpaceEvaluationLayout_.at(i).clear();
	}

	printf("SmallRouter::Init setting \n");
	//ISPD_set_track_capacity();
	ISPD_set_cost_array();
	ISPD_set_back_track_array();
	ISPD_set_back_track_Step_array_();
	//ISPD_Restore_Origin_Blockage_array(Blocage_Map, EOL_Blocage_Map);
	ISPD_set_blockage_array_(Blocage_Map);

	
	/*for (int i = 0; i < Blocage_Map.size(); i++)
	{
		for (int j = 0; j < Blocage_Map[i].size(); j++)
		{
			Blocage_Map[i][j].clear();
		}
		Blocage_Map[i].clear();
	}
	Blocage_Map.clear();*/
	
	
	ISPD_set_eol_blockage_array_(EOL_Blocage_Map);
	
	/*for (int i = 0; i < EOL_Blocage_Map.size(); i++)
	{
		for (int j = 0; j < EOL_Blocage_Map[i].size(); j++)
		{
			EOL_Blocage_Map[i][j].clear();
		}
		EOL_Blocage_Map[i].clear();
	}
	EOL_Blocage_Map.clear();*/
	
	ISPD_set_Viatype_array_();
	//ISPD_set_Viablockage_array_();
	ISPD_set_Congestion_map_(Pin_Cover_Map);
	
	/*for (int i = 0; i < Pin_Cover_Map.size(); i++)
	{
		for (int j = 0; j < Pin_Cover_Map[i].size(); j++)
		{
			Pin_Cover_Map[i][j].clear();
		}
		Pin_Cover_Map[i].clear();
	}
	Pin_Cover_Map.clear();*/
	
	//cin >> pause;
	Net_Open_Num = 0;
	//set_id_table();
	//SetNetPinsToTrack();
	//SetObstacleToTrack();
	printf("SmallRouter::Init complete\n");
}

void SmallRouter::ISPD_Restore_Origin_Blockage_array(vector<vector<vector<int> > > &Blocage_Map,
													 vector<vector<vector<int> > > &EOL_Blocage_Map)
{
	this->ISPD_origin_blockage_array_.resize((*ISPD_GridMap_layout).size());
	this->ISPD_origin_EOL_Blocage_Map.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		this->ISPD_origin_blockage_array_[i].resize((*ISPD_GridMap_layout)[i].size());
		this->ISPD_origin_EOL_Blocage_Map[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			this->ISPD_origin_blockage_array_[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
			this->ISPD_origin_EOL_Blocage_Map[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++)
			{
				this->ISPD_origin_blockage_array_[i][j][k] = Blocage_Map[i][j][k];
				this->ISPD_origin_EOL_Blocage_Map[i][j][k] = EOL_Blocage_Map[i][j][k];
			}
		}
	}
}

	/*SmallRouter::~SmallRouter() 
{
	if(track_capacity_ != NULL)
		delete [] track_capacity_;
	if(cost_array_ != NULL)
		delete [] cost_array_;
	if(back_track_array_ != NULL)
		delete [] back_track_array_;
}*/

	// Set the preferred direction
	void SmallRouter::SetPreferredDirection(
		const bool m1_is_verical,
		const bool m2_is_verical)
{
	const bool the_same_direction = (m1_is_verical == m2_is_verical);
	if(the_same_direction)
	{
		cout << "SmallRouter: ERROR: ";
		cout << "The preferred directions are the same in M1 & M2!" << endl;
		exit(1);
	}

	cout << "========== Set Preferred Direction ===========" << endl;
	cout << "1st routing layer vertical = ";
	cout << std::boolalpha << m1_is_verical << endl;
	cout << "2ed routing layer vertical = ";
	cout << std::boolalpha << m2_is_verical << endl;

	if(m1_is_verical)
	{
		kOddLayer = 1;
		kEvenLayer = 0;
	}
	else
	{
		kOddLayer = 0;
		kEvenLayer = 1;
	}

	return;
}


// Set the required line-end spacing.
void SmallRouter::SetLineEndSpacing(
		RoutingLayerInfos &routing_layer_infos,
		const int low_layer,
		const int high_layer)
{
	this->line_end_graph_.SetLineEndSpacing(
			routing_layer_infos,
			low_layer,
			high_layer
			);
	return;
}

// Set the via enclosure expansion.
/*void SmallRouter::SetViaEncExpansion(
		RoutingLayerInfos &routing_layer_infos,
		const int low_layer,
		const int high_layer
		)
{
	if(!kViaEncCostMode_)
		return;
	cout << "========== Via Enc Expansion ==========" << endl;
	cout << "Set via enc expansion!" << endl;
	int m1_pitch = 0;
	int m2_pitch = 0;
	for(LayerInfo &info : routing_layer_infos.GetLayerInfos())
	{
		if(((int)info.LayerNum) == low_layer)
		{
			m1_pitch = (info.is_vertical) ? 
				(info.v_track_spacing) : (info.h_track_spacing);
		}
		if (((int)info.LayerNum) == (low_layer + 2))
		{
			m2_pitch = (info.is_vertical) ? 
				(info.v_track_spacing) : (info.h_track_spacing);
			break;
		}
	}
	cout << "M1 pitch = " << m1_pitch << endl;
	cout << "M2 pitch = " << m2_pitch << endl;

	if(m1_pitch == 0 || m2_pitch == 0)
	{
		cout << "SmallRouter: Wrong M1 or M2 pitch ";
		cout << "for setting via enclosure expansion." << endl;
		exit(1);
	}

	const int v_pitch = 
		(routing_layer_infos[(low_layer/2)-1].is_vertical) ? 
		(m2_pitch) : (m1_pitch);
	const int h_pitch = 
		(!routing_layer_infos[(low_layer/2)-1].is_vertical) ? 
		(m2_pitch) : (m1_pitch);
	int idx = 0;
	for(int layer_num = low_layer; layer_num < high_layer; layer_num+=2)
	{
		if(layer_num == low_layer)
		{
			LayerInfo &info = routing_layer_infos[(layer_num / 2) - 1];
			const int lower_pitch = 
				(int)(info.is_vertical) ? (v_pitch) : (h_pitch);
			const int lower_spacing = (int)(info.spacing);
			const int lower_enc = (info.is_vertical) ?
				(info.upper_enc1_height) : (info.upper_enc1_width);
			const int half_lower_enc = lower_enc / 2;
			if(half_lower_enc > lower_pitch)
			{
				kViaEncExpansion_[0] = true;
			}
			else
			{
				const int pitch_enc_diff = lower_pitch - half_lower_enc;
				if(pitch_enc_diff >= lower_spacing)
				{
					kViaEncExpansion_[0] = false;
				}
				else
				{
					kViaEncExpansion_[0] = 
						((pitch_enc_diff * 2) >= lower_spacing) ?
						(false) : (true);
				}
			}
			const int upper_pitch = 
				(int)(!info.is_vertical) ? (v_pitch) : (h_pitch);
			LayerInfo &next_info = routing_layer_infos[(layer_num / 2)];
			const int upper_spacing = (int)(next_info.spacing);
			const int upper_enc = (next_info.is_vertical) ?
				(info.upper_enc2_height) : (info.upper_enc2_width);
			const int half_upper_enc = upper_enc / 2;
			if(half_upper_enc > upper_pitch)
			{
				kViaEncExpansion_[1] = true;
			}
			else
			{
				const int pitch_enc_diff = upper_pitch - half_upper_enc;
				if(pitch_enc_diff >= upper_spacing)
				{
					kViaEncExpansion_[1] = false;
				}
				else
				{
					kViaEncExpansion_[1] = 
						((pitch_enc_diff * 2) >= upper_spacing) ?
						(false) : (true);
				}
			}
			idx += 2;
		}
		else
		{
			LayerInfo &info = routing_layer_infos[(layer_num / 2)];
			const int pitch = 
				(int)(info.is_vertical) ? (v_pitch) : (h_pitch);
			const int spacing = (int)(info.spacing);
			const int upper_enc = (info.is_vertical) ?
				(info.upper_enc2_height) : (info.upper_enc2_width);
			const int half_upper_enc = upper_enc / 2;
			if(half_upper_enc > pitch)
			{
				kViaEncExpansion_[idx] = true;
			}
			else
			{
				const int pitch_enc_diff = pitch - half_upper_enc;
				if(pitch_enc_diff >= spacing)
				{
					kViaEncExpansion_[idx] = false;
				}
				else
				{
					kViaEncExpansion_[idx] = 
						((pitch_enc_diff * 2) >= spacing) ?
						(false) : (true);
				}
			}
			idx++;
		}
	}
	for(size_t idx = 0; idx < kViaEncExpansion_.size(); idx++)
	{
		cout << "Layer " << idx << ", Expand = ";
		cout << std::boolalpha << kViaEncExpansion_[idx] << endl;
	}
	return;
}*/


// Allocate memory and set the value of track_capacity_
void SmallRouter::set_track_capacity()
{
	const ULL array_size = 
		(ULL) layout_width_ * layout_height_ * layout_layer_;
	track_capacity_ = new int[array_size];
	for(ULL index = 0; index < array_size; index++)
		track_capacity_[index] = kNoBlockage;
}
void SmallRouter::ISPD_set_track_capacity(){
	this->ISPD_track_capacity_.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		this->ISPD_track_capacity_[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			this->ISPD_track_capacity_[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++){
				this->ISPD_track_capacity_[i][j][k] = kNoBlockage;
			}
		}
	}
}

// Allocate memory and set the value of cost_array_
void SmallRouter::set_cost_array()
{
	const ULL array_size = 
		(ULL) layout_width_ * layout_height_ * layout_layer_;
	cost_array_ = new float[array_size];
	for(ULL index = 0; index < array_size; index++)
		cost_array_[index] = INT_MAX;
}

void SmallRouter::ISPD_set_cost_array(){
	this->ISPD_cost_array_.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		this->ISPD_cost_array_[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			this->ISPD_cost_array_[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++){
				this->ISPD_cost_array_[i][j][k] = INT_MAX;
			}
		}
	}
}


// Allocate memory and set the value of back_track_array_
void SmallRouter::set_back_track_array()
{
	const ULL array_size = 
		(ULL) layout_width_ * layout_height_ * layout_layer_;
	back_track_array_ = new int[array_size];
	for(ULL index = 0; index < array_size; index++)
		back_track_array_[index] = kInitDirection;
}

void SmallRouter::ISPD_set_back_track_array(){
	this->ISPD_back_track_array_.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		this->ISPD_back_track_array_[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			this->ISPD_back_track_array_[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++){
				this->ISPD_back_track_array_[i][j][k] = kInitDirection;
			}
		}
	}
}


// Set the net id and two pin net id to the IdTable
// Map [two pin net id] to (net id, two pin net id)
void SmallRouter::set_id_table()
{
	int two_pin_net_index = 0;
	for(TwoPinRipUpNet &ripup_net : rip_up_two_pin_nets_)
	{
		const int net_id = ripup_net.net_id;
		const int two_pin_net_id = ripup_net.two_pin_net_id;
		
		NetIdPair temp_pair(net_id, two_pin_net_id, two_pin_net_index);
		net_id_table_[two_pin_net_id] = temp_pair;
		
		const bool net_id_key_exist = 
			(net_id_table_.find(net_id) != net_id_table_.end());
		if((!net_id_key_exist))
		{
			const int multi_net_two_pin_id = -1;
			const int multi_net_index = -1;
			NetIdPair temp_multi_net_pair(
					net_id, multi_net_two_pin_id, multi_net_index);
			net_id_table_[net_id] = temp_multi_net_pair;
		}

		two_pin_net_index++;
	}
	
}


// Reset the value of cost_array_
inline void SmallRouter::reset_cost_array(const float value)
{
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++){
				this->ISPD_cost_array_[i][j][k] = value;
			}
		}
	}
}


// Reset the value of back_track_array_
inline void SmallRouter::reset_back_track_array()
{
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++){
				this->ISPD_back_track_array_[i][j][k] = kInitDirection;
			}
		}
	}
}


void SmallRouter::SetNetPinsToTrack()
{
	cout << endl << "========== Set Pin To Track ==========" << endl;
	int wrong_grid_count = 0;
	/*for(TwoPinRipUpNet &ripup_net : rip_up_two_pin_nets_)
	{
		cout << "Net id = " << ripup_net.net_id << endl;
		cout << "2-Pin id = " << ripup_net.two_pin_net_id << endl;
		cout << "\t";
		ripup_net.GetConnection().PrintTargetTable();
	}*/
	for(TwoPinRipUpNet &ripup_net : rip_up_two_pin_nets_)
	{
		const int two_pin_net_id = ripup_net.two_pin_net_id;
		
		vector<Node> from_nodes;
		ripup_net.two_pin_net_connection.source.GetPinNodes(from_nodes);
		for(const Node &from_node : from_nodes)
		{
			const ULL from_index = Transform3dArrayTo1dIndex(from_node);
			const bool one_net_on_from_grid = 
				(track_capacity_[from_index] > 0);
			const bool no_net_on_grid = 
				(track_capacity_[from_index] == kNoBlockage);
			if(one_net_on_from_grid)
			{
				const int grid_id = track_capacity_[from_index];
				const NetIdPair &grid_id_pair = net_id_table_[grid_id];
				const int grid_net_id = grid_id_pair.net_id;
				const int grid_two_pin_net_id = grid_id_pair.two_pin_net_id;
				
				const int net_id = net_id_table_[two_pin_net_id].net_id;
				
				const bool belong_to_same_multi_pin_net = (net_id == grid_net_id);
				const bool grid_id_is_multi_net_id = (grid_two_pin_net_id == -1);
				if(belong_to_same_multi_pin_net && grid_id_is_multi_net_id)
				{
					// don't need to change capacity, just push two pin net id to layout_map
					layout_map_[from_index].push_back(two_pin_net_id);
				}
				else if(belong_to_same_multi_pin_net && !grid_id_is_multi_net_id)
				{
					// change capacity, and push grid two pin net id and two pin net id to layout_map
					layout_map_[from_index].push_back(two_pin_net_id);
					layout_map_[from_index].push_back(grid_two_pin_net_id);
					track_capacity_[from_index] = net_id;
					
				}
				else if(!belong_to_same_multi_pin_net && grid_id_is_multi_net_id)
				{
					cout << "(From node) Wrong grid location = " << from_node << endl;
					cout << "Net on grid : net id = " << grid_net_id << " ";
					cout << "two pin net id = " << grid_two_pin_net_id << endl;
					cout << "Net to be : net id = " << net_id << " ";
					cout << "two pin net id = " << two_pin_net_id << endl;
					track_capacity_[from_index] = kBlockage;
					ripup_net.GetConnection().EraseTargetKey(from_index);
					for(const int grid_two_pin_net_id : layout_map_[from_index])
					{
						const int grid_two_pin_net_index =
							net_id_table_[grid_two_pin_net_id].two_pin_net_index;
						TwoPinRipUpNet &occupy_net = 
							rip_up_two_pin_nets_[grid_two_pin_net_index];
						occupy_net.GetConnection().EraseTargetKey(from_index);
					}
					wrong_grid_count++;
				}
				else
				{
					cout << "(From node) Wrong grid location = " << from_node << endl;
					cout << "Net on grid : net id = " << grid_net_id << " ";
					cout << "two pin net id = " << grid_two_pin_net_id << endl;
					cout << "Net to be : net id = " << net_id << " ";
					cout << "two pin net id = " << two_pin_net_id << endl;
					track_capacity_[from_index] = kBlockage;
					ripup_net.GetConnection().EraseTargetKey(from_index);
					const int grid_two_pin_net_index = 
						net_id_table_[grid_two_pin_net_id].two_pin_net_index;
					TwoPinRipUpNet &occupy_net = 
						rip_up_two_pin_nets_[grid_two_pin_net_index];
					occupy_net.GetConnection().EraseTargetKey(from_index);
					wrong_grid_count++;
				}
				
			}
			else if(no_net_on_grid)
			{
				track_capacity_[from_index] = two_pin_net_id;
			}
			else
			{}
		}

		vector<Node> to_nodes;
		ripup_net.two_pin_net_connection.target.GetPinNodes(to_nodes);
		for(const Node &to_node : to_nodes)
		{
			const ULL to_index = Transform3dArrayTo1dIndex(to_node);
			
			const bool one_net_on_to_grid = 
				(track_capacity_[to_index] > 0);
			const bool no_net_on_grid =
				(track_capacity_[to_index] == kNoBlockage);
			if(one_net_on_to_grid)
			{
				const int grid_id = track_capacity_[to_index];
				const NetIdPair &grid_id_pair = net_id_table_[grid_id];
				const int grid_net_id = grid_id_pair.net_id;
				const int grid_two_pin_net_id = grid_id_pair.two_pin_net_id;
				
				const int net_id = net_id_table_[two_pin_net_id].net_id;
				
				const bool belong_to_same_multi_pin_net = (net_id == grid_net_id);
				const bool grid_id_is_multi_net_id = (grid_two_pin_net_id == -1);
				if(belong_to_same_multi_pin_net && grid_id_is_multi_net_id)
				{
					// don't need to change capacity, just push two pin net id to layout_map
					layout_map_[to_index].push_back(two_pin_net_id);
				}
				else if(belong_to_same_multi_pin_net && !grid_id_is_multi_net_id)
				{
					// change capacity, and push grid two pin net id and two pin net id to layout_map
					layout_map_[to_index].push_back(two_pin_net_id);
					layout_map_[to_index].push_back(grid_two_pin_net_id);
					track_capacity_[to_index] = net_id;
					
				}
				else if(!belong_to_same_multi_pin_net && grid_id_is_multi_net_id)
				{
					cout << "(To node) Wrong grid location = " << to_node << endl;
					cout << "Net on grid : net id = " << grid_net_id << " ";
					cout << "two pin net id = " << grid_two_pin_net_id << endl;
					cout << "Net to be : net id = " << net_id << " ";
					cout << "two pin net id = " << two_pin_net_id << endl;
					track_capacity_[to_index] = kBlockage;
					ripup_net.GetConnection().EraseTargetKey(to_index);
					for(const int grid_two_pin_net_id : layout_map_[to_index])
					{
						const int grid_two_pin_net_index =
							net_id_table_[grid_two_pin_net_id].two_pin_net_index;
						TwoPinRipUpNet &occupy_net = 
							rip_up_two_pin_nets_[grid_two_pin_net_index];
						occupy_net.GetConnection().EraseTargetKey(to_index);
					}
					wrong_grid_count++;
				}
				else
				{
					cout << "(To node) Wrong grid location = " << to_node << endl;
					cout << "Net on grid : net id = " << grid_net_id << " ";
					cout << "two pin net id = " << grid_two_pin_net_id << endl;
					cout << "Net to be : net id = " << net_id << " ";
					cout << "two pin net id = " << two_pin_net_id << endl;
					track_capacity_[to_index] = kBlockage;
					ripup_net.GetConnection().EraseTargetKey(to_index);
					const int grid_two_pin_net_index = 
						net_id_table_[grid_two_pin_net_id].two_pin_net_index;
					TwoPinRipUpNet &occupy_net = 
						rip_up_two_pin_nets_[grid_two_pin_net_index];
					occupy_net.GetConnection().EraseTargetKey(to_index);
					
					wrong_grid_count++;
				}
			}
			else if(no_net_on_grid)
			{
				track_capacity_[to_index] = two_pin_net_id;	
			}
			else
			{}
		}
	}

	cout << "Wrong Grid Sum = " << wrong_grid_count << endl;
}


// Set the blockage on track_capacity_ 
void SmallRouter::SetObstacleToTrack()
{
	for(const Node &ob_node : obstacle_)
	{
		const ULL obstacle_index = 
			Transform3dArrayTo1dIndex(ob_node);
		track_capacity_[obstacle_index] = kBlockage;
	}	
	return;
}


// Initial the pin postion for line-end graph
void SmallRouter::InitPinLocationLineEndGraph()
{
	if(rip_up_two_pin_nets_.empty())
	{
		cout << "SmallRouter: No nets to be ripped-up and rerouted." << endl;
		exit(1);
	}
	
	// this->line_end_graph_.PrintLineEndGraph();
	vector<Node> line_end_pin_nodes;
	for(TwoPinRipUpNet &ripup_net : rip_up_two_pin_nets_)
	{
		const TwoPinNetConnection &connection = 
			ripup_net.GetConnection();
		
		// source pin part initialization in the line-end graph
		vector<Node> source_pins;
		connection.source.GetWholePinNodes(source_pins);
		PinType source_pin_type = connection.source.pin_type;
		line_end_graph_.InitPartialPinLocation(
				source_pin_type, 
				source_pins
				);
	
		// get the line-end position of the source pin
		vector<Node> source_line_end_nodes;
		connection.source.GetPinLineEndPoints(source_line_end_nodes);
		if(!source_line_end_nodes.empty())
		{
			// append the line-end nodes
			line_end_pin_nodes.insert(
					line_end_pin_nodes.end(), 
					source_line_end_nodes.begin(), 
					source_line_end_nodes.end()
					);
		}

		// target pin part initialization in the line-end graph
		vector<Node> target_pins;
		connection.target.GetWholePinNodes(target_pins);
		PinType target_pin_type = connection.target.pin_type;
		this->line_end_graph_.InitPartialPinLocation(
				target_pin_type, 
				target_pins
				);
		
		// get the line-end position of the target pin
		vector<Node> target_line_end_nodes;
		connection.target.GetPinLineEndPoints(target_line_end_nodes);
		if(!target_line_end_nodes.empty())
		{
			line_end_pin_nodes.insert(
					line_end_pin_nodes.end(), 
					target_line_end_nodes.begin(), 
					target_line_end_nodes.end()
					);
		}
	}
	// this->line_end_graph_.PrintLineEndGraph();
	this->line_end_graph_.InsertBlockageForPin(line_end_pin_nodes);
	// this->line_end_graph_.PrintLineEndGraph();

	return;
}


// The Main Routing Flow 
void SmallRouter::StartRouting(
		RoutingLayerInfos &routing_layer_infos,
		const int low_layer,
		const int high_layer
		)
{
	/*history_congestion_graph_.SetUpGraph(
			layout_width_, layout_height_, layout_layer_, kLineEndModeRouting_);*/
	if(this->kLineEndModeRouting_)
	{
		line_end_graph_.SetViaEncCostMode(this->kViaEncCostMode_);
		// line_end_graph_.SetViaEncCostMode(false);
		// line_end_graph_.SetMaskNum(0, 2);
		// line_end_graph_.SetMaskNum(1, 2);
		line_end_graph_.SetUpGraph(
				layout_width_, layout_height_, layout_layer_,
				track_capacity_, &net_id_table_);
		line_end_graph_.SetPreferredDirection(kOddLayer, kEvenLayer);
		line_end_graph_.SetLineEndSpacing(
				routing_layer_infos, low_layer, high_layer);
		InitPinLocationLineEndGraph();
	}

	cout << endl <<"========== Start to Route the Layout ==========" << endl << endl;
	cout << "Basic info" << endl;
	cout << "Layout grid size: ";
	cout << " width " << layout_width_;
	cout << " height " << layout_height_;
	cout << " layer " << layout_layer_ << endl;
	const int multi_net_num = CountMultiNetNumber();
	cout << "Multi net num:  " << multi_net_num << endl;
	cout << "2-pin net num:  " << rip_up_two_pin_nets_.size() << endl;
	
	cout << endl << "@++++++++++ First Phase Routing +++++++++@" << endl << endl;
	FirstPhaseRouting();
	// line_end_graph_.PrintLineEndGraph();
	// PrintTrackGraph();

	cout << endl << "@++++++++++ Second Phase (Rip-up & Reroute) Routing ++++++++++@" << endl << endl;
	SecondPhaseRouting();

	cout << endl << "@++++++++++ Final Results ++++++++++@" << endl << endl;
	int short_net_num = 0;
	int open_net_num = 0;
	int overlap_violation_num = 0;
	AnalyzeTheResult(short_net_num, open_net_num, overlap_violation_num);
	if(this->kLineEndModeRouting_)
	{
		line_end_graph_.AnalyzeLineEndViolation();
	}
}

void SmallRouter::ISPD_set_back_track_Step_array_(){
	this->ISPD_back_track_Step_array_.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		this->ISPD_back_track_Step_array_[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			this->ISPD_back_track_Step_array_[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++){
				this->ISPD_back_track_Step_array_[i][j][k] = 0;
			}
		}
	}
}

void SmallRouter::ISPD_set_Viatype_array_(){
	this->ISPD_Viatype_array_.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		this->ISPD_Viatype_array_[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			this->ISPD_Viatype_array_[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++){
				this->ISPD_Viatype_array_[i][j][k].clear();
			}
		}
	}
}

void SmallRouter::ISPD_set_Target_array_()
{
	this->ISPD_Target_Map.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		this->ISPD_Target_Map[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			this->ISPD_Target_Map[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++)
			{
				this->ISPD_Target_Map[i][j][k] = false;
			}
		}
	}

	
}

void SmallRouter::ISPD_reset_Target_array_()
{

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++)
			{
				this->ISPD_Target_Map[i][j][k] = false;
			}
		}
	}
}
/*
void SmallRouter::ISPD_reset_TrueTarget_array_()
{

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++)
			{
				this->ISPD_True_Target_Map_[i][j][k] = false;
			}
		}
	}
}
*/
void SmallRouter::ISPD_Insert_Target_array_(vector<Pin> &element)
{

	for (int i = 0; i < element.size(); i++)
	{
		
		int s_x = element[i].pin_position.pin_point.x;
		int s_y = element[i].pin_position.pin_point.y;
		int s_z = element[i].pin_position.pin_point.z;

		ISPD_TargetMap_modify_list.push_back(make_tuple(s_x, s_y, s_z));
		this->ISPD_Target_Map[s_z][s_y][s_x] = true;
	}
}

void SmallRouter::ISPD_set_Subtree_array_()
{
	this->ISPD_Subtree_Map = new bool**[(*ISPD_GridMap_layout).size()];
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		this->ISPD_Subtree_Map[i] = new bool*[(*ISPD_GridMap_layout)[i].size()];
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			this->ISPD_Subtree_Map[i][j] = new bool[(*ISPD_GridMap_layout)[i][j].size()];
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			std::memset(this->ISPD_Subtree_Map[i][j], 0, sizeof(bool) * (*ISPD_GridMap_layout)[i][j].size());
			
		}
	}

}

void SmallRouter::ISPD_reset_Subtree_ViaHit_PathHit_array_()
{
	for (int index = 0; index < this->ISPD_TargetMap_modify_list.size(); index++)
	{
		int z = get<2>(this->ISPD_TargetMap_modify_list[index]);
		int y = get<1>(this->ISPD_TargetMap_modify_list[index]);
		int x = get<0>(this->ISPD_TargetMap_modify_list[index]);
		this->ISPD_Target_Map[z][y][x] = false;
	}

	for (int index = 0; index < this->ISPD_ViaHit_modify_list.size(); index++)
	{
		int z = get<2>(this->ISPD_ViaHit_modify_list[index]);
		int y = get<1>(this->ISPD_ViaHit_modify_list[index]);
		int x = get<0>(this->ISPD_ViaHit_modify_list[index]);
		this->ISPD_ViaHit_array_[z][y][x] = false;
	}

	for (int index = 0; index < this->ISPD_PathHit_modify_list.size(); index++)
	{
		int z = get<2>(this->ISPD_PathHit_modify_list[index]);
		int y = get<1>(this->ISPD_PathHit_modify_list[index]);
		int x = get<0>(this->ISPD_PathHit_modify_list[index]);
		this->ISPD_PathHit_array_[z][y][x] = false;
	}

	for (int index = 0; index < this->ISPD_Subtree_modify_list.size(); index++)
	{
		int z = get<2>(this->ISPD_Subtree_modify_list[index]);
		int y = get<1>(this->ISPD_Subtree_modify_list[index]);
		int x = get<0>(this->ISPD_Subtree_modify_list[index]);
		this->ISPD_Subtree_Map[z][y][x] = false;
	}

	this->ISPD_TargetMap_modify_list.clear();
	this->ISPD_ViaHit_modify_list.clear();
	this->ISPD_PathHit_modify_list.clear();
	this->ISPD_Subtree_modify_list.clear();
}

void SmallRouter::ISPD_Free_Subtree_ViaHit_PathHit_array_()
{
	
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			//delete[] this->ISPD_Subtree_Map[i][j];
			delete[] this->ISPD_PathHit_array_[i][j];
			delete[] this->ISPD_ViaHit_array_[i][j];
		}
		//delete[] this->ISPD_Subtree_Map[i];
		delete[] this->ISPD_PathHit_array_[i];
		delete[] this->ISPD_ViaHit_array_[i];
	}
	//delete[] this->ISPD_Subtree_Map;
	delete[] this->ISPD_PathHit_array_;
	delete[] this->ISPD_ViaHit_array_;
}

void SmallRouter::ISPD_reset_Subtree_array_()
{

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			std::memset(this->ISPD_Subtree_Map[i][j], 0, sizeof(bool) * (*ISPD_GridMap_layout)[i][j].size());
			/*
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++)
			{
				this->ISPD_Subtree_Map[i][j][k] = false;
			}
			*/
		}
	}

}

void SmallRouter::ISPD_Insert_Subtree_array_(vector<Pin> &element)
{

	for (int i = 0; i < element.size(); i++)
	{

		int s_x = element[i].pin_position.pin_point.x;
		int s_y = element[i].pin_position.pin_point.y;
		int s_z = element[i].pin_position.pin_point.z;

		ISPD_Subtree_modify_list.push_back(make_tuple(s_x, s_y, s_z));
		this->ISPD_Subtree_Map[s_z][s_y][s_x] = true;
	}
}

void SmallRouter::ISPD_Insert_ViaHit_array_(int &x, int &y, int z)
{

	ISPD_ViaHit_modify_list.push_back(make_tuple(x, y, z));
	this->ISPD_ViaHit_array_[z][y][x] = true;
}

void SmallRouter::ISPD_Insert_PathHit_array_(int &x, int &y, int z)
{

	ISPD_PathHit_modify_list.push_back(make_tuple(x, y, z));
	this->ISPD_PathHit_array_[z][y][x] = true;
}

void SmallRouter::ISPD_set_NetID_array_()
{
	this->ISPD_NetID_array.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		this->ISPD_NetID_array[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			this->ISPD_NetID_array[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++)
			{
				this->ISPD_NetID_array[i][j][k] = -1;
			}
		}
	}
}

void SmallRouter::ISPD_reset_NetID_array_()
{

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++)
			{
				this->ISPD_NetID_array[i][j][k] = -1;
			}
		}
	}
}

void SmallRouter::ISPD_set_Congestion_map_(vector<vector<vector<pair<int, int> >>> &Pin_Cover_Map)
{
	this->ISPD_Congestion_Map = &Pin_Cover_Map;
	/*
	this->ISPD_Congestion_Map.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		this->ISPD_Congestion_Map[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			this->ISPD_Congestion_Map[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++)
			{
				this->ISPD_Congestion_Map[i][j][k].first = Pin_Cover_Map[i][j][k].first;
				this->ISPD_Congestion_Map[i][j][k].second = Pin_Cover_Map[i][j][k].second;
			}
		}
	}
	*/
}

void SmallRouter::ISPD_reset_Congestion_map_()
{
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++)
			{
				(*ISPD_Congestion_Map)[i][j][k] = make_pair(-1,0);
			}
		}
	}
}

void SmallRouter::ISPD_Add_Congestion_map_(int x, int y, int z, int AddNum)
{
	(*ISPD_Congestion_Map)[z][y][x].second += AddNum;
}

void SmallRouter::ISPD_reset_Viatype_array_(){
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++){
				this->ISPD_Viatype_array_[i][j][k].clear();
			}
		}
	}
}

void SmallRouter::ISPD_set_BitViaType_array_(){
	this->ISPD_Via_Type_Map.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		this->ISPD_Via_Type_Map[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			this->ISPD_Via_Type_Map[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++)
			{
				this->ISPD_Via_Type_Map[i][j][k].SetType(-1);
			}
		}
	}
}

void SmallRouter::ISPD_set_Viablockage_array_(){
	this->ISPD_Viablockage_array_.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		this->ISPD_Viablockage_array_[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			this->ISPD_Viablockage_array_[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++){
				this->ISPD_Viablockage_array_[i][j][k].layer = -1;
				this->ISPD_Viablockage_array_[i][j][k].net_id = -1;
				this->ISPD_Viablockage_array_[i][j][k].via_type_id = -1;
			}
		}
	}
}

void SmallRouter::ISPD_set_blockage_array_(vector < vector < vector <int> > > &Blocage_Map){
	this->ISPD_blockage_array_ = &Blocage_Map;
	/*
	this->ISPD_blockage_array_.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		this->ISPD_blockage_array_[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			this->ISPD_blockage_array_[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++){
				this->ISPD_blockage_array_[i][j][k] = Blocage_Map[i][j][k];
			}
		}
	}
	*/
}

void SmallRouter::ISPD_set_eol_blockage_array_(vector < vector < vector <int> > > &EOL_Blocage_Map){
	this->ISPD_EOL_Blocage_Map = &EOL_Blocage_Map;
	/*
	this->ISPD_EOL_Blocage_Map.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		this->ISPD_EOL_Blocage_Map[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			this->ISPD_EOL_Blocage_Map[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++){
				this->ISPD_EOL_Blocage_Map[i][j][k] = EOL_Blocage_Map[i][j][k];
			}
		}
	}
	*/
}

void SmallRouter::ISPD_reset_back_track_Step_array_(int value){
	for (int i = 0; i < this->ISPD_back_track_Step_array_.size(); i++){
		for (int j = 0; j < this->ISPD_back_track_Step_array_[i].size(); j++){
			for (int k = 0; k < this->ISPD_back_track_Step_array_[i][j].size(); k++){
				this->ISPD_back_track_Step_array_[i][j][k] = value;
			}
		}
	}
}

void SmallRouter::ISPD_set_history_congestion_graph_(){
	this->history_congestion_graph_.ISPD_history_congestion_graph_.resize((*ISPD_GridMap_layout).size());
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		this->history_congestion_graph_.ISPD_history_congestion_graph_[i].resize((*ISPD_GridMap_layout)[i].size());
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			this->history_congestion_graph_.ISPD_history_congestion_graph_[i][j].resize((*ISPD_GridMap_layout)[i][j].size());
		}
	}

	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++){
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++){
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++){
				this->history_congestion_graph_.ISPD_history_congestion_graph_[i][j][k] = 1.0;
			}
		}
	}
}

// for rip-up
void SmallRouter::ISPD_reset_2PinNetStatus()
{
	for (auto &perNet : this->rip_up_two_pin_nets_)
	{
		perNet.routed = false;
	}
}

void SmallRouter::ISPD_RipUp2PinNet(int RipUpNet_index)
{
	this->Temp_target_path_list.clear();
	this->TempTargetTable.clear();

	//this->rip_up_two_pin_nets_.at(RipUpNet_index).routed = false;
	vector<int> EraseQueue;
	EraseQueue.push_back(RipUpNet_index);

	for( auto &temp : this->rip_up_two_pin_nets_.at(RipUpNet_index).ReferenceRipUpNet)
	{
		EraseQueue.push_back(temp);
	}

	for(int index = 0 ; index < EraseQueue.size() ; index++)
	{
		this->rip_up_two_pin_nets_.at(EraseQueue.at(index)).rip_up_times += 1;
		this->rip_up_two_pin_nets_.at(EraseQueue.at(index)).routed = false;
		// erase blockage map & net_id map
		for( auto &erase_node : (this->rip_up_two_pin_nets_.at(EraseQueue.at(index)).TwoPinNetData)->RipUp_path )
		{
			int _x = erase_node.x;
			int _y = erase_node.y;
			int _z = erase_node.z;

			this->ISPD_NetID_array[_z][_y][_x] = -1;
			(*ISPD_blockage_array_)[_z][_y][_x] = kNoBlockage;// initial
		}
		(this->rip_up_two_pin_nets_.at(EraseQueue.at(index)).TwoPinNetData)->RipUp_path.clear();

		// add new
		vector <int> &AddQueue = this->rip_up_two_pin_nets_.at(EraseQueue.at(index)).ReferenceRipUpNet;
		for(auto &newbie : AddQueue)
		{
			EraseQueue.push_back(newbie);
		}
		
	}


}

Parser::I_Rect WhichPinShape(const vector<Parser::I_Rect> &PinShape, Node &src)
{
	for (int i = 0; i < PinShape.size(); i++)
	{
		if (src.x >= PinShape[i].LB.first && src.x <= PinShape[i].RT.first)
		{
			if (src.y >= PinShape[i].LB.second && src.y <= PinShape[i].RT.second)
			{
				return PinShape[i];
			}
		}
	}
	printf("Error WhichPinShape Error\n");
	//exit(1);
}

void SmallRouter::ISPD_StartRouting()
{
	cout << endl << "========== Start Routing ==========" << endl << endl;

	//printf("ISPD_StartRouting:: Congestion Map Setting (unmodified)\n");
	//Congestion Map Setting
	//ISPD_set_history_congestion_graph_();

	//printf("ISPD_StartRouting:: Line-end Map Setting (unmodified)\n");
	//Line - end Map Setting

	printf("ISPD_StartRouting:: ISPD Congestion Map Setting\n");
	printf("ISPD_StartRouting:: ISPD NetID Map Setting\n");
	ISPD_set_NetID_array_();

	cout << "2-pin net num:  " << rip_up_two_pin_nets_.size() << endl;
	printf("Congestion Map Estimation Based Routing\n");
	//ISPD_Congestion_Map_Estimation_Based_Routing();
	printf("Congestion Map Estimation Based Routing Execute Successfully\n");

	printf("ISPD_StartRouting:: ISPD NetID Map Setting\n");
	ISPD_reset_NetID_array_(); // reset before every phase routing

	printf("ISPD_StartRouting:: ISPD 2PinNet Status Setting\n");
	ISPD_reset_2PinNetStatus();

//	ISPD_set_blockage_array_(ISPD_origin_blockage_array_);
//	ISPD_set_eol_blockage_array_(ISPD_origin_EOL_Blocage_Map);
	printf("ISPD_StartRouting::Initial Routing \n");
	ISPD_FirstPhaseRouting();
	printf("ISPD_StartRouting::Rip-up & Reroute (unmodified) \n");
	//FirstPhaseRouting();

	printf("ISPD_StartRouting::Free_Subtree_ViaHit_PathHit_array\n");
	ISPD_Free_Subtree_ViaHit_PathHit_array_();
}


void SmallRouter::ISPD_Congestion_Map_Estimation_Based_Routing(){
	int path_no_found_counter = 0;
	int bad_net_counter = 0;
	int success_counter = 0;

	float Total_Time_Of_Maze = 0.0;
	float Total_Time_Of_Other = 0.0;
	float Total_Time_Of_SEG = 0.0;
	ISPD_set_ViaHit_array_();
	ISPD_set_PathHit_array_();
	Net_wirePath.resize(Net_Num);
	map<tuple<int, int, int>, Parser::PathAndEnc> Net_Pin_Corresponding_Refinement;
	int last_rnd_two_pin_net_id = -1;
	int last_two_pin_net_id = 0;
	int minX = INT_MAX, minY = INT_MAX, maxX = -1, maxY = -1;
	for (int i = 0; i < this->rip_up_two_pin_nets_.size(); i++)
	{
		const int two_pin_net_index = i;
		const int two_pin_net_id = this->rip_up_two_pin_nets_[i].net_id;
		TwoPinRipUpNet &ripup_net = rip_up_two_pin_nets_[two_pin_net_index];

		if (ripup_net.netname != "net113")
		{
			continue;
		}

		//cout << ripup_net.netname << endl;

		if (ripup_net.routed == false)
		{
			ripup_net.routed = true;
			// for speed up
			if (last_rnd_two_pin_net_id != two_pin_net_id)
			{
				// change net
				this->CachePinTable.clear();
				this->Cache_pin_path_list.clear();
				this->TempTargetTable.clear();
				this->Temp_target_path_list.clear();
				ISPD_reset_Subtree_ViaHit_PathHit_array_();
				for(int tnet = last_two_pin_net_id; tnet < i ; tnet++){
					rip_up_two_pin_nets_[tnet].clear();
				}
				last_two_pin_net_id = i;
				Net_Pin_Corresponding_Refinement.clear();

				ISPD_SetNetPinRefinement(ripup_net, Net_Pin_Corresponding_Refinement, two_pin_net_id);
				minX = INT_MAX;
				minY = INT_MAX;
				maxX = -1;
				maxY = -1;
			}

			ripup_net.cachePinTable = &this->CachePinTable;
			ripup_net.cache_pin_path_list = &this->Cache_pin_path_list;
			ripup_net.tempTargetTable = &this->TempTargetTable;
			ripup_net.temp_target_path_list = &this->Temp_target_path_list;
			
			TwoPinNetConnection &connection = ripup_net.GetConnection();
			
			if (last_rnd_two_pin_net_id != two_pin_net_id)
			{
				int PinIndex;
				if ((ripup_net.TwoPinNetData)->TwoPinIndex.second != UndefineValue)
				{
					PinIndex = (ripup_net.TwoPinNetData)->TwoPinIndex.second;
				}
				else
				{
					printf("ripup_net.TwoPinNetData->TwoPinIndex.second == UndefineValue\n");
					exit(1);
				}
				if (ripup_net.DesignPinList->at(PinIndex).IsIRoutePath == 9453) // 9453 == IAMIROUTE
				{
					ISPD_Insert_Target_array_(ripup_net.two_pin_net_connection.Target_pin_path_list);
					ISPD_IRouteRevise(ripup_net, PinIndex);
				}
				else if (ripup_net.DesignPinList->at(PinIndex).OutSideMemory)
				{
					printf("Outside Memory\n");
					ISPD_Insert_Target_array_(ripup_net.two_pin_net_connection.Target_pin_path_list);
					//exit(1);
				}
				else
				{
					vector<Pin> Valid_Target;
					FindFirstValidTarget(ripup_net, Net_Pin_Corresponding_Refinement, Valid_Target, PinIndex);
					ISPD_Insert_Target_array_(Valid_Target);
				}

				for (auto &tarnode : ripup_net.two_pin_net_connection.Target_pin_path_list)
				{
					int x, y, z;
					tarnode.GetPinPointPosition(x, y, z);
					//printf("TAR (%d, %d, %d)\n", x, y, z);
					tuple <coor_t,coor_t,int> temp_coor;
					temp_coor = ISPD_real_coor_table.Index2Coor(x,y,z);
					if (get<0>(temp_coor) < minX)
						minX = get<0>(temp_coor);
					if (get<1>(temp_coor) < minY)
						minY = get<1>(temp_coor);
					if (get<0>(temp_coor) > maxX)
						maxX = get<0>(temp_coor);
					if (get<1>(temp_coor) > maxY)
						maxY = get<1>(temp_coor);
				}
			}
			else
			{
				/*or (int i = 0; i < ripup_net.PinToBitwise.size(); i++)
				{
					int x = ripup_net.PinToBitwise.at(i).pin_position.pin_point.x;
					int y = ripup_net.PinToBitwise.at(i).pin_position.pin_point.y;
					int z = ripup_net.PinToBitwise.at(i).pin_position.pin_point.z;
					printf("PinToBitwise: (%d,%d,%d)\n", x, y, z);
				}*/

				ISPD_Insert_Target_array_(ripup_net.PinToBitwise);
			}

			last_rnd_two_pin_net_id = two_pin_net_id;

			bool path_setup;
			vector<Parser::I_Rect> Global_guide = ripup_net.Global_guide;
			if (Global_guide.size() == 0)
			{
				printf("Error:: TwoPinNet(%d) : Global guide size is 0\n", i);
				exit(1);
			}
			const bool both_are_point_pins = (ripup_net.two_pin_net_connection.src_type == 0 &&
											  ripup_net.two_pin_net_connection.tar_type == 0);

			if (false)
			{
				;
			}
			else
			{
				//printf("Setting pins\n");
				Parser::I_Rect SrcEnc;
				Parser::I_Rect TarEnc;
				const int net_rr_times = 1;
				//printf("Src size = %d\n", connection.Source_pin_path_list.size());
				for (auto &srcnode : connection.Source_pin_path_list)
				{
					int x, y, z;
					srcnode.GetPinPointPosition(x, y, z);
					//printf("SRC (%d, %d, %d)\n", x, y, z);

					tuple<coor_t, coor_t, int> temp_coor;
					temp_coor = ISPD_real_coor_table.Index2Coor(x, y, z);
					if (get<0>(temp_coor) < minX)
						minX = get<0>(temp_coor);
					if (get<1>(temp_coor) < minY)
						minY = get<1>(temp_coor);
					if (get<0>(temp_coor) > maxX)
						maxX = get<0>(temp_coor);
					if (get<1>(temp_coor) > maxY)
						maxY = get<1>(temp_coor);
				}
				//printf("Tar size = %d\n", ripup_net.PinToBitwise.size());
				for (auto &tarnode : ripup_net.PinToBitwise)
				{
					int x, y, z;
					tarnode.GetPinPointPosition(x, y, z);
					//printf("TAR (%d, %d, %d)\n", x, y, z);
					tuple<coor_t, coor_t, int> temp_coor;
					temp_coor = ISPD_real_coor_table.Index2Coor(x, y, z);
					if (get<0>(temp_coor) < minX)
						minX = get<0>(temp_coor);
					if (get<1>(temp_coor) < minY)
						minY = get<1>(temp_coor);
					if (get<0>(temp_coor) > maxX)
						maxX = get<0>(temp_coor);
					if (get<1>(temp_coor) > maxY)
						maxY = get<1>(temp_coor);
				}

				//printf("min(%d,%d) max(%d,%d)\n", minX, minY, maxX, maxY);
				int LBfirst = minX;
				int LBsecond = minY;
				int RTfirst = maxX;
				int RTsecond = maxY;

				LBfirst = LBfirst - LUCKYNUMBER * 2000;
				LBsecond = LBsecond - LUCKYNUMBER * 2000;
				RTfirst = RTfirst + LUCKYNUMBER * 2000;
				RTsecond = RTsecond + LUCKYNUMBER * 2000;

				Parser::I_Rect BoundingBox = Parser::I_Rect(LBfirst, LBsecond, RTfirst, RTsecond, -1);
				//printf("bounding (%d,%d) (%d,%d)\n", LBfirst, LBsecond, RTfirst, RTsecond);
				// special case

				//SetBoundingBox(bounding_box, connection, net_rr_times);
				printf("============= Net(%d) TwoPinNet(%d) Success(%d) Short(%d) Failure(%d) Maze Routing Start =============\n", two_pin_net_id, i, success_counter, bad_net_counter, path_no_found_counter);
				Node target_point;

				map<tuple<int, int, int>, Parser::PathAndEnc> SourceRefinmentMap;
				map<Node, Parser::PathAndEnc>
					TargetRefinmentMap;

				int SrcPinIndex;
				if ((ripup_net.TwoPinNetData)->TwoPinIndex.first != UndefineValue)
				{
					SrcPinIndex = (ripup_net.TwoPinNetData)->TwoPinIndex.first;
				}
				else
				{
					printf("ripup_net.TwoPinNetData->TwoPinIndex.first == UndefineValue\n");
					exit(1);
				}
				bool need_refine_src = true;
				bool short_src = false;
				bool isIroute = ripup_net.DesignPinList->at(SrcPinIndex).IsIRoutePath == 9453;
				if (isIroute)
				{
					need_refine_src = false;
				}
				short_src = true;

				clock_t pro_start = clock();
				path_setup =
					LiteBackTrack(
						two_pin_net_index,
						target_point, connection, two_pin_net_id);
				printf("============= Net(%d) TwoPinNet(%d) Success(%d) Short(%d) Failure(%d) Detailed Path  =============\n", two_pin_net_id, i, success_counter, bad_net_counter, path_no_found_counter);
				if (!path_setup)
				{
					//printf("Path Set up error\n");
					path_no_found_counter++;
				}
				else
				{
					
					Push_path_into_CongestionMap(ripup_net.two_pin_net_detailed_path.detail_grid_path, two_pin_net_id);
					ISPD_Insert_Target_array_(ripup_net.PinToBitwise);

					printf("============= Net(%d) TwoPinNet(%d) Success(%d) Short(%d) Failure(%d) End  =============\n", two_pin_net_id, i, success_counter, bad_net_counter, path_no_found_counter);
					success_counter++;
				}
			}

		} // if unrouted
	}
}


// First Phase
// 1. Bounded Maze Routing
// 2. Strict Line-end Rule
void SmallRouter::FirstPhaseRouting()
{
	cout << "Entering First Phase!" << endl;

	InitialRerouteTwoPinNet();
	sort(
			reroute_two_pin_net_.begin(),
			reroute_two_pin_net_.end(), 
			LessWireLength
			);
	stable_sort(
			reroute_two_pin_net_.begin(),
			reroute_two_pin_net_.end(), 
			// NoBend
			MoreStraight
			);
	this->total_rip_up_times = 1;

	// Start the initial routing.
	for(const TwoPinRerouteNet &reroute_net : reroute_two_pin_net_)
	{
		const int two_pin_net_index = reroute_net.index;
		const int two_pin_net_id = reroute_net.two_pin_net_id;
		TwoPinRipUpNet &ripup_net = rip_up_two_pin_nets_[two_pin_net_index];
		const TwoPinNetConnection &connection = ripup_net.GetConnection();
		
		BoundingBox bounding_box;
		bool path_setup = false;
		const bool both_are_point_pins = (
				ripup_net.GetSource().pin_type == kPinPoint && 
				ripup_net.GetTarget().pin_type == kPinPoint);
		if(both_are_point_pins)
		{
			int source_x, source_y, source_z;
			ripup_net.GetSource().GetPinPointPosition(
					source_x, source_y, source_z);
			const Node source(source_x, source_y, source_z);
			
			int target_x, target_y, target_z;
			ripup_net.GetTarget().GetPinPointPosition(
					target_x, target_y, target_z);
			const Node target(target_x, target_y, target_z);
			
			const int net_rr_times = 1;
			SetBoundingBox(bounding_box, connection, net_rr_times);
			/*MazeRouting3D(
				two_pin_net_id, two_pin_net_index,
				source, target,
				bounding_box
				);*/
			path_setup = 
				BackTrack(
						two_pin_net_index,
						source, 
						target);
		}
		else
		{
			
			const int net_rr_times = 1;
			SetBoundingBox(bounding_box, connection, net_rr_times);
			Node target_point;
			/*MazeRouting3D(
					two_pin_net_id, two_pin_net_index,
					connection,
					bounding_box,
					target_point
				);*/

			/*path_setup = 
				BackTrack(
						two_pin_net_index, 
						target_point);*/
		}
		
		
		if(path_setup)
		{
			// if(this->kLineEndModeRouting_ || this->kViaEncCostMode_)
			FindPathTurns(two_pin_net_index);
			if(this->kLineEndModeRouting_)
			{
				const vector<TurnNode> &turn_nodes = 
					ripup_net.GetPathTurnNodes();
				line_end_graph_.InsertBlockage(turn_nodes);
			}
			SetTwoPinNetPathToTrack(two_pin_net_index, two_pin_net_id);
		}  
	}

	// Update line-end map info.
	if(this->kLineEndModeRouting_)
	{
		line_end_graph_.CleanBlockageWholeGraph();
		for(TwoPinRipUpNet &ripup_net : rip_up_two_pin_nets_)
		{
			const vector<TurnNode> &turn_nodes = 
				ripup_net.GetPathTurnNodes();
			line_end_graph_.InsertBlockage(turn_nodes);
		}
	}

	// Report the routing result.
	int short_net_num = 0;
	int open_net_num = 0;
	int overlap_violation_num = 0;
	AnalyzeTheResult(
			short_net_num, 
			open_net_num, 
			overlap_violation_num
		);
	int line_end_violation_num = 0;
	if(this->kLineEndModeRouting_)
	{
		line_end_violation_num = 
			line_end_graph_.AnalyzeLineEndViolation(); 
	}

	// Save the solution.
	best_solution_.SetSolution(
			short_net_num, 
			open_net_num, 
			overlap_violation_num, 
			line_end_violation_num
		);
	cout << "Completing 1st Phase!" << endl << endl;
}

void SmallRouter::PathConstruction(list<Node> &detail_grid_path, int two_pin_net_index, vector<Parser::wire_path> &wire_list,
								   vector<Parser::I_Rect> &Patch, vector<Parser::wire_path> &This_wire_list)
{

	wire_path path;
	path.clear();

	//printf("Patch\n");
	for (int i = 0; i < Patch.size(); i++){
		wire_path patch;
		patch.path_type = 3;
		patch.PatchLocate = make_pair(Patch[i].LB.first, Patch[i].LB.second);
		patch.Patch.Layer = Patch[i].Layer;
		patch.Patch.LB = make_pair(0, 0);
		patch.Patch.RT = make_pair(Patch[i].RT.first - Patch[i].LB.first, Patch[i].RT.second - Patch[i].LB.second);
		wire_list.push_back(patch);
	}

	//printf("Path\n");
	for (auto p = detail_grid_path.begin(); p != detail_grid_path.end(); p++){
		if (path.Src_Pin.first == UndefineValue){

			tuple<coor_t, coor_t, int> temp_coor;
			temp_coor = ISPD_real_coor_table.Index2Coor(p->x, p->y, p->z);
			path.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
			path.Layer = p->z;
			if( path.Layer > 100){
				printf("Layer Error (%d)\n",path.Layer);
				exit(1);
			}
		}
		else if (path.Tar_Pin.first == UndefineValue){	
			tuple<coor_t, coor_t, int> temp_coor;
			temp_coor = ISPD_real_coor_table.Index2Coor(p->x, p->y, p->z);
			if (p->z != path.Layer)
			{ // only src pin + via
				//printf("3\n");
				path.path_type = 2;
				if (path.Layer < p->z)
				{
					if (this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.first == -1 && this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.second == -1)
					{
						path.ViaName = this->ISPD_via_type_list.at(path.Layer).at(0).Name;
					}
					else
					{
						if (this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.first >= this->ISPD_via_type_list.size() || this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.second >= this->ISPD_via_type_list.at(this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.first).size())
						{
							//printf("this->ISPD_Viatype_array_[path.Layer][p->y][p->x].RaisingVia.first :: %d\n", this->ISPD_Viatype_array_[path.Layer][p->y][p->x].RaisingVia.first);
							path.ViaName = this->ISPD_via_type_list.at(path.Layer).at(0).Name;
						}
						else
							path.ViaName = this->ISPD_via_type_list.at(this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.first).at(this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.second).Name;
					}
				}
				else if (path.Layer > p->z)
				{
					if (this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.first == -1 &&
						this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.second == -1)
					{
						path.ViaName = this->ISPD_via_type_list.at(get<2>(temp_coor)).at(0).Name;
					}
					else
					{
						if (this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.first >= this->ISPD_via_type_list.size() || this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.second >= this->ISPD_via_type_list.at(this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.first).size())
						{
							//printf("this->ISPD_Viatype_array_[path.Layer][p->y][p->x].FallingVia.first :: %d\n", this->ISPD_Viatype_array_[path.Layer][p->y][p->x].FallingVia.first);
							path.ViaName = this->ISPD_via_type_list.at(get<2>(temp_coor)).at(0).Name;
						}
						else
							path.ViaName = this->ISPD_via_type_list.at(this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.first).at(this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.second).Name;
					}
				}
				wire_list.push_back(path);
				if( path.Layer > 100){
				printf("Layer Error (%d)\n",path.Layer);
				exit(1);
				}
				This_wire_list.push_back(path);
				path.clear();
				path.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
				path.Layer = p->z;
				if( path.Layer > 100){
				printf("Layer Error (%d)\n",path.Layer);
				exit(1);
				}
				continue;
			}
			else if (get<0>(temp_coor) == path.Src_Pin.first)
			{
				path.dir = false; //vertical
				path.Tar_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
			}
			else if (get<1>(temp_coor) == path.Src_Pin.second)
			{
				path.dir = true; //horizontal
				path.Tar_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
			}

		}
		else{
			tuple<coor_t, coor_t, int> temp_coor;
			temp_coor = ISPD_real_coor_table.Index2Coor(p->x, p->y, p->z);
			if (p->z != path.Layer)
			{ // only src pin + via
				path.path_type = 1;

				if (path.Layer < p->z)
				{
					//printf("8888888888\n");
					//printf("RaisingVia Via type (%d,%d)", this->ISPD_Viatype_array_[path.Layer][p->y][p->x].RaisingVia.first, this->ISPD_Viatype_array_[path.Layer][p->y][p->x].RaisingVia.second);
					if (this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.first == -1 && this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.second == -1){
						path.ViaName = this->ISPD_via_type_list.at(path.Layer).at(0).Name;
					}
					else{
						if (this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.first >= this->ISPD_via_type_list.size()
							|| this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.second >= this->ISPD_via_type_list.at(this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.first).size()){
							//printf("this->ISPD_Viatype_array_[path.Layer][p->y][p->x].RaisingVia.first :: %d\n", this->ISPD_Viatype_array_[path.Layer][p->y][p->x].RaisingVia.first);
							path.ViaName = this->ISPD_via_type_list.at(path.Layer).at(0).Name;
						}
						else
							path.ViaName = this->ISPD_via_type_list.at(this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.first).at(this->ISPD_Viatype_array_[p->z][p->y][p->x].FallingVia.second).Name;

					}
				}
				else if (path.Layer > p->z)
				{
					if (this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.first == -1 &&
						this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.second == -1){
						path.ViaName = this->ISPD_via_type_list.at(get<2>(temp_coor)).at(0).Name;
					}
					else{
						if (this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.first >= this->ISPD_via_type_list.size()
							|| this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.second >= this->ISPD_via_type_list.at(this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.first).size()){
							//printf("this->ISPD_Viatype_array_[path.Layer][p->y][p->x].FallingVia.first :: %d\n", this->ISPD_Viatype_array_[path.Layer][p->y][p->x].FallingVia.first);
							path.ViaName = this->ISPD_via_type_list.at(get<2>(temp_coor)).at(0).Name;
						}
						else
							path.ViaName = this->ISPD_via_type_list.at(this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.first).at(this->ISPD_Viatype_array_.at(p->z).at(p->y).at(p->x).RaisingVia.second).Name;
					}
				}
				wire_list.push_back(path);
				if( path.Layer > 100){
				printf("Layer Error (%d)\n",path.Layer);
				exit(1);
				}
				This_wire_list.push_back(path);
				path.clear();
				path.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
				path.Layer = p->z;
				if( path.Layer > 100){
				printf("Layer Error (%d)\n",path.Layer);
				exit(1);
				}
				//printf("Src(%d,%d) Tar(%d,%d)\n", path.Src_Pin.first, path.Src_Pin.second, path.Tar_Pin.first, path.Tar_Pin.second);
				continue;
			}

			if (path.dir == false && get<0>(temp_coor) == path.Tar_Pin.first)
			{ // make path longer
				//printf("8\n");
				path.Tar_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
			}
			else if (path.dir == true && get<1>(temp_coor) == path.Tar_Pin.second)
			{
				//printf("9\n");
				path.Tar_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
			}
			else if (path.dir == false && get<0>(temp_coor) != path.Tar_Pin.first)
			{ // turn node
				//printf("10\n");
				int x = path.Tar_Pin.first; int y = path.Tar_Pin.second;
				path.path_type = 0;
				wire_list.push_back(path);
				if( path.Layer > 100){
				printf("Layer Error (%d)\n",path.Layer);
				exit(1);
				}
				This_wire_list.push_back(path);
				path.clear();
				//printf("1");
				path.dir = true;
				path.Src_Pin = make_pair(x,y);
				path.Tar_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
				path.Layer = p->z;
				if( path.Layer > 100){
				printf("Layer Error (%d)\n",path.Layer);
				exit(1);
				}
			}
			else if (path.dir == true && get<1>(temp_coor) != path.Tar_Pin.second)
			{ // turn node
				//printf("11\n");
				int x = path.Tar_Pin.first; int y = path.Tar_Pin.second;
				path.path_type = 0;
				wire_list.push_back(path);
				if( path.Layer > 100){
				printf("Layer Error (%d)\n",path.Layer);
				exit(1);
				}
				This_wire_list.push_back(path);
				path.clear();
				//printf("2");
				path.dir = false;
				path.Src_Pin = make_pair(x, y);
				path.Tar_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
				path.Layer = p->z;
				if( path.Layer > 100){
				printf("Layer Error (%d)\n",path.Layer);
				exit(1);
				}
			}
		}
		//printf("Src(%d,%d) Tar(%d,%d) Layer(%d) Via(%s)\n", path.Src_Pin.first, path.Src_Pin.second, path.Tar_Pin.first, path.Tar_Pin.second, path.Layer, path.ViaName.c_str());
	}
	if (path.Src_Pin.first != UndefineValue  && path.Tar_Pin.first != UndefineValue){
		path.path_type = 0;
		wire_list.push_back(path);
		This_wire_list.push_back(path);
		path.clear();
	}
	/*else if (path.Src_Pin.first != UndefineValue){
		path.path_type = 2;
		path.ViaName = this->ISPD_via_type_list[min(wire_list[wire_list.size()-1].Layer, path.Layer)][0].Name;
		wire_list.push_back(path);
		path.clear();
	}*/
}

void SmallRouter::ISPD_HitRectPatch(Parser::I_Rect &Enclosure, vector <Parser::I_Rect> &PinShape,vector <Parser::I_Rect> &AdditionPatch,int net_id){

	/*bool in_shape = false;
	int center_x = (Enclosure.RT.first - Enclosure.LB.first) / 2;
	int center_y = (Enclosure.RT.second - Enclosure.LB.second) / 2;
	for (int i = 0; i < PinShape.size(); i++){
		if (center_x >= PinShape[i].LB.first && center_x <= PinShape[i].RT.first){
			if (center_y >= PinShape[i].LB.second && center_y <= PinShape[i].RT.second){
				in_shape = true;
				break;
			}
		}
	}

	if (in_shape)*/
	this->SpaceEvaluationLayout[0].OverlappedPatchEstimation(Enclosure, PinShape, AdditionPatch,net_id);

}


void SmallRouter::ISPD_SetNetPinRefinement(
	TwoPinRipUpNet &ripup_net,
	map<tuple<int, int, int>, Parser::PathAndEnc> &Net_Pin_Corresponding_Refinement,
	int net_id)
{
	//printf("ISPD_SetNetPinRefinement\n");
	if (ripup_net.DesignPinList == NULL)
	{
		cout << "886\n";
		exit(0);
	}
	//printf("ripup_net.DesignPinList size() : %d\n", ripup_net.DesignPinList->size());
	for (int i = 0; i < ripup_net.DesignPinList->size(); i++)
	{
		//printf("Outsidemem\n");
		if (ripup_net.DesignPinList->at(i).OutSideMemory){
			printf("Really Outsidemem\n");
			for (int pseudo = 0; pseudo < ripup_net.DesignPinList->at(i).LayoutNodeTRUE.size(); pseudo++)
			{
				int x = ripup_net.DesignPinList->at(i).LayoutNodeTRUE[pseudo].x;
				int y = ripup_net.DesignPinList->at(i).LayoutNodeTRUE[pseudo].y;
				int z = ripup_net.DesignPinList->at(i).LayoutNodeTRUE[pseudo].z;

				tuple<coor_t, coor_t, int> temp_coor;
				temp_coor = ISPD_real_coor_table.Index2Coor(x,y,z);

				int PinIndex = i;
				//printf("Pseudo Node(%d,%d,%d)\n", x, y, z);
				Node pesudo_pin(x, y, z);
				PathAndEnc PAE;
				bool valid_node = true;
				if (valid_node)
				{
					int center_x = (ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).RT.first + ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).LB.first)/2;
					int center_y = (ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).RT.second + ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).LB.second)/2;

					tuple<int, int, int> refine_node = make_tuple(x, y, z);
					if (get<0>(temp_coor) >= ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).LB.first && get<0>(temp_coor) <= ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).RT.first)
					{
						wire_path mem_path;
						mem_path.path_type = 0;
						mem_path.Layer = z;
						mem_path.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
						mem_path.Tar_Pin = make_pair(get<0>(temp_coor), center_y);

						PAE.path.push_back(mem_path);
					}
					else if (get<1>(temp_coor) >= ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).LB.second && get<1>(temp_coor) <= ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).RT.second)
					{
						wire_path mem_path;
						mem_path.path_type = 0;
						mem_path.Layer = z;
						mem_path.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
						mem_path.Tar_Pin = make_pair(center_x, get<1>(temp_coor));
						PAE.path.push_back(mem_path);
					}
					else
					{
						printf("MemRefine Error\n");
						exit(1);
					}
					Net_Pin_Corresponding_Refinement.emplace(refine_node, PAE);
				}
			}
			continue;
		}
		//printf("InsideGridNode\n");
		if (ripup_net.DesignPinList->at(i).InsideGridNode)
		{
			printf("Really InsideGridNode\n");
			bool free = false;
			int PinIndex = i;

			//printf("InsideGridNode:: LayoutNode size: %d\n", ripup_net.DesignPinList->at(i).LayoutNode.size());
			for (int pseudo = 0; pseudo < ripup_net.DesignPinList->at(i).LayoutNode.size(); pseudo++)
			{
				int s_x = ripup_net.DesignPinList->at(i).LayoutNode[pseudo].x;
				int s_y = ripup_net.DesignPinList->at(i).LayoutNode[pseudo].y;
				int s_z = ripup_net.DesignPinList->at(i).LayoutNode[pseudo].z;

				tuple<coor_t, coor_t, int> temp_coor;
				temp_coor = ISPD_real_coor_table.Index2Coor(s_x, s_y, s_z);
				vector<wire_path> need_refine_wire;
				int x = get<0>(temp_coor);
				int y = get<1>(temp_coor);
				//printf("Location Determin\n");
				if (x < ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).LB.first &&
					y < ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).LB.second)
				{
					// LB
					need_refine_wire = ripup_net.DesignPinList->at(PinIndex).IsolatedPinMetalSegmentLB.second;
				}
				else if (x < ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).LB.first &&
						 y > ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).RT.second)
				{
					// LT
					need_refine_wire = ripup_net.DesignPinList->at(PinIndex).IsolatedPinMetalSegmentLT.first;
				}
				else if (x > ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).RT.first &&
						 y > ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).RT.second)
				{
					// RT
					need_refine_wire = ripup_net.DesignPinList->at(PinIndex).IsolatedPinMetalSegmentRT.first;
				}
				else if (x > ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).RT.first &&
						 y < ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).LB.second)
				{
					// RB
					need_refine_wire = ripup_net.DesignPinList->at(PinIndex).IsolatedPinMetalSegmentRB.second;
				}
				if ((*ISPD_blockage_array_)[s_z][s_y][s_x] == NO_BLOCKAGE)
				{
					//printf("PAE (%d,%d,%d)\n",s_x,s_y,s_z);
					//printf("path size(%d)\n", need_refine_wire.size());
					//ISPD_TargetMap_modify_list.push_back(make_tuple(s_x, s_y, s_z));
					//this->ISPD_Target_Map[s_z][s_y][s_x] = true;
					free = true;
					PathAndEnc PAE;
					Parser::wire_path path;
					path.path_type = 2;
					path.Layer = s_z - 1;
					path.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
					path.ViaName = ISPD_via_type_list[s_z - 1][0].Name;
					for (int p1 = 0; p1 < need_refine_wire.size(); p1++)
					{
						//printf(" need_refine_wire Layer : %d\n",need_refine_wire[p1].Layer);
						PAE.path.push_back(need_refine_wire[p1]);
					}

					PAE.path.push_back(path);
					PAE.via_type = make_pair(s_z - 1, 0);
					tuple<int, int, int> node = make_tuple(s_x, s_y, s_z);
					Net_Pin_Corresponding_Refinement.emplace(node, PAE);
				}
				}

				if (!free)
				{
					//printf("Unfree\n");
					int s_x = ripup_net.DesignPinList->at(PinIndex).LayoutNode.at(0).x;
					int s_y = ripup_net.DesignPinList->at(PinIndex).LayoutNode.at(0).y;
					int s_z = ripup_net.DesignPinList->at(PinIndex).LayoutNode.at(0).z;

					tuple<coor_t, coor_t, int> temp_coor;
					temp_coor = ISPD_real_coor_table.Index2Coor(s_x, s_y, s_z);
					vector<wire_path> need_refine_wire;
					int x = get<0>(temp_coor);
					int y = get<1>(temp_coor);
					if (x < ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).LB.first &&
						y < ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).LB.second)
					{
						// LB
						need_refine_wire = ripup_net.DesignPinList->at(PinIndex).IsolatedPinMetalSegmentLB.second;
					}
					else if (x < ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).LB.first &&
							 y > ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).RT.second)
					{
						// LT
						need_refine_wire = ripup_net.DesignPinList->at(PinIndex).IsolatedPinMetalSegmentLT.first;
					}
					else if (x > ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).RT.first &&
							 y > ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).RT.second)
					{
						// RT
						need_refine_wire = ripup_net.DesignPinList->at(PinIndex).IsolatedPinMetalSegmentRT.first;
					}
					else if (x > ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).RT.first &&
							 y < ripup_net.DesignPinList->at(PinIndex).IRect_list.at(0).LB.second)
					{
						// RB
						need_refine_wire = ripup_net.DesignPinList->at(PinIndex).IsolatedPinMetalSegmentRB.second;
					}
					//ISPD_TargetMap_modify_list.push_back(make_tuple(s_x, s_y, s_z));
					//this->ISPD_Target_Map[s_z][s_y][s_x] = true;
					PathAndEnc PAE;
					Parser::wire_path path;
					path.path_type = 2;
					path.Layer = s_z - 1;
					path.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
					path.ViaName = ISPD_via_type_list[s_z - 1][0].Name;
					PAE.path.push_back(path);
					for (int p1 = 0; p1 < need_refine_wire.size(); p1++)
						PAE.path.push_back(need_refine_wire[i]);
					PAE.via_type = make_pair(s_z - 1, 0);
					tuple<int, int, int> node = make_tuple(s_x, s_y, s_z);
					Net_Pin_Corresponding_Refinement.emplace(node, PAE);
				}

				continue;
		}
		//printf("LayoutNodeTRUE\n");
		//printf("LayoutNodeTRUE(%d) size() : %d\n", i, ripup_net.DesignPinList->at(i).LayoutNodeTRUE.size());
		// real pin
		for (int real = 0; real < ripup_net.DesignPinList->at(i).LayoutNodeTRUE.size(); real++)
		{

			int x = ripup_net.DesignPinList->at(i).LayoutNodeTRUE[real].x;
			int y = ripup_net.DesignPinList->at(i).LayoutNodeTRUE[real].y;
			int z = ripup_net.DesignPinList->at(i).LayoutNodeTRUE[real].z;
			auto already = Net_Pin_Corresponding_Refinement.find(make_tuple(x, y, z));
			if (already != Net_Pin_Corresponding_Refinement.end())
			{
				continue;
			}
			//printf("Real Node(%d) (%d,%d,%d)\n", real,x,y,z , );
			Node real_pin(x, y, z);
			Node real_next_pin(x, y, z - 1);
			tuple<coor_t, coor_t, int> temp_coor;
			temp_coor = ISPD_real_coor_table.Index2Coor(x, y, z);
			//printf("Real Node(%d) (%d,%d,%d) - (%d,%d,%d)\n", real, x, y, z, get<0>(temp_coor), get<1>(temp_coor), z);
			
			Node real_pin_position(get<0>(temp_coor), get<1>(temp_coor) , z);
			// true source
			Parser::I_Rect shape = WhichPinShape(ripup_net.DesignPinList->at(i).IRect_list, real_pin_position);
			bool SourceOK = false;
			int best_via = BestVia(shape, real_pin);
			//printf("ViaEnclosureChecking\n");
			if (ViaEnclosureChecking(real_pin, real_next_pin, make_pair(z - 1, best_via), net_id))
			{
				PathAndEnc PAE;
				Parser::wire_path path;
				path.path_type = 2;
				path.Layer = z - 1;
				path.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
				path.ViaName = ISPD_via_type_list[z - 1][best_via].Name;
				PAE.path.push_back(path);
				PAE.Enclosure.Layer = z - 1;
				PAE.Enclosure.LB = make_pair(ISPD_via_type_list[z - 1][best_via].BotLayerIRect.LB.first + get<0>(temp_coor),
											 ISPD_via_type_list[z - 1][best_via].BotLayerIRect.LB.second + get<1>(temp_coor));

				PAE.Enclosure.RT = make_pair(ISPD_via_type_list[z - 1][best_via].BotLayerIRect.RT.first + get<0>(temp_coor),
											 ISPD_via_type_list[z - 1][best_via].BotLayerIRect.RT.second + get<1>(temp_coor));
				PAE.via_type = make_pair(z - 1, best_via);
				// emplace to map
				tuple<int, int, int>
					refine_node = make_tuple(x, y, z);
				//printf("(%d,%d)set via type (%d,%d)\n", x, y, z - 1, best_via);
				/*for(int P = 0; P < PAE.path.size();P++){
					printf("Real Path Layer(%d)\n",PAE.path[P].Layer);
				}*/
				Net_Pin_Corresponding_Refinement.emplace(refine_node, PAE);
				SourceOK = true;
			}
			//printf("SourceOK\n");
			if (!SourceOK)
			{

				for (int via_type = 0; via_type < ISPD_via_type_list[z - 1].size(); via_type++)
				{
					if (ViaEnclosureChecking(real_pin, real_next_pin, make_pair(z - 1, via_type), net_id))
					{
						PathAndEnc PAE;
						Parser::wire_path path;
						path.path_type = 2;
						path.Layer = z - 1;
						path.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
						path.ViaName = ISPD_via_type_list[z - 1][via_type].Name;
						PAE.path.push_back(path);
						PAE.Enclosure.Layer = z - 1;
						PAE.Enclosure.LB = make_pair(ISPD_via_type_list[z - 1][via_type].BotLayerIRect.LB.first + get<0>(temp_coor),
													 ISPD_via_type_list[z - 1][via_type].BotLayerIRect.LB.second + get<1>(temp_coor));

						PAE.Enclosure.RT = make_pair(ISPD_via_type_list[z - 1][via_type].BotLayerIRect.RT.first + get<0>(temp_coor),
													 ISPD_via_type_list[z - 1][via_type].BotLayerIRect.RT.second + get<1>(temp_coor));
						PAE.via_type = make_pair(z - 1, via_type);
						// emplace to map
						tuple<int, int, int> refine_node = make_tuple(x, y, z);
						Net_Pin_Corresponding_Refinement.emplace(refine_node, PAE);
						//printf("(%d,%d)set via type (%d,%d)\n", x,y,z - 1, via_type);
						/*for(int P = 0; P < PAE.path.size();P++){
							printf("Real Path Layer(%d)\n",PAE.path[P].Layer);
						}*/
						break;
					}
				}
			}
		}
		//printf("LayoutNode(%d) size() : %d\n", i,ripup_net.DesignPinList->at(i).LayoutNode.size());
		// pseudo pin
		for (int pseudo = 0; pseudo < ripup_net.DesignPinList->at(i).LayoutNode.size(); pseudo++)
		{
			int x = ripup_net.DesignPinList->at(i).LayoutNode[pseudo].x;
			int y = ripup_net.DesignPinList->at(i).LayoutNode[pseudo].y;
			int z = ripup_net.DesignPinList->at(i).LayoutNode[pseudo].z;

			auto already = Net_Pin_Corresponding_Refinement.find(make_tuple(x,y,z));
			if (already != Net_Pin_Corresponding_Refinement.end())
			{
				continue;
			}
			//printf("Pseudo Node(%d) (%d,%d,%d)\n", pseudo , x, y, z);
			Node pesudo_pin(x, y, z);
			PathAndEnc PAE;
			
			bool valid_node = OffGridViaPrediction(pesudo_pin, ripup_net.DesignPinList->at(i).IRect_list, PAE.path, PAE.Enclosure, PAE.via_type, net_id);
			if (valid_node)
			{
				tuple<int, int, int> refine_node = make_tuple(x, y, z);
				Net_Pin_Corresponding_Refinement.emplace(refine_node, PAE);
				/*for(int P = 0; P < PAE.path.size();P++){
					printf("Pseudo Path Layer(%d)\n",PAE.path[P].Layer);
				}*/
			}
			//printf("end\n");
		}
	}

	/*for (auto iter = Net_Pin_Corresponding_Refinement.begin(); iter != Net_Pin_Corresponding_Refinement.end(); iter++)
	{
		printf("x,y,z(%d,%d,%d) via type(%d,%d)\n", get<0>(iter->first), get<1>(iter->first), get<2>(iter->first), (*iter).second.via_type.first, (*iter).second.via_type.second);
		for(int P = 0; P < iter->second.path.size();P++){
					printf("Pseudo Path Layer(%d)\n",iter->second.path[P].Layer);
		}
	}*/
	printf("ISPD_SetNetPinRefinement Completed\n");
}

void SmallRouter::FindFirstValidTarget(TwoPinRipUpNet &ripup_net,
						  map<tuple<int, int, int>, Parser::PathAndEnc> &Net_Pin_Corresponding_Refinement,
						  vector<Pin> &Valid_Target,
						  int Pin_index)
{
	//printf("FindFirstValidTarget\n");
	for (int i = 0; i < ripup_net.two_pin_net_connection.Target_pin_path_list.size(); i++)
	{
		int s_x = ripup_net.two_pin_net_connection.Target_pin_path_list[i].pin_position.pin_point.x;
		int s_y = ripup_net.two_pin_net_connection.Target_pin_path_list[i].pin_position.pin_point.y;
		int s_z = ripup_net.two_pin_net_connection.Target_pin_path_list[i].pin_position.pin_point.z;
		//printf("Target_pin_path_list (%d,%d,%d)\n",s_x,s_y,s_z);
		tuple<int, int, int> target_node = make_tuple(s_x, s_y, s_z);
		auto find_target = Net_Pin_Corresponding_Refinement.find(target_node);

		if (find_target != Net_Pin_Corresponding_Refinement.end())
		{
			Pin pin;
			pin.SetPinPoint(kPinPoint, s_x, s_y, s_z);
			Valid_Target.push_back(pin);
		}
	}

	//printf("FindFirstValidTarget real size(%d), pseudo size(%d)\n", ripup_net.DesignPinList->at(Pin_index).LayoutNodeTRUE.size(),
	//	   ripup_net.DesignPinList->at(Pin_index).LayoutNode.size());

	if (Valid_Target.size() == 0){
		if (ripup_net.DesignPinList->at(Pin_index).LayoutNodeTRUE.size() > 0){
			int x = ripup_net.DesignPinList->at(Pin_index).LayoutNodeTRUE[0].x;
			int y = ripup_net.DesignPinList->at(Pin_index).LayoutNodeTRUE[0].y;
			int z = ripup_net.DesignPinList->at(Pin_index).LayoutNodeTRUE[0].z;

			Node real_pin(x, y, z);
			Node real_next_pin(x, y, z - 1);
			tuple<coor_t, coor_t, int> temp_coor;
			temp_coor = ISPD_real_coor_table.Index2Coor(x, y, z);
			Node real_pin_position(get<0>(temp_coor), get<1>(temp_coor), z);
			// true source
			Parser::I_Rect shape = WhichPinShape(ripup_net.DesignPinList->at(Pin_index).IRect_list, real_pin_position);
			int best_via = BestVia(shape, real_pin);
			PathAndEnc PAE;
			Parser::wire_path path;
			path.path_type = 2;
			path.Layer = z - 1;
			path.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
			path.ViaName = ISPD_via_type_list[z - 1][best_via].Name;
			PAE.path.push_back(path);
			PAE.Enclosure.Layer = z - 1;
			PAE.Enclosure.LB = make_pair(ISPD_via_type_list[z - 1][best_via].BotLayerIRect.LB.first + get<0>(temp_coor),
										 ISPD_via_type_list[z - 1][best_via].BotLayerIRect.LB.second + get<1>(temp_coor));

			PAE.Enclosure.RT = make_pair(ISPD_via_type_list[z - 1][best_via].BotLayerIRect.RT.first + get<0>(temp_coor),
										 ISPD_via_type_list[z - 1][best_via].BotLayerIRect.RT.second + get<1>(temp_coor));
			PAE.via_type = make_pair(z - 1, best_via);
			// emplace to map
			tuple<int, int, int> refine_node = make_tuple(x, y, z);
			Net_Pin_Corresponding_Refinement.emplace(refine_node, PAE);

			Pin pin;
			pin.SetPinPoint(kPinPoint, x, y, z);
			Valid_Target.push_back(pin);
		}
		else if (ripup_net.DesignPinList->at(Pin_index).LayoutNode.size() > 0){
			for (int pseudo = 0; pseudo < ripup_net.DesignPinList->at(Pin_index).LayoutNode.size(); pseudo++)
			{
				int x = ripup_net.DesignPinList->at(Pin_index).LayoutNode[pseudo].x;
				int y = ripup_net.DesignPinList->at(Pin_index).LayoutNode[pseudo].y;
				int z = ripup_net.DesignPinList->at(Pin_index).LayoutNode[pseudo].z;

				//printf("Pseudo Node(%d,%d,%d)\n", x, y, z);
				Node pesudo_pin(x, y, z);
				PathAndEnc PAE;
				bool valid_node = OffGridViaPrediction_Allow_Short(pesudo_pin, ripup_net.DesignPinList->at(Pin_index).IRect_list, PAE.path, PAE.Enclosure, PAE.via_type, ripup_net.net_id);
				if (valid_node)
				{
					tuple<int, int, int> refine_node = make_tuple(x, y, z);
					Net_Pin_Corresponding_Refinement.emplace(refine_node, PAE);

					Pin pin;
					pin.SetPinPoint(kPinPoint, x, y, z);
					Valid_Target.push_back(pin);
					break;
				}
			}
		}
		else{
			printf("Both real & pseudo are empty\n");
			exit(1);
		}
	}
	//printf("FindFirstValidTarget End\n");
}

void SmallRouter::ISPD_IRouteRevise(TwoPinRipUpNet &ripup_net, int &PinIndex)
{
	
	// revise iroute metal on layout
	ripup_net.DesignPinList->at(PinIndex).SynthesisSegList.clear();
	vector<Parser::RipUpNode> tempSegment;
	bool firstpoint = true;

	for (auto &node : ripup_net.DesignPinList->at(PinIndex).LayoutNodeOriginal)
	{
		if (!this->ISPD_PathHit_array_[node.z - 1][node.y][node.x] || firstpoint)
		{
			// add
			//printf("IRoute Seg Add(%d, %d, %d)\n", node.x, node.y, node.z - 1);
			tempSegment.push_back(Parser::RipUpNode(node.x, node.y, node.z));
			firstpoint = false;
		}
		else if (this->ISPD_PathHit_array_[node.z - 1][node.y][node.x] && tempSegment.size() != 0)
		{
			// cut & push
			//printf("IRoute Seg Add(%d, %d, %d)\n", node.x, node.y, node.z - 1);
			tempSegment.push_back(Parser::RipUpNode(node.x, node.y, node.z));
			ripup_net.DesignPinList->at(PinIndex).SynthesisSegList.push_back(tempSegment);
			tempSegment.clear();
			//printf("Push & clear\n");
			//printf("IRoute Seg Add(%d, %d, %d)\n", node.x, node.y, node.z - 1);
			tempSegment.push_back(Parser::RipUpNode(node.x, node.y, node.z));
		}
	}

	if (tempSegment.size() != 0)
	{
		// last segment
		ripup_net.DesignPinList->at(PinIndex).SynthesisSegList.push_back(tempSegment);
		tempSegment.clear();
		//printf("last push %d\n", PinIndex);
	}
}

void SmallRouter::ISPD_Net_Reset()
{
	for (int i = 0; i < (*ISPD_GridMap_layout).size(); i++)
	{
		for (int j = 0; j < (*ISPD_GridMap_layout)[i].size(); j++)
		{
			for (int k = 0; k < (*ISPD_GridMap_layout)[i][j].size(); k++)
			{
				ISPD_PathHit_array_[i][j][k] = false;
				ISPD_ViaHit_array_[i][j][k] = false;
				this->ISPD_Subtree_Map[i][j][k] = false;
				this->ISPD_Target_Map[i][j][k] = false;
			}
		}
	}
}

void SmallRouter::ISPD_FirstPhaseRouting(){

	int path_no_found_counter = 0;
	int bad_net_counter = 0;
	int success_counter = 0;

	double Normal_Maze_Time = 0.0;
	double Short_Maze_Time = 0.0;
	int Target_In_Box = 0;
	int SourceEmpty = 0;
	int PathNotFound = 0;

	float Total_Time_Of_Maze = 0.0;
	float Total_Time_Of_Other = 0.0;
	float Total_Time_Of_SEG = 0.0;
	ISPD_set_ViaHit_array_();
	ISPD_set_PathHit_array_();
	ISPD_set_BitViaType_array_();
#ifdef NCTU_GR_HEAP
	ISPD_set_Fib_Node_Location_array_();
#endif
	int last_two_pin_net_id = 0;
	Net_wirePath.resize(Net_Num);
	map<tuple<int, int, int>, Parser::PathAndEnc> Net_Pin_Corresponding_Refinement;
	int last_rnd_two_pin_net_id = -1;
	int minX = INT_MAX, minY = INT_MAX, maxX = -1, maxY = -1;
	for (int i = 0; i < this->rip_up_two_pin_nets_.size(); i++){
		const int two_pin_net_index = i;
		const int two_pin_net_id = this->rip_up_two_pin_nets_[i].net_id;
		TwoPinRipUpNet &ripup_net = rip_up_two_pin_nets_[two_pin_net_index];

		/*if (ripup_net.netname != "net10347")
		{
			continue;
		}*/
		/*if(two_pin_net_id != 3044){
			continue;
		}*/

		cout << ripup_net.netname << endl;
		if(ripup_net.DesignPinList == NULL)
			continue;
		
		if(ripup_net.routed == false)
		{
			
			ripup_net.routed = true;
			// for speed up
			if(last_rnd_two_pin_net_id != two_pin_net_id)
			{
				clock_t net_start = clock();
				// change net
				this->CachePinTable.clear();
				this->Cache_pin_path_list.clear();
				this->TempTargetTable.clear();
				this->Temp_target_path_list.clear();
				/*for(int tnet = last_two_pin_net_id; tnet < i ; tnet++){
					rip_up_two_pin_nets_[tnet].clear();
				}
				last_two_pin_net_id = i;*/

				ISPD_reset_Subtree_ViaHit_PathHit_array_();
				Net_Pin_Corresponding_Refinement.clear();
				ISPD_SetNetPinRefinement(ripup_net, Net_Pin_Corresponding_Refinement, two_pin_net_id);
				minX = INT_MAX;
				minY = INT_MAX;
				maxX = -1;
				maxY = -1;
				clock_t net_end = clock();
				double net_duration = (double)(net_end - net_start) / CLOCKS_PER_SEC;
				printf("Net Init Time Cost : %.2lf sec\n", net_duration);
			}
			
			ripup_net.cachePinTable = &this->CachePinTable;
			ripup_net.cache_pin_path_list = &this->Cache_pin_path_list;
			ripup_net.tempTargetTable = &this->TempTargetTable;
			ripup_net.temp_target_path_list = &this->Temp_target_path_list;
			printf("Get Connection\n");
			
			TwoPinNetConnection& connection = ripup_net.GetConnection();
			printf("END Get Connection\n");
			// mark on bitwise
			//ISPD_Insert_Subtree_array_(ripup_net.PinToBitwise);
			if (last_rnd_two_pin_net_id != two_pin_net_id){
				int PinIndex;
				if ((ripup_net.TwoPinNetData)->TwoPinIndex.second != UndefineValue)
				{
					PinIndex = (ripup_net.TwoPinNetData)->TwoPinIndex.second;
				}
				else{
					printf("ripup_net.TwoPinNetData->TwoPinIndex.second == UndefineValue\n");
					exit(1);
				}
				if (ripup_net.DesignPinList->at(PinIndex).IsIRoutePath == 9453) // 9453 == IAMIROUTE
				{
					ISPD_Insert_Target_array_(ripup_net.two_pin_net_connection.Target_pin_path_list);
					ISPD_IRouteRevise(ripup_net, PinIndex);
				}
				else if (ripup_net.DesignPinList->at(PinIndex).OutSideMemory)
				{
					//printf("Outside Memory\n");
					ISPD_Insert_Target_array_(ripup_net.two_pin_net_connection.Target_pin_path_list);
					//exit(1);
				}
				else
				{
					vector<Pin> Valid_Target;
					FindFirstValidTarget(ripup_net, Net_Pin_Corresponding_Refinement, Valid_Target, PinIndex);
					ISPD_Insert_Target_array_(Valid_Target);
				}

				for (auto &tarnode : ripup_net.two_pin_net_connection.Target_pin_path_list)
				{
					int x, y, z;
					tarnode.GetPinPointPosition(x, y, z);
					//printf("TAR (%d, %d, %d)\n", x, y, z);
					tuple<coor_t, coor_t, int> temp_coor;
					temp_coor = ISPD_real_coor_table.Index2Coor(x, y, z);
					if (get<0>(temp_coor) < minX)
						minX = get<0>(temp_coor);
					if (get<1>(temp_coor) < minY)
						minY = get<1>(temp_coor);
					if (get<0>(temp_coor) > maxX)
						maxX = get<0>(temp_coor);
					if (get<1>(temp_coor) > maxY)
						maxY = get<1>(temp_coor);
				}
				
			}
			else{
				/*or (int i = 0; i < ripup_net.PinToBitwise.size(); i++)
				{
					int x = ripup_net.PinToBitwise.at(i).pin_position.pin_point.x;
					int y = ripup_net.PinToBitwise.at(i).pin_position.pin_point.y;
					int z = ripup_net.PinToBitwise.at(i).pin_position.pin_point.z;
					printf("PinToBitwise: (%d,%d,%d)\n", x, y, z);
				}*/

				ISPD_Insert_Target_array_(ripup_net.PinToBitwise);
			}

			last_rnd_two_pin_net_id = two_pin_net_id;

			bool path_setup;
			vector <Parser::I_Rect> Global_guide = ripup_net.Global_guide;
			if (Global_guide.size() == 0){
				printf("Error:: TwoPinNet(%d) : Global guide size is 0\n", i);
				exit(1);
			}
			const bool both_are_point_pins = (
				ripup_net.two_pin_net_connection.src_type == 0 &&
				ripup_net.two_pin_net_connection.tar_type == 0);

			if (false)
			{
				;
			}
			else
			{
				//printf("Setting pins\n");
				Parser::I_Rect SrcEnc;
				Parser::I_Rect TarEnc;
				const int net_rr_times = 1;
				//printf("Src size = %d\n", connection.Source_pin_path_list.size());
				for (auto &srcnode : connection.Source_pin_path_list)
				{
					int x, y, z;
					srcnode.GetPinPointPosition(x, y, z);
					//printf("SRC (%d, %d, %d)\n", x, y, z);

					tuple<coor_t, coor_t, int> temp_coor;
					temp_coor = ISPD_real_coor_table.Index2Coor(x, y, z);
					if (get<0>(temp_coor) < minX)
						minX = get<0>(temp_coor);
					if (get<1>(temp_coor) < minY)
						minY = get<1>(temp_coor);
					if (get<0>(temp_coor) > maxX)
						maxX = get<0>(temp_coor);
					if (get<1>(temp_coor) > maxY)
						maxY = get<1>(temp_coor);
				}
				//printf("Tar size = %d\n", ripup_net.PinToBitwise.size());
				for (auto &tarnode : ripup_net.PinToBitwise)
				{
					int x, y, z;
					tarnode.GetPinPointPosition(x, y, z);
					//printf("TAR (%d, %d, %d)\n", x, y, z);
					tuple<coor_t, coor_t, int> temp_coor;
					temp_coor = ISPD_real_coor_table.Index2Coor(x, y, z);
					if (get<0>(temp_coor) < minX)
						minX = get<0>(temp_coor);
					if (get<1>(temp_coor) < minY)
						minY = get<1>(temp_coor);
					if (get<0>(temp_coor) > maxX)
						maxX = get<0>(temp_coor);
					if (get<1>(temp_coor) > maxY)
						maxY = get<1>(temp_coor);
				}

				
				//printf("min(%d,%d) max(%d,%d)\n", minX, minY, maxX, maxY);
				int LBfirst = minX;
				int LBsecond = minY;
				int RTfirst = maxX;
				int RTsecond = maxY;

				LBfirst = LBfirst - LUCKYNUMBER * 2000;
				LBsecond = LBsecond - LUCKYNUMBER * 2000;
				RTfirst = RTfirst + LUCKYNUMBER * 2000;
				RTsecond = RTsecond + LUCKYNUMBER * 2000;

				Parser::I_Rect BoundingBox = Parser::I_Rect(LBfirst, LBsecond, RTfirst, RTsecond, -1);
				//printf("bounding (%d,%d) (%d,%d)\n", LBfirst, LBsecond, RTfirst, RTsecond);
				// special case

				//SetBoundingBox(bounding_box, connection, net_rr_times);
				printf("============= Net(%d) TwoPinNet(%d) Success(%d) Short(%d) Failure(%d) Maze Routing Start =============\n", two_pin_net_id, i , success_counter,bad_net_counter,path_no_found_counter);
				Node target_point;

				map<tuple<int, int, int>, Parser::PathAndEnc> SourceRefinmentMap;
				map<Node, Parser::PathAndEnc>
					TargetRefinmentMap;

				int SrcPinIndex;
				if ((ripup_net.TwoPinNetData)->TwoPinIndex.first != UndefineValue)
				{
					SrcPinIndex = (ripup_net.TwoPinNetData)->TwoPinIndex.first;
				}
				else
				{
					printf("ripup_net.TwoPinNetData->TwoPinIndex.first == UndefineValue\n");
					exit(1);
				}
				bool need_refine_src = true;
				bool short_src = false;
				bool isIroute = ripup_net.DesignPinList->at(SrcPinIndex).IsIRoutePath == 9453;
				if (isIroute)
				{
					need_refine_src = false;
				}

				vector<Pin> Valid_Source_list;
				clock_t maze_start = clock();

				Parser::I_Rect Target_Box = ripup_net.TargetBound;

				int maze = ISPD_MazeRouting3D(
					two_pin_net_id, two_pin_net_index,
					connection,
					Global_guide,
					BoundingBox,
					target_point,
					SourceRefinmentMap,
					TargetRefinmentMap,
					Net_Pin_Corresponding_Refinement, isIroute, Valid_Source_list, Target_Box);

				clock_t maze_end = clock();
				double maze_duration = (double)(maze_end - maze_start) / CLOCKS_PER_SEC;
				Normal_Maze_Time += maze_duration;
				printf("Maze Route Time Cost : %.2lf sec\n", maze_duration);
				printf("============= Net(%d) TwoPinNet(%d) Success(%d) Short(%d) Failure(%d) Maze Routing End =============\n", two_pin_net_id, i, success_counter, bad_net_counter, path_no_found_counter);
				if (maze == -1 || maze == -2){
					//exit(1);

					if(maze == -1)
						PathNotFound ++;
					if (maze == -2)
						SourceEmpty++;
#ifdef ALLOW_SHORT
							short_src = true;
							clock_t short_maze_start = clock();
							int short_maze = ISPD_MazeRouting3D_Allow_Short(two_pin_net_id, two_pin_net_index,
																			connection,
																			Global_guide,
																			BoundingBox,
																			target_point,
																			SourceRefinmentMap,
																			TargetRefinmentMap, isIroute, Target_Box);
							clock_t short_maze_end = clock();
							double short_maze_duration = (double)(maze_end - maze_start) / CLOCKS_PER_SEC;
							Short_Maze_Time += short_maze_duration;
							if (short_maze == -1)
							{
								path_no_found_counter++;
								exit(1);

								continue;
					}
#else
					bad_net_counter++;
					continue;
#endif
					bad_net_counter++;
				}
				clock_t pro_start = clock();
				Node source;
				Node target;
				path_setup =
					BackTrack(
						two_pin_net_index,
						target_point, connection, two_pin_net_id, SrcEnc, TarEnc, source, target);
				printf("============= Net(%d) TwoPinNet(%d) Success(%d) Short(%d) Failure(%d) Detailed Path  =============\n", two_pin_net_id, i , success_counter,bad_net_counter,path_no_found_counter);
				if (!path_setup){
					//printf("Path Set up error\n");
					path_no_found_counter++;
				}
				else{

					if(short_src){
						PseusoNodeDetermin_FromNetTable_Short(connection, source, target, two_pin_net_id, SrcEnc, TarEnc, Net_Pin_Corresponding_Refinement, SourceRefinmentMap);
						if (!isIroute)
							this->Temp_target_path_list = Valid_Source_list;
					}
					if (!short_src){
						PseusoNodeDetermin_FromNetTable(connection, source, target, two_pin_net_id, SrcEnc, TarEnc, Net_Pin_Corresponding_Refinement);
						if (!isIroute)
							this->Temp_target_path_list = Valid_Source_list;
					}

					vector<Parser::I_Rect> Patch;
					printf("Hit Patch\n");
					if (SrcEnc.Layer != UndefineValue)
					{
						ISPD_HitRectPatch(SrcEnc, connection.Src_Pin_Shape, Patch, two_pin_net_id);
					}
					if (TarEnc.Layer != UndefineValue){
						ISPD_HitRectPatch(TarEnc, connection.Tar_Pin_Shape, Patch, two_pin_net_id);
					}
					

					//printf("PathConstruction\n");
					vector<Parser::wire_path> This_wire_list;
					//printf("====================================\n");
					PathConstruction(ripup_net.two_pin_net_detailed_path.detail_grid_path, i, Net_wirePath[two_pin_net_id], Patch, This_wire_list);
					/*for (int path = 0; path < Net_wirePath[two_pin_net_id].size(); path++){
						if (Net_wirePath[two_pin_net_id][path].path_type == 3){
							printf("Patch Layer(%d) Locate(%d,%d) LB(%d,%d) RT(%d,%d)\n", Net_wirePath[two_pin_net_id][path].Patch.Layer, Net_wirePath[two_pin_net_id][path].PatchLocate.first, Net_wirePath[two_pin_net_id][path].PatchLocate.second, Net_wirePath[two_pin_net_id][path].Patch.LB.first, Net_wirePath[two_pin_net_id][path].Patch.LB.second, Net_wirePath[two_pin_net_id][path].Patch.RT.first, Net_wirePath[two_pin_net_id][path].Patch.RT.second);
						}
						else
						printf("Src(%d,%d) - Tar(%d,%d) Layer(%d)\n", Net_wirePath[two_pin_net_id][path].Src_Pin.first, Net_wirePath[two_pin_net_id][path].Src_Pin.second,
							Net_wirePath[two_pin_net_id][path].Tar_Pin.first, Net_wirePath[two_pin_net_id][path].Tar_Pin.second, Net_wirePath[two_pin_net_id][path].Layer);
					}*/
					//printf("Push_path_into_SEG\n");
					//Push_path_into_SEG(This_wire_list, two_pin_net_id);
					//printf("Push_path_into_blockage\n");
					Push_path_into_blockage(ripup_net.two_pin_net_detailed_path.detail_grid_path, two_pin_net_id);
					/*
					void ISPD_set_Congestion_map_();
					void ISPD_reset_Congestion_map_();
					void ISPD_Add_Congestion_map_(int x, int y, int z, int AddNum);
					*/

					printf("DetailPath2RipUpPath\n");
					DetailPath2RipUpPath(ripup_net.two_pin_net_detailed_path.detail_grid_path, ripup_net, two_pin_net_index);

					
					//cout << "IS IROUTE : " << isIroute << endl;
					if (isIroute)
					{
						//printf("Iroute revise\n");
						ISPD_IRouteRevise(ripup_net, SrcPinIndex);
					}

					//printf("bitwise\n");
					//ISPD_Insert_Subtree_array_(ripup_net.PinToBitwise);
					ISPD_Insert_Target_array_(ripup_net.PinToBitwise);

					printf("============= Net(%d) TwoPinNet(%d) Success(%d) Short(%d) Failure(%d) End  =============\n", two_pin_net_id, i ,  success_counter,bad_net_counter,path_no_found_counter);
					if(maze != -1)
						success_counter++;
				}
				clock_t pro_end = clock();
				double pro_duration = (double)(pro_end - pro_start) / CLOCKS_PER_SEC;
				printf("Processing Time Cost : %.2lf sec\n", pro_duration);
			}


		} // if unrouted
	}
	/*
double Normal_Maze_Time = 0.0;
	double Short_Maze_Time = 0.0;
	int Target_In_Box = 0;
	int SourceEmpty = 0;
	int PathNotFound = 0;*/
	printf("Statistical Data:: Normal Maze Time : %.5f\n",Normal_Maze_Time);
	printf("Statistical Data:: Short Maze Time : %.5f\n",Short_Maze_Time);
	printf("Statistical Data:: Source Empty  : %d\n",SourceEmpty);
	printf("Statistical Data:: Path Not Found : %d\n",PathNotFound);
	//printf("FirstPhaseRoute::Total_Net_Num(%d), Path_Not_Found(%d)\n", this->rip_up_two_pin_nets_.size(), path_no_found_counter);
}

void SmallRouter::Push_path_into_SEG(vector<Parser::wire_path> This_wire_list, int two_pin_net_id)
{
	for (int i = 0; i < This_wire_list.size(); i++){
		if (This_wire_list[i].path_type == 0 || This_wire_list[i].path_type == 1){
			if (This_wire_list[i].Src_Pin.first == This_wire_list[i].Tar_Pin.first){
				// vertical;
				Parser::I_Rect PathBlock;
				PathBlock.Layer = This_wire_list[i].Layer;
				PathBlock.LB.first = This_wire_list[i].Src_Pin.first - (ISPD_MinWidth_list[PathBlock.Layer] / 2);
				PathBlock.RT.first = This_wire_list[i].Src_Pin.first + (ISPD_MinWidth_list[PathBlock.Layer] / 2);
				PathBlock.LB.second = min(This_wire_list[i].Src_Pin.second, This_wire_list[i].Tar_Pin.second) - (ISPD_MinWidth_list[PathBlock.Layer] / 2);
				PathBlock.RT.second = max(This_wire_list[i].Src_Pin.second, This_wire_list[i].Tar_Pin.second) + (ISPD_MinWidth_list[PathBlock.Layer] / 2);
				printf("1Path blockage: LB(%d,%d) RT(%d,%d)\n", PathBlock.LB.first, PathBlock.LB.second, PathBlock.RT.first, PathBlock.RT.second);
				this->SpaceEvaluationLayout[PathBlock.Layer].PushNetBlockageToTree(PathBlock, two_pin_net_id);
			}
			else if (This_wire_list[i].Src_Pin.second == This_wire_list[i].Tar_Pin.second)
			{
				// horizontal;
				Parser::I_Rect PathBlock;
				PathBlock.Layer = This_wire_list[i].Layer;
				PathBlock.LB.second = This_wire_list[i].Src_Pin.second - (ISPD_MinWidth_list[PathBlock.Layer] / 2);
				PathBlock.RT.second = This_wire_list[i].Src_Pin.second + (ISPD_MinWidth_list[PathBlock.Layer] / 2);
				PathBlock.LB.first = min(This_wire_list[i].Src_Pin.first, This_wire_list[i].Tar_Pin.first) - (ISPD_MinWidth_list[PathBlock.Layer] / 2);
				PathBlock.RT.first = max(This_wire_list[i].Src_Pin.first, This_wire_list[i].Tar_Pin.first) + (ISPD_MinWidth_list[PathBlock.Layer] / 2);
				printf("2Path blockage: %d LB(%d,%d) RT(%d,%d)\n", ISPD_MinWidth_list[PathBlock.Layer] / 2 , PathBlock.LB.first, PathBlock.LB.second, PathBlock.RT.first, PathBlock.RT.second);
				this->SpaceEvaluationLayout[PathBlock.Layer].PushNetBlockageToTree(PathBlock, two_pin_net_id);
			}
		}
	}
}

// Second Phase
// 1. Bounded Maze Routing
// 2. Strict Line-end Rule->Relaxed Line-end Rule
void SmallRouter::SecondPhaseRouting()
{
	cout << "Entering Second Phase!" << endl;
	int rip_up_times = 1;
	const int max_iteration_times = kSecondPhaseRipUpIteration_;
	SetRerouteTwoPinNet();
	
	bool reach_rip_up_time_threshold = 
		(rip_up_times > kSecondPhaseRipUpIteration_);
	bool has_violated_nets = 
		(reroute_two_pin_net_.size() != 0);
	bool line_end_converge = false;
	int last_line_end_violation_num = 0;
	int line_end_converge_counter = 0;
	while((!reach_rip_up_time_threshold) && (has_violated_nets))
	{
		int short_net_num = 0;
		int open_net_num = 0;
		int overlap_violation_num = 0;
		AnalyzeTheResult(
				short_net_num, 
				open_net_num, 
				overlap_violation_num
				);

		int line_violation_num = 0;
		if(this->kLineEndModeRouting_)
		{
			line_violation_num = 
				line_end_graph_.AnalyzeLineEndViolation(); 
		}

		// Count the violated via enclosure nodes.
		int via_enc_vio_num = 0;
		for(TwoPinRerouteNet &reroute_net: reroute_two_pin_net_)
			via_enc_vio_num += (int)reroute_net.via_enc_vio_nodes.size();

		// Save the current solution and keep the best one.
		current_solution_.SetSolution(
				short_net_num, 
				open_net_num, 
				overlap_violation_num, 
				line_violation_num
			);
		if(IsBetterSolution(this->current_solution_, this->best_solution_))
		{
			this->best_solution_ = this->current_solution_;
		}

		if(this->kLineEndModeRouting_)
		{
			if(short_net_num == 0 && open_net_num == 0 && via_enc_vio_num == 0 && 
					line_violation_num == last_line_end_violation_num)
			{
				line_end_converge_counter++;
			}
			else if(short_net_num == 0 && open_net_num == 0 && via_enc_vio_num == 0 &&
					line_violation_num != last_line_end_violation_num)
			{
				line_end_converge_counter = 0;
				last_line_end_violation_num = line_violation_num;
			}
			else
			{}

			if(line_end_converge_counter >= 2/*kSecondPhaseRipUpIteration_/100*/)
				line_end_converge = true;
		} 
		
		if(line_end_converge && open_net_num == 0 && 
			short_net_num == 0 && overlap_violation_num == 0)
		{
			cout << "Line-end Vios Converges. Exit!" << endl;
			break;
		}

		SetNetScore(rip_up_times, max_iteration_times);
		// sort each net by its net score
		stable_sort(
				reroute_two_pin_net_.begin(), 
				reroute_two_pin_net_.end(), 
				LessWireLength);
		stable_sort(
				reroute_two_pin_net_.begin(), 
				reroute_two_pin_net_.end(), 
				LessNetScore);
		
		if(this->kLineEndModeRouting_ &&
				(short_net_num == 0 && open_net_num == 0)) 
		{
			stable_sort(
					reroute_two_pin_net_.begin(), 
					reroute_two_pin_net_.end(), 
					LessLineEndVioaltion);
		} 
		
		// Show the info of rerouted nets every iteration.
		ShowRerouteNetInfo(rip_up_times);
		// Rip-Up and Reroute each Net.
		for(TwoPinRerouteNet &reroute_net : reroute_two_pin_net_)
		{
			const int two_pin_net_index = reroute_net.index;
			const int two_pin_net_id = reroute_net.two_pin_net_id;

			// if(need_to_be_ripup)
			// {
			vector<Node> &net_history_node = reroute_net.history_node;
			ReCheckAndUpdateHistoryGraph(net_history_node);
			if(this->kLineEndModeRouting_)
			{
				const vector<TurnNode> &turn_nodes = 
					rip_up_two_pin_nets_[two_pin_net_index].GetPathTurnNodes();
				line_end_graph_.CleanBlockage(turn_nodes);
			}
			// }
			// else
			// {
				// continue;
			// }
			RipUpTwoPinNetFromTrack(two_pin_net_index, two_pin_net_id);
			
			TwoPinRipUpNet &ripup_net = 
				rip_up_two_pin_nets_[two_pin_net_index];
			const TwoPinNetConnection &connection = 
				ripup_net.GetConnection();
			BoundingBox bounding_box;
			// SetBoundingBox(bounding_box);

			bool path_setup = false;
			const bool both_are_point_pins = (
				ripup_net.GetSource().pin_type == kPinPoint && 
				ripup_net.GetTarget().pin_type == kPinPoint);
			if(both_are_point_pins)
			{
				int source_x, source_y, source_z;
				ripup_net.GetSource().GetPinPointPosition(
						source_x, source_y, source_z);
				const Node source(source_x, source_y, source_z);

				int target_x, target_y, target_z;
				ripup_net.GetTarget().GetPinPointPosition(
						target_x, target_y, target_z);
				const Node target(target_x, target_y, target_z);
			
				const int net_rr_times = ripup_net.rip_up_times;
				// SetBoundingBox(bounding_box, connection, net_rr_times);
				if(kLineEndModeRouting_)
				{
					SetBoundingBox(bounding_box, connection, net_rr_times);
				}
				else
				{
					if(rip_up_times < (kSecondPhaseRipUpIteration_ - kSecondPhaseRipUpIteration_/10))
						SetBoundingBox(bounding_box, connection, net_rr_times);
					else
						SetBoundingBox(bounding_box);
				}


				/*MazeRouting3D(
					two_pin_net_id, two_pin_net_index,
					source, target,
					bounding_box);*/

				path_setup = 
					BackTrack(
							two_pin_net_index,
							source,
							target);
			}
			else
			{
				const int net_rr_times = ripup_net.rip_up_times; 
				SetBoundingBox(bounding_box, connection, net_rr_times);
				if(kLineEndModeRouting_)
				{
					SetBoundingBox(bounding_box, connection, net_rr_times);
				}
				else
				{
					if(rip_up_times < (kSecondPhaseRipUpIteration_ - kSecondPhaseRipUpIteration_/10))
						SetBoundingBox(bounding_box, connection, net_rr_times);
					else
						SetBoundingBox(bounding_box);
				}

				Node target_point;
				/*MazeRouting3D(
					two_pin_net_id, two_pin_net_index,
					connection,
					bounding_box,
					target_point
					);*/
	
				/*path_setup = 
					BackTrack(
							two_pin_net_index, 
							target_point);*/
			}
			

			if(path_setup)
			{
				FindPathTurns(two_pin_net_index);
				if(this->kLineEndModeRouting_) 
				{
					const vector<TurnNode> &new_turn_nodes = 
						ripup_net.GetPathTurnNodes();
					line_end_graph_.InsertBlockage(new_turn_nodes);
				} 
				
				SetTwoPinNetPathToTrack(two_pin_net_index, two_pin_net_id);
			}
		}

		if(this->kLineEndModeRouting_)
		{
			line_end_graph_.CleanBlockageWholeGraph();
			for(TwoPinRipUpNet &ripup_net : rip_up_two_pin_nets_)
			{
				const vector<TurnNode> &turn_nodes = 
					ripup_net.GetPathTurnNodes();
				line_end_graph_.InsertBlockage(turn_nodes);
			}
		}

		UpdateRipUpTwoPinNetFromRerouteTwoPinNet();
		// Check whether need to rip-up and reroute again or not.
		this->total_rip_up_times++;
		rip_up_times++;
		reach_rip_up_time_threshold = 
			(rip_up_times > kSecondPhaseRipUpIteration_);
		
		SetRerouteTwoPinNet();
		has_violated_nets = (reroute_two_pin_net_.size() != 0);
	}
	
	cout << endl << "~~~~~~~~~~ 2ed Phase Result ~~~~~~~~~~" << endl;
	if(this->kLineEndModeRouting_)
	{
		line_end_graph_.CleanBlockageWholeGraph();
		for(TwoPinRipUpNet &ripup_net : rip_up_two_pin_nets_)
		{
			const vector<TurnNode> &turn_nodes = 
				ripup_net.GetPathTurnNodes();
			line_end_graph_.InsertBlockage(turn_nodes);
		}
	}

	int short_net_num = 0;
	int open_net_num = 0;
	int overlap_violation_num = 0;
	AnalyzeTheResult(short_net_num, open_net_num, overlap_violation_num);
	int line_end_violation_num = 0;
	if(this->kLineEndModeRouting_)
	{
		line_end_graph_.AnalyzeLineEndViolation();
	}

	current_solution_.SetSolution(
			short_net_num, 
			open_net_num, 
			overlap_violation_num, 
			line_end_violation_num
		);
	if(IsBetterSolution(this->current_solution_, this->best_solution_))
	{
		this->best_solution_ = this->current_solution_;
	}

	cout << endl << endl << "2nd Phase Total Rip-Up Iteration Times ";
	cout << (rip_up_times - 1) << endl;
	cout << "Completing 2ed Phase!" << endl;
}


inline void SmallRouter::SetBoundingBox(
		BoundingBox &bounding_box, 
		const TwoPinNetConnection &connection, 
		const int rip_up_times
		)
{
	const Pin &source = connection.source;
	const PinType source_pin_type = source.pin_type;
	int source_x1, source_y1, source_z1;
	int source_x2, source_y2, source_z2;
	switch(source_pin_type)
	{
		case kPinPoint:
			source.GetPinPointPosition(source_x1, source_y1, source_z1);
			break;
		case kPinLine:
			source.GetPinLinePosition(
					source_x1, source_y1, source_z1, 
					source_x2, source_y2, source_z2);
			break;
		case kPinRectangle:
			source.GetPinRectanglePosition(
					source_x1, source_y1, source_z1, 
					source_x2, source_y2, source_z2);
			break;
		default:
			;
	}

	const Pin &target = connection.target;
	const PinType target_pin_type = target.pin_type;
	int target_x1, target_y1, target_z1;
	int target_x2, target_y2, target_z2;
	switch(target_pin_type)
	{
		case kPinPoint:
			target.GetPinPointPosition(target_x1, target_y1, target_z1);
			break;
		case kPinLine:
			target.GetPinLinePosition(
					target_x1, target_y1, target_z1, 
					target_x2, target_y2, target_z2);
			break;
		case kPinRectangle:
			target.GetPinRectanglePosition(
					target_x1, target_y1, target_z1, 
					target_x2, target_y2, target_z2);
			break;
		default:
			;
	}
	
	
	const bool source_is_point = (source_pin_type == kPinPoint);
	const bool target_is_point = (target_pin_type == kPinPoint);
	int min_x = -1, min_y = -1, min_z = -1;
	int max_x = -1, max_y = -1, max_z = -1;
	if(source_is_point && target_is_point)
	{
		min_x = min(source_x1, target_x1);
		min_y = min(source_y1, target_y1);
		min_z = min(source_z1, target_z1);
		max_x = max(source_x1, target_x1);
		max_y = max(source_y1, target_y1);
		max_z = max(source_z1, target_z1);
	}
	else if(source_is_point && !target_is_point)
	{
		min_x = min(source_x1, min(target_x1, target_x2));
		min_y = min(source_y1, min(target_y1, target_y2));
		min_z = min(source_z1, min(target_z1, target_z2));
		max_x = max(source_x1, max(target_x1, target_x2));
		max_y = max(source_y1, max(target_y1, target_y2));
		max_z = max(source_z1, max(target_z1, target_z2));
	}
	else if(!source_is_point && target_is_point)
	{
		min_x = min(min(source_x1, source_x2), target_x1);
		min_y = min(min(source_y1, source_y2), target_y1);
		min_z = min(min(source_z1, source_z2), target_z1);
		max_x = max(max(source_x1, source_x2), target_x1);
		max_y = max(max(source_y1, source_y2), target_y1);
		max_z = max(max(source_z1, source_z2), target_z1);
	}
	else if(!source_is_point && !target_is_point)
	{
		min_x = min(min(source_x1, source_x2), min(target_x1, target_x2));
		min_y = min(min(source_y1, source_y2), min(target_y1, target_y2));
		min_z = min(min(source_z1, source_z2), min(target_z1, target_z2));
		max_x = max(max(source_x1, source_x2), max(target_x1, target_x2));
		max_y = max(max(source_y1, source_y2), max(target_y1, target_y2));
		max_z = max(max(source_z1, source_z2), max(target_z1, target_z2));
	}
	else
	{}

	const int bbox_width = max_x - min_x + 1;
	const int bbox_height = max_y - min_y + 1;
	const int bbox_layer = max_z - min_z + 1;

	const double alpha = kBBoxAlpha_;
	const double beta = kBBoxBeta_;
	const double expand_ratio = 
		(1 + beta) + atan((double)rip_up_times - alpha);

	const double expanded_width = bbox_width * expand_ratio;
	const double expanded_height = bbox_height * expand_ratio;
	const double expanded_layer = bbox_layer * expand_ratio;

	const int expanded_width_track = 
		(int)((expanded_width - bbox_width) / 2);
	const int expanded_height_track = 
		(int)((expanded_height - bbox_height) / 2);
	const int expanded_layer_track = 
		(int)((expanded_layer - bbox_layer) / 2);

	// set the Bbox range.
	int bbox_left = min_x - expanded_width_track;
	int bbox_right = max_x + expanded_width_track;
	int bbox_top = max_y + expanded_height_track;
	int bbox_bottom = min_y - expanded_height_track;
	int bbox_top_layer = max_z + expanded_layer_track;
	int bbox_bottom_layer = min_z - expanded_layer_track;

	// check if the boundary of Bbox exceeds that of layout
	if(bbox_left < 0)
		bbox_left = 0;
	
	if(bbox_right >= layout_width_)
		bbox_right = layout_width_ - 1;
	
	if(bbox_top >= layout_height_)
		bbox_top = layout_height_ - 1;
	
	if(bbox_bottom < 0)
		bbox_bottom = 0;
	
	if(bbox_top_layer >= layout_layer_)
		bbox_top_layer = layout_layer_ - 1;
	
	if(bbox_bottom_layer < 0)
		bbox_bottom_layer = 0;


	// setting Bbox boundary.
	bounding_box.left = bbox_left;
	bounding_box.right = bbox_right;
	bounding_box.top = bbox_top;
	bounding_box.bottom = bbox_bottom;
	bounding_box.top_layer = bbox_top_layer;
	bounding_box.bottom_layer = bbox_bottom_layer;
	
	return;
}



// set the bounding box to be full.
inline void SmallRouter::SetBoundingBox(
		BoundingBox &bounding_box
		)
{
	bounding_box.left = 0;
	bounding_box.right = layout_width_ - 1;
	bounding_box.top = layout_height_ - 1;
	bounding_box.bottom = 0;
	bounding_box.top_layer = layout_layer_ - 1;
	bounding_box.bottom_layer = 0; 
	
	return;
}


// Set the Scores of each ripped-up net.
void SmallRouter::SetNetScore(
		const int iteration_times, 
		const int max_iteration_times
		)
{
	const float alpha = kNetScoreAlpha_; // wire-length factor
	const float beta = kNetScoreBeta_; // violation number factor
	const float gama = kNetScoreGama_; // rip-up-times factor
	
	const bool has_reroute_net = (reroute_two_pin_net_.size() > 0);
	if(has_reroute_net)
	{
		// compute the average wire-length, 
		// average violation numbers and 
		// average rip-up times of rerouted nets
		int total_wire_length = 0;
		int total_violation_num = 0;
		int total_rip_up_times = 0;

		int max_wire_langth = 0;
		int max_violation_num = 0;
		int max_rip_up_tiems = 0;

		int min_wire_langth = INT_MAX;
		int min_violation_num = INT_MAX;
		int min_rip_up_tiems = INT_MAX;

		for(const TwoPinRerouteNet &reroute_net : reroute_two_pin_net_)
		{
			const int net_wire_length = reroute_net.wire_length;
			total_wire_length += net_wire_length;
			
			if(net_wire_length > max_wire_langth)
				max_wire_langth = net_wire_length;
			if(net_wire_length < min_wire_langth)
				min_wire_langth = net_wire_length;
			
			const int net_violation_num = reroute_net.violation_num;
			total_violation_num += net_violation_num;
			
			if(net_violation_num > max_violation_num)
				max_violation_num = net_violation_num;
			if(net_violation_num < min_violation_num)
				min_violation_num = net_violation_num;
			
			const int net_rip_up_times = reroute_net.rip_up_times;
			total_rip_up_times += net_rip_up_times;
			
			if(net_rip_up_times > max_rip_up_tiems)
				max_rip_up_tiems = net_rip_up_times;
			if(net_rip_up_times < min_rip_up_tiems)
				min_rip_up_tiems = net_rip_up_times;
		}

		const int wire_length_diff = max_wire_langth - min_wire_langth;
		const int violatin_num_diff = max_violation_num - min_violation_num;
		const int rip_up_times_diff = max_rip_up_tiems - min_rip_up_tiems;
		// compute the net score of each rerouted net 
		for(TwoPinRerouteNet &reroute_net : reroute_two_pin_net_)
		{
			const int tmp_wire_length = reroute_net.wire_length;
			const int tmp_violation_num = reroute_net.violation_num;
			const int tmp_rip_up_times = reroute_net.rip_up_times;
			
			float wire_length_score = 0.0;
			if(wire_length_diff)
			{
				wire_length_score = 
					(float) (tmp_wire_length - min_wire_langth) 
					/ wire_length_diff;
			}
			
			float violation_num_score = 0.0;
			if(violatin_num_diff)
			{
				violation_num_score = 
					(float) (tmp_violation_num - min_violation_num) 
					/ violatin_num_diff;
			}
			
			float rip_up_times_score = 0.0;
			if(rip_up_times_diff)
			{
				rip_up_times_score = 
					(float) (tmp_rip_up_times - min_rip_up_tiems) 
					/ rip_up_times_diff;
			}
			
			const float tmp_net_score = 
				alpha * wire_length_score +
				beta * violation_num_score + 
				gama * rip_up_times_score;

			reroute_net.net_score = tmp_net_score;
		}
	}
}


// Routing Algorithm
/*void SmallRouter::MazeRouting3D(
		const int two_pin_net_id, 
		const int two_pin_net_index,
		const Node &source, 
		const Node &target,
		const BoundingBox &bounding_box
		)
{
	// Init Setting
	priority_queue< MazeNode, vector<MazeNode>, GreaterMazeNode >  maze_queue;
	reset_cost_array(FLT_MAX);
	MazeNode source_node(source, 0.0);
	MazeNode target_node(target, 0.0);

	// Assume the path can be found first.
	rip_up_two_pin_nets_[two_pin_net_index].path_is_found = true;
	
	const ULL source_node_index = 
		Transform3dArrayTo1dIndex(source);
	cost_array_[source_node_index] = 0;
	back_track_array_[source_node_index] = kNoDirection;
	
	// Start the pin-to-pin Maze Routing.
	maze_queue.push(source_node);
	bool reach_target = target_node.IsEqualCoord(maze_queue.top());
	while( !reach_target )
	{
		// The path of the net can be found and built!!!
		if(maze_queue.empty())
		{
			rip_up_two_pin_nets_[two_pin_net_index].path_is_found = false;
			break;
		} 
		
		MazeNode current_node(maze_queue.top());
		maze_queue.pop();
		for(int dir_index = kRight; dir_index <= kDown; dir_index++)
		{
			const int direction = dir_index;
			MazeNode next_node(current_node);
			switch(direction)
			{
				case kRight:
					next_node.x += kMoveStep;
					break;
				case kLeft:
					next_node.x -= kMoveStep;
					break;
				case kTop:
					next_node.y += kMoveStep;
					break;
				case kBottom:
					next_node.y -= kMoveStep;
					break;
				case kUp:
					next_node.z += kMoveStep;
					break;
				case kDown:
					next_node.z -= kMoveStep;
					break;
				default:
					;
			}

			const ULL next_node_index = 
				Transform3dArrayTo1dIndex(next_node);
			const bool via_direction = 
				(direction == kUp || direction == kDown);
			const bool is_node_blocked = 
				IsNodeBlocked(next_node, bounding_box, two_pin_net_id, direction);
			if(!is_node_blocked)
			{
				const float next_node_temp_cost = 
					UpdateCost(
							current_node, next_node, 
							direction, via_direction,
							two_pin_net_id, two_pin_net_index
							);
				if(cost_array_[next_node_index] > next_node_temp_cost)
				{
					cost_array_[next_node_index] = next_node_temp_cost;
					back_track_array_[next_node_index] = direction;
					next_node.cost = cost_array_[next_node_index];
					maze_queue.push(next_node);
				}
			}
			
		}
		reach_target = target_node.IsEqualCoord(maze_queue.top());
	}
	// PrintCostArray();
	// PrintBackTrackArray();
}

void SmallRouter::MazeRouting3D(
		const int two_pin_net_id, 
		const int two_pin_net_index,
		const TwoPinNetConnection &connection,
		const BoundingBox &bounding_box,
		Node &target_point
		)
{
	const Pin &source = connection.source;
	
	// Init Setting
	vector<MazeNode> maze_queue;
	maze_queue.clear();
	reset_cost_array(FLT_MAX);

	// Set the source nodes.
	vector<Node> source_nodes;
	source_nodes.resize(1);
	// source.GetPinNodes(source_nodes);
	source.GetCenterNode(source_nodes[0]);
	for(const Node &source_node : source_nodes)
	{
		const ULL source_index = Transform3dArrayTo1dIndex(source_node);
		const bool has_blockage_on_grid = 
			(track_capacity_[source_index] == kBlockage);
		if(has_blockage_on_grid)
			continue;
		cost_array_[source_index] = 0;
		back_track_array_[source_index] = kNoDirection;
		maze_queue.push_back(MazeNode(source_node, 0));
	}
	make_heap(maze_queue.begin(), maze_queue.end(), GreaterMazeNode());
	
	// Assume the path can be found first.
	rip_up_two_pin_nets_[two_pin_net_index].path_is_found = true;

	// If the both pins are overlapped.
	if(maze_queue.size() <= 0)
		return;

	// Start to search path
	ULL target_key = Transform3dArrayTo1dIndex(maze_queue.front());
	bool reach_target = connection.IsTargetReached(target_key);
	while(!reach_target)
	{
		// The path of the net can't be found and built!!!
		if(maze_queue.empty())
		{
			rip_up_two_pin_nets_[two_pin_net_index].path_is_found = false;
			break;
		} 
		
		// explore the node
		MazeNode current_node(maze_queue.front());
		pop_heap(maze_queue.begin(), maze_queue.end(), GreaterMazeNode());
		maze_queue.pop_back();
		
		for(int dir_index = kRight; dir_index <= kDown; dir_index++)
		{
			const int direction = dir_index;
			
			// the neighbor of the current node
			MazeNode next_node(current_node);
			switch(direction)
			{
				case kRight:
					next_node.x += kMoveStep;
					break;
				case kLeft:
					next_node.x -= kMoveStep;
					break;
				case kTop:
					next_node.y += kMoveStep;
					break;
				case kBottom:
					next_node.y -= kMoveStep;
					break;
				case kUp:
					next_node.z += kMoveStep;
					break;
				case kDown:
					next_node.z -= kMoveStep;
					break;
				default:
					// do nothing
					;
			}
			
			const ULL next_node_index = Transform3dArrayTo1dIndex(next_node);
			const bool via_direction = 
				(direction == kUp || direction == kDown);
			const bool is_node_blocked = 
				IsNodeBlocked(next_node, bounding_box, two_pin_net_id, direction);
			if(!is_node_blocked)
			{
				const float next_node_temp_cost = 
					UpdateCost(
							current_node, next_node, 
							direction, via_direction,
							two_pin_net_id, two_pin_net_index);
				
				if(cost_array_[next_node_index] > next_node_temp_cost)
				{
					cost_array_[next_node_index] = next_node_temp_cost;
					back_track_array_[next_node_index] = direction;
					next_node.cost = cost_array_[next_node_index];

					maze_queue.push_back(next_node);
					push_heap(
							maze_queue.begin(), 
							maze_queue.end(), 
							GreaterMazeNode()
							);
					// update the priority queue.
					// see below the routing function.
				}
			}
		}
			
		// if find the target, then leave.
		target_key = Transform3dArrayTo1dIndex(maze_queue.front());
		reach_target = connection.IsTargetReached(target_key);
		if(reach_target)
		{
			target_point.SetNode(
					maze_queue.front().x, 
					maze_queue.front().y, 
					maze_queue.front().z);
		}
	}
	// PrintCostArray();
	// PrintBackTrackArray();
	return;
}

void SmallRouter::ISPD_Setting_Switching_Node(const vector<Parser::I_Rect> &GlobalGuide, 
	vector < vector <Node> > &switching_node_Raise, vector < vector <Node> > &switching_node_Fall , int range){
	switching_node_Raise.resize(this->layout_layer_);
	switching_node_Fall.resize(this->layout_layer_);
	for (int i = 0; i < GlobalGuide.size(); i++){
		int layer = GlobalGuide[i].Layer - 1;
		auto Lxp = (*ISPD_fast_coor_map).at(layer).first.upper_bound(GlobalGuide[i].LB.first);
		auto Rxp = (*ISPD_fast_coor_map).at(layer).first.lower_bound(GlobalGuide[i].RT.first);

		auto Byp = (*ISPD_fast_coor_map).at(layer).second.upper_bound(GlobalGuide[i].LB.second);
		auto Typ = (*ISPD_fast_coor_map).at(layer).second.lower_bound(GlobalGuide[i].RT.second);

		if (Lxp->second > Rxp->second || Byp->second > Typ->second){
			printf("WARNING:: Guide with no node inside\n");
		}
		else{
			for (int y = Byp->second - range; y <= Typ->second + range; y++){
				for (int x = Lxp->second - range; x <= Rxp->second + range; x++){
					Node node_R(x, y, layer - 1);
					Node node_F(x, y, layer + 1);
					switching_node_Raise[layer - 1].push_back(node_R);
					switching_node_Fall	[layer + 1].push_back(node_F);
				}
			}
		}
	}
	
}*/

bool SmallRouter::NodeCanGo(int x, int y, int z, bool UpperSwitching, bool LowerSwitching){

	if (x < 0 || x >= (*ISPD_GridMap_layout)[z][y].size()){
		return false;
	}
	if (y < 0 || y >= (*ISPD_GridMap_layout)[z].size()) return false;
	/*printf("node(%d,%d,%d)  UP/DOWN(%d,%d)\n", (*ISPD_GridMap_layout)[z][y][x].RaiseAble, (*ISPD_GridMap_layout)[z][y][x].FallAble
		, (*ISPD_GridMap_layout)[z][y][x].InPlane, UpperSwitching, LowerSwitching);*/
	if (UpperSwitching && !LowerSwitching){
		if ((*ISPD_GridMap_layout)[z][y][x].RaiseAble || (*ISPD_GridMap_layout)[z][y][x].InPlane){
			return true;
		}
	}
	else if (LowerSwitching && !UpperSwitching){
		if ((*ISPD_GridMap_layout)[z][y][x].FallAble || (*ISPD_GridMap_layout)[z][y][x].InPlane){
			return true;
		}
	}
	else if(!LowerSwitching && !UpperSwitching){
		if ((*ISPD_GridMap_layout)[z][y][x].InPlane){
			return true;
		}
	}
	else if(LowerSwitching && UpperSwitching){
		return true;
	}

	return false;
}

Node SmallRouter::ToNextNode(Node current_node, int direction, vector < vector <Parser::I_Rect> > &Layer_Global_guide){
	
	bool UpperSwitching = true;
	bool LowerSwitching = true;

	int x = current_node.x;
	int y = current_node.y;
	int z = current_node.z;

	tuple<coor_t, coor_t, int> temp_coor;
	temp_coor = ISPD_real_coor_table.Index2Coor(x, y, z);

	coor_t coor_x = get<0>(temp_coor);
	coor_t coor_y = get<1>(temp_coor);
	/*if (z - 1 >= 0){
		for (int i = 0; i < Layer_Global_guide[z - 1].size(); i++){
			if (coor_x >= Layer_Global_guide[z - 1][i].LB.first && coor_x <= Layer_Global_guide[z - 1][i].RT.first){
				if (coor_y >= Layer_Global_guide[z - 1][i].LB.second && coor_y <= Layer_Global_guide[z - 1][i].RT.second){
					LowerSwitching = true;
				}
			}
		}
	}
	
	if (z + 1 < Layer_Global_guide.size()){
		for (int i = 0; i < Layer_Global_guide[z + 1].size(); i++){
			if (coor_x >= Layer_Global_guide[z + 1][i].LB.first && coor_x <= Layer_Global_guide[z + 1][i].RT.first){
				if (coor_y >= Layer_Global_guide[z + 1][i].LB.second && coor_y <= Layer_Global_guide[z + 1][i].RT.second){
					UpperSwitching = true;
				}
			}
		}
	}*/
	int offset = 0;
	Node node(x,y,z);
	switch (direction)
	{
		case kRight:{
			offset++;
			bool FoundNode = false;
			while (!NodeCanGo(x + offset, y, z, UpperSwitching, LowerSwitching)){
				offset++;
				if (x + offset >= (*ISPD_GridMap_layout)[z][y].size()){
					node.x = -1;
					return node;
				}
			}
			node.x += offset;
			ISPD_back_track_Step_array_[node.z][node.y][node.x] = offset;
			return node;
		}
		case kLeft:{
			offset++;
			bool FoundNode = false;
			while (!NodeCanGo(x - offset, y, z, UpperSwitching, LowerSwitching)){
				offset++;
				if (x - offset < 0){
					node.x = -1;
					return node;
				}
			}
			node.x -= offset ;
			ISPD_back_track_Step_array_[node.z][node.y][node.x] = offset;
			return node;
		}
		case kTop:{
			offset++;
			bool FoundNode = false;
			while (!NodeCanGo(x, y + offset, z, UpperSwitching, LowerSwitching)){
				offset++;
				if (y + offset >= (*ISPD_GridMap_layout)[z].size() ){
					node.y = -1;
					return node;
				}
			}
			node.y += offset ;
			ISPD_back_track_Step_array_[node.z][node.y][node.x] = offset;
			return node;
		}
		case kBottom:{
			offset++;
			bool FoundNode = false;
			while (!NodeCanGo(x, y - offset, z, UpperSwitching, LowerSwitching)){
				offset++;
				if (y - offset < 0){
					node.y = -1;
					return node;
				}
			}
			node.y -= offset ;
			ISPD_back_track_Step_array_[node.z][node.y][node.x] = offset;
			return node;
		}
		case kUp:{
			//printf("UpperSwitching : %d ,RaiseAble %d\n", UpperSwitching, (*ISPD_GridMap_layout)[z][y][x].RaiseAble);
			if (UpperSwitching && (*ISPD_GridMap_layout)[z][y][x].RaiseAble){
				auto raising_node = ISPD_connection_table.RaisingNode_Index(x,y,z);
				//printf("Raising Node (%d,%d,%d)\n", get<0>(raising_node), get<1>(raising_node), get<2>(raising_node));
				Node r_node(get<0>(raising_node), get<1>(raising_node), get<2>(raising_node));
				return r_node;
			}
			else{
				return node;
			}
		}
		case kDown:{
			//printf("LowerSwitching : %d ,FallAble %d\n", LowerSwitching, (*ISPD_GridMap_layout)[z][y][x].FallAble);
			if (LowerSwitching && (*ISPD_GridMap_layout)[z][y][x].FallAble){
				auto falling_node = ISPD_connection_table.FallingNode_Index(x, y, z);
				//printf("Falling Node (%d,%d,%d)\n", get<0>(falling_node), get<1>(falling_node), get<2>(falling_node));
				Node f_node(get<0>(falling_node), get<1>(falling_node), get<2>(falling_node));
				/*printf("Node Falling: (%d,%d,%d) -> (%d,%d,%d) \n", node.x, node.y, node.z, 
					(*ISPD_GridMap_layout)[z][y][x].FallingNode.x, (*ISPD_GridMap_layout)[z][y][x].FallingNode.y, (*ISPD_GridMap_layout)[z][y][x].FallingNode.z);
				*/
				return f_node;
			}
			else{
				return node;
			}
		}
	default:
		// do nothing
		;
	}

	printf("Error::ToNextNode:: Direction Error\n");
}

void SettingLayerGlobalGuide(const vector<Parser::I_Rect> &GlobalGuide ,
	vector < vector <Parser::I_Rect> > &Layer_Global_guide, int total_layer){
	Layer_Global_guide.resize(total_layer);
	for (int i = 0; i < GlobalGuide.size(); i++){
		Layer_Global_guide[GlobalGuide[i].Layer - 1].push_back(GlobalGuide[i]);
		
		/*printf("Layer(%d) : Global guide : LB(%d,%d) RT(%d,%d)\n", GlobalGuide[i].Layer - 1, 
			GlobalGuide[i].LB.first, GlobalGuide[i].LB.second, GlobalGuide[i].RT.first, GlobalGuide[i].RT.second);
		*/
	}

}

void SmallRouter::Unmark_Blockage(const vector <Parser::I_Rect> &PinShape){
	for (int i = 0; i < PinShape.size(); i++){

		int width = min(PinShape[i].RT.first - PinShape[i].LB.first,
			PinShape[i].RT.second - PinShape[i].LB.second);
		int layer = PinShape[i].Layer - 1;
		int spacing = -1;
		for (int st = 0; st < ISPD_Spacing_Table[layer].size(); st++){
			if (width >= ISPD_Spacing_Table[layer][st].first)
				spacing = ISPD_Spacing_Table[layer][st].second;
			else{
				break;
			}
		}
		if (spacing == -1){
			printf("SetAllShapeToBlockage:: Error for spacing = -1\n");
		}
		spacing += 0.5 * ISPD_MinWidth_list[layer];
		// Parallel Run Length Spacing
		auto Lxp_spacing = (*ISPD_fast_coor_map).at(layer).first.upper_bound(PinShape[i].LB.first - spacing-1);
		auto Rxp_spacing = (*ISPD_fast_coor_map).at(layer).first.lower_bound(PinShape[i].RT.first + spacing-1);
		Rxp_spacing--;
		auto Byp_spacing = (*ISPD_fast_coor_map).at(layer).second.upper_bound(PinShape[i].LB.second - spacing+1);
		auto Typ_spacing = (*ISPD_fast_coor_map).at(layer).second.lower_bound(PinShape[i].RT.second + spacing+1);
		Typ_spacing--;

		for (int y = Byp_spacing->second; y <= Typ_spacing->second; y++){
			for (int x = Lxp_spacing->second; x <= Rxp_spacing->second; x++){
				if ((*ISPD_blockage_array_)[layer][y][x] == SPACING_BLOCKAGE)
					(*ISPD_blockage_array_)[layer][y][x] = SAMENET;
			}
		}

		int eol_spacing = ISPD_EndOfLine_Spacing[layer].SPACING + 0.5 * ISPD_MinWidth_list[layer];
		auto Lxp_eol = (*ISPD_fast_coor_map).at(layer).first.upper_bound(PinShape[i].LB.first - eol_spacing-1);
		auto Rxp_eol = (*ISPD_fast_coor_map).at(layer).first.lower_bound(PinShape[i].RT.first + eol_spacing-1);

		Rxp_eol--;
		auto Byp_eol = (*ISPD_fast_coor_map).at(layer).second.upper_bound(PinShape[i].LB.second - eol_spacing+1);
		auto Typ_eol = (*ISPD_fast_coor_map).at(layer).second.lower_bound(PinShape[i].RT.second + eol_spacing+1);
		Typ_eol--;
		for (int y = Byp_eol->second; y <= Typ_eol->second; y++){
			for (int x = Lxp_eol->second; x <= Rxp_eol->second; x++){
				if ((*ISPD_EOL_Blocage_Map)[layer][y][x] == SPACING_BLOCKAGE)
					(*ISPD_EOL_Blocage_Map)[layer][y][x] = SAMENET;
			}
		}


	}
}

void SmallRouter::Remark_Blockage(const vector <Parser::I_Rect> &PinShape){
	for (int i = 0; i < PinShape.size(); i++){

		int width = min(PinShape[i].RT.first - PinShape[i].LB.first,
			PinShape[i].RT.second - PinShape[i].LB.second);
		int layer = PinShape[i].Layer - 1;
		int spacing = -1;
		for (int st = 0; st < ISPD_Spacing_Table[layer].size(); st++){
			if (width >= ISPD_Spacing_Table[layer][st].first)
				spacing = ISPD_Spacing_Table[layer][st].second;
			else{
				break;
			}
		}
		if (spacing == -1){
			printf("SetAllShapeToBlockage:: Error for spacing = -1\n");
		}
		spacing += 0.5 * ISPD_MinWidth_list[layer];
		// Parallel Run Length Spacing
		auto Lxp_spacing = (*ISPD_fast_coor_map).at(layer).first.upper_bound(PinShape[i].LB.first - spacing-1);
		auto Rxp_spacing = (*ISPD_fast_coor_map).at(layer).first.lower_bound(PinShape[i].RT.first + spacing-1);
		Rxp_spacing--;
		auto Byp_spacing = (*ISPD_fast_coor_map).at(layer).second.upper_bound(PinShape[i].LB.second - spacing+1);
		auto Typ_spacing = (*ISPD_fast_coor_map).at(layer).second.lower_bound(PinShape[i].RT.second + spacing+1);
		Typ_spacing--;
		for (int y = Byp_spacing->second; y <= Typ_spacing->second; y++){
			for (int x = Lxp_spacing->second; x <= Rxp_spacing->second; x++){
				if ((*ISPD_blockage_array_)[layer][y][x] == SAMENET)
					(*ISPD_blockage_array_)[layer][y][x] = SPACING_BLOCKAGE;
			}
		}


		int eol_spacing = ISPD_EndOfLine_Spacing[layer].SPACING + 0.5 * ISPD_MinWidth_list[layer];

		auto Lxp_eol = (*ISPD_fast_coor_map).at(layer).first.upper_bound(PinShape[i].LB.first - eol_spacing-1);
		auto Rxp_eol = (*ISPD_fast_coor_map).at(layer).first.lower_bound(PinShape[i].RT.first + eol_spacing-1);
		Rxp_eol--;
		auto Byp_eol = (*ISPD_fast_coor_map).at(layer).second.upper_bound(PinShape[i].LB.second - eol_spacing+1);
		auto Typ_eol = (*ISPD_fast_coor_map).at(layer).second.lower_bound(PinShape[i].RT.second + eol_spacing+1);
		Typ_eol--;
		for (int y = Byp_eol->second; y <= Typ_eol->second; y++){
			for (int x = Lxp_eol->second; x <= Rxp_eol->second; x++){
				if ((*ISPD_EOL_Blocage_Map)[layer][y][x] == SAMENET)
					(*ISPD_EOL_Blocage_Map)[layer][y][x] = SPACING_BLOCKAGE;
			}
		}


	}
}

bool SmallRouter::VIA_BIT_MAP_CHECKING(const Node &node, pair <int,int> type , int net_id, I_Rect &Enclosure,bool up_down/* up : true , down : false*/)
{
	auto node_coor = ISPD_real_coor_table.Index2Coor(node.x,node.y,node.z);
	
	//printf("eol_width (%d) \n",ISPD_EndOfLine_Spacing[node.z].ENDOFLINE);
	//printf("Enclosure width (%d) , height (%d)\n",Enclosure.RT.first - Enclosure.LB.first,Enclosure.RT.second - Enclosure.LB.second);
	// I_Rect Enclosure; 
	int eol_width = ISPD_EndOfLine_Spacing[node.z].ENDOFLINE;
	bool horizontal_eol = false;
	bool vertical_eol = false;
	if((Enclosure.RT.first - Enclosure.LB.first) < eol_width){
		// vertical eol
		vertical_eol = true;
	}
	if((Enclosure.RT.second - Enclosure.LB.second) < eol_width){
		// horizontal eol
		horizontal_eol = true;
	}

	if(net_id == 2225 && get<0>(node_coor) == 232200 && get<1>(node_coor) == 318250 && node.z == 2){
		printf("eol_width (%d) \n",ISPD_EndOfLine_Spacing[node.z].ENDOFLINE);
		printf("Enclosure width (%d) , height (%d)\n",Enclosure.RT.first - Enclosure.LB.first,Enclosure.RT.second - Enclosure.LB.second);
		printf("vertical_eol (%d) , horizontal_eol (%d)\n",vertical_eol,horizontal_eol);
		printf("block-left(%d) , block-right(%d) , block-up(%d) , block-down(%d)\n",(*ISPD_blockage_array_)[node.z][node.y][node.x - 1],(*ISPD_blockage_array_)[node.z][node.y][node.x + 1],(*ISPD_blockage_array_)[node.z][node.y+1][node.x],(*ISPD_blockage_array_)[node.z][node.y-1][node.x]);
	}

	bool left_wire = false;
	if(node.x - 1 >= 0){
		if ((*ISPD_blockage_array_)[node.z][node.y][node.x - 1] != NO_BLOCKAGE &&
			(*ISPD_blockage_array_)[node.z][node.y][node.x - 1] != net_id)
		{
			if(horizontal_eol) return false;
		}
	}

	bool right_wire = false;
	if(node.x + 1 < (*ISPD_GridMap_layout)[node.z][0].size()){
		if ((*ISPD_blockage_array_)[node.z][node.y][node.x + 1] != NO_BLOCKAGE &&
			(*ISPD_blockage_array_)[node.z][node.y][node.x + 1] != net_id)
		{
			if(horizontal_eol) return false;
		}
	}

	bool up_wire = false;
	if(node.y + 1 < (*ISPD_GridMap_layout)[node.z].size()){
		if ((*ISPD_blockage_array_)[node.z][node.y + 1][node.x] != NO_BLOCKAGE &&
			(*ISPD_blockage_array_)[node.z][node.y + 1][node.x] != net_id)
		{
			if(vertical_eol) return false;
		}
	}

	bool down_wire = false;
	if(node.y - 1 >= 0){
		if ((*ISPD_blockage_array_)[node.z][node.y - 1][node.x] != NO_BLOCKAGE &&
			(*ISPD_blockage_array_)[node.z][node.y - 1][node.x] != net_id)
		{
			if(vertical_eol) return false;
		}
	}

	if(net_id == 2225 && get<0>(node_coor) == 232200 && get<1>(node_coor) == 318250 && node.z == 2){
		printf("path pass\n");
	}

	
	int i_type =  ISPD_enc_id_HashTable.GetViaBitID(type,up_down,node.z);
	for(int y = node.y - 1; y <= node.y + 1; y++){
		if( y < 0 || y >= (*ISPD_GridMap_layout)[node.z].size()) continue;
		for(int x = node.x - 1; x <= node.x + 1; x++){
			if( x < 0 || x >= (*ISPD_GridMap_layout)[node.z][y].size()) continue;

			

			if(ISPD_Via_Type_Map[node.z][y][x].getType() != -1){
				int j_type =  ISPD_Via_Type_Map[node.z][y][x].getType();
				auto other_node_coor = ISPD_real_coor_table.Index2Coor(x,y,node.z);
				if(x == node.x && y == node.y) return false;

				if(x == node.x){ // same vertical
					int spacing = ISPD_Enc_Relation_Pair_Table.at(node.z).at(i_type).at(j_type).same_vertical_spacing;

					if(abs(get<1>(other_node_coor) - get<1>(node_coor)) < spacing){
						return false;
					}
				}
				else if(y == node.y){ // same horizontal
					int spacing = ISPD_Enc_Relation_Pair_Table.at(node.z).at(i_type).at(j_type).same_horizontal_spacing;

					if(abs(get<0>(other_node_coor) - get<0>(node_coor)) < spacing){
						return false;
					}
				}
				else{
					int H_spacing = ISPD_Enc_Relation_Pair_Table.at(node.z).at(i_type).at(j_type).same_horizontal_spacing;
					int V_spacing = ISPD_Enc_Relation_Pair_Table.at(node.z).at(i_type).at(j_type).same_vertical_spacing;
					if(abs(get<0>(other_node_coor) - get<0>(node_coor)) < H_spacing || abs(get<1>(other_node_coor) - get<1>(node_coor)) < V_spacing){
						return false;
					}
				}

			}

		}	
	}

	return true;

}

bool SmallRouter::ViaEnclosureChecking(const Node &cur_node, const Node &next_node, pair <int,int> type , int net_id){

#ifdef _SpaceEvaluationGraph_
	vector<Parser::I_Rect> NetShape;
	// find the via enclosure bounding box of 2 layer (PRL , EOL)
	Parser::I_Rect cur_enclosure;
	Parser::I_Rect next_enclosure;
	if (type.first == cur_node.z){
		cur_enclosure = this->ISPD_via_type_list[type.first][type.second].BotLayerIRect;
		next_enclosure = this->ISPD_via_type_list[type.first][type.second].TopLayerIRect;
	}
	else{
		next_enclosure = this->ISPD_via_type_list[type.first][type.second].BotLayerIRect;
		cur_enclosure = this->ISPD_via_type_list[type.first][type.second].TopLayerIRect;
	}
	//printf("SpaceEvaluationLayout size %d\n", SpaceEvaluationLayout.size());
	/*printf("cur_layer %d\n", cur_node.z);
	printf("nex_layer %d\n", next_node.z);*/
	return  SpaceEvaluationLayout[cur_node.z].NodeEvluationByTree(cur_enclosure, NetShape, cur_node.x, cur_node.y, net_id) &
		SpaceEvaluationLayout[next_node.z].NodeEvluationByTree(next_enclosure, NetShape, next_node.x, next_node.y, net_id);
#endif

#ifdef _VIA_BIT_MAP_
	vector<Parser::I_Rect> NetShape;
	if(min(cur_node.z,next_node.z) == 0){
		Parser::I_Rect _cur_enclosure;
		Parser::I_Rect _next_enclosure;
		if (type.first == cur_node.z){
			_cur_enclosure = this->ISPD_via_type_list[type.first][type.second].BotLayerIRect;
			_next_enclosure = this->ISPD_via_type_list[type.first][type.second].TopLayerIRect;
			return  SpaceEvaluationLayout[cur_node.z].NodeEvluationByTree(_cur_enclosure, NetShape, cur_node.x, cur_node.y, net_id) &
				VIA_BIT_MAP_CHECKING(next_node,type,net_id,_next_enclosure,!(type.first == next_node.z));
		}
		else{
			_next_enclosure = this->ISPD_via_type_list[type.first][type.second].BotLayerIRect;
			_cur_enclosure = this->ISPD_via_type_list[type.first][type.second].TopLayerIRect;
			return  VIA_BIT_MAP_CHECKING(cur_node,type,net_id,_cur_enclosure,!(type.first == cur_node.z)) &
				SpaceEvaluationLayout[next_node.z].NodeEvluationByTree(_next_enclosure, NetShape, next_node.x, next_node.y, net_id);
		}
	
		
	}
	Parser::I_Rect cur_enclosure;
	Parser::I_Rect next_enclosure;
	if (type.first == cur_node.z){
		cur_enclosure = this->ISPD_via_type_list[type.first][type.second].BotLayerIRect;
		next_enclosure = this->ISPD_via_type_list[type.first][type.second].TopLayerIRect;
	}
	else{
		next_enclosure = this->ISPD_via_type_list[type.first][type.second].BotLayerIRect;
		cur_enclosure = this->ISPD_via_type_list[type.first][type.second].TopLayerIRect;
	}

	auto node_coor = ISPD_real_coor_table.Index2Coor(cur_node.x,cur_node.y,cur_node.z);

	if(net_id == 2225 && get<0>(node_coor) == 232200 && get<1>(node_coor) == 318250 && cur_node.z == 2 && next_node.z == 1){
		bool cur = VIA_BIT_MAP_CHECKING(cur_node,type,net_id,cur_enclosure,!(type.first == cur_node.z));
		bool nex = VIA_BIT_MAP_CHECKING(next_node,type,net_id,next_enclosure,!(type.first == next_node.z));

		printf("============== via_type(%d,%d) - cur (%d) , nex(%d)\n",type.first,type.second,cur,nex);
	}

	return VIA_BIT_MAP_CHECKING(cur_node,type,net_id,cur_enclosure,!(type.first == cur_node.z)) &
		VIA_BIT_MAP_CHECKING(next_node,type,net_id,next_enclosure,!(type.first == next_node.z));
#endif
	



}

bool SmallRouter::OffGridViaEnclosureChecking(int _x_coor_ , int _y_coor_ , int cur_layer ,
 int next_layer , pair<int,int> type , int net_id)
{
	Parser::I_Rect cur_enclosure;
	Parser::I_Rect next_enclosure;
	if (type.first == cur_layer)
	{
		cur_enclosure = this->ISPD_via_type_list[type.first][type.second].BotLayerIRect;
		cur_enclosure.Layer = cur_layer;
		cur_enclosure.LB.first += _x_coor_;
		cur_enclosure.RT.first += _x_coor_;
		cur_enclosure.LB.second += _y_coor_;
		cur_enclosure.RT.second += _y_coor_;

		next_enclosure = this->ISPD_via_type_list[type.first][type.second].TopLayerIRect;
		next_enclosure.Layer = next_layer;
		next_enclosure.LB.first += _x_coor_;
		next_enclosure.RT.first += _x_coor_;
		next_enclosure.LB.second += _y_coor_;
		next_enclosure.RT.second += _y_coor_;
	}
	else
	{
		next_enclosure = this->ISPD_via_type_list[type.first][type.second].BotLayerIRect;
		next_enclosure.Layer = next_layer;
		next_enclosure.LB.first += _x_coor_;
		next_enclosure.RT.first += _x_coor_;
		next_enclosure.LB.second += _y_coor_;
		next_enclosure.RT.second += _y_coor_;

		cur_enclosure = this->ISPD_via_type_list[type.first][type.second].TopLayerIRect;
		cur_enclosure.Layer = cur_layer;
		cur_enclosure.LB.first += _x_coor_;
		cur_enclosure.RT.first += _x_coor_;
		cur_enclosure.LB.second += _y_coor_;
		cur_enclosure.RT.second += _y_coor_;

	}

	return SpaceEvaluationLayout[cur_layer].GeneralViaEvaluation(cur_enclosure,net_id) &
		   SpaceEvaluationLayout[next_layer].GeneralViaEvaluation(next_enclosure, net_id);
}

void NetShape_Construction(vector <Parser::I_Rect> &NetShape,const vector <Parser::I_Rect> &A, const vector <Parser::I_Rect> &B){

	for (int i = 0; i < A.size(); i++){
		NetShape.push_back(A[i]);
	}
	for (int i = 0; i < B.size(); i++){
		NetShape.push_back(B[i]);
	}

}

bool SmallRouter::OffGridViaPrediction(Node &pseudo_target, const vector<Parser::I_Rect> PinShape, vector<Parser::wire_path> &OffGridPath, Parser::I_Rect &Enc, pair<int,int> &via_type_pair,int net_id)
{

	tuple<coor_t, coor_t, int> temp_coor;
	temp_coor = ISPD_real_coor_table.Index2Coor(pseudo_target.x, pseudo_target.y, pseudo_target.z);
	//printf("Layer : %d\n",pseudo_target.z);
	int pt_x = get<0>(temp_coor);
	int pt_y = get<1>(temp_coor);
	int pt_z = get<2>(temp_coor);
	for (int i = 0; i < PinShape.size(); i++)
	{
		if (pseudo_target.z - 1 == PinShape[i].Layer - 1)
		{
			// Right Check
			if (pseudo_target.x + 1 < (*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y].size())
			{
				tuple<coor_t, coor_t, int> temp_coor_right;
				temp_coor_right = ISPD_real_coor_table.Index2Coor(pseudo_target.x + 1, pseudo_target.y, pseudo_target.z);
				int pt_x_right = get<0>(temp_coor_right);
				if (pt_x <= PinShape.at(i).LB.first && pt_x_right >= PinShape.at(i).LB.first)
				{
					if (pt_y >= PinShape.at(i).LB.second && pt_y <= PinShape.at(i).RT.second)
					{
						// Right Hit
						int Hit_place_x = (PinShape.at(i).RT.first + PinShape.at(i).LB.first) / 2;
						int width = PinShape.at(i).RT.first - PinShape.at(i).LB.first;
						/*(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
						offGrid_Refine.path_type = 1;
						offGrid_Refine.Layer = pseudo_target.z;
						offGrid_Refine.Src_Pin = make_pair(pt_x, pt_y);
						offGrid_Refine.Tar_Pin = make_pair(Hit_place_x, pt_y);
						offGrid_Refine.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;
						*/
						if ((*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] != NO_BLOCKAGE)
						{
							return false;
						}
						for (int via_type = 0; via_type < this->ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
						{
							bool via_can_hit = OffGridViaEnclosureChecking(pt_x, pt_y, pseudo_target.z, pseudo_target.z - 1,
																		   make_pair(pseudo_target.z - 1, via_type), net_id);
							int width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
							int height = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;

							if (via_can_hit && height < width)
							{
								// path setting
								/*Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z;
								path.Src_Pin = make_pair(pt_x, pt_y);
								path.Tar_Pin = make_pair(Hit_place_x, pt_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + Hit_place_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
								Enc.RT = make_pair(Hit_place_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
								OffGridPath.push_back(path);*/

								Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z - 1;
								path.Src_Pin = make_pair(Hit_place_x, pt_y);
								path.Tar_Pin = make_pair(pt_x, pt_y);
								if (height < width)
								{
									path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
									via_type_pair = make_pair(pseudo_target.z - 1, via_type);
									Parser::wire_path patch;
									patch.path_type = 3;
									patch.Patch.Layer = pseudo_target.z - 1;
									patch.PatchLocate = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first,
																  pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second);

									patch.Patch.LB = make_pair(0, 0);
									patch.Patch.RT = make_pair(Hit_place_x - patch.PatchLocate.first + (ISPD_MinWidth_list[0]/2),
															   pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - patch.PatchLocate.second);
									//printf("Enclosure LB(%d,%d) RT(%d,%d)\n", ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
									//printf("pt_x(%d) pt_y(%d), Hit_x(%d) Hit_y(%d) \n", pt_x, pt_y, Hit_place_x, pt_y);
									//printf("Right::Patch Hit Layer(%d) Locate(%d,%d) , Patch LB(%d,%d) Patch RT(%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second,
									//	   patch.Patch.LB.first, patch.Patch.LB.second, patch.Patch.RT.first, patch.Patch.RT.second);
									/*int pause;
									cin >> pause;*/
#ifdef PseudoPatch
									OffGridPath.push_back(patch);
#endif
									Enc.Layer = pseudo_target.z - 1;
									//Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
									//Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
									Enc.LB = make_pair(patch.Patch.LB.first + patch.PatchLocate.first, patch.Patch.LB.second + patch.PatchLocate.second);
									Enc.RT = make_pair(patch.Patch.RT.first + patch.PatchLocate.first, patch.Patch.RT.second + patch.PatchLocate.second);
								}

								

								OffGridPath.push_back(path);

								return true;
							}
						}
						for (int via_type = 0; via_type < this->ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
						{
							bool via_can_hit = OffGridViaEnclosureChecking(pt_x, pt_y, pseudo_target.z, pseudo_target.z - 1,
																		make_pair(pseudo_target.z - 1, via_type), net_id);
							if (via_can_hit)
							{
								// path setting
								/*Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z;
								path.Src_Pin = make_pair(pt_x, pt_y);
								path.Tar_Pin = make_pair(Hit_place_x, pt_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + Hit_place_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
								Enc.RT = make_pair(Hit_place_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
								OffGridPath.push_back(path);*/

								Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z - 1;
								path.Src_Pin = make_pair(Hit_place_x, pt_y);
								path.Tar_Pin = make_pair(pt_x, pt_y);
								int width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
								int height = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								via_type_pair = make_pair(pseudo_target.z - 1, via_type);
								/*Parser::wire_path patch;
								patch.path_type = 3;
								patch.Patch.Layer = pseudo_target.z - 1;
								patch.PatchLocate = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first,
																	  pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second);
										
								patch.Patch.LB = make_pair(0, 0);
								patch.Patch.RT = make_pair(Hit_place_x - patch.PatchLocate.first,
															   pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - patch.PatchLocate.second);

								printf("Right::Patch Hit Layer(%d) Locate(%d,%d) , Patch LB(%d,%d) Patch RT(%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second, 
											patch.Patch.LB.first, patch.Patch.LB.second, patch.Patch.RT.first, patch.Patch.RT.second);
*/
#ifdef PseudoPatch
								//OffGridPath.push_back(patch);
#endif
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
								Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
								OffGridPath.push_back(path);

								return true;
							}
						}

						return false;
					}
				}
			}
			// Left Check
			if(pseudo_target.x - 1 >= 0)
			{
				tuple<coor_t, coor_t, int> temp_coor_left;
				temp_coor_left = ISPD_real_coor_table.Index2Coor(pseudo_target.x - 1, pseudo_target.y, pseudo_target.z);

				int pt_x_left = get<0>(temp_coor_left);
				if (pt_x_left <= PinShape.at(i).LB.first && pt_x >= PinShape.at(i).LB.first)
				{
					if (pt_y >= PinShape.at(i).LB.second && pt_y <= PinShape.at(i).RT.second)
					{
						// Left Hit
						int Hit_place_x = (PinShape.at(i).RT.first + PinShape.at(i).LB.first) / 2;
						int width = PinShape.at(i).RT.first - PinShape.at(i).LB.first;

						/*(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
						offGrid_Refine.path_type = 1;
						offGrid_Refine.Layer = pseudo_target.z;
						offGrid_Refine.Src_Pin = make_pair(pt_x, pt_y);
						offGrid_Refine.Tar_Pin = make_pair(Hit_place_x, pt_y);
						offGrid_Refine.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;
						*/
						if ((*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] != NO_BLOCKAGE)
						{
							return false;
						}
						for (int via_type = 0; via_type < this->ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
						{
							bool via_can_hit = OffGridViaEnclosureChecking(pt_x, pt_y, pseudo_target.z, pseudo_target.z - 1,
																		   make_pair(pseudo_target.z - 1, via_type), net_id);
							int width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
							int height = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;

							if (via_can_hit && height < width)
							{
								// path setting
								/*Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z;
								path.Src_Pin = make_pair(pt_x, pt_y);
								path.Tar_Pin = make_pair(Hit_place_x, pt_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + Hit_place_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
								Enc.RT = make_pair(Hit_place_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
*/
								Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z - 1;
								path.Src_Pin = make_pair(Hit_place_x, pt_y);
								path.Tar_Pin = make_pair(pt_x, pt_y);
								if (height < width)
								{
									path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
									via_type_pair = make_pair(pseudo_target.z - 1, via_type);
									Parser::wire_path patch;
									patch.path_type = 3;
									patch.Patch.Layer = pseudo_target.z - 1;
									patch.PatchLocate = make_pair(Hit_place_x,
																  pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second);
									//printf("Patch Hit Layer(%d) (%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second);

									patch.Patch.LB = make_pair(0, 0);
									patch.Patch.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first - patch.PatchLocate.first - (ISPD_MinWidth_list[0]/2),
															   pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - patch.PatchLocate.second);
									//printf("Enclosure LB(%d,%d) RT(%d,%d)\n", ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
									//printf("pt_x(%d) pt_y(%d), Hit_x(%d) Hit_y(%d) \n", pt_x, pt_y, Hit_place_x, pt_y);
									
									//printf("Left::Patch Hit Layer(%d) Locate(%d,%d) , Patch LB(%d,%d) Patch RT(%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second,
									//	   patch.Patch.LB.first, patch.Patch.LB.second, patch.Patch.RT.first, patch.Patch.RT.second);

#ifdef PseudoPatch
									OffGridPath.push_back(patch);
#endif
									Enc.Layer = pseudo_target.z - 1;
									Enc.LB = make_pair(patch.Patch.LB.first + patch.PatchLocate.first, patch.Patch.LB.second + patch.PatchLocate.second);
									Enc.RT = make_pair(patch.Patch.RT.first + patch.PatchLocate.first, patch.Patch.RT.second + patch.PatchLocate.second);
								}
								/*Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
								Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
*/
								OffGridPath.push_back(path);
								return true;
							}
						}
						for (int via_type = 0; via_type < this->ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
						{
							bool via_can_hit = OffGridViaEnclosureChecking(pt_x, pt_y, pseudo_target.z, pseudo_target.z - 1,
																		make_pair(pseudo_target.z - 1, via_type), net_id);
							if (via_can_hit)
							{
								// path setting
								/*Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z;
								path.Src_Pin = make_pair(pt_x, pt_y);
								path.Tar_Pin = make_pair(Hit_place_x, pt_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + Hit_place_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
								Enc.RT = make_pair(Hit_place_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
*/
								Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z - 1;
								path.Src_Pin = make_pair(Hit_place_x, pt_y);
								path.Tar_Pin = make_pair(pt_x, pt_y);
								int width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
								int height = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
									path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
									via_type_pair = make_pair(pseudo_target.z - 1, via_type);
									/*Parser::wire_path patch;
									patch.path_type = 3;
									patch.Patch.Layer = pseudo_target.z - 1;
									patch.PatchLocate = make_pair(Hit_place_x,
																	  pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second);
										//printf("Patch Hit Layer(%d) (%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second);

									patch.Patch.LB = make_pair(0, 0);
									patch.Patch.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first - patch.PatchLocate.first,
																   pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - patch.PatchLocate.second);
									printf("Left::Patch Hit Layer(%d) Locate(%d,%d) , Patch LB(%d,%d) Patch RT(%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second,
								   	patch.Patch.LB.first, patch.Patch.LB.second, patch.Patch.RT.first, patch.Patch.RT.second);
*/
#ifdef PseudoPatch
									//OffGridPath.push_back(patch);
#endif
									Enc.Layer = pseudo_target.z - 1;
									Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
									Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);

									OffGridPath.push_back(path);
									return true;
							}
						}

						return false;
					}
				}
			} // if
			// Lower Check
			if(pseudo_target.y - 1 >= 0 )
			{
				tuple<coor_t, coor_t, int> temp_coor_low;
				temp_coor_low = ISPD_real_coor_table.Index2Coor(pseudo_target.x, pseudo_target.y - 1, pseudo_target.z);

				int pt_y_low = get<1>(temp_coor_low);
				if (pt_y_low <= PinShape.at(i).LB.second && pt_y >= PinShape.at(i).LB.second)
				{
					if (pt_x >= PinShape.at(i).LB.first && pt_x <= PinShape.at(i).RT.first)
					{
						// Low Hit
						int Hit_place_y = (PinShape.at(i).RT.second + PinShape.at(i).LB.second) / 2;
						int width = PinShape.at(i).RT.second - PinShape.at(i).LB.second;

						/*(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
						offGrid_Refine.path_type = 1;
						offGrid_Refine.Layer = pseudo_target.z;
						offGrid_Refine.Src_Pin = make_pair(pt_x, pt_y);
						offGrid_Refine.Tar_Pin = make_pair(pt_x, Hit_place_y);
						offGrid_Refine.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;
						*/

						if ((*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y-1][pseudo_target.x] != NO_BLOCKAGE)
						{
							return false;
						}
						for (int via_type = 0; via_type < this->ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
						{
							bool via_can_hit = OffGridViaEnclosureChecking(pt_x, pt_y, pseudo_target.z, pseudo_target.z - 1,
																		   make_pair(pseudo_target.z - 1, via_type), net_id);
							int width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
							int height = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;

							if (via_can_hit && height > width)
							{
								Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z - 1;
								path.Src_Pin = make_pair(pt_x, Hit_place_y);
								path.Tar_Pin = make_pair(pt_x, pt_y);
								if (height > width)
								{
									path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
									via_type_pair = make_pair(pseudo_target.z - 1, via_type);
									Parser::wire_path patch;
									patch.path_type = 3;
									patch.Patch.Layer = pseudo_target.z - 1;
									patch.PatchLocate = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first,
																  Hit_place_y);
									//printf("Patch Hit Layer(%d) (%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second);

									patch.Patch.LB = make_pair(0, 0);
									patch.Patch.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - patch.PatchLocate.first,
															   pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second - patch.PatchLocate.second - (ISPD_MinWidth_list[0]/2));

									//printf("Enclosure LB(%d,%d) RT(%d,%d)\n", ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
									//printf("pt_x(%d) pt_y(%d), Hit_x(%d) Hit_y(%d) \n", pt_x, pt_y, pt_x, Hit_place_y);
									//printf("Lower::Patch Hit Layer(%d) Locate(%d,%d) , Patch LB(%d,%d) Patch RT(%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second,
									//	   patch.Patch.LB.first, patch.Patch.LB.second, patch.Patch.RT.first, patch.Patch.RT.second);
#ifdef PseudoPatch
									OffGridPath.push_back(patch);
#endif
									Enc.Layer = pseudo_target.z - 1;
									Enc.LB = make_pair(patch.Patch.LB.first + patch.PatchLocate.first, patch.Patch.LB.second + patch.PatchLocate.second);
									Enc.RT = make_pair(patch.Patch.RT.first + patch.PatchLocate.first, patch.Patch.RT.second + patch.PatchLocate.second);
								}
								/*Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
								Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
*/
								OffGridPath.push_back(path);
								return true;
							}
						}

							for (int via_type = 0; via_type < this->ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
							{
								bool via_can_hit = OffGridViaEnclosureChecking(pt_x, pt_y, pseudo_target.z, pseudo_target.z - 1,
																			   make_pair(pseudo_target.z - 1, via_type), net_id);
								if (via_can_hit)
								{
									// path setting
									/*Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z;
								path.Src_Pin = make_pair(pt_x, pt_y);
								path.Tar_Pin = make_pair(pt_x, Hit_place_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + Hit_place_y);
								Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, Hit_place_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
*/
									Parser::wire_path path;
									(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
									path.path_type = 1;
									path.Layer = pseudo_target.z - 1;
									path.Src_Pin = make_pair(pt_x, Hit_place_y);
									path.Tar_Pin = make_pair(pt_x, pt_y);
									int width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
									int height = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
										path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
										via_type_pair = make_pair(pseudo_target.z - 1, via_type);
										/*Parser::wire_path patch;
										patch.path_type = 3;
										patch.Patch.Layer = pseudo_target.z - 1;
										patch.PatchLocate = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first,
																	  Hit_place_y);
										//printf("Patch Hit Layer(%d) (%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second);

										patch.Patch.LB = make_pair(0, 0);
										patch.Patch.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - patch.PatchLocate.first,
																   pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second - patch.PatchLocate.second);
										printf("Lower::Patch Hit Layer(%d) Locate(%d,%d) , Patch LB(%d,%d) Patch RT(%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second,
											   patch.Patch.LB.first, patch.Patch.LB.second, patch.Patch.RT.first, patch.Patch.RT.second);
*/
#ifdef PseudoPatch
										//OffGridPath.push_back(patch);
#endif
										Enc.Layer = pseudo_target.z - 1;
										Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
										Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);

										OffGridPath.push_back(path);
										return true;
								}
							}

							return false;
					}
				}
			} // if
			// Upper Check
			if (pseudo_target.y + 1 < (*ISPD_blockage_array_)[pseudo_target.z].size() )
			{
				tuple<coor_t, coor_t, int> temp_coor_up;
				temp_coor_up = ISPD_real_coor_table.Index2Coor(pseudo_target.x, pseudo_target.y + 1, pseudo_target.z);

				int pt_y_up = get<1>(temp_coor_up);
				if (pt_y <= PinShape.at(i).LB.second && pt_y_up >= PinShape.at(i).LB.second)
				{
					if (pt_x >= PinShape.at(i).LB.first && pt_x <= PinShape.at(i).RT.first)
					{
						// Upper Hit
						int Hit_place_y = (PinShape.at(i).RT.second + PinShape.at(i).LB.second) / 2;
						int width = PinShape.at(i).RT.second - PinShape.at(i).LB.second;

						/*(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
						offGrid_Refine.path_type = 1;
						offGrid_Refine.Layer = pseudo_target.z;
						offGrid_Refine.Src_Pin = make_pair(pt_x, pt_y);
						offGrid_Refine.Tar_Pin = make_pair(pt_x, Hit_place_y);
						offGrid_Refine.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;
						*/
						if ((*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] != NO_BLOCKAGE)
						{
							return false;
						}
						for (int via_type = 0; via_type < this->ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
						{
							bool via_can_hit = OffGridViaEnclosureChecking(pt_x, pt_y, pseudo_target.z, pseudo_target.z - 1,
																   make_pair(pseudo_target.z - 1, via_type), net_id);
							int width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
							int height = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
							if (via_can_hit && height > width)
							{
								Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z - 1;
								path.Src_Pin = make_pair(pt_x, Hit_place_y);
								path.Tar_Pin = make_pair(pt_x, pt_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								via_type_pair = make_pair(pseudo_target.z - 1, via_type);
								Parser::wire_path patch;
								patch.path_type = 3;
								patch.Patch.Layer = pseudo_target.z - 1;
								patch.PatchLocate = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first,
																  pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
									//printf("Patch Hit Layer(%d) (%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second);
								patch.Patch.LB = make_pair(0, 0);
								patch.Patch.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - patch.PatchLocate.first + (ISPD_MinWidth_list[0]/2),
															   Hit_place_y - patch.PatchLocate.second);
								//printf("Enclosure LB(%d,%d) RT(%d,%d)\n", ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
								//printf("pt_x(%d) pt_y(%d), Hit_x(%d) Hit_y(%d) \n", pt_x, pt_y, pt_x, Hit_place_y);
								//printf("Upper::Patch Hit Layer(%d) Locate(%d,%d) , Patch LB(%d,%d) Patch RT(%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second,
								//		   patch.Patch.LB.first, patch.Patch.LB.second, patch.Patch.RT.first, patch.Patch.RT.second);
#ifdef PseudoPatch
								OffGridPath.push_back(patch);
#endif
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(patch.Patch.LB.first + patch.PatchLocate.first, patch.Patch.LB.second + patch.PatchLocate.second);
								Enc.RT = make_pair(patch.Patch.RT.first + patch.PatchLocate.first, patch.Patch.RT.second + patch.PatchLocate.second);
								OffGridPath.push_back(path);
								return true;
							}
						}

						for (int via_type = 0; via_type < this->ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
						{
							bool via_can_hit = OffGridViaEnclosureChecking(pt_x, pt_y, pseudo_target.z, pseudo_target.z - 1,
																		make_pair(pseudo_target.z - 1, via_type), net_id);
							if (via_can_hit)
							{
								// path setting
								/*Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z;
								path.Src_Pin = make_pair(pt_x, pt_y);
								path.Tar_Pin = make_pair(pt_x, Hit_place_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + Hit_place_y);
								Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, Hit_place_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
								OffGridPath.push_back(path);*/

								Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z - 1;
								path.Src_Pin = make_pair(pt_x, Hit_place_y);
								path.Tar_Pin = make_pair(pt_x, pt_y);
								int width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
								int height = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
										path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
										via_type_pair = make_pair(pseudo_target.z - 1, via_type);
										/*Parser::wire_path patch;
										patch.path_type = 3;
										patch.Patch.Layer = pseudo_target.z - 1;
										patch.PatchLocate = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first,
																	  pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
										//printf("Patch Hit Layer(%d) (%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second);
										patch.Patch.LB = make_pair(0, 0);
										patch.Patch.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - patch.PatchLocate.first,
																   Hit_place_y - patch.PatchLocate.second);
										printf("Upper::Patch Hit Layer(%d) Locate(%d,%d) , Patch LB(%d,%d) Patch RT(%d,%d)\n", patch.Layer, patch.PatchLocate.first, patch.PatchLocate.second,
											   patch.Patch.LB.first, patch.Patch.LB.second, patch.Patch.RT.first, patch.Patch.RT.second);
*/
#ifdef PseudoPatch
										//OffGridPath.push_back(patch);
#endif
										Enc.Layer = pseudo_target.z - 1;
										Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
										Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
										OffGridPath.push_back(path);
										return true;
							}
						}

						return false;
					}
				}
			} // if
		}
	}

	return false;
	// We need to avoid these cases if possible
	if (OffGridPath.size() == 0)
	{

		// left upper , left lower , right upper , right lower
		int dis = INT_MAX;
		int pin_index = -1;
		int edge_id = -1;
		for (int i = 0; i < PinShape.size(); i++)
		{
			if (pseudo_target.z - 1 == PinShape[i].Layer - 1)
			{
				int left_edge_x = PinShape[i].LB.first;
				int left_edge_y = (PinShape[i].RT.second + PinShape[i].LB.second) / 2;
				int han_dis_L = abs(left_edge_x - pt_x) + abs(left_edge_y - pt_y);
				if (han_dis_L < dis)
				{
					dis = han_dis_L;
					pin_index = i;
					edge_id = 1;
				}
				int right_edge_x = PinShape[i].RT.first;
				int right_edge_y = (PinShape[i].RT.second + PinShape[i].LB.second) / 2;
				int han_dis_R = abs(right_edge_x - pt_x) + abs(right_edge_y - pt_y);
				if (han_dis_R < dis)
				{
					dis = han_dis_R;
					pin_index = i;
					edge_id = 2;
				}
				int up_edge_x = (PinShape[i].RT.first + PinShape[i].LB.first) / 2;
				int up_edge_y = PinShape[i].RT.second;
				int han_dis_U = abs(up_edge_x - pt_x) + abs(up_edge_y - pt_y);
				if (han_dis_U < dis)
				{
					dis = han_dis_U;
					pin_index = i;
					edge_id = 3;
				}
				int low_edge_x = (PinShape[i].RT.first + PinShape[i].LB.first) / 2;
				int low_edge_y = PinShape[i].LB.second;
				int han_dis_B = abs(low_edge_x - pt_x) + abs(low_edge_y - pt_y);
				if (han_dis_B < dis)
				{
					dis = han_dis_B;
					pin_index = i;
					edge_id = 4;
				}
			}
		}

		if (edge_id == 1)
		{
			int left_edge_x = PinShape[pin_index].LB.first;
			int left_edge_y = (PinShape[pin_index].RT.second + PinShape[pin_index].LB.second) / 2;
			Parser::wire_path wire1;
			wire1.path_type = 0;
			wire1.Layer = pseudo_target.z;
			wire1.Src_Pin = make_pair(pt_x, pt_y);
			wire1.Tar_Pin = make_pair(pt_x, left_edge_y);
			if (pt_y <= PinShape[pin_index].LB.second)
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x + 1] = net_id;
			}
			else
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x + 1] = net_id;
			}

			/*offGrid_Refine.path_type = 1;
			offGrid_Refine.path_type = 1;*/
			int best_via = -1;
			int w = INT_MAX;
			if ((PinShape[pin_index].RT.second - PinShape[pin_index].LB.second) >
				(PinShape[pin_index].RT.first - PinShape[pin_index].LB.first))
			{
				// find less width
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}
			else
			{
				// find less height
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}

			Parser::wire_path wire2;
			wire2.path_type = 1;
			wire2.Layer = pseudo_target.z;
			wire2.Src_Pin = make_pair(pt_x, left_edge_y);
			wire2.Tar_Pin = make_pair(left_edge_x, left_edge_y);
			wire2.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;

			/*Net_wirePath[net_id].push_back(wire1);
			Net_wirePath[net_id].push_back(wire2);*/
		}
		else if (edge_id == 2)
		{
			int right_edge_x = PinShape[pin_index].RT.first;
			int right_edge_y = (PinShape[pin_index].RT.second + PinShape[pin_index].LB.second) / 2;
			Parser::wire_path wire1;
			wire1.path_type = 0;
			wire1.Layer = pseudo_target.z;
			wire1.Src_Pin = make_pair(pt_x, pt_y);
			wire1.Tar_Pin = make_pair(pt_x, right_edge_y);
			if (pt_y <= PinShape[pin_index].LB.second)
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x - 1] = net_id;
			}
			else
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x - 1] = net_id;
			}
			int best_via = -1;
			int w = INT_MAX;
			if ((PinShape[pin_index].RT.second - PinShape[pin_index].LB.second) >
				(PinShape[pin_index].RT.first - PinShape[pin_index].LB.first))
			{
				// find less width
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}
			else
			{
				// find less height
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}

			Parser::wire_path wire2;
			wire2.path_type = 1;
			wire2.Layer = pseudo_target.z;
			wire2.Src_Pin = make_pair(pt_x, right_edge_y);
			wire2.Tar_Pin = make_pair(right_edge_x, right_edge_y);
			wire2.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;

			/*Net_wirePath[net_id].push_back(wire1);
			Net_wirePath[net_id].push_back(wire2);*/
		}
		else if (edge_id == 3)
		{
			int Up_edge_x = (PinShape[pin_index].RT.first + PinShape[pin_index].LB.first) / 2;
			int Up_edge_y = PinShape[pin_index].RT.second;
			Parser::wire_path wire1;
			wire1.path_type = 0;
			wire1.Layer = pseudo_target.z;
			wire1.Src_Pin = make_pair(pt_x, pt_y);
			wire1.Tar_Pin = make_pair(Up_edge_x, pt_y);
			if (pt_x <= PinShape[pin_index].LB.first)
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x + 1] = net_id;
			}
			else
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x - 1] = net_id;
			}

			int best_via = -1;
			int w = INT_MAX;
			if ((PinShape[pin_index].RT.second - PinShape[pin_index].LB.second) >
				(PinShape[pin_index].RT.first - PinShape[pin_index].LB.first))
			{
				// find less width
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}
			else
			{
				// find less height
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}

			Parser::wire_path wire2;
			wire2.path_type = 1;
			wire2.Layer = pseudo_target.z;
			wire2.Src_Pin = make_pair(Up_edge_x, pt_y);
			wire2.Tar_Pin = make_pair(Up_edge_x, Up_edge_y);
			wire2.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;

			/*Net_wirePath[net_id].push_back(wire1);
			Net_wirePath[net_id].push_back(wire2);*/
		}
		else if (edge_id == 4)
		{
			int low_edge_x = (PinShape[pin_index].RT.first + PinShape[pin_index].LB.first) / 2;
			int low_edge_y = PinShape[pin_index].LB.second;
			Parser::wire_path wire1;
			wire1.path_type = 0;
			wire1.Layer = pseudo_target.z;
			wire1.Src_Pin = make_pair(pt_x, pt_y);
			wire1.Tar_Pin = make_pair(low_edge_x, pt_y);
			if (pt_x <= PinShape[pin_index].LB.first)
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x + 1] = net_id;
			}
			else
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x - 1] = net_id;
			}
			int best_via = -1;
			int w = INT_MAX;
			if ((PinShape[pin_index].RT.second - PinShape[pin_index].LB.second) >
				(PinShape[pin_index].RT.first - PinShape[pin_index].LB.first))
			{
				// find less width
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}
			else
			{
				// find less height
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}

			Parser::wire_path wire2;
			wire2.path_type = 1;
			wire2.Layer = pseudo_target.z;
			wire2.Src_Pin = make_pair(low_edge_x, pt_y);
			wire2.Tar_Pin = make_pair(low_edge_x, low_edge_y);
			wire2.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;

			/*Net_wirePath[net_id].push_back(wire1);
			Net_wirePath[net_id].push_back(wire2);*/
		}
		else
		{
			printf("Error of Refine connection");
			exit(1);
		}
	}
}

bool SmallRouter::OffGridViaPrediction_Allow_Short(Node &pseudo_target, const vector<Parser::I_Rect> PinShape, vector<Parser::wire_path> &OffGridPath, Parser::I_Rect &Enc, pair<int ,int> &via_type_pair,int net_id)
{
	tuple<coor_t, coor_t, int> temp_coor;
	temp_coor = ISPD_real_coor_table.Index2Coor(pseudo_target.x, pseudo_target.y, pseudo_target.z);

	int pt_x = get<0>(temp_coor);
	int pt_y = get<1>(temp_coor);
	int pt_z = get<2>(temp_coor);
	for (int i = 0; i < PinShape.size(); i++)
	{
		if (pseudo_target.z - 1 == PinShape[i].Layer - 1)
		{
			// Right Check
			if (pseudo_target.x + 1 < (*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y].size())
			{
				tuple<coor_t, coor_t, int> temp_coor_right;
				temp_coor_right = ISPD_real_coor_table.Index2Coor(pseudo_target.x + 1, pseudo_target.y, pseudo_target.z);
				int pt_x_right = get<0>(temp_coor_right);
				if (pt_x <= PinShape.at(i).LB.first && pt_x_right >= PinShape.at(i).LB.first)
				{
					if (pt_y >= PinShape.at(i).LB.second && pt_y <= PinShape.at(i).RT.second)
					{
						// Right Hit
						int Hit_place_x = (PinShape.at(i).RT.first + PinShape.at(i).LB.first) / 2;
						int width = PinShape.at(i).RT.first - PinShape.at(i).LB.first;
						/*(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
						offGrid_Refine.path_type = 1;
						offGrid_Refine.Layer = pseudo_target.z;
						offGrid_Refine.Src_Pin = make_pair(pt_x, pt_y);
						offGrid_Refine.Tar_Pin = make_pair(Hit_place_x, pt_y);
						offGrid_Refine.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;
						*/
						for (int via_type = 0; via_type < this->ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
						{
							bool via_can_hit = OffGridViaEnclosureChecking(pt_x, pt_y, pseudo_target.z, pseudo_target.z - 1,
																		   make_pair(pseudo_target.z - 1, via_type), net_id);
							if (via_can_hit)
							{
								// path setting
								/*Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z;
								path.Src_Pin = make_pair(pt_x, pt_y);
								path.Tar_Pin = make_pair(Hit_place_x, pt_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + Hit_place_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
								Enc.RT = make_pair(Hit_place_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
								OffGridPath.push_back(path);*/

								Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z - 1;
								path.Src_Pin = make_pair(Hit_place_x, pt_y);
								path.Tar_Pin = make_pair(pt_x, pt_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								via_type_pair = make_pair(pseudo_target.z - 1, via_type);
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
								Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
								OffGridPath.push_back(path);

								return true;
							}
						}
						Parser::wire_path path;
						(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
						path.path_type = 1;
						path.Layer = pseudo_target.z - 1;
						path.Src_Pin = make_pair(Hit_place_x, pt_y);
						path.Tar_Pin = make_pair(pt_x, pt_y);
						path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][0].Name;
						via_type_pair = make_pair(pseudo_target.z - 1, 0);
						//Enc.Layer = pseudo_target.z - 1;
						//Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + Hit_place_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
						//Enc.RT = make_pair(Hit_place_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
						OffGridPath.push_back(path);

						return true;
					}
				}
			}
			// Left Check
			if (pseudo_target.x - 1 >= 0)
			{
				tuple<coor_t, coor_t, int> temp_coor_left;
				temp_coor_left = ISPD_real_coor_table.Index2Coor(pseudo_target.x - 1, pseudo_target.y, pseudo_target.z);

				int pt_x_left = get<0>(temp_coor_left);
				if (pt_x_left <= PinShape.at(i).LB.first && pt_x >= PinShape.at(i).LB.first)
				{
					if (pt_y >= PinShape.at(i).LB.second && pt_y <= PinShape.at(i).RT.second)
					{
						// Left Hit
						int Hit_place_x = (PinShape.at(i).RT.first + PinShape.at(i).LB.first) / 2;
						int width = PinShape.at(i).RT.first - PinShape.at(i).LB.first;

						/*(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
						offGrid_Refine.path_type = 1;
						offGrid_Refine.Layer = pseudo_target.z;
						offGrid_Refine.Src_Pin = make_pair(pt_x, pt_y);
						offGrid_Refine.Tar_Pin = make_pair(Hit_place_x, pt_y);
						offGrid_Refine.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;
						*/

						for (int via_type = 0; via_type < this->ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
						{
							bool via_can_hit = OffGridViaEnclosureChecking(pt_x, pt_y, pseudo_target.z, pseudo_target.z - 1,
																		   make_pair(pseudo_target.z - 1, via_type), net_id);
							if (via_can_hit)
							{
								// path setting
								/*Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z;
								path.Src_Pin = make_pair(pt_x, pt_y);
								path.Tar_Pin = make_pair(Hit_place_x, pt_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + Hit_place_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
								Enc.RT = make_pair(Hit_place_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
*/
								Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z - 1;
								path.Src_Pin = make_pair(Hit_place_x, pt_y);
								path.Tar_Pin = make_pair(pt_x, pt_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								via_type_pair = make_pair(pseudo_target.z - 1, via_type);
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
								Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);

								OffGridPath.push_back(path);
								return true;
							}
						}

						Parser::wire_path path;
						(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
						path.path_type = 1;
						path.Layer = pseudo_target.z - 1;
						path.Src_Pin = make_pair(Hit_place_x, pt_y);
						path.Tar_Pin = make_pair(pt_x, pt_y);
						path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][0].Name;
						via_type_pair = make_pair(pseudo_target.z - 1, 0);
						//Enc.Layer = pseudo_target.z - 1;
						//Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + Hit_place_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
						//Enc.RT = make_pair(Hit_place_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);

						OffGridPath.push_back(path);
						return true;
					}
				}
			} // if
			// Lower Check
			if (pseudo_target.y - 1 >= 0)
			{
				tuple<coor_t, coor_t, int> temp_coor_low;
				temp_coor_low = ISPD_real_coor_table.Index2Coor(pseudo_target.x , pseudo_target.y - 1, pseudo_target.z);

				int pt_y_low = get<1>(temp_coor_low);
				if (pt_y_low <= PinShape.at(i).LB.second && pt_y >= PinShape.at(i).LB.second)
				{
					if (pt_x >= PinShape.at(i).LB.first && pt_x <= PinShape.at(i).RT.first)
					{
						// Low Hit
						int Hit_place_y = (PinShape.at(i).RT.second + PinShape.at(i).LB.second) / 2;
						int width = PinShape.at(i).RT.second - PinShape.at(i).LB.second;

						/*(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
						offGrid_Refine.path_type = 1;
						offGrid_Refine.Layer = pseudo_target.z;
						offGrid_Refine.Src_Pin = make_pair(pt_x, pt_y);
						offGrid_Refine.Tar_Pin = make_pair(pt_x, Hit_place_y);
						offGrid_Refine.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;
						*/

						for (int via_type = 0; via_type < this->ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
						{
							bool via_can_hit = OffGridViaEnclosureChecking(pt_x, pt_y, pseudo_target.z, pseudo_target.z - 1,
																		   make_pair(pseudo_target.z - 1, via_type), net_id);
							if (via_can_hit)
							{
								// path setting
								/*Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z;
								path.Src_Pin = make_pair(pt_x, pt_y);
								path.Tar_Pin = make_pair(pt_x, Hit_place_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + Hit_place_y);
								Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, Hit_place_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
*/
								Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z - 1;
								path.Src_Pin = make_pair(pt_x, Hit_place_y);
								path.Tar_Pin = make_pair(pt_x, pt_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								via_type_pair = make_pair(pseudo_target.z - 1, via_type);
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
								Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);

								OffGridPath.push_back(path);
								return true;
							}
						}

						Parser::wire_path path;
						(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
						path.path_type = 1;
						path.Layer = pseudo_target.z - 1;
						path.Src_Pin = make_pair(pt_x, Hit_place_y);
						path.Tar_Pin = make_pair(pt_x, pt_y);
						path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][0].Name;
						via_type_pair = make_pair(pseudo_target.z - 1, 0);
						//Enc.Layer = pseudo_target.z - 1;
						//Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + Hit_place_y);
						//Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, Hit_place_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);

						OffGridPath.push_back(path);
						return true;
					}
				}
			} // if
			// Upper Check
			if (pseudo_target.y + 1 < (*ISPD_blockage_array_)[pseudo_target.z].size())
			{
				tuple<coor_t, coor_t, int> temp_coor_up;
				temp_coor_up = ISPD_real_coor_table.Index2Coor(pseudo_target.x, pseudo_target.y + 1, pseudo_target.z);

				int pt_y_up = get<1>(temp_coor_up);
				if (pt_y <= PinShape.at(i).LB.second && pt_y_up >= PinShape.at(i).LB.second)
				{
					if (pt_x >= PinShape.at(i).LB.first && pt_x <= PinShape.at(i).RT.first)
					{
						// Upper Hit
						int Hit_place_y = (PinShape.at(i).RT.second + PinShape.at(i).LB.second) / 2;
						int width = PinShape.at(i).RT.second - PinShape.at(i).LB.second;

						/*(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
						offGrid_Refine.path_type = 1;
						offGrid_Refine.Layer = pseudo_target.z;
						offGrid_Refine.Src_Pin = make_pair(pt_x, pt_y);
						offGrid_Refine.Tar_Pin = make_pair(pt_x, Hit_place_y);
						offGrid_Refine.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;
						*/

						for (int via_type = 0; via_type < this->ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
						{
							bool via_can_hit = OffGridViaEnclosureChecking(pt_x, pt_y, pseudo_target.z, pseudo_target.z - 1,
																		   make_pair(pseudo_target.z - 1, via_type), net_id);
							if (via_can_hit)
							{
								// path setting
								/*Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z;
								path.Src_Pin = make_pair(pt_x, pt_y);
								path.Tar_Pin = make_pair(pt_x, Hit_place_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + Hit_place_y);
								Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, Hit_place_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
								OffGridPath.push_back(path);*/

								Parser::wire_path path;
								(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
								path.path_type = 1;
								path.Layer = pseudo_target.z - 1;
								path.Src_Pin = make_pair(pt_x, Hit_place_y);
								path.Tar_Pin = make_pair(pt_x, pt_y);
								path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][via_type].Name;
								via_type_pair = make_pair(pseudo_target.z - 1, via_type);
								Enc.Layer = pseudo_target.z - 1;
								Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + pt_y);
								Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, pt_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
								OffGridPath.push_back(path);
								return true;
							}
						}

						Parser::wire_path path;
						(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
						path.path_type = 1;
						path.Layer = pseudo_target.z - 1;
						path.Src_Pin = make_pair(pt_x, Hit_place_y);
						path.Tar_Pin = make_pair(pt_x, pt_y);
						path.ViaName = ISPD_via_type_list[pseudo_target.z - 1][0].Name;
						via_type_pair = make_pair(pseudo_target.z - 1, 0);
						//Enc.Layer = pseudo_target.z - 1;
						//Enc.LB = make_pair(ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first + pt_x, ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second + Hit_place_y);
						//Enc.RT = make_pair(pt_x + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first, Hit_place_y + ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second);
						OffGridPath.push_back(path);
						return true;
					}
				}
			} // if
		}
	}

	return false;
	// We need to avoid these cases if possible
	if (OffGridPath.size() == 0)
	{

		// left upper , left lower , right upper , right lower
		int dis = INT_MAX;
		int pin_index = -1;
		int edge_id = -1;
		for (int i = 0; i < PinShape.size(); i++)
		{
			if (pseudo_target.z - 1 == PinShape[i].Layer - 1)
			{
				int left_edge_x = PinShape[i].LB.first;
				int left_edge_y = (PinShape[i].RT.second + PinShape[i].LB.second) / 2;
				int han_dis_L = abs(left_edge_x - pt_x) + abs(left_edge_y - pt_y);
				if (han_dis_L < dis)
				{
					dis = han_dis_L;
					pin_index = i;
					edge_id = 1;
				}
				int right_edge_x = PinShape[i].RT.first;
				int right_edge_y = (PinShape[i].RT.second + PinShape[i].LB.second) / 2;
				int han_dis_R = abs(right_edge_x - pt_x) + abs(right_edge_y - pt_y);
				if (han_dis_R < dis)
				{
					dis = han_dis_R;
					pin_index = i;
					edge_id = 2;
				}
				int up_edge_x = (PinShape[i].RT.first + PinShape[i].LB.first) / 2;
				int up_edge_y = PinShape[i].RT.second;
				int han_dis_U = abs(up_edge_x - pt_x) + abs(up_edge_y - pt_y);
				if (han_dis_U < dis)
				{
					dis = han_dis_U;
					pin_index = i;
					edge_id = 3;
				}
				int low_edge_x = (PinShape[i].RT.first + PinShape[i].LB.first) / 2;
				int low_edge_y = PinShape[i].LB.second;
				int han_dis_B = abs(low_edge_x - pt_x) + abs(low_edge_y - pt_y);
				if (han_dis_B < dis)
				{
					dis = han_dis_B;
					pin_index = i;
					edge_id = 4;
				}
			}
		}

		if (edge_id == 1)
		{
			int left_edge_x = PinShape[pin_index].LB.first;
			int left_edge_y = (PinShape[pin_index].RT.second + PinShape[pin_index].LB.second) / 2;
			Parser::wire_path wire1;
			wire1.path_type = 0;
			wire1.Layer = pseudo_target.z;
			wire1.Src_Pin = make_pair(pt_x, pt_y);
			wire1.Tar_Pin = make_pair(pt_x, left_edge_y);
			if (pt_y <= PinShape[pin_index].LB.second)
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x + 1] = net_id;
			}
			else
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x + 1] = net_id;
			}

			/*offGrid_Refine.path_type = 1;
			offGrid_Refine.path_type = 1;*/
			int best_via = -1;
			int w = INT_MAX;
			if ((PinShape[pin_index].RT.second - PinShape[pin_index].LB.second) >
				(PinShape[pin_index].RT.first - PinShape[pin_index].LB.first))
			{
				// find less width
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}
			else
			{
				// find less height
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}

			Parser::wire_path wire2;
			wire2.path_type = 1;
			wire2.Layer = pseudo_target.z;
			wire2.Src_Pin = make_pair(pt_x, left_edge_y);
			wire2.Tar_Pin = make_pair(left_edge_x, left_edge_y);
			wire2.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;

			/*Net_wirePath[net_id].push_back(wire1);
			Net_wirePath[net_id].push_back(wire2);*/
		}
		else if (edge_id == 2)
		{
			int right_edge_x = PinShape[pin_index].RT.first;
			int right_edge_y = (PinShape[pin_index].RT.second + PinShape[pin_index].LB.second) / 2;
			Parser::wire_path wire1;
			wire1.path_type = 0;
			wire1.Layer = pseudo_target.z;
			wire1.Src_Pin = make_pair(pt_x, pt_y);
			wire1.Tar_Pin = make_pair(pt_x, right_edge_y);
			if (pt_y <= PinShape[pin_index].LB.second)
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x - 1] = net_id;
			}
			else
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x - 1] = net_id;
			}
			int best_via = -1;
			int w = INT_MAX;
			if ((PinShape[pin_index].RT.second - PinShape[pin_index].LB.second) >
				(PinShape[pin_index].RT.first - PinShape[pin_index].LB.first))
			{
				// find less width
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}
			else
			{
				// find less height
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}

			Parser::wire_path wire2;
			wire2.path_type = 1;
			wire2.Layer = pseudo_target.z;
			wire2.Src_Pin = make_pair(pt_x, right_edge_y);
			wire2.Tar_Pin = make_pair(right_edge_x, right_edge_y);
			wire2.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;

			/*Net_wirePath[net_id].push_back(wire1);
			Net_wirePath[net_id].push_back(wire2);*/
		}
		else if (edge_id == 3)
		{
			int Up_edge_x = (PinShape[pin_index].RT.first + PinShape[pin_index].LB.first) / 2;
			int Up_edge_y = PinShape[pin_index].RT.second;
			Parser::wire_path wire1;
			wire1.path_type = 0;
			wire1.Layer = pseudo_target.z;
			wire1.Src_Pin = make_pair(pt_x, pt_y);
			wire1.Tar_Pin = make_pair(Up_edge_x, pt_y);
			if (pt_x <= PinShape[pin_index].LB.first)
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x + 1] = net_id;
			}
			else
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x - 1] = net_id;
			}

			int best_via = -1;
			int w = INT_MAX;
			if ((PinShape[pin_index].RT.second - PinShape[pin_index].LB.second) >
				(PinShape[pin_index].RT.first - PinShape[pin_index].LB.first))
			{
				// find less width
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}
			else
			{
				// find less height
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}

			Parser::wire_path wire2;
			wire2.path_type = 1;
			wire2.Layer = pseudo_target.z;
			wire2.Src_Pin = make_pair(Up_edge_x, pt_y);
			wire2.Tar_Pin = make_pair(Up_edge_x, Up_edge_y);
			wire2.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;

			/*Net_wirePath[net_id].push_back(wire1);
			Net_wirePath[net_id].push_back(wire2);*/
		}
		else if (edge_id == 4)
		{
			int low_edge_x = (PinShape[pin_index].RT.first + PinShape[pin_index].LB.first) / 2;
			int low_edge_y = PinShape[pin_index].LB.second;
			Parser::wire_path wire1;
			wire1.path_type = 0;
			wire1.Layer = pseudo_target.z;
			wire1.Src_Pin = make_pair(pt_x, pt_y);
			wire1.Tar_Pin = make_pair(low_edge_x, pt_y);
			if (pt_x <= PinShape[pin_index].LB.first)
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x + 1] = net_id;
			}
			else
			{
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x - 1] = net_id;
			}
			int best_via = -1;
			int w = INT_MAX;
			if ((PinShape[pin_index].RT.second - PinShape[pin_index].LB.second) >
				(PinShape[pin_index].RT.first - PinShape[pin_index].LB.first))
			{
				// find less width
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}
			else
			{
				// find less height
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++)
				{
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
					if (via_width < w)
					{
						best_via = via_type;
						w = via_width;
						;
					}
				}
			}

			Parser::wire_path wire2;
			wire2.path_type = 1;
			wire2.Layer = pseudo_target.z;
			wire2.Src_Pin = make_pair(low_edge_x, pt_y);
			wire2.Tar_Pin = make_pair(low_edge_x, low_edge_y);
			wire2.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;

			/*Net_wirePath[net_id].push_back(wire1);
			Net_wirePath[net_id].push_back(wire2);*/
		}
		else
		{
			printf("Error of Refine connection");
			exit(1);
		}
	}
}

void SmallRouter::OffGridViaRefinment(Node &pseudo_target, vector<Parser::I_Rect> &PinShape, Parser::wire_path &offGrid_Refine /*Must be type 1*/, int net_id)
{

	tuple <coor_t,coor_t,int> temp_coor;
	temp_coor = ISPD_real_coor_table.Index2Coor(pseudo_target.x, pseudo_target.y, pseudo_target.z);
	int pt_x = get<0>(temp_coor);
	int pt_y = get<1>(temp_coor);
	int pt_z = get<2>(temp_coor);
	for (int i = 0; i < PinShape.size(); i++){
		if (pseudo_target.z - 1 == PinShape[i].Layer - 1){
			// Right Check
			tuple<coor_t, coor_t, int> temp_coor_right;
			temp_coor_right = ISPD_real_coor_table.Index2Coor(pseudo_target.x + 1, pseudo_target.y, pseudo_target.z);
			int pt_x_right = get<0>(temp_coor_right);
			if (pt_x <= PinShape.at(i).LB.first && pt_x_right >= PinShape.at(i).LB.first){
				if (pt_y >= PinShape.at(i).LB.second && pt_y <= PinShape.at(i).RT.second){
					// Right Hit
					int Hit_place_x = (PinShape.at(i).RT.first + PinShape.at(i).LB.first) / 2;
					int width = PinShape.at(i).RT.first - PinShape.at(i).LB.first;
					int best_via = -1;
					int w = INT_MAX;
					for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++){
						int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
						if (via_width < w){
							best_via = via_type;
							w = via_width;;
						}
					}
					(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
					offGrid_Refine.path_type = 1;
					offGrid_Refine.Layer = pseudo_target.z;
					offGrid_Refine.Src_Pin = make_pair(pt_x, pt_y);
					offGrid_Refine.Tar_Pin = make_pair(Hit_place_x, pt_y);
					offGrid_Refine.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;
					break;
				}
			}
			// Left Check
			tuple<coor_t, coor_t, int> temp_coor_left;
			temp_coor_left = ISPD_real_coor_table.Index2Coor(pseudo_target.x - 1, pseudo_target.y, pseudo_target.z);
			int pt_x_left = get<0>(temp_coor_left);
			if (pt_x_left <= PinShape.at(i).LB.first && pt_x >= PinShape.at(i).LB.first){
				if (pt_y >= PinShape.at(i).LB.second && pt_y <= PinShape.at(i).RT.second){
					// Left Hit
					int Hit_place_x = (PinShape.at(i).RT.first + PinShape.at(i).LB.first) / 2;
					int width = PinShape.at(i).RT.first - PinShape.at(i).LB.first;
					int best_via = -1;
					int w = INT_MAX;
					for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++){
						int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
						if (via_width < w){
							best_via = via_type;
							w = via_width;;
						}
					}
					(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
					offGrid_Refine.path_type = 1;
					offGrid_Refine.Layer = pseudo_target.z;
					offGrid_Refine.Src_Pin = make_pair(pt_x, pt_y);
					offGrid_Refine.Tar_Pin = make_pair(Hit_place_x, pt_y);
					offGrid_Refine.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;
					break;
				}
			}
			// Lower Check
			tuple<coor_t, coor_t, int> temp_coor_low;
			temp_coor_low = ISPD_real_coor_table.Index2Coor(pseudo_target.x , pseudo_target.y - 1, pseudo_target.z);

			int pt_y_low = get<1>(temp_coor_low);
			if (pt_y_low <= PinShape.at(i).LB.second && pt_y >= PinShape.at(i).LB.second){
				if (pt_x >= PinShape.at(i).LB.first && pt_x <= PinShape.at(i).RT.first){
					// Low Hit
					int Hit_place_y = (PinShape.at(i).RT.second + PinShape.at(i).LB.second) / 2;
					int width = PinShape.at(i).RT.second - PinShape.at(i).LB.second;
					int best_via = -1;
					int w = INT_MAX;
					for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++){
						int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
						if (via_width < w){
							best_via = via_type;
							w = via_width;;
						}
					}
					(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
					offGrid_Refine.path_type = 1;
					offGrid_Refine.Layer = pseudo_target.z;
					offGrid_Refine.Src_Pin = make_pair(pt_x, pt_y);
					offGrid_Refine.Tar_Pin = make_pair(pt_x, Hit_place_y);
					offGrid_Refine.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;
					break;
				}
			}
			// Upper Check
			tuple<coor_t, coor_t, int> temp_coor_up;
			temp_coor_up = ISPD_real_coor_table.Index2Coor(pseudo_target.x, pseudo_target.y + 1, pseudo_target.z);

			int pt_y_up = get<1>(temp_coor_up);
			if (pt_y <= PinShape.at(i).LB.second && pt_y_up >= PinShape.at(i).LB.second){
				if (pt_x >= PinShape.at(i).LB.first && pt_x <= PinShape.at(i).RT.first){
					// Upper Hit
					int Hit_place_y = (PinShape.at(i).RT.second + PinShape.at(i).LB.second) / 2;
					int width = PinShape.at(i).RT.second - PinShape.at(i).LB.second;
					int best_via = -1;
					int w = INT_MAX;
					for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++){
						int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
						if (via_width < w){
							best_via = via_type;
							w = via_width;;
						}
					}
					(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
					offGrid_Refine.path_type = 1;
					offGrid_Refine.Layer = pseudo_target.z;
					offGrid_Refine.Src_Pin = make_pair(pt_x, pt_y);
					offGrid_Refine.Tar_Pin = make_pair(pt_x, Hit_place_y);
					offGrid_Refine.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;
					break;
				}
			}


		}
	}
	// We need to avoid these cases if possible
	if (offGrid_Refine.Layer == UndefineValue){

		// left upper , left lower , right upper , right lower
		int dis = INT_MAX;
		int pin_index = -1;
		int edge_id = -1;
		for (int i = 0; i < PinShape.size(); i++){
			if (pseudo_target.z - 1 == PinShape[i].Layer - 1){
				int left_edge_x = PinShape[i].LB.first;
				int left_edge_y = (PinShape[i].RT.second + PinShape[i].LB.second) / 2;
				int han_dis_L = abs(left_edge_x - pt_x) + abs(left_edge_y - pt_y);
				if (han_dis_L < dis){
					dis = han_dis_L;
					pin_index = i;
					edge_id = 1;
				}
				int right_edge_x = PinShape[i].RT.first;
				int right_edge_y = (PinShape[i].RT.second + PinShape[i].LB.second) / 2;
				int han_dis_R = abs(right_edge_x - pt_x) + abs(right_edge_y - pt_y);
				if (han_dis_R < dis){
					dis = han_dis_R;
					pin_index = i;
					edge_id = 2;
				}
				int up_edge_x = (PinShape[i].RT.first + PinShape[i].LB.first) / 2;
				int up_edge_y = PinShape[i].RT.second;
				int han_dis_U = abs(up_edge_x - pt_x) + abs(up_edge_y - pt_y);
				if (han_dis_U < dis){
					dis = han_dis_U;
					pin_index = i;
					edge_id = 3;
				}
				int low_edge_x = (PinShape[i].RT.first + PinShape[i].LB.first) / 2;
				int low_edge_y = PinShape[i].LB.second;
				int han_dis_B = abs(low_edge_x - pt_x) + abs(low_edge_y - pt_y);
				if (han_dis_B < dis){
					dis = han_dis_B;
					pin_index = i;
					edge_id = 4;
				}
			}
		}


		if (edge_id == 1){
			int left_edge_x = PinShape[pin_index].LB.first;
			int left_edge_y = (PinShape[pin_index].RT.second + PinShape[pin_index].LB.second) / 2;
			Parser::wire_path wire1;
			wire1.path_type = 0;
			wire1.Layer = pseudo_target.z;
			wire1.Src_Pin = make_pair(pt_x, pt_y);
			wire1.Tar_Pin = make_pair(pt_x, left_edge_y);
			if (pt_y <= PinShape[pin_index].LB.second){
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x + 1] = net_id;
			}
			else {
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x + 1] = net_id;
			}

			offGrid_Refine.path_type = 1;
			offGrid_Refine.path_type = 1;
			int best_via = -1;
			int w = INT_MAX;
			if ((PinShape[pin_index].RT.second - PinShape[pin_index].LB.second)
				>
				(PinShape[pin_index].RT.first - PinShape[pin_index].LB.first))
			{
				// find less width
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++){
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
					if (via_width < w){
						best_via = via_type;
						w = via_width;;
					}
				}
			}
			else{
				// find less height
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++){
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
					if (via_width < w){
						best_via = via_type;
						w = via_width;;
					}
				}
			}

			Parser::wire_path wire2;
			wire2.path_type = 1;
			wire2.Layer = pseudo_target.z;
			wire2.Src_Pin = make_pair(pt_x, left_edge_y);
			wire2.Tar_Pin = make_pair(left_edge_x, left_edge_y);
			wire2.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;

			Net_wirePath[net_id].push_back(wire1);
			Net_wirePath[net_id].push_back(wire2);
		}
		else if (edge_id == 2){
			int right_edge_x = PinShape[pin_index].RT.first;
			int right_edge_y = (PinShape[pin_index].RT.second + PinShape[pin_index].LB.second) / 2;
			Parser::wire_path wire1;
			wire1.path_type = 0;
			wire1.Layer = pseudo_target.z;
			wire1.Src_Pin = make_pair(pt_x, pt_y);
			wire1.Tar_Pin = make_pair(pt_x, right_edge_y);
			if (pt_y <= PinShape[pin_index].LB.second){
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x - 1] = net_id;
			}
			else {
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x - 1] = net_id;
			}
			int best_via = -1;
			int w = INT_MAX;
			if ((PinShape[pin_index].RT.second - PinShape[pin_index].LB.second)
					>
					(PinShape[pin_index].RT.first - PinShape[pin_index].LB.first))
			{
				// find less width
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++){
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
					if (via_width < w){
						best_via = via_type;
						w = via_width;;
					}
				}
			}
			else{
				// find less height
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++){
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
					if (via_width < w){
						best_via = via_type;
						w = via_width;;
					}
				}
			}

			Parser::wire_path wire2;
			wire2.path_type = 1;
			wire2.Layer = pseudo_target.z;
			wire2.Src_Pin = make_pair(pt_x, right_edge_y);
			wire2.Tar_Pin = make_pair(right_edge_x, right_edge_y);
			wire2.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;

			Net_wirePath[net_id].push_back(wire1);
			Net_wirePath[net_id].push_back(wire2);
		}
		else if (edge_id == 3){
			int Up_edge_x = (PinShape[pin_index].RT.first + PinShape[pin_index].LB.first) / 2;
			int Up_edge_y = PinShape[pin_index].RT.second;
			Parser::wire_path wire1;
			wire1.path_type = 0;
			wire1.Layer = pseudo_target.z;
			wire1.Src_Pin = make_pair(pt_x, pt_y);
			wire1.Tar_Pin = make_pair(Up_edge_x, pt_y);
			if (pt_x <= PinShape[pin_index].LB.first){
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x + 1] = net_id;
			}
			else {
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y - 1][pseudo_target.x - 1] = net_id;
			}
			
			int best_via = -1;
			int w = INT_MAX;
			if ((PinShape[pin_index].RT.second - PinShape[pin_index].LB.second)
					>
					(PinShape[pin_index].RT.first - PinShape[pin_index].LB.first))
			{
				// find less width
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++){
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
					if (via_width < w){
						best_via = via_type;
						w = via_width;;
					}
				}
			}
			else{
				// find less height
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++){
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
					if (via_width < w){
						best_via = via_type;
						w = via_width;;
					}
				}
			}

			Parser::wire_path wire2;
			wire2.path_type = 1;
			wire2.Layer = pseudo_target.z;
			wire2.Src_Pin = make_pair(Up_edge_x, pt_y);
			wire2.Tar_Pin = make_pair(Up_edge_x, Up_edge_y);
			wire2.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;

			Net_wirePath[net_id].push_back(wire1);
			Net_wirePath[net_id].push_back(wire2);
		}
		else if (edge_id == 4){
			int low_edge_x = (PinShape[pin_index].RT.first + PinShape[pin_index].LB.first) / 2;
			int low_edge_y = PinShape[pin_index].LB.second;
			Parser::wire_path wire1;
			wire1.path_type = 0;
			wire1.Layer = pseudo_target.z;
			wire1.Src_Pin = make_pair(pt_x, pt_y);
			wire1.Tar_Pin = make_pair(low_edge_x, pt_y);
			if (pt_x <= PinShape[pin_index].LB.first){
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x + 1] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x + 1] = net_id;
			}
			else {
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y][pseudo_target.x - 1] = net_id;
				(*ISPD_blockage_array_)[pseudo_target.z][pseudo_target.y + 1][pseudo_target.x - 1] = net_id;
			}
			int best_via = -1;
			int w = INT_MAX;
			if ((PinShape[pin_index].RT.second - PinShape[pin_index].LB.second)
					>
					(PinShape[pin_index].RT.first - PinShape[pin_index].LB.first))
			{
				// find less width
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++){
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.first;
					if (via_width < w){
						best_via = via_type;
						w = via_width;;
					}
				}
			}
			else{
				// find less height
				for (int via_type = 0; via_type < ISPD_via_type_list[pseudo_target.z - 1].size(); via_type++){
					int via_width = ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[pseudo_target.z - 1][via_type].BotLayerIRect.LB.second;
					if (via_width < w){
						best_via = via_type;
						w = via_width;;
					}
				}
			}

			Parser::wire_path wire2;
			wire2.path_type = 1;
			wire2.Layer = pseudo_target.z;
			wire2.Src_Pin = make_pair(low_edge_x, pt_y);
			wire2.Tar_Pin = make_pair(low_edge_x, low_edge_y);
			wire2.ViaName = ISPD_via_type_list[pseudo_target.z - 1][best_via].Name;

			Net_wirePath[net_id].push_back(wire1);
			Net_wirePath[net_id].push_back(wire2);
		}
		else{
			printf("Error of Refine connection");
			exit(1);
		}

	}
}

bool FindSameNode(const vector <Pin> &True, Node &src){
	for (int i = 0; i < True.size(); i++){
		int x = True[i].pin_position.pin_point.x;
		int y = True[i].pin_position.pin_point.y;
		int z = True[i].pin_position.pin_point.z;
		if (src.x == x && src.y == y && src.y == y)
			return true;
	}
	return false;
}



int SmallRouter::BestVia(Parser::I_Rect PinShape, Node &src){
	int best_via = -1;
	int w = INT_MAX;
	if ((PinShape.RT.second - PinShape.LB.second)
	>
	(PinShape.RT.first - PinShape.LB.first))
	{
		// find less width
		for (int via_type = 0; via_type < ISPD_via_type_list[src.z - 1].size(); via_type++){
			int via_width = ISPD_via_type_list[src.z - 1][via_type].BotLayerIRect.RT.first - ISPD_via_type_list[src.z - 1][via_type].BotLayerIRect.LB.first;
			if (via_width < w){
				best_via = via_type;
				w = via_width;;
			}
		}
	}
	else{
		// find less height
		for (int via_type = 0; via_type < ISPD_via_type_list[src.z - 1].size(); via_type++){
			int via_width = ISPD_via_type_list[src.z - 1][via_type].BotLayerIRect.RT.second - ISPD_via_type_list[src.z - 1][via_type].BotLayerIRect.LB.second;
			if (via_width < w){
				best_via = via_type;
				w = via_width;;
			}
		}
	}
}

void SmallRouter::SetValidSourcePin(
	const TwoPinNetConnection &connection,
	map<tuple<int, int, int>, Parser::PathAndEnc> &RefinementTable,
	vector<Node> &maze_queue, int two_pin_net_id, bool SrcIsIRoute)
{
	/*
	for (auto &pinpin : connection.Src_Pin_Shape)
	{
		printf("V2 Src ->(%d,%d) (%d,%d)\n", pinpin.LB.first, pinpin.LB.second, pinpin.RT.first, pinpin.RT.second);
	}
	*/

	for (int i = 0; i < connection.Source_pin_path_list.size(); i++)
	{
		int x = connection.Source_pin_path_list[i].pin_position.pin_point.x;
		int y = connection.Source_pin_path_list[i].pin_position.pin_point.y;
		int z = connection.Source_pin_path_list[i].pin_position.pin_point.z;

		if ((*ISPD_blockage_array_)[z][y][x] != NO_BLOCKAGE && (*ISPD_blockage_array_)[z][y][x] != two_pin_net_id){
			//printf("Node (%d,%d,%d) Net(%d) CurNet(%d)\n", x, y, z, (*ISPD_blockage_array_)[z][y][x], two_pin_net_id);
			continue;
		}

		if (SrcIsIRoute){
			ISPD_cost_array_[z][y][x] = 0;
			ISPD_back_track_array_[z][y][x] = kNoDirection;
			ISPD_back_track_Step_array_[z][y][x] = 0;
			Node node(x,y,z,0.0);
			auto temp_coor = ISPD_real_coor_table.Index2Coor(x, y, z);
			printf("Source IRoute (%d,%d,%d) coor(%d,%d,%d)\n", x, y, z, get<0>(temp_coor), get<1>(temp_coor), get<2>(temp_coor));
			maze_queue.push_back(node);
		}
		else{
			tuple<int, int, int> node = make_tuple(x, y, z);
			auto find_source = RefinementTable.find(node);
			if (find_source != RefinementTable.end())
			{
				
				/*bool Dummy_Left = false;
				if (x - 1 >= 0){
					Dummy_Left = ISPD_ViaHit_array_[z][y][x - 1];
					if (Dummy_Left){
						x = x - 1;
						ISPD_cost_array_[z][y][x] = 0;
						ISPD_back_track_array_[z][y][x] = kNoDirection;
						ISPD_back_track_Step_array_[z][y][x] = 0;
						Node node(x, y, z, 0.0);
						maze_queue.push_back(node);
						continue;
					}
				}
				bool Dummy_Right = false;
				if (x + 1 < (*ISPD_GridMap_layout)[z][y].size()){
					Dummy_Right = ISPD_ViaHit_array_[z][y][x + 1];
					if (Dummy_Right){
						x = x + 1;
						ISPD_cost_array_[z][y][x] = 0;
						ISPD_back_track_array_[z][y][x] = kNoDirection;
						ISPD_back_track_Step_array_[z][y][x] = 0;
						Node node(x, y, z, 0.0);
						maze_queue.push_back(node);
						continue;
					}
				}
				bool Dummy_Up = false;
				if (y - 1 >= 0){
					Dummy_Up = ISPD_ViaHit_array_[z][y - 1][x];
					if (Dummy_Up)
					{
						y = y - 1;
						ISPD_cost_array_[z][y][x] = 0;
						ISPD_back_track_array_[z][y][x] = kNoDirection;
						ISPD_back_track_Step_array_[z][y][x] = 0;
						Node node(x, y, z, 0.0);
						maze_queue.push_back(node);
						continue;
					}
				}
					
				bool Dummy_Down = false;
				if (y + 1 < (*ISPD_GridMap_layout)[z].size()){
					Dummy_Down = ISPD_ViaHit_array_[z][y + 1][x];
					if (Dummy_Down)
					{
						y = y + 1;
						ISPD_cost_array_[z][y][x] = 0;
						ISPD_back_track_array_[z][y][x] = kNoDirection;
						ISPD_back_track_Step_array_[z][y][x] = 0;
						Node node(x, y, z, 0.0);
						maze_queue.push_back(node);
						continue;
					}
				}*/
				auto temp_coor = ISPD_real_coor_table.Index2Coor(x,y,z);
				printf("Source Pin (%d,%d,%d) coor(%d,%d,%d)\n", x, y, z, get<0>(temp_coor), get<1>(temp_coor), get<2>(temp_coor));
				ISPD_cost_array_[z][y][x] = 0;
				ISPD_back_track_array_[z][y][x] = kNoDirection;
				ISPD_back_track_Step_array_[z][y][x] = 0;
				ISPD_modify_list.push_back(make_tuple(x, y, z));
				Node node(x, y, z, 0.0);
				maze_queue.push_back(node);
			}
		}
	}
}

bool SmallRouter::NodeInBoundingBox(const vector<Parser::I_Rect> &BoundingBox, Node &next)
{
	auto temp_coor = ISPD_real_coor_table.Index2Coor(next.x, next.y, next.z);
	for (int i = 0; i < BoundingBox.size(); i++)
	{
		if (get<0>(temp_coor) >= BoundingBox[i].LB.first && get<0>(temp_coor) <= BoundingBox[i].RT.first && get<1>(temp_coor) >= BoundingBox[i].LB.second && get<1>(temp_coor) <= BoundingBox[i].RT.second)
		{
			return true;
		}
	}
	return false;
}

void SmallRouter::SettingBoundBoxGuide(const vector<vector<Parser::I_Rect>> &Layer_Global_guide, vector<Node> maze_queue, vector<Parser::I_Rect> &BB)
{
	for (int pin = 0; pin < maze_queue.size(); pin++)
	{
		int x = maze_queue[pin].x;
		int y = maze_queue[pin].y;
		int z = maze_queue[pin].z;
		auto temp_coor = ISPD_real_coor_table.Index2Coor(x,y,z);
		coor_t coor_x = get<0>(temp_coor);
		coor_t coor_y = get<1>(temp_coor);

		for (int i = 0; i < Layer_Global_guide[z].size(); i++)
		{
			if ((coor_x >= Layer_Global_guide[z][i].LB.first && coor_x <= Layer_Global_guide[z][i].RT.first))
			{
				if ((coor_y >= Layer_Global_guide[z][i].LB.second && coor_y <= Layer_Global_guide[z][i].RT.second))
				{
					BB.push_back(Layer_Global_guide[z][i]);
					break;
				}
			}
		}
	}
}

void SmallRouter::ResetFunction()
{
	for (int m = 0; m < ISPD_modify_list.size();m++){
		int i = get<2>(ISPD_modify_list[m]);
		int j = get<1>(ISPD_modify_list[m]);
		int k = get<0>(ISPD_modify_list[m]);
		ISPD_cost_array_[i][j][k] = FLT_MAX;
		ISPD_back_track_array_[i][j][k] = kNoDirection;
		ISPD_back_track_Step_array_[i][j][k] = -1;
		this->ISPD_Viatype_array_[i][j][k].clear();
#ifdef NCTU_GR_HEAP
		ISPD_Fib_Node_Location[i][j][k] = NULL;
		ISPD_Node_In_Heap[i][j][k] = false;
#endif
	}
}

/*bool SmallRouter::FastInGuide(Node &next){
	return ISPD_Guide_Map[next.z][next.y][next.x];
}*/

int SmallRouter::ISPD_MazeRouting3D(
	const int two_pin_net_id,
	const int two_pin_net_index,
	const TwoPinNetConnection &connection,
	const vector<Parser::I_Rect> &GlobalGuide,
	const Parser::I_Rect &BoundingBox,
	Node &target_point,
	map<tuple<int, int, int>, Parser::PathAndEnc> SourceRefinmentMap,
	map<Node, Parser::PathAndEnc> &TargetRefinmentMap,
	map<tuple<int, int, int>, Parser::PathAndEnc> &RefinmentMap,
	bool SrcIsIRoute, vector<Pin> &ValidPinList,
	Parser::I_Rect &Target_Box)
{
	// Init Setting
	//clock_t init_start = clock();
#ifdef NCTU_GR_HEAP
	//FibonacciHeap <Node> Fib_heap;
	//pheap<Node> Fib_heap;
	Fib_heap.initial() ;
	vector<Node>
		maze_queue;
	maze_queue.clear();
#else
	vector<Node> maze_queue;
	maze_queue.clear();
#endif // FIB_HEAP

	ResetFunction();
	ISPD_modify_list.clear();

	printf("Target Box LB(%d,%d) RT(%d,%d)\n", Target_Box.LB.first, Target_Box.LB.second, Target_Box.RT.first,Target_Box.RT.second);
	//printf("Src_Pin_shape size: %d\n", connection.Src_Pin_Shape.size());
	vector<Parser::I_Rect> NetShape;
	//map< Node, vector<Parser::wire_path> > PseudoSource_PathTable;
	//printf("source\n");
	// Pin Process No need to do now
	if (connection.src_type == 0){
		//printf("ISPD_MazeRouting3D:: Source Pin Setting\n");
		int x = connection.source.pin_position.pin_point.x;
		int y = connection.source.pin_position.pin_point.y;
		int z = connection.source.pin_position.pin_point.z;
		ISPD_cost_array_[z][y][x] = 0;
		ISPD_back_track_array_[z][y][x] = kNoDirection;
		Node node_src(x, y, z);
		ISPD_modify_list.push_back(make_tuple(x,y,z));
		maze_queue.push_back(node_src);
		printf("Set Source Pin Error, src type\n");
		exit(1);
	}
	else{
		SetValidSourcePin(connection, RefinmentMap, maze_queue, two_pin_net_id, SrcIsIRoute);
	}

	ValidPinList.clear();

	for (int i = 0; i < maze_queue.size(); i++)
	{
		
		int x = maze_queue[i].x;
		int y = maze_queue[i].y;
		int z = maze_queue[i].z;
		//printf("過濾清單:: (%d,%d,%d)\n",x,y,z);
		Pin pin;
		pin.SetPinPoint(kPinPoint,x,y,z);
		ValidPinList.push_back(pin);
	}


	// Assume the path can be found first.
	rip_up_two_pin_nets_[two_pin_net_index].path_is_found = true;

	//printf("ISPD_MazeRouting3D:: Setting Layer-based GlobalGuide\n");
	vector < vector <Parser::I_Rect> > Layer_Global_guide;
	SettingLayerGlobalGuide(GlobalGuide, Layer_Global_guide,this->layout_layer_);
	
	map<Node, int> TargetTable;

	if (maze_queue.empty())
	{
		printf("Error::source pin empty\n");
		return -2;
	}

	printf("Push Source Node Into Fib_Heap\n");
	for (int i = 0; i < maze_queue.size(); i++)
	{
		if (this->ISPD_Target_Map[maze_queue[i].z][maze_queue[i].y][maze_queue[i].x]){
			target_point.SetNode(
				maze_queue[i].x,
				maze_queue[i].y,
				maze_queue[i].z);
			return 0;
		}
#ifdef NCTU_GR_HEAP

		//ISPD_Fib_Node_Location[maze_queue[i].z][maze_queue[i].y][maze_queue[i].x] = Fib_heap.insert(maze_queue[i]);
		ISPD_Fib_Node_Location[maze_queue[i].z][maze_queue[i].y][maze_queue[i].x] = new pheap_el<Node>;
		ISPD_Fib_Node_Location[maze_queue[i].z][maze_queue[i].y][maze_queue[i].x]->data = &maze_queue[i];
		Fib_heap.insert(ISPD_Fib_Node_Location[maze_queue[i].z][maze_queue[i].y][maze_queue[i].x]);
		ISPD_Node_In_Heap[maze_queue[i].z][maze_queue[i].y][maze_queue[i].x] = true;

#endif // NCTU_GR_HEAP
	}
	printf("Push Source Node Into Fib_Heap Successful\n");
#ifndef NCTU_GR_HEAP
	make_heap(maze_queue.begin(), maze_queue.end(), GreaterMazeNode());
#endif
	Node pre_node;
	int max_queue_size = 0;
	bool reach_target = false;
	while (!reach_target)
	{

		// The path of the net can't be found and built!!!
#ifdef NCTU_GR_HEAP
		/*if (Fib_heap.isEmpty()){
			rip_up_two_pin_nets_[two_pin_net_index].path_is_found = false;
			printf("Error::Path Not Found\n");
			return -1;
		}*/
		if (Fib_heap.root == NULL)
		{
			rip_up_two_pin_nets_[two_pin_net_index].path_is_found = false;
			printf("Error::Path Not Found\n");
			return -1;
		}
#else
		if (maze_queue.empty())
		{
			rip_up_two_pin_nets_[two_pin_net_index].path_is_found = false;
			printf("Error::Path Not Found\n");
			return -1;
		}
#endif

				// explore the node
#ifdef NCTU_GR_HEAP
		//Node current_node = Fib_heap.removeMinimum();
		Node current_node = *((Fib_heap.extract())->data); // NCTU_GR
#else
		/*if (maze_queue.size() > max_queue_size)
			max_queue_size = maze_queue.size();*/
		Node current_node(maze_queue.front());
		pop_heap(maze_queue.begin(), maze_queue.end(), GreaterMazeNode());
		maze_queue.pop_back();
#endif // FIB_HEAP

		// Here to check current node
		int cur_layer = current_node.z;
		int cur_x = current_node.x;
		int cur_y = current_node.y;

		//printf("current node : (%d,%d,%d)  cost : %.2f\n", cur_x, cur_y, cur_layer, current_node.cost);
		if (current_node.cost == 0){
			if (pre_node.x == cur_x && pre_node.y == cur_y &&pre_node.z == cur_layer) return -1;

			else
				pre_node.SetNode(cur_x, cur_y, cur_layer);
		}
		for (int dir_index = kRight; dir_index <= kDown; dir_index++)
		{
			const int direction = dir_index;

			// the neighbor of the current node
			Node next = ToNextNode(current_node, direction, Layer_Global_guide);
			if (current_node.z == next.z && current_node.x == next.x && current_node.y == next.y){
				continue;
			}
			if (next.x == -1 || next.y == -1){
				continue;
			}
			//printf("next node : (%d,%d,%d) \n", next.x, next.y, next.z);
			// Reject M1 Routing will make case simpler
#ifdef REJECT_M1_ROUTING
			if (next.z == 0){
				continue;
			}
#endif
			if (!NodeInBoundingBox(GlobalGuide, next))
				continue;

				//const ULL next_node_index = Transform3dArrayTo1dIndex(next_node);
				const bool via_direction =
					(direction == kUp || direction == kDown);
				Node cur_node(cur_x, cur_y, cur_layer);

				// Move In Plane
				if (!via_direction)
				{
					const bool is_node_blocked = ISPD_IsNodeBlocked(cur_node, next, two_pin_net_id, direction, 0, NetShape);
					//IsNodeBlocked(next_node, bounding_box, two_pin_net_id, direction);

				if (!is_node_blocked)
				{
					ISPD_modify_list.push_back(make_tuple(next.x, next.y, next.z));

					float next_node_temp_cost = ISPD_UpdateCost(
						current_node, next,
						direction, via_direction,
						two_pin_net_id, two_pin_net_index, Layer_Global_guide, TargetTable, RefinmentMap, Target_Box); // = Cost function
					if (ISPD_cost_array_[next.z][next.y][next.x] > next_node_temp_cost)
					{
						ISPD_cost_array_[next.z][next.y][next.x] = next_node_temp_cost;
						ISPD_back_track_array_[next.z][next.y][next.x] = direction;
						
						//Node *next_node = new Node(next, next_node_temp_cost);
						Node next_node(next,next_node_temp_cost);
#ifdef NCTU_GR_HEAP
						//printf("push heap: (%d,%d,%d)  cost : %.2f\n", next_node->x, next_node->y, next_node->z, next_node->cost);
						//node<Node> *temp = ISPD_Fib_Node_Location[next_node.z][next_node.y][next_node.x];
						//pheap_el<Node> *temp = ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x];
						if (temp != NULL)
						{ // node is in heap, just decrease key
							//Fib_heap.decreaseKey(temp, next_node);
							Fib_heap.decreaseKey(temp, next_node_temp_cost);
						}
						else
						{ // node is not in heap, insert new node
							//ISPD_Fib_Node_Location[next_node.z][next_node.y][next_node.x] = Fib_heap.insert(next_node);
							//ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x] = new pheap_el<Node>;
							//printf("push heap: (%d,%d,%d)  cost : %.2f\n", next_node.x, next_node.y, next_node.z, next_node.cost);
							ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x]->data = next_node;
							//printf("push heap: (%d,%d,%d)  cost : %.2f\n", next_node.x, next_node.y, next_node.z, next_node.cost);
							Fib_heap.insert(ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x]);
						}
						//Fib_heap.insert(next_node);
						//printf("Push successful\n");
#else
						maze_queue.push_back(next_node);
						push_heap(
							maze_queue.begin(),
							maze_queue.end(),
							GreaterMazeNode());
#endif // FIB_HEAP
						
					}
				}
			}
			// Move Up/Down with via
			else{
				for (int via_type = 0; via_type < this->ISPD_via_type_list[min(cur_layer, (int)next.z)].size(); via_type++){
				//	printf("%d\n", via_type);
					const bool is_node_blocked = ISPD_IsNodeBlocked(cur_node, next, two_pin_net_id, direction, via_type, NetShape);
					if (!is_node_blocked)
					{
						ISPD_modify_list.push_back(make_tuple(next.x, next.y, next.z));
						//Node next_node(next, 0.0);
						//printf("%d\n", via_type);
						float next_node_temp_cost = ISPD_UpdateCost(
							current_node, next,
							direction, via_direction,
							two_pin_net_id, two_pin_net_index, Layer_Global_guide, TargetTable, RefinmentMap, Target_Box); // = Cost function
						// Via priority
						next_node_temp_cost += via_type;
						if (ISPD_cost_array_[next.z][next.y][next.x] > next_node_temp_cost)
						{
							ISPD_cost_array_[next.z][next.y][next.x] = next_node_temp_cost;
							ISPD_back_track_array_[next.z][next.y][next.x] = direction;
							Node next_node(next,next_node_temp_cost);

							// Setting valid via in Initial route
							if (via_direction){
								if (next.z > cur_layer){
									ISPD_Viatype_array_[cur_layer][cur_y][cur_x].RaisingVia = make_pair(min(cur_layer, (int)next.z), via_type);
									ISPD_Viatype_array_[next.z][next.y][next.x].FallingVia = make_pair(min(cur_layer, (int)next.z), via_type);
								}
								else{
									ISPD_Viatype_array_[cur_layer][cur_y][cur_x].FallingVia = make_pair(min(cur_layer, (int)next.z), via_type);
									ISPD_Viatype_array_[next.z][next.y][next.x].RaisingVia = make_pair(min(cur_layer, (int)next.z), via_type);
								}
							}

#ifdef NCTU_GR_HEAP
							//printf("push heap: (%d,%d,%d)  cost : %.2f\n", next_node->x, next_node->y, next_node->z, next_node->cost);
							//node<Node> *temp = ISPD_Fib_Node_Location[next_node.z][next_node.y][next_node.x];
							//pheap_el<Node> *temp = ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x];
							if (temp != NULL)
							{ // node is in heap, just decrease key
								//Fib_heap.decreaseKey(temp, next_node);
								Fib_heap.decreaseKey(temp, next_node_temp_cost);
							}
							else
							{ // node is not in heap, insert new node
								//ISPD_Fib_Node_Location[next_node.z][next_node.y][next_node.x] = Fib_heap.insert(next_node);
								//ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x] = new pheap_el<Node>;
								//printf("push heap: (%d,%d,%d)  cost : %.2f\n", next_node.x, next_node.y, next_node.z, next_node.cost);
								ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x]->data = next_node;

								Fib_heap.insert(ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x]);
							}
							//Fib_heap.insert(next_node);
							//printf("Push successful\n");
#else
							maze_queue.push_back(next_node);
							push_heap(
								maze_queue.begin(),
								maze_queue.end(),
								GreaterMazeNode());
#endif // FIB_HEAP
						}
						break; // This Break is important
					}
				}
			}


		}
#ifdef NCTU_GR_HEAP
		//if (Fib_heap.isEmpty())
			//continue;
		if (Fib_heap.root == NULL)
			continue;
			// Target Find

		//Node mn = Fib_heap.getMinimum();
		Node mn = *(Fib_heap.root->data);
#else
		Node mn = maze_queue.front();
#endif // FIB_HEAP

		Node n(mn.x,mn.y,mn.z);
		if (TargetTableFind(n, SrcIsIRoute, two_pin_net_id, Target_Box)) // find target
		{
			printf("Target (%d,%d,%d)\n", mn.x, mn.y, mn.z);
			target_point.SetNode(mn.x, mn.y, mn.z);
			reach_target = true;
		
		}

	}

#ifdef NCTU_GR_HEAP
	Fib_heap.memory_clear();
#endif // FIB_HEAP
	//printf("Max Maze Queue Size : %d\n", max_queue_size);
	//printf("Final Maze Queue Size : %d\n", maze_queue.size());

	return 0;
}
void SmallRouter::SetSourcePin_Allow_Short_v2(const TwoPinNetConnection &connection,
										   map<tuple<int,int,int>, Parser::PathAndEnc> &Source_PathTable,
										   vector<Node> &maze_queue, int two_pin_net_id, bool isIroute)
{

	for (int i = 0; i < connection.Source_pin_path_list.size(); i++)
	{
		int x = connection.Source_pin_path_list[i].pin_position.pin_point.x;
		int y = connection.Source_pin_path_list[i].pin_position.pin_point.y;
		int z = connection.Source_pin_path_list[i].pin_position.pin_point.z;
		tuple<coor_t, coor_t, int> temp_coor;
		temp_coor = ISPD_real_coor_table.Index2Coor(x,y,z);
		/*if ((*ISPD_blockage_array_)[z][y][x] != NO_BLOCKAGE && (*ISPD_blockage_array_)[z][y][x] != two_pin_net_id)
			continue;*/
		Node node_src(x, y, z);
		Node node_nex(x, y, z - 1);
		bool SourceOK = true;
		bool find_pseudo = FindSameNode(connection.pseudo_Source_pin_path_list, node_src);
		bool find_true = FindSameNode(connection.real_Source_pin_path_list, node_src);
		if (find_true)
		{
			// true source
			Node real_pin_position(get<0>(temp_coor), get<1>(temp_coor), z);

			Parser::I_Rect shape = WhichPinShape(connection.Src_Pin_Shape, real_pin_position);
			int best_via = BestVia(shape, node_src);
			PathAndEnc PAE;
			Parser::wire_path path;
			path.path_type = 2;
			path.Layer = z - 1;
			path.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
			path.ViaName = ISPD_via_type_list[z - 1][best_via].Name;
			PAE.path.push_back(path);
			PAE.Enclosure = ISPD_via_type_list[z - 1][best_via].BotLayerIRect;
			PAE.via_type = make_pair(z - 1, best_via);
			// emplace to map
			tuple<int, int, int> refine_node = make_tuple(x, y, z);
			Source_PathTable.emplace(refine_node, PAE);
		}
		else if (find_pseudo)
		{
			// pseudo source
			Parser::PathAndEnc PAE;
			bool valid_node = OffGridViaPrediction_Allow_Short(node_src, connection.Src_Pin_Shape, PAE.path, PAE.Enclosure, PAE.via_type, two_pin_net_id);
			if (valid_node)
			{
				//printf("PseudoSource_PathTable:: emplace node(%d,%d,%d)\n", node_src.x, node_src.y, node_src.z);
				tuple<int, int, int> refine_node = make_tuple(x, y, z);
				Source_PathTable.emplace(refine_node, PAE);
				SourceOK = true;
			}
			else
			{
				auto DG = Source_PathTable.find(make_tuple(x,y,z));
				if (DG != Source_PathTable.end()){
					SourceOK = true;
				}
				else{
					continue;
				}
			}
		}
		if (SourceOK)
		{
			ISPD_modify_list.push_back(make_tuple(x,y,z));
			ISPD_cost_array_[z][y][x] = 0;
			ISPD_back_track_array_[z][y][x] = kNoDirection;
			ISPD_back_track_Step_array_[z][y][x] = 0;
			Node node(x,y,z,0.0);

			printf("Short::Source Pin (%d,%d,%d) coor(%d,%d,%d)\n", x, y, z, get<0>(temp_coor), get<1>(temp_coor), get<2>(temp_coor));
			maze_queue.push_back(node);
		}
	}
}

void SmallRouter::SetSourcePin_Allow_Short(const TwoPinNetConnection &connection,
											   map<Node, Parser::PathAndEnc> &PseudoSource_PathTable,
											   vector<Node> &maze_queue, int two_pin_net_id, bool isIroute)
{
	/*for (int i = 0; i < connection.Src_Pin_Shape.size(); i++){
		printf("SrcPin :: LB(%d,%d) RT(%d,%d)\n", connection.Src_Pin_Shape[i].LB.first, connection.Src_Pin_Shape[i].LB.second,
			   connection.Src_Pin_Shape[i].RT.first, connection.Src_Pin_Shape[i].RT.second);
	}*/
	for (int i = 0; i < connection.Source_pin_path_list.size(); i++)
	{
		int x = connection.Source_pin_path_list[i].pin_position.pin_point.x;
		int y = connection.Source_pin_path_list[i].pin_position.pin_point.y;
		int z = connection.Source_pin_path_list[i].pin_position.pin_point.z;

		/*if ((*ISPD_blockage_array_)[z][y][x] != NO_BLOCKAGE && (*ISPD_blockage_array_)[z][y][x] != two_pin_net_id)
			continue;*/
		Node node_src(x, y, z);
		Node node_nex(x, y, z - 1);
		bool SourceOK = true;
		bool find_pseudo = FindSameNode(connection.pseudo_Source_pin_path_list, node_src);
		bool find_true = FindSameNode(connection.real_Source_pin_path_list, node_src);
		if (find_true)
		{
			// true source
			Parser::I_Rect shape = WhichPinShape(connection.Src_Pin_Shape, node_src);
			SourceOK = false;
			int best_via = BestVia(shape, node_src);
			ISPD_Viatype_array_[z][y][x].FallingVia = make_pair(z - 1, best_via);
			ISPD_Viatype_array_[z - 1][y][x].RaisingVia = make_pair(z - 1, best_via);
			SourceOK = true;
		}
		else if (find_pseudo)
		{
			// pseudo source
			Parser::PathAndEnc PAE;
			vector<Parser::wire_path> pseudo_path;
			bool valid_node = OffGridViaPrediction_Allow_Short(node_src, connection.Src_Pin_Shape, PAE.path, PAE.Enclosure, PAE.via_type, two_pin_net_id);
			if (valid_node)
			{
				//printf("PseudoSource_PathTable:: emplace node(%d,%d,%d)\n", node_src.x, node_src.y, node_src.z);

				PseudoSource_PathTable.emplace(node_src, PAE);
				SourceOK = true;
			}
			else
			{
				continue;
			}
		}
		if (SourceOK)
		{
			
			ISPD_cost_array_[z][y][x] = 0;
			ISPD_back_track_array_[z][y][x] = kNoDirection;
			ISPD_back_track_Step_array_[z][y][x] = 0;
			Node node(x, y, z, 0.0);
			maze_queue.push_back(node);
		}
	}
}

int SmallRouter::ISPD_MazeRouting3D_Allow_Short(
	const int two_pin_net_id,
	const int two_pin_net_index,
	const TwoPinNetConnection &connection,
	const vector<Parser::I_Rect> &GlobalGuide,
	const Parser::I_Rect &BoundingBox,
	Node &target_point,
	map<tuple<int, int, int>, Parser::PathAndEnc> &SourceRefinmentMap,
	map<Node, Parser::PathAndEnc> &TargetRefinmentMap, bool isIroute, Parser::I_Rect &Target_Box)
{
	// Init Setting
#ifdef NCTU_GR_HEAP
	//FibonacciHeap <Node> Fib_heap;
	pheap<Node> Fib_heap;
	Fib_heap.initial();
	vector<Node>
		maze_queue;
	maze_queue.clear();
#else
	vector<Node> maze_queue;
	maze_queue.clear();
#endif // FIB_HEAP
	ResetFunction();
	ISPD_modify_list.clear();
	vector<Parser::I_Rect> NetShape;
	//map< Node, vector<Parser::wire_path> > PseudoSource_PathTable;

	// Pin Process No need to do now
	if (connection.src_type == 0)
	{
		//printf("ISPD_MazeRouting3D:: Source Pin Setting\n");
		int x = connection.source.pin_position.pin_point.x;
		int y = connection.source.pin_position.pin_point.y;
		int z = connection.source.pin_position.pin_point.z;
		ISPD_cost_array_[z][y][x] = 0;
		ISPD_back_track_array_[z][y][x] = kNoDirection;
		Node node_src(x, y, z,0.0);
		maze_queue.push_back(node_src);
		printf("Set Source Pin Error, src type\n");
		exit(1);
	}
	else
	{
		SetSourcePin_Allow_Short_v2(connection, SourceRefinmentMap,
								 maze_queue, two_pin_net_id,isIroute);
	}

	if (two_pin_net_id == 2660){
		for (int i = 0; i < ISPD_TargetMap_modify_list.size();i++){
			printf("Short::Target(%d,%d,%d)\n", get<0>(ISPD_TargetMap_modify_list[i]), get<1>(ISPD_TargetMap_modify_list[i]), get<2>(ISPD_TargetMap_modify_list[i]));
		}
	}

#ifndef NCTU_GR_HEAP
	make_heap(maze_queue.begin(), maze_queue.end(), GreaterMazeNode());
#endif
	// Assume the path can be found first.
	rip_up_two_pin_nets_[two_pin_net_index].path_is_found = true;

	//printf("ISPD_MazeRouting3D:: Setting Layer-based GlobalGuide\n");
	vector<vector<Parser::I_Rect>> Layer_Global_guide;
	SettingLayerGlobalGuide(GlobalGuide, Layer_Global_guide, this->layout_layer_);

	map<Node, int> TargetTable;
	

	if (maze_queue.empty())
	{
		printf("Error:: Allow Short Routing - Source Empty\n");
		exit(1);
		return -1;
	}

	printf("Push Source Node Into Fib_Heap\n");
	for (int i = 0; i < maze_queue.size(); i++)
	{
		if (this->ISPD_Target_Map[maze_queue[i].z][maze_queue[i].y][maze_queue[i].x])
		{
			target_point.SetNode(
				maze_queue[i].x,
				maze_queue[i].y,
				maze_queue[i].z);
			return 0;
		}
#ifdef NCTU_GR_HEAP

		//ISPD_Fib_Node_Location[maze_queue[i].z][maze_queue[i].y][maze_queue[i].x] = Fib_heap.insert(maze_queue[i]);
		ISPD_Fib_Node_Location[maze_queue[i].z][maze_queue[i].y][maze_queue[i].x] = new pheap_el<Node>;
		ISPD_Fib_Node_Location[maze_queue[i].z][maze_queue[i].y][maze_queue[i].x]->data = &maze_queue[i];
		Fib_heap.insert(ISPD_Fib_Node_Location[maze_queue[i].z][maze_queue[i].y][maze_queue[i].x]);
		ISPD_Node_In_Heap[maze_queue[i].z][maze_queue[i].y][maze_queue[i].x] = true;

#endif // NCTU_GR_HEAP
	}
	printf("Push Source Node Into Fib_Heap Successful\n");

	Node pre_node;

	bool reach_target = false;
	while (!reach_target)
	{

		// The path of the net can't be found and built!!!
#ifdef NCTU_GR_HEAP
		/*if (Fib_heap.isEmpty()){
			rip_up_two_pin_nets_[two_pin_net_index].path_is_found = false;
			printf("Error::Path Not Found\n");
			return -1;
		}*/
		if (Fib_heap.root == NULL)
		{
			rip_up_two_pin_nets_[two_pin_net_index].path_is_found = false;
			printf("Error::Path Not Found\n");
			return -1;
		}
#else
		if (maze_queue.empty())
		{
			rip_up_two_pin_nets_[two_pin_net_index].path_is_found = false;
			printf("Error::Path Not Found\n");
			return -1;
		}
#endif

		// explore the node
#ifdef NCTU_GR_HEAP
		//Node current_node = Fib_heap.removeMinimum();
		Node current_node = *((Fib_heap.extract())->data); // NCTU_GR
#else
		/*if (maze_queue.size() > max_queue_size)
			max_queue_size = maze_queue.size();*/
		Node current_node(maze_queue.front());
		pop_heap(maze_queue.begin(), maze_queue.end(), GreaterMazeNode());
		maze_queue.pop_back();
#endif // FIB_HEAP

		// Here to check current node
		int cur_layer = current_node.z;
		int cur_x = current_node.x;
		int cur_y = current_node.y;
		//if (two_pin_net_id == 2660)
			//printf("current node : (%d,%d,%d)  cost : %.2f\n", cur_x, cur_y, cur_layer, current_node.cost);
			if (current_node.cost == 0)
			{
				if (pre_node.x == cur_x && pre_node.y == cur_y && pre_node.z == cur_layer)
					return -1;

				else
					pre_node.SetNode(cur_x, cur_y, cur_layer);
			}
		for (int dir_index = kRight; dir_index <= kDown; dir_index++)
		{
			const int direction = dir_index;

			// the neighbor of the current node
			Node next = ToNextNode(current_node, direction, Layer_Global_guide);

			

			if (current_node.z == next.z && current_node.x == next.x && current_node.y == next.y)
			{
				continue;
			}
			if (next.x == -1 || next.y == -1)
			{
				continue;
			}

			// Reject M1 Routing will make case simpler
#ifdef REJECT_M1_ROUTING
			if (next.z == 0)
			{
				continue;
			}
#endif

#ifdef Short_Routing_Region
				// Using Bounding Box to filter region
			auto temp_coor = ISPD_real_coor_table.Index2Coor(next.x, next.y, next.z);
			if (get<0>(temp_coor) < BoundingBox.LB.first ||
				get<0>(temp_coor) > BoundingBox.RT.first ||
				get<1>(temp_coor) < BoundingBox.LB.second ||
				get<1>(temp_coor) > BoundingBox.RT.second)
			{
				continue;
			}
#endif

			//const ULL next_node_index = Transform3dArrayTo1dIndex(next_node);
			const bool via_direction =
				(direction == kUp || direction == kDown);
			Node cur_node(cur_x, cur_y, cur_layer);

			// Move In Plane
			if (!via_direction)
			{
				const bool is_node_blocked = false;
				if (!is_node_blocked)
				{
					ISPD_modify_list.push_back(make_tuple(next.x, next.y, next.z));

					float next_node_temp_cost = ISPD_UpdateCost_Allow_Short(
						current_node, next,
						direction, via_direction,
						two_pin_net_id, two_pin_net_index, Layer_Global_guide, TargetTable, SourceRefinmentMap, Target_Box); // = Cost function

					if (ISPD_cost_array_[next.z][next.y][next.x] > next_node_temp_cost)
					{
						ISPD_cost_array_[next.z][next.y][next.x] = next_node_temp_cost;
						ISPD_back_track_array_[next.z][next.y][next.x] = direction;

						Node next_node(next, next_node_temp_cost);


#ifdef NCTU_GR_HEAP
						//printf("push heap: (%d,%d,%d)  cost : %.2f\n", next_node->x, next_node->y, next_node->z, next_node->cost);
						//node<Node> *temp = ISPD_Fib_Node_Location[next_node.z][next_node.y][next_node.x];
						pheap_el<Node> *temp = ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x];
						if (temp != NULL)
						{ // node is in heap, just decrease key
							//Fib_heap.decreaseKey(temp, next_node);
							Fib_heap.decreaseKey(temp, next_node_temp_cost);
						}
						else
						{ // node is not in heap, insert new node
							//ISPD_Fib_Node_Location[next_node.z][next_node.y][next_node.x] = Fib_heap.insert(next_node);
							ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x] = new pheap_el<Node>;
							//printf("push heap: (%d,%d,%d)  cost : %.2f\n", next_node.x, next_node.y, next_node.z, next_node.cost);
							ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x]->data = next_node;
							//printf("push heap: (%d,%d,%d)  cost : %.2f\n", next_node.x, next_node.y, next_node.z, next_node.cost);
							Fib_heap.insert(ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x]);
						}
							//Fib_heap.insert(next_node);
							//printf("Push successful\n");
#else
						maze_queue.push_back(next_node);
						push_heap(
							maze_queue.begin(),
							maze_queue.end(),
							GreaterMazeNode());
#endif // FIB_HEAP
					}
				}
			}
			// Move Up/Down with via
			else
			{
				bool hit = false;
				for (int via_type = 0; via_type < this->ISPD_via_type_list[min(cur_layer, (int)next.z)].size(); via_type++)
				{
					const bool is_node_blocked = !ViaEnclosureChecking(cur_node, next, make_pair(min(cur_layer, (int)next.z), via_type), two_pin_net_id);
					if (!is_node_blocked)
					{
						hit = true;
						ISPD_modify_list.push_back(make_tuple(next.x, next.y, next.z));
						//Node next_node(next, 0.0);
						//printf("%d\n", via_type);
						float next_node_temp_cost = ISPD_UpdateCost_Allow_Short(
							current_node, next,
							direction, via_direction,
							two_pin_net_id, two_pin_net_index, Layer_Global_guide, TargetTable, SourceRefinmentMap, Target_Box); // = Cost function
						// Via priority
						next_node_temp_cost += via_type;
						if (ISPD_cost_array_[next.z][next.y][next.x] > next_node_temp_cost)
						{
							ISPD_cost_array_[next.z][next.y][next.x] = next_node_temp_cost;
							ISPD_back_track_array_[next.z][next.y][next.x] = direction;
							Node next_node(next, next_node_temp_cost);

							// Setting valid via in Initial route
							if (via_direction)
							{
								if (next.z > cur_layer)
								{
									ISPD_Viatype_array_[cur_layer][cur_y][cur_x].RaisingVia = make_pair(min(cur_layer, (int)next.z), via_type);
									ISPD_Viatype_array_[next.z][next.y][next.x].FallingVia = make_pair(min(cur_layer, (int)next.z), via_type);
								}
								else
								{
									ISPD_Viatype_array_[cur_layer][cur_y][cur_x].FallingVia = make_pair(min(cur_layer, (int)next.z), via_type);
									ISPD_Viatype_array_[next.z][next.y][next.x].RaisingVia = make_pair(min(cur_layer, (int)next.z), via_type);
								}
							}

#ifdef NCTU_GR_HEAP
							//printf("push heap: (%d,%d,%d)  cost : %.2f\n", next_node->x, next_node->y, next_node->z, next_node->cost);
							//node<Node> *temp = ISPD_Fib_Node_Location[next_node.z][next_node.y][next_node.x];
							pheap_el<Node> *temp = ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x];
							if (temp != NULL)
							{ // node is in heap, just decrease key
								//Fib_heap.decreaseKey(temp, next_node);
								Fib_heap.decreaseKey(temp, next_node_temp_cost);
							}
							else
							{ // node is not in heap, insert new node
								//ISPD_Fib_Node_Location[next_node.z][next_node.y][next_node.x] = Fib_heap.insert(next_node);
								ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x] = new pheap_el<Node>;
								//printf("push heap: (%d,%d,%d)  cost : %.2f\n", next_node.x, next_node.y, next_node.z, next_node.cost);
								ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x]->data = next_node;

								Fib_heap.insert(ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x]);
							}
								//Fib_heap.insert(next_node);
								//printf("Push successful\n");
#else
							maze_queue.push_back(next_node);
							push_heap(
								maze_queue.begin(),
								maze_queue.end(),
								GreaterMazeNode());
#endif // FIB_HEAP
						}
						break; // This Break is important
					}
				}

				if(!hit){
					for (int via_type = 0; via_type < this->ISPD_via_type_list[min(cur_layer, (int)next.z)].size(); via_type++)
					{
						const bool is_node_blocked = false;
						if (!is_node_blocked)
						{
							ISPD_modify_list.push_back(make_tuple(next.x, next.y, next.z));
							//Node next_node(next, 0.0);
							//printf("%d\n", via_type);
							float next_node_temp_cost = ISPD_UpdateCost_Allow_Short(
								current_node, next,
								direction, via_direction,
								two_pin_net_id, two_pin_net_index, Layer_Global_guide, TargetTable, SourceRefinmentMap, Target_Box); // = Cost function
							// Via priority
							next_node_temp_cost += via_type;
							if (ISPD_cost_array_[next.z][next.y][next.x] > next_node_temp_cost)
							{
								ISPD_cost_array_[next.z][next.y][next.x] = next_node_temp_cost;
								ISPD_back_track_array_[next.z][next.y][next.x] = direction;
								Node *next_node = new Node(next, next_node_temp_cost);

								// Setting valid via in Initial route
								if (via_direction)
								{
									if (next.z > cur_layer)
									{
										ISPD_Viatype_array_[cur_layer][cur_y][cur_x].RaisingVia = make_pair(min(cur_layer, (int)next.z), via_type);
										ISPD_Viatype_array_[next.z][next.y][next.x].FallingVia = make_pair(min(cur_layer, (int)next.z), via_type);
									}
									else
									{
										ISPD_Viatype_array_[cur_layer][cur_y][cur_x].FallingVia = make_pair(min(cur_layer, (int)next.z), via_type);
										ISPD_Viatype_array_[next.z][next.y][next.x].RaisingVia = make_pair(min(cur_layer, (int)next.z), via_type);
									}
								}

#ifdef NCTU_GR_HEAP
								//printf("push heap: (%d,%d,%d)  cost : %.2f\n", next_node->x, next_node->y, next_node->z, next_node->cost);
								//node<Node> *temp = ISPD_Fib_Node_Location[next_node.z][next_node.y][next_node.x];
								pheap_el<Node> *temp = ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x];
								if (temp != NULL)
								{ // node is in heap, just decrease key
									//Fib_heap.decreaseKey(temp, next_node);
									Fib_heap.decreaseKey(temp, next_node_temp_cost);
								}
								else
								{ // node is not in heap, insert new node
									//ISPD_Fib_Node_Location[next_node.z][next_node.y][next_node.x] = Fib_heap.insert(next_node);
									ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x] = new pheap_el<Node>;
									//printf("push heap: (%d,%d,%d)  cost : %.2f\n", next_node.x, next_node.y, next_node.z, next_node.cost);
									ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x]->data = next_node;

									Fib_heap.insert(ISPD_Fib_Node_Location[next_node->z][next_node->y][next_node->x]);
								}
									//Fib_heap.insert(next_node);
									//printf("Push successful\n");
#else
							maze_queue.push_back(*next_node);
							push_heap(
								maze_queue.begin(),
								maze_queue.end(),
								GreaterMazeNode());
#endif // FIB_HEAP
							}
							break; // This Break is important
						}
					}
				}
			}
		}

#ifdef NCTU_GR_HEAP
		//if (Fib_heap.isEmpty())
		//continue;
		if (Fib_heap.root == NULL)
			continue;
		// Target Find

		//Node mn = Fib_heap.getMinimum();
		Node mn = *(Fib_heap.root->data);
#else
		Node mn = maze_queue.front();
#endif // FIB_HEAP

		Node n(mn.x, mn.y, mn.z);
		if (TargetTableFind(n, isIroute, two_pin_net_id, Target_Box)) // find target
		{
			printf("Target (%d,%d,%d)\n", mn.x, mn.y, mn.z);
			target_point.SetNode(mn.x, mn.y, mn.z);
			reach_target = true;
		}
	}
#ifdef NCTU_GR_HEAP
	Fib_heap.memory_clear();
#endif // FIB_HEAP
	return 0;
}

bool SmallRouter::TargetTableFind(Node &node, bool IsSrcIRoute, int two_pin_net_id,I_Rect &Target_Box)
{
	/*if (IsSrcIRoute)
		return ISPD_iroute_array[node.z][node.y][node.x] == two_pin_net_id;
	else
		return ISPD_Target_Map[node.z][node.y][node.x] | ISPD_iroute_array[node.z][node.y][node.x] == two_pin_net_id;
*/

	return ISPD_Target_Map[node.z][node.y][node.x];
/*
	if (IsSrcIRoute){
		return ISPD_Target_Map[node.z][node.y][node.x];
	}
	else{
		if (node.x <= Target_Box.RT.first && node.x >= Target_Box.LB.first && node.y <= Target_Box.RT.second && node.y >= Target_Box.LB.second)
			return ISPD_Target_Map[node.z][node.y][node.x];
		else
			return false;
	}
		*/
}

bool SmallRouter::Stack_Via_Estimation(const Node &cur_node){
	if ((*ISPD_EOL_Blocage_Map)[cur_node.z][cur_node.y][cur_node.x - 1] == NO_BLOCKAGE){
		return true;
	}
	else if ((*ISPD_EOL_Blocage_Map)[cur_node.z][cur_node.y][cur_node.x + 1] == NO_BLOCKAGE){
		return true;
	}
	else if ((*ISPD_EOL_Blocage_Map)[cur_node.z][cur_node.y - 1][cur_node.x] == NO_BLOCKAGE){
		return true;
	}
	else if ((*ISPD_EOL_Blocage_Map)[cur_node.z][cur_node.y + 1][cur_node.x] == NO_BLOCKAGE){
		return true;
	}
	else{
		return false;
	}
}

bool SmallRouter::ISPD_IsNodeBlocked(
	const Node &cur_node,
	const Node &next_node,
	const int two_pin_net_id,
	const int direction,
	const int via_type,
	vector <Parser::I_Rect> &NetShape
	){

	//int Cut_Spacing = this->ISPD_Via_cut_Spacing[next_node.z];
	//printf("Block\n");
	if ((*ISPD_blockage_array_)[next_node.z][next_node.y][next_node.x] != NO_BLOCKAGE &&
		(*ISPD_blockage_array_)[next_node.z][next_node.y][next_node.x] != two_pin_net_id)
	{
		return true;
	}

	if (direction == kUp || direction == kDown){
		// check eol blockage map - EOL spacing
		if ((*ISPD_EOL_Blocage_Map)[next_node.z][next_node.y][next_node.x] != NO_BLOCKAGE &&
			(*ISPD_EOL_Blocage_Map)[next_node.z][next_node.y][next_node.x] != two_pin_net_id)
		{
			return true;
		}
	}
		
	if (direction == kUp || direction == kDown){
		//printf("ViaEnclosureChecking\n");
		if (ViaEnclosureChecking(cur_node, next_node, make_pair(min(cur_node.z, next_node.z), via_type), two_pin_net_id) == false){
			
			return true;
		}

		// Stack Via
		/*if (ISPD_back_track_array_[cur_node.z][cur_node.y][cur_node.x] == kUp && direction == kUp){
			if (Stack_Via_Estimation(cur_node) == false){
				return true;
			}
			return true;
		}
		else if (ISPD_back_track_array_[cur_node.z][cur_node.y][cur_node.x] == kDown && direction == kDown){
			if (Stack_Via_Estimation(cur_node) == false){
				return true;
			}
			return true;
		}*/
		//printf("ViaEnclosureChecking End\n");
	}
	
	//printf("Block END\n");
	// check if via enclosure violate spacing rule to wire
	/*if (direction == kUp || direction == kDown){
		Parser::Via this_via = this->ISPD_via_type_list[min(cur_node.z, next_node.z)][via_type];
		for (int y = cur_node.y - 1; y <= cur_node.y + 1; y++){
			for (int x = cur_node.x - 1; x <= cur_node.x + 1; x++){
				if (y >= 0 && y < (*ISPD_blockage_array_)[cur_node.z].size() && x >= 0 && x < (*ISPD_blockage_array_)[cur_node.z][y].size())
				if ((*ISPD_blockage_array_)[cur_node.z][y][x] != NO_BLOCKAGE || (*ISPD_EOL_Blocage_Map)[cur_node.z][y][x] != NO_BLOCKAGE){
					return true;
				}
			}
		}
		for (int y = next_node.y - 1; y <= next_node.y + 1; y++){
			for (int x = next_node.x - 1; x <= next_node.x + 1; x++){
				if (y >= 0 && y < (*ISPD_blockage_array_)[cur_node.z].size() && x >= 0 && x < (*ISPD_blockage_array_)[cur_node.z][y].size())
				if ((*ISPD_blockage_array_)[next_node.z][y][x] != NO_BLOCKAGE || (*ISPD_EOL_Blocage_Map)[next_node.z][y][x] != NO_BLOCKAGE){
					return true;
				}
			}
		}
	}*/
	
	// check if via violate cut spacing rule to other vias
	/*if (direction == kUp || direction == kDown){
		if (ISPD_Viablockage_array_[cur_node.z][cur_node.y][cur_node.x].layer != -1
			&& ISPD_Viablockage_array_[cur_node.z][cur_node.y][cur_node.x].net_id != two_pin_net_id){
			return true;
		}
	}*/


	return false;
}


/*inline bool SmallRouter::IsNodeBlocked(
		const MazeNode &next_node,
		const BoundingBox &bounding_box,
		const int two_pin_net_id,
		const int direction
		)
{
	bool is_blocked = false;
			
	const bool node_inside_bounding_box = (
			(next_node.x >= bounding_box.left) && 
			(next_node.x <= bounding_box.right) &&
			(next_node.y <= bounding_box.top) &&
			(next_node.y >= bounding_box.bottom) &&
			(next_node.z <= bounding_box.top_layer) &&
			(next_node.z >= bounding_box.bottom_layer));
	
	const ULL next_node_index = Transform3dArrayTo1dIndex(next_node);
	const bool to_node_can_be_reached = (
			(node_inside_bounding_box) && 
			(track_capacity_[next_node_index] != kBlockage)
		);
	
	// For parity line-end issue.
	bool next_node_has_pseudo_blockage = false;
	if(this->kLineEndModeRouting_ && node_inside_bounding_box)
	{
		if(this->total_rip_up_times > kBlockageTimes_)
			next_node_has_pseudo_blockage = 
				line_end_graph_.HasNormalBlockage(next_node);
		else if(this->total_rip_up_times <= kBlockageTimes_)
			next_node_has_pseudo_blockage =
				line_end_graph_.HasBlockage(next_node);
		else
			;
	}

	// For preferred direction of each layer.
	const bool horizontal_direction_preferred_layer = 
		( (direction == kRight || direction == kLeft) && 
		  (next_node.z % 2 == kOddLayer));
	const bool vertical_direction_preferred_layer = 
		( (direction == kTop || direction == kBottom) && 
		  (next_node.z % 2 == kEvenLayer));
	const bool via_direction = 
		( direction == kUp || direction == kDown);
	const bool preferred_direction = 
		( horizontal_direction_preferred_layer || 
		  vertical_direction_preferred_layer || 
		  via_direction);

	is_blocked = (
			!node_inside_bounding_box ||
			!to_node_can_be_reached ||
			!preferred_direction ||
			next_node_has_pseudo_blockage  
			);
	return is_blocked;
}

// Evaluate overlapping cost for a grid.
inline float SmallRouter::UpdateOverlappedCost(
		const MazeNode &maze_node,
		const int two_pin_net_id, 
		const int two_pin_net_index)
{
	float penalty_num = 0.0;

	const ULL grid_index = Transform3dArrayTo1dIndex(maze_node);
	const float beta = kOverlapBeta_;
	const float slope = kOverlapSlope_;
	const float offset = kOverlapOffset_;
	
	const bool over_two_illegal_nets_on_grid = 
		(track_capacity_[grid_index] <= -2);
	const bool one_net_on_grid = (track_capacity_[grid_index] > 0);
	const bool no_net_on_grid = (track_capacity_[grid_index] == 0);
	if(over_two_illegal_nets_on_grid)
	{
		const int overlap_net_num = (-track_capacity_[grid_index]);
		penalty_num = 
			1.0 + ((beta) / (exp(slope * (overlap_net_num + offset)) + 1.0));
	}
	else if(one_net_on_grid)
	{
		
		const int net_id = net_id_table_[two_pin_net_id].net_id;
		const int grid_id = track_capacity_[grid_index];
		const int grid_net_id = net_id_table_[grid_id].net_id;
		
		const bool belong_to_same_multi_pin_net = (net_id == grid_net_id);
		if(belong_to_same_multi_pin_net)
		{
			penalty_num = 0.0;
		}
		else
		{
			int rip_up_times = 1;
			const int grid_two_pin_net_id = 
				net_id_table_[grid_id].two_pin_net_id;
			if(grid_two_pin_net_id == -1)
			{
				rip_up_times = 100;
			}
			else
			{
				const int grid_two_pin_net_index = 
					net_id_table_[grid_id].two_pin_net_index;
				TwoPinRipUpNet &occupied_net = 
					rip_up_two_pin_nets_[grid_two_pin_net_index];
				rip_up_times = occupied_net.rip_up_times + 1;
			}
			penalty_num = 
				1.0 + ((beta) / (exp(slope * (rip_up_times  + offset)) + 1.0));
		}
	}
	else if(no_net_on_grid)
	{
		penalty_num = 0.5;
	}
	else
	{
		penalty_num = 1000;
	}

	return penalty_num;
}


// Evaluate via enclosure overlapping cost.
inline float SmallRouter::UpdateViaEncOverlappedCost(
		const Node &node,
		const int two_pin_net_id, 
		const int two_pin_net_index
		)
{
	float penalty_num = 0.0;

	const ULL grid_index = Transform3dArrayTo1dIndex(node);
	const float beta = kViaEncOverlapBeta_;
	// const float beta = 3.0;
	const float slope = kViaEncOverlapSlope_;
	const float offset = kViaEncOverlapOffset_;
	
	const bool over_two_illegal_nets_on_grid = 
		(track_capacity_[grid_index] <= -2);
	const bool one_net_on_grid = (track_capacity_[grid_index] > 0);
	const bool no_net_on_grid = (track_capacity_[grid_index] == 0);
	if(over_two_illegal_nets_on_grid)
	{
		const int overlap_net_num = (-track_capacity_[grid_index]);
		penalty_num = 
			1.0 + ((beta) / (exp(slope * (overlap_net_num + offset)) + 1.0));
		if(kLineEndModeRouting_ || kEndEndSpacingMode_)
			penalty_num *= 6;
		else
			penalty_num *= 11;
	}
	else if(one_net_on_grid)
	{
		
		const int net_id = net_id_table_[two_pin_net_id].net_id;
		const int grid_id = track_capacity_[grid_index];
		const int grid_net_id = net_id_table_[grid_id].net_id;
		
		const bool belong_to_same_multi_pin_net = (net_id == grid_net_id);
		if(belong_to_same_multi_pin_net)
		{
			penalty_num = 0.0;
		}
		else
		{
			int rip_up_times = 1;
			const int grid_two_pin_net_id = 
				net_id_table_[grid_id].two_pin_net_id;
			if(grid_two_pin_net_id == -1)
			{
				rip_up_times = 100;
			}
			else
			{
				const int grid_two_pin_net_index = 
					net_id_table_[grid_id].two_pin_net_index;
				TwoPinRipUpNet &occupied_net = 
					rip_up_two_pin_nets_[grid_two_pin_net_index];
				rip_up_times = occupied_net.rip_up_times + 1;
			}
			penalty_num = 
				1.0 + ((beta) / (exp(slope * (rip_up_times  + offset)) + 1.0));
			if(kLineEndModeRouting_ || kEndEndSpacingMode_)
				penalty_num *= 6;
			else
				penalty_num *= 11;
		}
	}
	else if(no_net_on_grid)
	{
		penalty_num = 0.5;
	}
	else
	{
		penalty_num = 1000;
	}

	return penalty_num;
}


inline float SmallRouter::UpdateCost(
		const MazeNode &current_node,
		const MazeNode &next_node,
		const int direction,
		const bool is_via_direction,
		const int two_pin_net_id,
		const int two_pin_net_index
		)
{
	float edge_cost = 0.0;
	
	const float current_node_cost = 
		cost_array_[Transform3dArrayTo1dIndex(current_node)];
	const float base_cost = kBaseCost_;
	const float via_cost = (is_via_direction) ? (kViaCost_) : (0.0);
	// const float via_cost = (is_via_direction) ? (0) : (0.0);
	const float overlap_cost = 
		UpdateOverlappedCost(
				next_node,
				two_pin_net_id, 
				two_pin_net_index
				);
	const float history_cost = 
		(this->total_rip_up_times % kIgnoreHistoryInterval != 0) ? 
		(history_congestion_graph_.GetHistoryCongestion(
			next_node.x, next_node.y, next_node.z)) : (1.0);
	// line-end cost
	float line_end_cost = 0.0;
	if(this->kLineEndModeRouting_ && 
		this->total_rip_up_times > kBlockageTimes_)
	{
		line_end_cost = 
			line_end_graph_.GetLineEndStatus(
					next_node.x, next_node.y, next_node.z, direction);
		if(total_rip_up_times < 990)
			line_end_cost = 
				line_end_cost * (M_PI/2 - atan(0.01*total_rip_up_times-10));
		else
			line_end_cost =
				line_end_cost * (M_PI/2 - atan(0.01*990-10));
	}
	
	// via enclosre cost
	float via_enc_cost = 0.0;
	if(this->kViaEncCostMode_ && is_via_direction)
	{
		vector<Node> enc_nodes;
		GetViaEncNodes(direction, current_node, next_node, enc_nodes);
		
		if(this->kLineEndModeRouting_)
		{
			for(const Node &enc_node : enc_nodes)
			{
				via_enc_cost +=
					UpdateViaEncOverlappedCost(
						enc_node,
						two_pin_net_id, 
						two_pin_net_index
						); 
			}
		}
		else
		{
			for(const Node &enc_node : enc_nodes)
			{
				via_enc_cost +=
				UpdateViaEncOverlappedCost(
					enc_node,
					two_pin_net_id, 
					two_pin_net_index
					) *
				history_congestion_graph_.GetHistoryCongestion(
						enc_node.x, 
						enc_node.y,
						enc_node.z
						);
			}
		}
		if(overlap_cost == 0.0 && via_enc_cost <= 1.0)
			via_enc_cost *= 0.1;
		/*if(overlap_cost == 0.0 && via_enc_cost == 0.0)
			via_enc_cost *= 0.001;
		else if(overlap_cost == 0.0 && via_enc_cost <= 0.5)
			via_enc_cost *= 0.02;
		else if(overlap_cost == 0.0 && via_enc_cost <= 1.0)
			via_enc_cost *= 0.1;
		else if(overlap_cost == 0.0 && via_enc_cost <= 1.5)
			via_enc_cost *= 0.5;
		else
			;
		
	}

	edge_cost = current_node_cost;
	const bool not_belong_to_same_net = (overlap_cost != 0.0);
	if(not_belong_to_same_net)
		edge_cost += ((via_cost + overlap_cost) * history_cost);
	
	edge_cost += via_enc_cost;
	edge_cost += line_end_cost;
	if(not_belong_to_same_net)
		edge_cost += base_cost;
	return edge_cost;
}*/

float Target_Distance(int x, int y, int z, Parser::I_Rect &TargetBox)
{
	if (x >= TargetBox.LB.first && x <= TargetBox.RT.first){
		if (y >= TargetBox.LB.second && y <= TargetBox.RT.second){
			return 0.0;
		}
		else {
			return min(abs(y - TargetBox.LB.second) , abs(y - TargetBox.RT.second));
		}
	}
	if (y >= TargetBox.LB.second && y <= TargetBox.RT.second){
		return min(abs(x - TargetBox.LB.first) , abs(x - TargetBox.RT.first));
	}

	int x_dis = min(abs(x - TargetBox.LB.first) , abs(x - TargetBox.RT.first));
	int y_dis = min(abs(y - TargetBox.LB.second) , abs(y - TargetBox.RT.second));

	return x_dis + y_dis;
}

float SmallRouter::ISPD_UpdateCost(const Node &current_node, const Node &next_node, const int direction, const bool is_via_direction,
								   const int two_pin_net_id, const int two_pin_net_index, vector<vector<Parser::I_Rect>> &Layer_Global_guide, map<Node, int> &TargetTable, map<tuple<int, int, int>, Parser::PathAndEnc> &Refinementmap ,Parser::I_Rect &TargetBox)
{
	int next_x = next_node.x;
	int next_y = next_node.y;
	int next_z = next_node.z;

	int cur_x = current_node.x;
	int cur_y = current_node.y;
	int cur_z = current_node.z;


	float edge_cost = 0.0;

	float wrong_dir_penalty = 0.0;
	if (next_z == 0)  edge_cost += SHORT_PENALTY;
	if (next_z == 0 && cur_z == 0)  edge_cost += 2.0;

	// stack via penalty
	if (is_via_direction){
		if (ISPD_back_track_array_[cur_z][cur_y][cur_x] == kUp && direction == kUp) edge_cost += STACK_VIA_PENALTY;
		else if (ISPD_back_track_array_[cur_z][cur_y][cur_x] == kDown && direction == kDown) edge_cost += STACK_VIA_PENALTY;
		else if (ISPD_back_track_array_[cur_z][cur_y][cur_x] == kNoDirection){
			edge_cost += STACK_VIA_PENALTY;
		}
		else if (ISPD_Target_Map[next_z][next_y][next_x]){
			edge_cost += STACK_VIA_PENALTY;
		}
	}
	//printf("Congestion\n");
	float CongestionCost = 0.0;
	if ((*ISPD_Congestion_Map)[next_z][next_y][next_x].second > 0 && (*ISPD_Congestion_Map)[next_z][next_y][next_x].first != two_pin_net_id)
			CongestionCost = ((*ISPD_Congestion_Map)[next_z][next_y][next_x].second - 1) * CONGESTION_PENALTY;
	//printf("Congestion End\n");
	float SameNet_Spacing = 0.0;
	// wrong dir penalty
	if (!is_via_direction){
		if (prefer_dir[cur_z] == true){  // prefer horizontal
			if (direction == kTop || direction == kBottom){
				wrong_dir_penalty = WRONG_DIR_PENALTY;
				/*printf("WARNING:: Wrong dir for edge (%d,%d,%d)-(%d,%d,%d)\n", current_node.x, current_node.y, current_node.z,
					next_node.x, next_node.y, next_node.z);*/
			}
		}
		else{							 // prefer vertical
			if (direction == kRight || direction == kLeft){
				wrong_dir_penalty = WRONG_DIR_PENALTY;
				/*printf("WARNING:: Wrong dir for edge (%d,%d,%d)-(%d,%d,%d)\n", current_node.x, current_node.y, current_node.z,
					next_node.x, next_node.y, next_node.z);*/
			}
		}
	}

	if((*ISPD_GridMap_layout)[next_z][next_y][next_x].InPlane == false){
		edge_cost += OUY_OF_TRACK_PENALTY;
	}

	float DummyViaCost = 0.0;
	auto NeedRefinement = Refinementmap.find(make_tuple(next_x, next_y, next_z));

	if (ISPD_Target_Map[next_z][next_y][next_x] && NeedRefinement != Refinementmap.end())
	{
		bool Dummy_Left = false;
		if (next_x - 1 >= 0)
			Dummy_Left = ISPD_ViaHit_array_[next_z][next_y][next_x - 1];
		bool Dummy_Right = false;
		if (next_x + 1 < (*ISPD_GridMap_layout)[next_z][next_y].size())
			Dummy_Right = ISPD_ViaHit_array_[next_z][next_y][next_x + 1];
		bool Dummy_Up = false;
		if (next_y - 1 >= 0)
			Dummy_Up = ISPD_ViaHit_array_[next_z][next_y - 1][next_x];
		bool Dummy_Down = false;
		if (next_y + 1 < (*ISPD_GridMap_layout)[next_z].size())
			Dummy_Down = ISPD_ViaHit_array_[next_z][next_y + 1][next_x];
		bool Dummy = Dummy_Left | Dummy_Right | Dummy_Up | Dummy_Down;
		if (ISPD_ViaHit_array_[next_z][next_y][next_x] == false && Dummy)
		{
			DummyViaCost += DUMMY_VIA_PENALTY;
		}
	}

#ifdef A_star_TargetBox
	float TargetDis = Target_Distance(next_x, next_y, next_z,TargetBox);
	if(TargetDis < 0){
		printf("TargetDis < 0 Error\n");
		exit(1);
	}
#endif

	const float current_node_cost = ISPD_cost_array_[current_node.z][current_node.y][current_node.x];
	//const float base_cost = abs(next_x - cur_x) + abs(next_y - cur_y);
	const float base_cost = 1;
	//const float via_cost = 0.0;
	float via_cost = 0.0;
	if (is_via_direction)
	{
		Parser::I_Rect Bot_Rect = ISPD_via_type_list.at(min(next_z, cur_z)).at(0).BotLayerIRect;
		int bot_size = (Bot_Rect.RT.first - Bot_Rect.LB.first) * (Bot_Rect.RT.second - Bot_Rect.LB.second);
		Parser::I_Rect Top_Rect = ISPD_via_type_list.at(min(next_z, cur_z)).at(0).TopLayerIRect;
		int top_size = (Top_Rect.RT.first - Top_Rect.LB.first) * (Top_Rect.RT.second - Top_Rect.LB.second);
		via_cost = ViaCost/* + bot_size + top_size*/;
	}


	const float overlap_cost = 0.0;
	const float history_cost = 0.0;
	// line-end cost
	float line_end_cost = 0.0;

	// via enclosre cost
	float via_enc_cost = 0.0;

	edge_cost += current_node_cost;
	//edge_cost += ((via_cost + overlap_cost) * history_cost);
	//edge_cost += SameNet_Spacing;
	edge_cost += wrong_dir_penalty;
	//edge_cost += via_enc_cost;
	//edge_cost += line_end_cost;
	edge_cost += base_cost;
	edge_cost += via_cost;
	edge_cost += DummyViaCost;
#ifdef PinCoverCost
	edge_cost += CongestionCost;
#endif
#ifdef A_star_TargetBox
	edge_cost += TargetDis * A_Star_Distance_Penalty;
#endif

	// out of guide penalty
	bool InGuide = false;
	auto temp_coor = ISPD_real_coor_table.Index2Coor(next_x, next_y, next_z);
	coor_t coor_x = get<0>(temp_coor);
	coor_t coor_y = get<1>(temp_coor);
	for (int i = 0; i < Layer_Global_guide[next_z].size(); i++){
		if ((coor_x >= Layer_Global_guide[next_z][i].LB.first && coor_x <= Layer_Global_guide[next_z][i].RT.first)){
			if ((coor_y >= Layer_Global_guide[next_z][i].LB.second && coor_y <= Layer_Global_guide[next_z][i].RT.second)){
				InGuide = true;
			}
		} 
	}
	if (InGuide == false){
		edge_cost += OUT_OF_GUIDE_PENALTY;
	}

	return edge_cost;
}

float SmallRouter::ISPD_UpdateCost_Allow_Short(const Node &current_node, const Node &next_node, const int direction, const bool is_via_direction,
											   const int two_pin_net_id, const int two_pin_net_index, vector<vector<Parser::I_Rect>> &Layer_Global_guide, map<Node, int> &TargetTable,map<tuple<int, int, int>, Parser::PathAndEnc> &Refinementmap, Parser::I_Rect &TargetBox)
{
	int next_x = next_node.x;
	int next_y = next_node.y;
	int next_z = next_node.z;

	int cur_x = current_node.x;
	int cur_y = current_node.y;
	int cur_z = current_node.z;


	float edge_cost = 0.0;

	float wrong_dir_penalty = 0.0;
	if (next_z == 0)  edge_cost += SHORT_PENALTY;
	if (next_z == 0 && cur_z == 0)  edge_cost += 2.0;

	// check blockage map - parallel run length
	if ((*ISPD_blockage_array_)[next_node.z][next_node.y][next_node.x] != NO_BLOCKAGE &&
		(*ISPD_blockage_array_)[next_node.z][next_node.y][next_node.x] != two_pin_net_id){
		edge_cost += SHORT_PENALTY;
	}
	// check eol blockage map - EOL spacing
	if ((*ISPD_EOL_Blocage_Map)[next_node.z][next_node.y][next_node.x] != NO_BLOCKAGE &&
		(*ISPD_EOL_Blocage_Map)[next_node.z][next_node.y][next_node.x] != two_pin_net_id){
		edge_cost += SHORT_PENALTY;
	}

	// stack via penalty
	if (is_via_direction)
	{
		if (ISPD_back_track_array_[cur_z][cur_y][cur_x] == kUp && direction == kUp)
			edge_cost += STACK_VIA_PENALTY;
		else if (ISPD_back_track_array_[cur_z][cur_y][cur_x] == kDown && direction == kDown)
			edge_cost += STACK_VIA_PENALTY;
		else if (ISPD_back_track_array_[cur_z][cur_y][cur_x] == kNoDirection)
		{
			edge_cost += STACK_VIA_PENALTY;
		}
		else if (ISPD_Target_Map[next_z][next_y][next_x])
		{
			edge_cost += STACK_VIA_PENALTY;
		}
	}
	//printf("Congestion\n");
	float CongestionCost = 0.0;
	if ((*ISPD_Congestion_Map)[next_z][next_y][next_x].second > 0 && (*ISPD_Congestion_Map)[next_z][next_y][next_x].first != two_pin_net_id)
		CongestionCost = ((*ISPD_Congestion_Map)[next_z][next_y][next_x].second - 1) * CONGESTION_PENALTY;
	//printf("Congestion End\n");
	float SameNet_Spacing = 0.0;
	// wrong dir penalty
	if (!is_via_direction)
	{
		if (prefer_dir[cur_z] == true)
		{ // prefer horizontal
			if (direction == kTop || direction == kBottom)
			{
				wrong_dir_penalty = WRONG_DIR_PENALTY;
				/*printf("WARNING:: Wrong dir for edge (%d,%d,%d)-(%d,%d,%d)\n", current_node.x, current_node.y, current_node.z,
					next_node.x, next_node.y, next_node.z);*/
			}
		}
		else
		{ // prefer vertical
			if (direction == kRight || direction == kLeft)
			{
				wrong_dir_penalty = WRONG_DIR_PENALTY;
				/*printf("WARNING:: Wrong dir for edge (%d,%d,%d)-(%d,%d,%d)\n", current_node.x, current_node.y, current_node.z,
					next_node.x, next_node.y, next_node.z);*/
			}
		}
	}

	if ((*ISPD_GridMap_layout)[next_z][next_y][next_x].InPlane == false)
	{
		edge_cost += OUY_OF_TRACK_PENALTY;
	}

	float DummyViaCost = 0.0;
	auto NeedRefinement = Refinementmap.find(make_tuple(next_x, next_y, next_z));

	if (ISPD_Target_Map[next_z][next_y][next_x] && NeedRefinement != Refinementmap.end())
	{
		bool Dummy_Left = false;
		if (next_x - 1 >= 0)
			Dummy_Left = ISPD_ViaHit_array_[next_z][next_y][next_x - 1];
		bool Dummy_Right = false;
		if (next_x + 1 < (*ISPD_GridMap_layout)[next_z][next_y].size())
			Dummy_Right = ISPD_ViaHit_array_[next_z][next_y][next_x + 1];
		bool Dummy_Up = false;
		if (next_y - 1 >= 0)
			Dummy_Up = ISPD_ViaHit_array_[next_z][next_y - 1][next_x];
		bool Dummy_Down = false;
		if (next_y + 1 < (*ISPD_GridMap_layout)[next_z].size())
			Dummy_Down = ISPD_ViaHit_array_[next_z][next_y + 1][next_x];
		bool Dummy = Dummy_Left | Dummy_Right | Dummy_Up | Dummy_Down;
		if (ISPD_ViaHit_array_[next_z][next_y][next_x] == false && Dummy)
		{
			DummyViaCost += DUMMY_VIA_PENALTY;
		}
	}

#ifdef A_star_TargetBox
	float TargetDis = Target_Distance(next_x, next_y, next_z, TargetBox);
	if (TargetDis < 0)
	{
		printf("TargetDis < 0 Error\n");
		exit(1);
	}
#endif

	const float current_node_cost = ISPD_cost_array_[current_node.z][current_node.y][current_node.x];
	//const float base_cost = abs(next_x - cur_x) + abs(next_y - cur_y);
	const float base_cost = 1;
	//const float via_cost = 0.0;
	float via_cost = 0.0;
	if (is_via_direction)
	{
		Parser::I_Rect Bot_Rect = ISPD_via_type_list.at(min(next_z, cur_z)).at(0).BotLayerIRect;
		int bot_size = (Bot_Rect.RT.first - Bot_Rect.LB.first) * (Bot_Rect.RT.second - Bot_Rect.LB.second);
		Parser::I_Rect Top_Rect = ISPD_via_type_list.at(min(next_z, cur_z)).at(0).TopLayerIRect;
		int top_size = (Top_Rect.RT.first - Top_Rect.LB.first) * (Top_Rect.RT.second - Top_Rect.LB.second);
		via_cost = ViaCost /* + bot_size + top_size*/;
	}

	const float overlap_cost = 0.0;
	const float history_cost = 0.0;
	// line-end cost
	float line_end_cost = 0.0;

	// via enclosre cost
	float via_enc_cost = 0.0;

	edge_cost += current_node_cost;
	//edge_cost += ((via_cost + overlap_cost) * history_cost);
	//edge_cost += SameNet_Spacing;
	edge_cost += wrong_dir_penalty;
	//edge_cost += via_enc_cost;
	//edge_cost += line_end_cost;
	edge_cost += base_cost;
	edge_cost += via_cost;
	edge_cost += DummyViaCost;
#ifdef PinCoverCost
	edge_cost += CongestionCost;
#endif
#ifdef A_star_TargetBox
	edge_cost += TargetDis * A_Star_Distance_Penalty;
#endif

	// out of guide penalty
	bool InGuide = false;
	auto temp_coor = ISPD_real_coor_table.Index2Coor(next_x, next_y, next_z);
	coor_t coor_x = get<0>(temp_coor);
	coor_t coor_y = get<1>(temp_coor);
	for (int i = 0; i < Layer_Global_guide[next_z].size(); i++)
	{
		if ((coor_x >= Layer_Global_guide[next_z][i].LB.first && coor_x <= Layer_Global_guide[next_z][i].RT.first))
		{
			if ((coor_y >= Layer_Global_guide[next_z][i].LB.second && coor_y <= Layer_Global_guide[next_z][i].RT.second))
			{
				InGuide = true;
			}
		}
	}
	if (InGuide == false)
	{
		edge_cost += OUT_OF_GUIDE_PENALTY;
	}

	return edge_cost;
}


/*inline void SmallRouter::GetViaEncNodes(
		const int direction,
		const MazeNode &current_node,
		const MazeNode &next_node,
		vector<Node> &enc_nodes
		)
{
	if(direction != kUp && direction != kDown)
		return;
	const int kEncWidth = 1;
	if(current_node.z % 2 == kOddLayer)
	{
		// Current Node.
		if((current_node.x - kEncWidth) >= 0)
			enc_nodes.push_back(Node( \
					current_node.x-kEncWidth, current_node.y, current_node.z));
		if(current_node.x + kEncWidth < layout_width_) 
			enc_nodes.push_back(Node( \
					current_node.x+kEncWidth, current_node.y, current_node.z));
		// Back Next Node.
		if((next_node.y - kEncWidth) >= 0 )
			enc_nodes.push_back(Node( \
					next_node.x, next_node.y-kEncWidth, next_node.z));
		if(next_node.y + kEncWidth < layout_height_) 
			enc_nodes.push_back( Node( \
					next_node.x, next_node.y+kEncWidth, next_node.z));
	}
	else
	{
		// Current Node.
		if((current_node.y - kEncWidth) >= 0 )
			enc_nodes.push_back(Node( \
					current_node.x, current_node.y-kEncWidth, current_node.z));
		if(current_node.y + kEncWidth < layout_height_) 
			enc_nodes.push_back( Node( \
					current_node.x, current_node.y+kEncWidth, current_node.z));
		// Back Next Node.
		if((next_node.x - kEncWidth) >= 0)
			enc_nodes.push_back(Node( \
					next_node.x-kEncWidth, next_node.y, next_node.z));
		if(next_node.x + kEncWidth < layout_width_) 
			enc_nodes.push_back(Node( \
					next_node.x+kEncWidth, next_node.y, next_node.z));
	}
}*/

bool SmallRouter::BackTrack(
		const int two_pin_net_index,
		const int from_x, const int from_y, const int from_z, 
		const int to_x, const int to_y, const int to_z)
{
	
	const bool path_is_found = rip_up_two_pin_nets_[two_pin_net_index].path_is_found;
	if(path_is_found)
	{
		int back_from_x = to_x;
		int back_from_y = to_y;
		int back_from_z = to_z;
		
		TwoPinNetDetailedPath temp_path;
		Node back_from_node;
		
		back_from_node.x = to_x;
		back_from_node.y = to_y;
		back_from_node.z = to_z;
		temp_path.detail_grid_path.push_front(back_from_node);
		

		while((back_from_x != from_x) || (back_from_y != from_y) || (back_from_z != from_z))
		{

			unsigned long long back_track_index = 
				Transform3dArrayTo1dIndex(back_from_x, back_from_y, back_from_z);
			Node temp_node;
			
			if(back_track_array_[back_track_index] == kRight)
			{
				temp_node.x = back_from_x - kMoveStep;
				temp_node.y = back_from_y;
				temp_node.z = back_from_z;
				
				back_from_x -= kMoveStep;
			}
			else if(back_track_array_[back_track_index] == kLeft)
			{
				temp_node.x = back_from_x + kMoveStep;
				temp_node.y = back_from_y;
				temp_node.z = back_from_z;
				
				back_from_x += kMoveStep;
			}
			else if(back_track_array_[back_track_index] == kTop)
			{
				temp_node.x = back_from_x;
				temp_node.y = back_from_y - kMoveStep;
				temp_node.z = back_from_z;
				
				back_from_y -= kMoveStep;
			}
			else if(back_track_array_[back_track_index] == kBottom)
			{
				temp_node.x = back_from_x;
				temp_node.y = back_from_y + kMoveStep;
				temp_node.z = back_from_z;
				
				back_from_y += kMoveStep;
			}
			else if(back_track_array_[back_track_index] == kUp)
			{
				temp_node.x = back_from_x;
				temp_node.y = back_from_y;
				temp_node.z = back_from_z - kMoveStep;
				
				back_from_z -= kMoveStep;
			}
			else if(back_track_array_[back_track_index] == kDown)
			{
				temp_node.x = back_from_x;
				temp_node.y = back_from_y;
				temp_node.z = back_from_z + kMoveStep;
				
				back_from_z += kMoveStep;
			}
			else
			{
				cout << "something wrong in back track array!!!" << endl;
				cout << "old bc = " << back_track_array_[back_track_index] << endl;
				break;
				// do nothing
			}
			
			temp_path.detail_grid_path.push_front(temp_node);
		}
		
		rip_up_two_pin_nets_[two_pin_net_index].two_pin_net_detailed_path.detail_grid_path.assign(temp_path.detail_grid_path.begin(), temp_path.detail_grid_path.end());
		
		return true;
	}
	else
	{
		rip_up_two_pin_nets_[two_pin_net_index].two_pin_net_detailed_path.detail_grid_path.clear();
		
		return false;
	}
	
	return false;
}

bool SmallRouter::MemBlockRefine(vector <Parser::I_Rect> &PinShape, Node &n,Parser::wire_path &RefinePath)
{

	auto temp_coor = ISPD_real_coor_table.Index2Coor(n.x,n.y,n.z);

	Node node(get<0>(temp_coor), get<1>(temp_coor), n.z);
	for (int i = 0; i < PinShape.size(); i++)
	{
		if (PinShape[i].Layer == node.z)
		{
			int center_x = (PinShape[i].LB.first + PinShape[i].RT.first) / 2;
			int center_y = (PinShape[i].LB.second + PinShape[i].RT.second) / 2;
			//printf("center x %d , center y %d\n", center_x, center_y);
			int max_y = (*ISPD_GridMap_layout)[node.z].size() - 1;
			int max_x = (*ISPD_GridMap_layout)[node.z][(*ISPD_GridMap_layout)[node.z].size() - 1].size() - 1;
			auto temp_coor_min = ISPD_real_coor_table.Index2Coor(0, 0, n.z);
			auto temp_coor_max = ISPD_real_coor_table.Index2Coor(max_x, max_y, n.z);

			if (center_x < (get<0>(temp_coor_min)) ||
				center_x > (get<0>(temp_coor_max)) )
			{
				if (node.y >= PinShape[i].LB.second && node.y <= PinShape[i].RT.second)
				{
					RefinePath.path_type = 0;
					RefinePath.Src_Pin = make_pair(node.x, node.y);
					RefinePath.Tar_Pin = make_pair(center_x, node.y);
					RefinePath.Layer = node.z;

					//printf("MemBlockRefine::Add Path SRC(%d,%d) to TAR(%d,%d)\n", node.x, node.y, center_x, node.y);
					return true;
				}
				else
				{
					printf("MemBlockRefine Error\n");
					int pause;
					cin >> pause;
				}
			}
			else if (center_y < (get<1>(temp_coor_min)) ||
					 center_y > (get<1>(temp_coor_max)))
			{
				if (node.y >= PinShape[i].LB.second && node.y <= PinShape[i].RT.second)
				{
					RefinePath.path_type = 0;
					RefinePath.Src_Pin = make_pair(node.x, node.y);
					RefinePath.Tar_Pin = make_pair(node.x, center_y);
					RefinePath.Layer = node.z;

					//printf("MemBlockRefine::Add Path SRC(%d,%d) to TAR(%d,%d)\n", node.x, node.y, node.x, center_y);
					return true;
				}
				else
				{
					printf("MemBlockRefine Error\n");
					int pause;
					cin >> pause;
				}
			}
		}
	}
	return false;
}

void SmallRouter::PseusoNodeDetermin_FromNetTable(TwoPinNetConnection &connection, Node &source,
												  Node &Target, int net_id, Parser::I_Rect &SrcEnc, Parser::I_Rect &TarEnc,
												  map<tuple<int,int,int>, Parser::PathAndEnc> &RefinmentMap)
{
	tuple <coor_t,coor_t,int> source_coor;
	source_coor = ISPD_real_coor_table.Index2Coor(source.x, source.y, source.z);
	printf("source : (%d,%d,%d) coor(%d,%d,%d)\n", source.x, source.y, source.z, get<0>(source_coor), get<1>(source_coor), get<2>(source_coor));

	if (ISPD_ViaHit_array_[source.z][source.y][source.x] == false){
		tuple<int, int, int> s_node = make_tuple(source.x, source.y, source.z);
		auto find_source = RefinmentMap.find(s_node);
		if (find_source != RefinmentMap.end())
		{
			SrcEnc = find_source->second.Enclosure;
			for (int path = 0; path < find_source->second.path.size(); path++)
			{
				Net_wirePath[net_id].push_back(find_source->second.path[path]);
			}
			Node src(source.x, source.y, source.z);
			Node tar(source.x, source.y, source.z - 1);
			SetViaEnclosureBlockage(src, tar,kUp ,find_source->second.via_type,net_id);

			ISPD_Insert_ViaHit_array_(source.x, source.y, (int)source.z);
		}
	}
	tuple<coor_t, coor_t, int> Target_coor;
	Target_coor = ISPD_real_coor_table.Index2Coor(Target.x, Target.y, Target.z);
	printf("Target : (%d,%d,%d) coor(%d,%d,%d)\n", Target.x, Target.y, Target.z, get<0>(Target_coor), get<1>(Target_coor), get<2>(Target_coor));
	if (ISPD_ViaHit_array_[Target.z][Target.y][Target.x] == false)
	{
		tuple<int, int, int> t_node = make_tuple(Target.x, Target.y, Target.z);
		auto find_target = RefinmentMap.find(t_node);
		if (find_target != RefinmentMap.end())
		{
			/*if (Target.x== 498 && Target.y == 598 && Target.z==1 ){

				printf("refine via type (%d,%d)\n", find_target->second.via_type.first, find_target->second.via_type.second);

				for (auto iter = RefinmentMap.begin(); iter != RefinmentMap.end(); iter++)
				{
					printf("x,y,z(%d,%d,%d) via type(%d,%d)\n", get<0>(iter->first), get<1>(iter->first), get<2>(iter->first), (*iter).second.via_type.first, (*iter).second.via_type.second);
				}
			}*/
			TarEnc = find_target->second.Enclosure;
			for (int path = 0; path < find_target->second.path.size(); path++)
			{
				Net_wirePath[net_id].push_back(find_target->second.path[path]);
			}
			Node src(Target.x, Target.y, Target.z);
			Node tar(Target.x, Target.y, Target.z - 1);
			SetViaEnclosureBlockage(src, tar, kUp, find_target->second.via_type, net_id);
			//ISPD_ViaHit_array_[Target.z][Target.y][Target.x] = true;

			ISPD_Insert_ViaHit_array_(Target.x, Target.y, (int)Target.z);
		}
	}
}

void SmallRouter::PseusoNodeDetermin_FromNetTable_Short(TwoPinNetConnection &connection, Node &source,
												  Node &Target, int net_id, Parser::I_Rect &SrcEnc, Parser::I_Rect &TarEnc,
												  map<tuple<int, int, int>, Parser::PathAndEnc> &RefinmentMap,
												  map<tuple<int, int, int>, Parser::PathAndEnc> &SourceRefinmentMap)
{
	if (ISPD_ViaHit_array_[source.z][source.y][source.x] == false)
	{
		tuple<int, int, int> s_node = make_tuple(source.x, source.y, source.z);
		auto find_source = SourceRefinmentMap.find(s_node);
		if (find_source != SourceRefinmentMap.end())
		{
			SrcEnc = find_source->second.Enclosure;
			for (int path = 0; path < find_source->second.path.size(); path++)
			{
				Net_wirePath[net_id].push_back(find_source->second.path[path]);
			}
			Node src(source.x, source.y, source.z);
			Node tar(source.x, source.y, source.z - 1);
			SetViaEnclosureBlockage(src, tar, kUp, find_source->second.via_type, net_id);

			ISPD_Insert_ViaHit_array_(source.x, source.y, (int)source.z);
		}

		
	}
	if (ISPD_ViaHit_array_[Target.z][Target.y][Target.x] == false){
		tuple<int, int, int> t_node = make_tuple(Target.x, Target.y, Target.z);
		auto find_target = RefinmentMap.find(t_node);
		if (find_target != RefinmentMap.end())
		{
			TarEnc = find_target->second.Enclosure;
			for (int path = 0; path < find_target->second.path.size(); path++)
			{
				Net_wirePath[net_id].push_back(find_target->second.path[path]);
			}
			Node src(Target.x, Target.y, Target.z);
			Node tar(Target.x, Target.y, Target.z - 1);
			SetViaEnclosureBlockage(src, tar, kUp, find_target->second.via_type, net_id);
			ISPD_Insert_ViaHit_array_(Target.x, Target.y, (int)Target.z);
		}

		
	}
}

void SmallRouter::PseusoNodeDetermin(TwoPinNetConnection &connection, Node &source,
									 Node &Target, int net_id, Parser::I_Rect &SrcEnc, Parser::I_Rect &TarEnc,
									 map<Node, Parser::PathAndEnc> &SourceRefinmentMap, map<Node, Parser::PathAndEnc> &TargetRefinmentMap)
{

	bool SPseudo = true;

	tuple <coor_t,coor_t,int> source_coor;
	source_coor = ISPD_real_coor_table.Index2Coor(source.x,source.y,source.z);
	tuple<coor_t, coor_t, int> target_coor;
	target_coor = ISPD_real_coor_table.Index2Coor(Target.x, Target.y, Target.z);

	for (int i = 0; i < connection.real_Source_pin_path_list.size(); i++)
	{
		int x = connection.real_Source_pin_path_list[i].pin_position.pin_point.x;
		int y = connection.real_Source_pin_path_list[i].pin_position.pin_point.y;
		int z = connection.real_Source_pin_path_list[i].pin_position.pin_point.z;
		bool belong_pseudo_s = (x == source.x) & (y == source.y) & (z == source.z);
		//printf("source (%d,%d,%d)\n", x, y, z);
		if (belong_pseudo_s && z == 1)
		{
			//printf("True source\n");
			Parser::wire_path wire;
			wire.path_type = 2;
			wire.Layer = source.z;
			tuple<coor_t, coor_t, int> temp_coor;
			temp_coor = ISPD_real_coor_table.Index2Coor(x, y, z);
			wire.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
			//printf("Source FallingVia(1) : %d\n", ISPD_Viatype_array_[source.z][source.y][source.x].FallingVia.first);
			//printf("Source FallingVia(2) : %d\n", ISPD_Viatype_array_[source.z][source.y][source.x].FallingVia.second);
			if (ISPD_Viatype_array_[source.z][source.y][source.x].FallingVia.first == -1 ||
				ISPD_Viatype_array_[source.z][source.y][source.x].FallingVia.second == -1)
			{
				wire.ViaName = ISPD_via_type_list.at(0).at(0).Name;
				SrcEnc.Layer = 0;
				SrcEnc.LB = make_pair(ISPD_via_type_list.at(0).at(0).BotLayerIRect.LB.first + get<0>(source_coor), ISPD_via_type_list.at(0).at(0).BotLayerIRect.LB.second + get<1>(source_coor));
				SrcEnc.RT = make_pair(ISPD_via_type_list.at(0).at(0).BotLayerIRect.RT.first + get<0>(source_coor), ISPD_via_type_list.at(0).at(0).BotLayerIRect.RT.second + get<1>(source_coor));
			}
			else
			{
				wire.ViaName = ISPD_via_type_list.at(ISPD_Viatype_array_[source.z][source.y][source.x].FallingVia.first).at(ISPD_Viatype_array_[source.z][source.y][source.x].FallingVia.second).Name;
				int id_1 = ISPD_Viatype_array_[source.z][source.y][source.x].FallingVia.first;
				int id_2 = ISPD_Viatype_array_[source.z][source.y][source.x].FallingVia.second;
				SrcEnc.Layer = 0;
				SrcEnc.LB = make_pair(ISPD_via_type_list.at(id_1).at(id_2).BotLayerIRect.LB.first + get<0>(source_coor), ISPD_via_type_list.at(id_1).at(id_2).BotLayerIRect.LB.second + get<1>(source_coor));
				SrcEnc.RT = make_pair(ISPD_via_type_list.at(id_1).at(id_2).BotLayerIRect.RT.first + get<0>(source_coor), ISPD_via_type_list.at(id_1).at(id_2).BotLayerIRect.RT.second + get<2>(source_coor));
			}
			SPseudo = false;
			Net_wirePath[net_id].push_back(wire);
			break;
		}
		else if (belong_pseudo_s && z > 1)
		{
			Parser::wire_path path;
			if (MemBlockRefine(connection.Src_Pin_Shape, source, path))
			{
				Net_wirePath[net_id].push_back(path);
				SPseudo = false;
			}
			
		}
	}
	if (SPseudo == true)
	{
		for (int i = 0; i < connection.pseudo_Source_pin_path_list.size(); i++)
		{
			int x = connection.pseudo_Source_pin_path_list[i].pin_position.pin_point.x;
			int y = connection.pseudo_Source_pin_path_list[i].pin_position.pin_point.y;
			int z = connection.pseudo_Source_pin_path_list[i].pin_position.pin_point.z;
			bool belong_pseudo_s = (x == source.x) & (y == source.y) & (z == source.z);
			//printf("p-source (%d,%d,%d)\n", x, y, z);
			if (belong_pseudo_s && z == 1)
			{
				//printf("pseudo source\n");
				SPseudo = true;
				//OffGridViaRefinment(source, connection.Src_Pin_Shape, wire, net_id);
				
				auto RPath = SourceRefinmentMap.find(source);
				if (RPath != SourceRefinmentMap.end())
				{
					SrcEnc = RPath->second.Enclosure;
					for (int path = 0; path < RPath->second.path.size(); path++)
					{
						Net_wirePath[net_id].push_back(RPath->second.path[path]);
					}
				}
				else
				{
					auto RPathi = SourceRefinmentMap.begin();
					for (; RPathi != SourceRefinmentMap.end(); RPathi++)
					{
						if (RPathi->first.x == Target.x && RPathi->first.y == Target.y)
						{
							SrcEnc = RPathi->second.Enclosure;
							for (int path = 0; path < RPathi->second.path.size(); path++)
							{
								Net_wirePath[net_id].push_back(RPathi->second.path[path]);
							}
								break;
						}
					}
					if (RPathi == SourceRefinmentMap.end())
					{
						//printf("TargetRefinmentMap Error\n");
						;
					}
				}
				break;
			}
		}
	}


	bool TPseudo = true;
	map<Node, int> TargetTable;
	for (int i = 0; i < connection.real_Target_pin_path_list.size(); i++)
	{
		int x = connection.real_Target_pin_path_list[i].pin_position.pin_point.x;
		int y = connection.real_Target_pin_path_list[i].pin_position.pin_point.y;
		int z = connection.real_Target_pin_path_list[i].pin_position.pin_point.z;
		bool belong_pseudo_s = (x == Target.x) & (y == Target.y) & (z == Target.z);
		//printf("target (%d,%d,%d)\n", x, y, z);
		if (belong_pseudo_s && z == 1)
		{
			Parser::wire_path wire;
			wire.path_type = 2;
			wire.Layer = Target.z;
			tuple<coor_t, coor_t, int> temp_coor;
			temp_coor = ISPD_real_coor_table.Index2Coor(x, y, z);
			//printf("Target FallingVia(1) : %d\n", ISPD_Viatype_array_[Target.z][Target.y][Target.x].FallingVia.first);
			//printf("Target FallingVia(2) : %d\n", ISPD_Viatype_array_[Target.z][Target.y][Target.x].FallingVia.second);
			if (ISPD_Viatype_array_[Target.z][Target.y][Target.x].FallingVia.first == -1 ||
				ISPD_Viatype_array_[Target.z][Target.y][Target.x].FallingVia.second == -1)
			{
				wire.ViaName = ISPD_via_type_list.at(0).at(0).Name;
				TarEnc.Layer = 0;
				TarEnc.LB = make_pair(ISPD_via_type_list.at(0).at(0).BotLayerIRect.LB.first + get<0>(target_coor), ISPD_via_type_list.at(0).at(0).BotLayerIRect.LB.second + get<1>(target_coor));
				TarEnc.RT = make_pair(ISPD_via_type_list.at(0).at(0).BotLayerIRect.RT.first + get<0>(target_coor), ISPD_via_type_list.at(0).at(0).BotLayerIRect.RT.second + get<1>(target_coor));
			}
			else
			{
				wire.ViaName = ISPD_via_type_list.at(ISPD_Viatype_array_[Target.z][Target.y][Target.x].FallingVia.first).at(ISPD_Viatype_array_[Target.z][Target.y][Target.x].FallingVia.second).Name;
				int id_1 = ISPD_Viatype_array_[Target.z][Target.y][Target.x].FallingVia.first;
				int id_2 = ISPD_Viatype_array_[Target.z][Target.y][Target.x].FallingVia.second;
				TarEnc.Layer = 0;
				TarEnc.LB = make_pair(ISPD_via_type_list.at(id_1).at(id_2).BotLayerIRect.LB.first + get<0>(target_coor), ISPD_via_type_list.at(id_1).at(id_2).BotLayerIRect.LB.second + get<1>(target_coor));
				TarEnc.RT = make_pair(ISPD_via_type_list.at(id_1).at(id_2).BotLayerIRect.RT.first + get<0>(target_coor), ISPD_via_type_list.at(id_1).at(id_2).BotLayerIRect.RT.second + get<1>(target_coor));
			}
			wire.Src_Pin = make_pair(get<0>(temp_coor), get<1>(temp_coor));
			Net_wirePath[net_id].push_back(wire);

			TPseudo = false;
			break;
		}
		else if (belong_pseudo_s && z > 1)
		{
			Parser::wire_path path;
			if (MemBlockRefine(connection.Tar_Pin_Shape, Target, path))
			{
				Net_wirePath[net_id].push_back(path);
			}
		}
	}
	if (TPseudo)
	{
		for (int i = 0; i < connection.pseudo_Target_pin_path_list.size(); i++)
		{

			int x = connection.pseudo_Target_pin_path_list[i].pin_position.pin_point.x;
			int y = connection.pseudo_Target_pin_path_list[i].pin_position.pin_point.y;
			int z = connection.pseudo_Target_pin_path_list[i].pin_position.pin_point.z;
			//printf("p-target (%d,%d,%d)\n", x, y, z);
			bool belong_pseudo_t = (x == Target.x) & (y == Target.y) & (z == Target.z);
			if (belong_pseudo_t && z == 1)
			{
				TPseudo = true;
				//Parser::wire_path wire;
				//OffGridViaRefinment(Target, connection.Tar_Pin_Shape, wire, net_id);
				auto RPath = TargetRefinmentMap.find(Target);
				if (RPath != TargetRefinmentMap.end())
				{
					TarEnc = RPath->second.Enclosure;
					for (int path = 0; path < RPath->second.path.size(); path++)
					{
						Net_wirePath[net_id].push_back(RPath->second.path[path]);
					}
				}
				else
				{
					auto RPathi = TargetRefinmentMap.begin();
					for (; RPathi != TargetRefinmentMap.end(); RPathi++)
					{
						if (RPathi->first.x == Target.x && RPathi->first.y == Target.y){
							TarEnc = RPathi->second.Enclosure;
							for (int path = 0; path < RPathi->second.path.size(); path++)
							{
								Net_wirePath[net_id].push_back(RPathi->second.path[path]);
							}

								break;
						}
					}
					if (RPathi == TargetRefinmentMap.end()){
						;
					}
				}
			}
		}
	}
}

// for maze route and ISPD_2PinNet
void SmallRouter::DetailPath2RipUpPath(list<Node> &detail_grid_path, TwoPinRipUpNet &ripup_net, int net_index)
{
	(*ripup_net.TwoPinNetData).RipUp_path.clear();
	ripup_net.PinToBitwise.clear();
	//cout << "Path length : " << detail_grid_path.size() << endl ;

	for (auto p = detail_grid_path.begin(); p != detail_grid_path.end(); p++)
	{

		(*ripup_net.TwoPinNetData).RipUp_path.push_back(Parser::RipUpNode(p->x, p->y, p->z));
		Pin newpin;
		newpin.SetPinPoint(kPinPoint, p->x, p->y, p->z);
		ripup_net.PinToBitwise.push_back(newpin);
		ripup_net.PreNetPath.push_back(newpin);
		if(this->ISPD_NetID_array[p->z][p->y][p->x] == -1)
		{
			this->ISPD_NetID_array[p->z][p->y][p->x] = net_index;
		}
		else 
		{
			//printf("Overlapped (%d, %d, %d) (%d) (%d)\n",p->x, p->y, p->z, this->ISPD_NetID_array[p->z][p->y][p->x], net_index);
			this->rip_up_two_pin_nets_.at(this->ISPD_NetID_array[p->z][p->y][p->x]).ReferenceRipUpNet.push_back(net_index);
			this->ISPD_NetID_array[p->z][p->y][p->x] = net_index;
		}
		
	}
}

void SmallRouter::Push_path_into_blockage(list<Node> &detail_grid_path, int id){
	
	for (auto p = detail_grid_path.begin(); p != detail_grid_path.end(); p++){	
		(*ISPD_blockage_array_)[p->z][p->y][p->x] = id;

		ISPD_Insert_PathHit_array_(p->x, p->y, (int)p->z);
	}

}

void SmallRouter::Push_path_into_CongestionMap(list<Node> &detail_grid_path, int id){

	for (auto p = detail_grid_path.begin(); p != detail_grid_path.end(); p++){
		(*ISPD_Congestion_Map)[p->z][p->y][p->x].second++;
	}

}

bool SmallRouter::BackTrack(
		const int two_pin_net_index,
		const Node &source,
		const Node &target
		)
{
	
	const bool path_is_found = 
		rip_up_two_pin_nets_[two_pin_net_index].path_is_found;
	if(!path_is_found)
	{
		rip_up_two_pin_nets_[two_pin_net_index].GetDetailedGridPath().clear();
		return false;
	}

	TwoPinNetDetailedPath temp_path;
	temp_path.detail_grid_path.push_front(target);
	

	bool is_tracing_sucessful = true;
	int back_from_x = target.x;
	int back_from_y = target.y;
	int back_from_z = target.z;
	while((back_from_x != source.x) || 
			(back_from_y != source.y) || 
			(back_from_z != source.z))
	{
		const ULL back_track_index = 
			Transform3dArrayTo1dIndex(back_from_x, back_from_y, back_from_z);
		
		if(back_track_array_[back_track_index] == kRight)
		{
			back_from_x -= kMoveStep;
		}
		else if(back_track_array_[back_track_index] == kLeft)
		{
			back_from_x += kMoveStep;
		}
		else if(back_track_array_[back_track_index] == kTop)
		{
			back_from_y -= kMoveStep;
		}
		else if(back_track_array_[back_track_index] == kBottom)
		{
			back_from_y += kMoveStep;
		}
		else if(back_track_array_[back_track_index] == kUp)
		{
			back_from_z -= kMoveStep;
		}
		else if(back_track_array_[back_track_index] == kDown)
		{
			back_from_z += kMoveStep;
		}
		else
		{
			cout << "something wrong in back track array!!!" << endl;
			cout << "old bc = " << back_track_array_[back_track_index] << endl;
			is_tracing_sucessful = false;
			break;
		}
		
		Node back_node(back_from_x, back_from_y, back_from_z);
		temp_path.detail_grid_path.push_front(back_node);
	}

	if(is_tracing_sucessful)
		rip_up_two_pin_nets_[two_pin_net_index].GetDetailedGridPath().assign( 
				temp_path.detail_grid_path.begin(), temp_path.detail_grid_path.end());
	else
		rip_up_two_pin_nets_[two_pin_net_index].GetDetailedGridPath().clear();

	return is_tracing_sucessful;
}

// Extract Line-End block from rect and set them to blockage map
void SmallRouter::ExtractLineEndBlock(Parser::I_Rect &block,int net_id)
{
	int layer = block.Layer;
	Parser::I_Rect EOL1;
	Parser::I_Rect EOL2;
	EOL1.Layer = layer;
	EOL2.Layer = layer;
	int spacing = this->SpaceEvaluationLayout[layer].spacing_table[0].second;
	Parser::INT_EOL_Rule eol_rule = this->SpaceEvaluationLayout[layer].eol_rule;
	if (block.RT.first - block.LB.first < block.RT.second - block.LB.second)
	{
		// Bot EOL
		EOL1.LB.first = block.LB.first - eol_rule.WITHIN;
		EOL1.RT.first = block.RT.first + eol_rule.WITHIN;
		EOL1.LB.second = block.LB.second - eol_rule.SPACING - spacing;
		EOL1.RT.second = block.LB.second;
		// Top EOL
		EOL2.LB.first = block.LB.first - eol_rule.WITHIN;
		EOL2.RT.first = block.RT.first + eol_rule.WITHIN;
		EOL2.LB.second = block.RT.second;
		EOL2.RT.second = block.RT.second + eol_rule.SPACING + spacing;
	}
	else
	{
		// Left EOL
		EOL1.LB.first = block.LB.first - eol_rule.SPACING - spacing;
		EOL1.RT.first = block.LB.first;
		EOL1.LB.second = block.LB.second - eol_rule.WITHIN;
		EOL1.RT.second = block.RT.second + eol_rule.WITHIN;
		// Right EOL
		EOL2.LB.first = block.RT.first;
		EOL2.RT.first = block.RT.first + eol_rule.SPACING + spacing;
		EOL2.LB.second = block.LB.second - eol_rule.WITHIN;
		EOL2.RT.second = block.RT.second + eol_rule.WITHIN;
	}

	Parser::I_Rect block_range;
	this->RectangleMapping.MapRect_ShrinkedRange(EOL1, block_range);
	for (int x = block_range.LB.first; x <= block_range.RT.first; x++)
	{
		for (int y = block_range.LB.second; y <= block_range.RT.second; y++)
		{
			(*this->ISPD_blockage_array_)[layer][y][x] = net_id;
		}
	}

	this->RectangleMapping.MapRect_ShrinkedRange(EOL2, block_range);
	for (int x = block_range.LB.first; x <= block_range.RT.first; x++)
	{
		for (int y = block_range.LB.second; y <= block_range.RT.second; y++)
		{
			(*this->ISPD_blockage_array_)[layer][y][x] = net_id;
		}
	}
}
// Set Rect To  3 Block Map 
void SmallRouter::PushRectIntoBlockage(Parser::I_Rect &block,int net_id,int layer){
	block.Layer = layer;
	// Blockage Map
	Parser::I_Rect block_range;
	block_range.clear();
	Parser::I_Rect b_block = block;
	int half_width = (ISPD_MinWidth_list[layer]/2);
	b_block.LB.first -= (this->SpaceEvaluationLayout[layer].spacing_table[0].second + half_width);
	b_block.LB.second -= (this->SpaceEvaluationLayout[layer].spacing_table[0].second + half_width);
	b_block.RT.first += (this->SpaceEvaluationLayout[layer].spacing_table[0].second + half_width);
	b_block.RT.second += (this->SpaceEvaluationLayout[layer].spacing_table[0].second + half_width);
	//printf("block LB(%d,%d) RT(%d,%d)\n", block.LB.first, block.LB.second, block.RT.first, block.RT.second);
	//printf("inflate block LB(%d,%d) RT(%d,%d)\n", b_block.LB.first, b_block.LB.second, b_block.RT.first, b_block.RT.second);
	this->RectangleMapping.MapRect_ShrinkedRange(b_block, block_range);
	//printf("block LB(%d,%d) RT(%d,%d)\n", block.LB.first, block.LB.second, block.RT.first, block.RT.second);
	//printf("block range LB(%d,%d) RT(%d,%d)\n", block_range.LB.first, block_range.LB.second, block_range.RT.first, block_range.RT.second);
	//printf("coor range LB(%d,%d) RT(%d,%d)\n", block_range.LB.first, block_range.LB.second, block_range.RT.first, block_range.RT.second);

	for (int x = block_range.LB.first; x <= block_range.RT.first; x++)
	{
		for (int y = block_range.LB.second; y <= block_range.RT.second; y++){
			(*this->ISPD_blockage_array_)[layer][y][x] = net_id;
		}
	}
	// Line end block
	ExtractLineEndBlock(block,net_id);
	// EOL Map
	Parser::I_Rect eol_block = block;
	eol_block.LB.first -= (this->SpaceEvaluationLayout[layer].eol_rule.SPACING + half_width);
	eol_block.LB.second -= (this->SpaceEvaluationLayout[layer].eol_rule.SPACING + half_width);
	eol_block.RT.first += (this->SpaceEvaluationLayout[layer].eol_rule.SPACING + half_width);
	eol_block.RT.second += (this->SpaceEvaluationLayout[layer].eol_rule.SPACING + half_width);
	this->RectangleMapping.MapRect_ShrinkedRange(eol_block, block_range);
	for (int x = block_range.LB.first; x <= block_range.RT.first; x++)
	{
		for (int y = block_range.LB.second; y <= block_range.RT.second; y++)
		{
			(*this->ISPD_EOL_Blocage_Map)[layer][y][x] = net_id;
		}
	}

	if(layer == 0){
		// Space Evaluation Graph
		this->SpaceEvaluationLayout[layer].PushNetBlockageToTree(block, net_id);
	}
	
}

void SmallRouter::SetViaEnclosureBlockage(Node back_from,Node new_node,int dir,pair <int , int> via_id,int net_id){

	if (dir == kUp){
		int via_1id = via_id.first;
		int via_2id = via_id.second;
		int new_x = new_node.x;
		int new_y = new_node.y;
		int new_z = new_node.z;
		int back_from_x = back_from.x;
		int back_from_y = back_from.y;
		int back_from_z = back_from.z;

		//printf("new z : %d / back z : %d\n", new_z, back_from_z);
		tuple<coor_t, coor_t, int> new_coor;
		new_coor = ISPD_real_coor_table.Index2Coor(new_x, new_y, new_z);

		tuple<coor_t, coor_t, int> back_coor;
		back_coor = ISPD_real_coor_table.Index2Coor(back_from_x, back_from_y, back_from_z);

		I_Rect BotEnclosure;
		BotEnclosure.LB.first = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.LB.first + get<0>(new_coor);
		BotEnclosure.LB.second = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.LB.second + get<1>(new_coor);
		BotEnclosure.RT.first = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.RT.first + get<0>(new_coor);
		BotEnclosure.RT.second = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.RT.second + get<1>(new_coor);

		I_Rect TopEnclosure;
		TopEnclosure.LB.first = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.LB.first + get<0>(back_coor);
		TopEnclosure.LB.second = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.LB.second + get<1>(back_coor);
		TopEnclosure.RT.first = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.RT.first + get<0>(back_coor);
		TopEnclosure.RT.second = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.RT.second + get<1>(back_coor);

#ifdef _SpaceEvaluationGraph_
		this->SpaceEvaluationLayout[new_z].PushNetBlockageToTree(BotEnclosure, net_id);
		this->SpaceEvaluationLayout[back_from_z].PushNetBlockageToTree(TopEnclosure, net_id);
#endif

#ifdef _VIA_BIT_MAP_
		if(new_z == 0) {
			this->SpaceEvaluationLayout[new_z].PushNetBlockageToTree(BotEnclosure, net_id);
		}
		else{
			int enc_type_id = ISPD_enc_id_HashTable.GetViaBitID(via_id, false ,new_z);
			ISPD_Via_Type_Map[new_z][new_y][new_x].SetType(enc_type_id);
		}
		if(back_from_z == 0){
			this->SpaceEvaluationLayout[back_from_z].PushNetBlockageToTree(TopEnclosure, net_id);
		}
		else{
			int enc_type_id = ISPD_enc_id_HashTable.GetViaBitID(via_id, true ,back_from_z);
			ISPD_Via_Type_Map[back_from_z][back_from_y][back_from_x].SetType(enc_type_id);
		}
#endif
/*#ifdef _CrossLineGraph_
		this->SpaceEvaluationLayout[new_z].HitViaEnclosure(new_x, new_y, BotEnclosure, net_id);
		this->SpaceEvaluationLayout[back_from_z].HitViaEnclosure(back_from_x, back_from_y, TopEnclosure, net_id);
#endif*/

		PushRectIntoBlockage(BotEnclosure, net_id, new_z);
		PushRectIntoBlockage(TopEnclosure, net_id, back_from_z);
	}
	else if (dir == kDown){
		int via_1id = via_id.first;
		int via_2id = via_id.second;
		int new_x = new_node.x;
		int new_y = new_node.y;
		int new_z = new_node.z;
		int back_from_x = back_from.x;
		int back_from_y = back_from.y;
		int back_from_z = back_from.z;

		tuple<coor_t, coor_t, int> new_coor;
		new_coor = ISPD_real_coor_table.Index2Coor(new_x, new_y, new_z);

		tuple<coor_t, coor_t, int> back_coor;
		back_coor = ISPD_real_coor_table.Index2Coor(back_from_x, back_from_y, back_from_z);
		//printf("new z : %d / back z : %d\n", new_z, back_from_z);
		I_Rect BotEnclosure;
		BotEnclosure.LB.first = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.LB.first + get<0>(back_coor);
		BotEnclosure.LB.second = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.LB.second + get<1>(back_coor);
		BotEnclosure.RT.first = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.RT.first + get<0>(back_coor);
		BotEnclosure.RT.second = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.RT.second + get<1>(back_coor);

		I_Rect TopEnclosure;
		TopEnclosure.LB.first = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.LB.first + get<0>(new_coor);
		TopEnclosure.LB.second = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.LB.second + get<1>(new_coor);
		TopEnclosure.RT.first = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.RT.first + get<0>(new_coor);
		TopEnclosure.RT.second = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.RT.second + get<1>(new_coor);

#ifdef _SpaceEvaluationGraph_
		this->SpaceEvaluationLayout[new_z].PushNetBlockageToTree(TopEnclosure, net_id);
		this->SpaceEvaluationLayout[back_from_z].PushNetBlockageToTree(BotEnclosure, net_id);
#endif

#ifdef _VIA_BIT_MAP_
		if(new_z == 0) {
			this->SpaceEvaluationLayout[new_z].PushNetBlockageToTree(TopEnclosure, net_id);
		}
		else{
			int enc_type_id = ISPD_enc_id_HashTable.GetViaBitID(via_id, true ,new_z);
			ISPD_Via_Type_Map[new_z][new_y][new_x].SetType(enc_type_id);
		}
		if(back_from_z == 0){
			this->SpaceEvaluationLayout[back_from_z].PushNetBlockageToTree(BotEnclosure, net_id);
		}
		else{
			int enc_type_id = ISPD_enc_id_HashTable.GetViaBitID(via_id, false ,back_from_z);
			ISPD_Via_Type_Map[back_from_z][back_from_y][back_from_x].SetType(enc_type_id);
		}
#endif

/*#ifdef _CrossLineGraph_
		this->SpaceEvaluationLayout[new_z].HitViaEnclosure(new_x, new_y, TopEnclosure, net_id);
		this->SpaceEvaluationLayout[back_from_z].HitViaEnclosure(back_from_x, back_from_y, BotEnclosure, net_id);
#endif*/

		PushRectIntoBlockage(BotEnclosure, net_id, back_from_z);
		PushRectIntoBlockage(TopEnclosure, net_id, new_z);
	}
}

void SmallRouter::ExtractViaFromBackTrack(int back_from_z,int back_from_y, int back_from_x,Parser::I_Rect &Enc,bool src_tar){
	
	tuple <coor_t,coor_t,int > temp_coor;
	temp_coor = ISPD_real_coor_table.Index2Coor(back_from_x, back_from_y, back_from_z);
	if (src_tar)
	{
		if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kUp){
			int via_1id = ISPD_Viatype_array_[back_from_z][back_from_y][back_from_x].FallingVia.first;
			int via_2id = ISPD_Viatype_array_[back_from_z][back_from_y][back_from_x].FallingVia.second;
			Enc.LB.first = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.LB.first + get<0>(temp_coor);
			Enc.LB.second = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.LB.second + get<1>(temp_coor);
			Enc.RT.first = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.RT.first + get<0> (temp_coor);
			Enc.RT.second = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.RT.second + get<1>(temp_coor);
			Enc.Layer = back_from_z - 1;
		}
		else if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kDown){
			int via_1id = ISPD_Viatype_array_[back_from_z][back_from_y][back_from_x].RaisingVia.first;
			int via_2id = ISPD_Viatype_array_[back_from_z][back_from_y][back_from_x].RaisingVia.second;
			Enc.LB.first = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.LB.first + get<0> (temp_coor);
			Enc.LB.second = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.LB.second + get<1>(temp_coor);
			Enc.RT.first = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.RT.first + get<0> (temp_coor);
			Enc.RT.second = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.RT.second + get<1>(temp_coor);
			Enc.Layer = back_from_z + 1;
		}
	}
	else{

		if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kUp){
			int via_1id = ISPD_Viatype_array_[back_from_z][back_from_y][back_from_x].FallingVia.first;
			int via_2id = ISPD_Viatype_array_[back_from_z][back_from_y][back_from_x].FallingVia.second;
			Enc.LB.first = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.LB.first + get<0> (temp_coor);
			Enc.LB.second = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.LB.second + get<1> (temp_coor);
			Enc.RT.first = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.RT.first + get<0> (temp_coor);
			Enc.RT.second = this->ISPD_via_type_list[via_1id][via_2id].TopLayerIRect.RT.second + get<1>(temp_coor);
			Enc.Layer = back_from_z;
		}
		else if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kDown){
			int via_1id = ISPD_Viatype_array_[back_from_z][back_from_y][back_from_x].RaisingVia.first;
			int via_2id = ISPD_Viatype_array_[back_from_z][back_from_y][back_from_x].RaisingVia.second;
			Enc.LB.first = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.LB.first + get<0> (temp_coor);
			Enc.LB.second = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.LB.second + get<1>(temp_coor);
			Enc.RT.first = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.RT.first + get<0>(temp_coor);
			Enc.RT.second = this->ISPD_via_type_list[via_1id][via_2id].BotLayerIRect.RT.second + get<1>(temp_coor);
			Enc.Layer = back_from_z;
		}
	}
}

void SmallRouter::LongerStackVia(Node &cur_node, wire_path & addition){
	;
}

bool SmallRouter::BackTrack(
	const int two_pin_net_index,
	const Node &back_target_point,
	TwoPinNetConnection &connection,
	const int net_id, Parser::I_Rect &SrcEnc, Parser::I_Rect &TarEnc,
	Node &source, Node &target)
{

	const bool path_is_found = 
		rip_up_two_pin_nets_[two_pin_net_index].path_is_found;
	if(!path_is_found)
	{
		rip_up_two_pin_nets_[two_pin_net_index].GetDetailedGridPath().clear();
		return false;
	}

	bool path_okay = true;
	TwoPinNetDetailedPath temp_path;
	int back_from_x = back_target_point.x;
	int back_from_y = back_target_point.y;
	int back_from_z = back_target_point.z;
	target.SetNode(back_from_x, back_from_y, back_from_z);
	// store the target node to its path
	temp_path.detail_grid_path.push_front(back_target_point);
	/*if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kUp || ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kDown)
		ExtractViaFromBackTrack(back_from_z, back_from_y, back_from_x, TarEnc,false);*/
	//printf("target (%d,%d,%d)\n", back_from_x, back_from_y, back_from_z);
	//const ULL target_index = Transform3dArrayTo1dIndex(back_target_point);
	bool reach_source = (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kNoDirection);
	while(!reach_source)
	{
		const ULL back_track_index = 
			Transform3dArrayTo1dIndex(back_from_x, back_from_y, back_from_z);
		
		if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kRight)
		{
			source.SetNode(back_from_x, back_from_y, back_from_z); 
			back_from_x -= ISPD_back_track_Step_array_[back_from_z][back_from_y][back_from_x];
			
		}
		else if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kLeft)
		{
			source.SetNode(back_from_x, back_from_y, back_from_z);
			back_from_x += ISPD_back_track_Step_array_[back_from_z][back_from_y][back_from_x];
			
		}
		else if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kTop)
		{
			source.SetNode(back_from_x, back_from_y, back_from_z);
			back_from_y -= ISPD_back_track_Step_array_[back_from_z][back_from_y][back_from_x];
			
		}
		else if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kBottom)
		{
			source.SetNode(back_from_x, back_from_y, back_from_z);
			back_from_y += ISPD_back_track_Step_array_[back_from_z][back_from_y][back_from_x];
			
		}
		else if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kUp)
		{
			source.SetNode(back_from_x, back_from_y, back_from_z);
			auto falling_node = ISPD_connection_table.FallingNode_Index(back_from_x, back_from_y, back_from_z);
			int new_x = get<0>(falling_node);
			int new_y = get<1>(falling_node);
			int new_z = get<2>(falling_node);
			//ExtractViaFromBackTrack(back_from_z, back_from_y, back_from_x, SrcEnc,true);
			// Set Via enclosure to blockage
			Node back_from(back_from_x, back_from_y, back_from_z);
			Node new_node(new_x, new_y, new_z);
			SetViaEnclosureBlockage(back_from, new_node, kUp, 
				ISPD_Viatype_array_[back_from_z][back_from_y][back_from_x].FallingVia,net_id);
			back_from_x = new_x;
			back_from_y = new_y;
			back_from_z = new_z;
			/*if (stack_via){
				wire_path addition;
				LongerStackVia(back_from, addition);
				Net_wirePath[net_id].push_back(addition);
			}*/
			
		}
		else if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kDown)
		{
			source.SetNode(back_from_x, back_from_y, back_from_z);
			auto raising_node = ISPD_connection_table.RaisingNode_Index(back_from_x, back_from_y, back_from_z);
			int new_x = get<0>(raising_node);
			int new_y = get<1>(raising_node);
			int new_z = get<2>(raising_node);
			//ExtractViaFromBackTrack(back_from_z, back_from_y, back_from_x, SrcEnc,true);
			Node back_from(back_from_x, back_from_y, back_from_z);
			Node new_node(new_x, new_y, new_z);
			SetViaEnclosureBlockage(back_from, new_node, kDown,
				ISPD_Viatype_array_[back_from_z][back_from_y][back_from_x].RaisingVia, net_id);

			back_from_x = new_x;
			back_from_y = new_y;
			back_from_z = new_z;
			/*if (stack_via){
				wire_path addition;
				LongerStackVia(back_from, addition);
				Net_wirePath[net_id].push_back(addition);
			}*/
			
		}
		else
		{
			//cout << "something wrong in back track array!!!" << endl;
			//cout << "new bc = " << ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] << endl;
			path_okay = false;
			break;
		}
		
		Node back_node(back_from_x, back_from_y, back_from_z);
		temp_path.detail_grid_path.push_front(back_node);
		//const ULL back_node_index = Transform3dArrayTo1dIndex(back_node);
		//printf("Node (%d,%d,%d)\n", back_from_x, back_from_y, back_from_z);
		reach_source = (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kNoDirection);
	}

	source.SetNode(back_from_x, back_from_y, back_from_z);
	rip_up_two_pin_nets_[two_pin_net_index].GetDetailedGridPath().assign(
			temp_path.detail_grid_path.begin(), temp_path.detail_grid_path.end());

	return path_okay;
}


bool SmallRouter::LiteBackTrack(
	const int two_pin_net_index,
	const Node &back_target_point,
	TwoPinNetConnection &connection,
	const int net_id
	)
{

	const bool path_is_found =
		rip_up_two_pin_nets_[two_pin_net_index].path_is_found;
	if (!path_is_found)
	{
		rip_up_two_pin_nets_[two_pin_net_index].GetDetailedGridPath().clear();
		return false;
	}

	bool path_okay = true;
	TwoPinNetDetailedPath temp_path;
	int back_from_x = back_target_point.x;
	int back_from_y = back_target_point.y;
	int back_from_z = back_target_point.z;
	Node target(back_from_x, back_from_y, back_from_z);
	Node source;
	// store the target node to its path
	temp_path.detail_grid_path.push_front(back_target_point);
	/*if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kUp || ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kDown)
	ExtractViaFromBackTrack(back_from_z, back_from_y, back_from_x, TarEnc,false);*/
	bool stack_via = false;
	//const ULL target_index = Transform3dArrayTo1dIndex(back_target_point);
	bool reach_source = (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kNoDirection);
	while (!reach_source)
	{
		const ULL back_track_index =
			Transform3dArrayTo1dIndex(back_from_x, back_from_y, back_from_z);

		if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kRight)
		{
			source.SetNode(back_from_x, back_from_y, back_from_z);
			back_from_x -= ISPD_back_track_Step_array_[back_from_z][back_from_y][back_from_x];
			stack_via = false;
		}
		else if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kLeft)
		{
			source.SetNode(back_from_x, back_from_y, back_from_z);
			back_from_x += ISPD_back_track_Step_array_[back_from_z][back_from_y][back_from_x];
			stack_via = false;
		}
		else if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kTop)
		{
			source.SetNode(back_from_x, back_from_y, back_from_z);
			back_from_y -= ISPD_back_track_Step_array_[back_from_z][back_from_y][back_from_x];
			stack_via = false;
		}
		else if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kBottom)
		{
			source.SetNode(back_from_x, back_from_y, back_from_z);
			back_from_y += ISPD_back_track_Step_array_[back_from_z][back_from_y][back_from_x];
			stack_via = false;
		}
		else if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kUp)
		{
			source.SetNode(back_from_x, back_from_y, back_from_z);
			auto falling_node = ISPD_connection_table.FallingNode_Index(back_from_x, back_from_y, back_from_z);
			int new_x = get<0>(falling_node);
			int new_y = get<1>(falling_node);
			int new_z = get<2>(falling_node);
			//ExtractViaFromBackTrack(back_from_z, back_from_y, back_from_x, SrcEnc,true);
			// Set Via enclosure to blockage
			Node back_from(back_from_x, back_from_y, back_from_z);
			Node new_node(new_x, new_y, new_z);
			/*SetViaEnclosureBlockage(back_from, new_node, kUp,
				ISPD_Viatype_array_[back_from_z][back_from_y][back_from_x].FallingVia, net_id);*/
			back_from_x = new_x;
			back_from_y = new_y;
			back_from_z = new_z;
			/*if (stack_via){
			wire_path addition;
			LongerStackVia(back_from, addition);
			Net_wirePath[net_id].push_back(addition);
			}*/
			stack_via = true;
		}
		else if (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kDown)
		{
			source.SetNode(back_from_x, back_from_y, back_from_z);
			auto raising_node = ISPD_connection_table.RaisingNode_Index(back_from_x, back_from_y, back_from_z);
			int new_x = get<0>(raising_node);
			int new_y = get<1>(raising_node);
			int new_z = get<2>(raising_node);
			//ExtractViaFromBackTrack(back_from_z, back_from_y, back_from_x, SrcEnc,true);
			Node back_from(back_from_x, back_from_y, back_from_z);
			Node new_node(new_x, new_y, new_z);
			/*SetViaEnclosureBlockage(back_from, new_node, kDown,
				ISPD_Viatype_array_[back_from_z][back_from_y][back_from_x].RaisingVia, net_id);*/

			back_from_x = new_x;
			back_from_y = new_y;
			back_from_z = new_z;
			/*if (stack_via){
			wire_path addition;
			LongerStackVia(back_from, addition);
			Net_wirePath[net_id].push_back(addition);
			}*/
			stack_via = true;
		}
		else
		{
			//cout << "something wrong in back track array!!!" << endl;
			//cout << "new bc = " << ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] << endl;
			path_okay = false;
			break;
		}

		Node back_node(back_from_x, back_from_y, back_from_z);
		temp_path.detail_grid_path.push_front(back_node);
		//const ULL back_node_index = Transform3dArrayTo1dIndex(back_node);
		//printf("Node (%d,%d,%d)\n", back_from_x, back_from_y, back_from_z);
		reach_source = (ISPD_back_track_array_[back_from_z][back_from_y][back_from_x] == kNoDirection);
	}
	//SrcEnc.Layer--;
	//ExtractViaFromBackTrack(back_from_z, back_from_y, back_from_x, SrcEnc);
	source.SetNode(back_from_x, back_from_y, back_from_z);
	rip_up_two_pin_nets_[two_pin_net_index].GetDetailedGridPath().assign(
		temp_path.detail_grid_path.begin(), temp_path.detail_grid_path.end());
	/*printf("PseusoNodeDetermin\n");
	PseusoNodeDetermin(connection, source, target, net_id, SrcEnc, TarEnc);
	printf("PseusoNodeDetermin End\n");*/
	return path_okay;
}



// Find the Turns(or Bends) of a Path after Backtracing.
/*void SmallRouter::FindPathTurns(const int two_pin_net_index)
{
	TwoPinRipUpNet &net = rip_up_two_pin_nets_[two_pin_net_index];
	vector<TurnNode> &net_path_turn_nodes = net.GetPathTurnNodes();
	net_path_turn_nodes.clear();
	
	// If the path of a net is found, then find the turns of the path.
	// Otherwise, not do it.
	const bool path_is_found = net.path_is_found;
	if(!path_is_found)
		return;
	
	const list<Node> &grid_path = net.GetDetailedGridPath();
	// 1. Target part
	const Node &target = *(grid_path.rbegin());
	LineEndType target_le_type = kSegNone;
	const ULL target_index = Transform3dArrayTo1dIndex(target);
	const int target_direction = this->back_track_array_[target_index];
	switch(target_direction)
	{
		case kRight:
			target_le_type = kSegRight;
			break;
		case kLeft:
			target_le_type = kSegLeft;
			break;
		case kTop:
			target_le_type = kSegTop;
			break;
		case kBottom:
			target_le_type = kSegBottom;
			break;
		case kUp:
		case kDown:
			target_le_type = kSegNone;
			break;
		default:
			// do nothing
			;
	}
	// the target
	net_path_turn_nodes.push_back(TurnNode(target, target_le_type));
	
	// check the node that is connected to pin through a via.
	const bool target_connected_via = 
		(target_direction == kUp || target_direction == kDown);
	if(target_connected_via)
	{
		Node next_node = Node(target);
		switch(target_direction)
		{
			case kUp:
				next_node.z -= kMoveStep;
				break;
			case kDown:
				next_node.z += kMoveStep;
				break;
			default:
				// do nothing
				;
		}

		LineEndType next_node_le_type = kSegNone;
		const ULL next_node_index = Transform3dArrayTo1dIndex(next_node);
		const int next_node_direction = back_track_array_[next_node_index];
		switch(next_node_direction)
		{
			case kRight:
				next_node_le_type = kSegRight;
				break;
			case kLeft:
				next_node_le_type = kSegLeft;
				break;
			case kTop:
				next_node_le_type = kSegTop;
				break;
			case kBottom:
				next_node_le_type = kSegBottom;
				break;
			case kUp:
			case kDown:
				next_node_le_type = kSegNone;
				break;
			default:
					// do nothing
				;
		}

		const bool next_node_not_connect_another_via = 
			(next_node_le_type != kSegNone);
		if(next_node_not_connect_another_via)
			net_path_turn_nodes.push_back( \
					TurnNode(next_node, next_node_le_type));
	}


	// 2. Mediate Path part
	for(list<Node>::const_reverse_iterator path_node_iter = 
			(++grid_path.crbegin());
		path_node_iter != (--grid_path.crend());
		path_node_iter++)
	{
		const Node &current_node = (*path_node_iter);
		const ULL current_node_index = Transform3dArrayTo1dIndex(current_node);
		const bool go_through_via = 
			(
				(back_track_array_[current_node_index] == kUp) ||
				(back_track_array_[current_node_index] == kDown)
			);
		if(go_through_via)
		{
			list<Node>::const_reverse_iterator pre_iter = path_node_iter;
			pre_iter--;
			const Node &pre_node = (*pre_iter);
			const ULL pre_node_index = Transform3dArrayTo1dIndex(pre_node);
			const int pre_node_direction = back_track_array_[pre_node_index];
			
			LineEndType current_node_le_type = kSegNone;
			switch(pre_node_direction)
			{
				case kRight:
					current_node_le_type = kSegLeft;
					break;
				case kLeft:
					current_node_le_type = kSegRight;
					break;
				case kTop:
					current_node_le_type = kSegBottom;
					break;
				case kBottom:
					current_node_le_type = kSegTop;
					break;
				case kUp:
				case kDown:
					current_node_le_type = kSegNone;
					break;
				default:
						// do nothing
					;
			}
			net_path_turn_nodes.push_back(
					TurnNode(current_node, current_node_le_type));

			list<Node>::const_reverse_iterator post_iter = path_node_iter;
			post_iter++;
			const Node &post_node = (*post_iter);
			const ULL post_node_index = Transform3dArrayTo1dIndex(post_node);
			const int post_node_direction = back_track_array_[post_node_index];
			
			LineEndType post_node_le_type = kSegNone;
			switch(post_node_direction)
			{
				case kRight:
					post_node_le_type = kSegRight;
					break;
				case kLeft:
					post_node_le_type = kSegLeft;
					break;
				case kTop:
					post_node_le_type = kSegTop;
					break;
				case kBottom:
					post_node_le_type = kSegBottom;
					break;
				case kUp:
				case kDown:
					post_node_le_type = kSegNone;
					break;
				default:
						// do nothing
					;
			}
			const bool post_node_not_connect_another_via = 
				(post_node_le_type != kSegNone);
			if(post_node_not_connect_another_via)
				net_path_turn_nodes.push_back( \
						TurnNode(post_node, post_node_le_type));
		}
	}

	// 3. Source Part
	const Node &source = *(grid_path.begin()); 
	list<Node>::const_iterator second_node_iter = grid_path.begin();
	second_node_iter++;
	const Node &second_node = (*second_node_iter);
	const ULL second_node_index = Transform3dArrayTo1dIndex(second_node);
	const int second_node_direction = back_track_array_[second_node_index];

	LineEndType source_le_type = kSegNone;
	switch(second_node_direction)
	{
		case kRight:
			source_le_type = kSegLeft;
			break;
		case kLeft:
			source_le_type = kSegRight;
			break;
		case kTop:
			source_le_type = kSegBottom;
			break;
		case kBottom:
			source_le_type = kSegTop;
			break;
		case kUp:
		case kDown:
			source_le_type = kSegNone;
			break;
		default:
				// do nothing
			;
	}
	net_path_turn_nodes.push_back(TurnNode(source, source_le_type));

	return;
}*/

// Find the via enclosures of a grid path.
void SmallRouter::FindPathTurns(const int two_pin_net_index)
{
	TwoPinRipUpNet &net = rip_up_two_pin_nets_[two_pin_net_index];
	vector<TurnNode> &turn_nodes = net.GetPathTurnNodes();
	turn_nodes.clear();
	
	// If the path of a net is found, then find the turns of the path.
	// Otherwise, not do it.
	const bool path_is_found = net.path_is_found;
	if(!path_is_found)
		return;
	
	const list<Node> &grid_path = net.GetDetailedGridPath();
	for(list<Node>::const_reverse_iterator node_iter = grid_path.crbegin();
		node_iter != (--grid_path.crend()); node_iter++)
	{
		const Node &node = (*node_iter);
		if(node_iter != grid_path.crbegin())
		{
			const bool is_rbegin_node = false;
			// Current Node.
			list<Node>::const_reverse_iterator p_pre_node = node_iter;
			p_pre_node--;
			const Node &pre_node = (*p_pre_node);
			// Back Next Node.
			list<Node>::const_reverse_iterator p_post_node = node_iter;
			p_post_node++;
			const Node &post_node = (*p_post_node);
			GetTurnNodes(is_rbegin_node, node, pre_node, post_node, turn_nodes);
		}
		else if(node_iter == grid_path.crbegin())
		{
			const bool is_rbegin_node = true;
			// Back Next Node.
			list<Node>::const_reverse_iterator p_post_node = node_iter;
			p_post_node++;
			const Node &post_node = (*p_post_node);
			GetTurnNodes(is_rbegin_node, node, post_node, post_node, turn_nodes);
		}
		else
		{}
	}
	return;
}



inline void SmallRouter::GetTurnNodes(
		const bool is_rbegin_node,
		const Node &node, 
		const Node &pre_node, 
		const Node &post_node,
		vector<TurnNode> &turn_nodes
		)
{
	const int kEncWidth = 1;
	
	const ULL node_index = Transform3dArrayTo1dIndex(node);
	const int direction = back_track_array_[node_index];
	if(direction != kUp && direction != kDown)
		return;

	// The sequence of nodes is reverse.
	const int next_layer = (direction == kUp) ? 
		(node.z - 1) : (node.z + 1);
	// Current Node.
	const ULL pre_index = Transform3dArrayTo1dIndex(pre_node);
	const int pre_direction = back_track_array_[pre_index];
	// Back Next Node.
	const ULL post_index = Transform3dArrayTo1dIndex(post_node);
	const int post_direction = back_track_array_[post_index];
	if(!is_rbegin_node)
	{
		if(node.z % 2 == kOddLayer)
		{
			// Current Node.
			if(kViaEncExpansion_[node.z])
				if((node.x - kEncWidth) >= 0 && 
						pre_direction == kRight)
					turn_nodes.push_back( TurnNode( 
						Node(node.x-kEncWidth, node.y, node.z), kSegLeft));
				else if(node.x + kEncWidth < layout_width_ && 
						pre_direction == kLeft) 
					turn_nodes.push_back( TurnNode( 
						Node(node.x+kEncWidth, node.y, node.z), kSegRight));
				else
					;
			else
				if(pre_direction == kRight)
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y, node.z), kSegLeft));
				else if(pre_direction == kLeft)
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y, node.z), kSegRight));
				else
					;
			// Back Next Node.
			if(kViaEncExpansion_[next_layer])
			{
				if(post_direction != kNoDirection)
				{
					if((node.y - kEncWidth) >= 0 &&
							post_direction == kBottom)
						turn_nodes.push_back( TurnNode( 
							Node(node.x, node.y-kEncWidth, next_layer), kSegBottom));
					else if(node.y + kEncWidth < layout_height_ &&
							post_direction == kTop) 
						turn_nodes.push_back( TurnNode( 
							Node(node.x, node.y+kEncWidth, next_layer), kSegTop));
					else
						;
				}
				else
				{
					if((node.y - kEncWidth) >= 0)
						turn_nodes.push_back( TurnNode( 
							Node(node.x, node.y-kEncWidth, next_layer), kSegBottom));
					if(node.y + kEncWidth < layout_height_) 
						turn_nodes.push_back( TurnNode( 
							Node(node.x, node.y+kEncWidth, next_layer), kSegTop));
				}
			}
			else
			{
				if(post_direction != kNoDirection)
				{
					if(post_direction == kBottom)
						turn_nodes.push_back( TurnNode( 
							Node(node.x, node.y, next_layer), kSegBottom));
					else if(post_direction == kTop) 
						turn_nodes.push_back( TurnNode( 
							Node(node.x, node.y, next_layer), kSegTop));
					else
						;
				}
				else
				{
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y, next_layer), kSegNone));
				}
			}
		}
		else
		{
			// Current Node.
			if(kViaEncExpansion_[node.z])
				if((node.y - kEncWidth) >= 0 && 
						pre_direction == kTop) 
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y-kEncWidth, node.z), kSegBottom));
				else if(node.y + kEncWidth < layout_height_ &&
						pre_direction == kBottom) 
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y+kEncWidth, node.z), kSegTop));
				else
					;
			else
				if(pre_direction == kTop) 
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y, node.z), kSegBottom));
				else if(pre_direction == kBottom) 
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y, node.z), kSegTop));
				else
					;
			// Back Next Node.
			if(kViaEncExpansion_[next_layer])
			{
				if(post_direction != kNoDirection)
				{
					if((node.x - kEncWidth) >= 0 &&
							post_direction == kLeft) 
						turn_nodes.push_back( TurnNode( 
							Node(node.x-kEncWidth, node.y, next_layer), kSegLeft));
					else if(node.x + kEncWidth < layout_width_ && 
							post_direction == kRight)
						turn_nodes.push_back( TurnNode( 
							Node(node.x+kEncWidth, node.y, next_layer), kSegRight));
					else
						;
				}
				else
				{
					if((node.x - kEncWidth) >= 0) 
						turn_nodes.push_back( TurnNode( 
							Node(node.x-kEncWidth, node.y, next_layer), kSegLeft));
					if(node.x + kEncWidth < layout_width_)
						turn_nodes.push_back( TurnNode( 
							Node(node.x+kEncWidth, node.y, next_layer), kSegRight));
				}
			}
			else
			{
				if(post_direction != kNoDirection)
				{
					if(post_direction == kLeft) 
						turn_nodes.push_back( TurnNode( 
							Node(node.x, node.y, next_layer), kSegLeft));
					else if( post_direction == kRight)
						turn_nodes.push_back( TurnNode( 
							Node(node.x, node.y, next_layer), kSegRight));
					else
						;
				}
				else
				{
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y, next_layer), kSegNone));
				}
			}
		}
	}
	else
	{
		if(node.z % 2 == kOddLayer)
		{
			// Current Node.
			if(kViaEncExpansion_[node.z])
			{
				if((node.x - kEncWidth) >= 0)
					turn_nodes.push_back( TurnNode( 
						Node(node.x-kEncWidth, node.y, node.z), kSegLeft));
				if(node.x + kEncWidth < layout_width_) 
					turn_nodes.push_back( TurnNode( 
						Node(node.x+kEncWidth, node.y, node.z), kSegRight));
			}
			else
			{
				turn_nodes.push_back( TurnNode( 
					Node(node.x, node.y, node.z), kSegNone));
			}

			// Back Next Node.
			if(kViaEncExpansion_[next_layer])
				if((node.y - kEncWidth) >= 0 &&
						post_direction == kBottom)
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y-kEncWidth, next_layer), kSegBottom));
				else if(node.y + kEncWidth < layout_height_ &&
						post_direction == kTop) 
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y+kEncWidth, next_layer), kSegTop));
				else
					;
			else
				if(post_direction == kBottom)
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y, next_layer), kSegBottom));
				else if(post_direction == kTop) 
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y, next_layer), kSegTop));
				else
					;
		}
		else
		{
			// Current Node.
			if(kViaEncExpansion_[node.z])
			{
				if((node.y - kEncWidth) >= 0) 
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y-kEncWidth, node.z), kSegBottom));
				if(node.y + kEncWidth < layout_height_) 
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y+kEncWidth, node.z), kSegTop));
			}
			else
			{
				turn_nodes.push_back( TurnNode( 
					Node(node.x, node.y, node.z), kSegNone));
			}
			// Back Next Node.
			if(kViaEncExpansion_[next_layer])
				if((node.x - kEncWidth) >= 0 &&
						post_direction == kLeft) 
					turn_nodes.push_back( TurnNode( 
						Node(node.x-kEncWidth, node.y, next_layer), kSegLeft));
				else if(node.x + kEncWidth < layout_width_ && 
						post_direction == kRight)
					turn_nodes.push_back( TurnNode( 
						Node(node.x+kEncWidth, node.y, next_layer), kSegRight));
				else
					;
			else
				if(post_direction == kLeft) 
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y, next_layer), kSegLeft));
				else if(post_direction == kRight)
					turn_nodes.push_back( TurnNode( 
						Node(node.x, node.y, next_layer), kSegRight));
				else
					;
		}
	}
}


// initialize the reroute_two_pin_net_ vector 
// at the begining of first phase routing
void SmallRouter::InitialRerouteTwoPinNet()
{
	// check if some stuffs remained in reroute_two_pin_net_ vector
	if(reroute_two_pin_net_.size() > 0)
		reroute_two_pin_net_.clear();

	
	// set all the nets to be rerouted and then record it
	for(unsigned int two_pin_net_index = 0;
		two_pin_net_index < rip_up_two_pin_nets_.size();
		two_pin_net_index++)
	{
		TwoPinRipUpNet &ripup_net = 
			rip_up_two_pin_nets_[two_pin_net_index];
		const bool both_pins_are_overlapped = ripup_net.both_pins_are_overlapped;
		const bool has_only_one_pin = ripup_net.has_only_one_pin;
		if(!(both_pins_are_overlapped || has_only_one_pin))
		{
			TwoPinRerouteNet temp_net;
			temp_net.index = two_pin_net_index;
			temp_net.two_pin_net_id = ripup_net.two_pin_net_id;
			temp_net.rip_up_times = 0;
			temp_net.violation_num = 0;
			temp_net.wire_length = 
				ripup_net.GetConnection().GetEstimatedWireLength();
			temp_net.net_score = 0.0;
			temp_net.straight_ratio = 
				ripup_net.GetConnection().GetStraightRatio();
			temp_net.is_straight = 
				ripup_net.GetConnection().IsStraight();
			
			// set the history node of the net
			temp_net.history_node.clear();
			
			reroute_two_pin_net_.push_back(temp_net);
		}
	}
}


// find the two-pin nets having violations.
void SmallRouter::SetRerouteTwoPinNet()
{
	// cout << "Set Reroute Two Pin Net " << endl;
	// check if some nets remained in reroute_two_pin_net_ vector
	if(reroute_two_pin_net_.size() > 0)
		reroute_two_pin_net_.clear();
	
	unordered_set<int> reroute_id_set;
	unordered_set<int> violation_id_set;
	// find nets that need to be rerouted and then record it;
	for(unsigned int two_pin_net_index = 0;
		two_pin_net_index < rip_up_two_pin_nets_.size();
		two_pin_net_index++)
	{
		TwoPinRipUpNet &ripup_net = 
			rip_up_two_pin_nets_[two_pin_net_index];
		
		const bool both_pins_are_overlapped = 
			ripup_net.both_pins_are_overlapped;
		const bool has_only_one_pin = 
			ripup_net.has_only_one_pin;
		if(both_pins_are_overlapped || has_only_one_pin)
			continue;

		const bool has_two_pin_path = 
			(ripup_net.GetDetailedGridPath().size() > 0);
		if(has_two_pin_path)
		{
			TwoPinRerouteNet temp_net;
			temp_net.index = two_pin_net_index;
			temp_net.two_pin_net_id = ripup_net.two_pin_net_id;
			temp_net.rip_up_times = (ripup_net.rip_up_times) + 1;
			temp_net.violation_num = 0;
			temp_net.wire_length = CountWireLength(two_pin_net_index);
			temp_net.net_score = 0.0;
			temp_net.straight_ratio = 
				ripup_net.GetConnection().GetStraightRatio();
			temp_net.is_straight = ripup_net.GetConnection().IsStraight();
			temp_net.line_end_violation_num = 0;

			// Check path violation.
			for(const Node &node : 
				ripup_net.GetDetailedGridPath())
			{
				const ULL grid_index = Transform3dArrayTo1dIndex(node);
				const bool has_violation_on_grid = 
					(track_capacity_[grid_index] <= -2);
				
				if(has_violation_on_grid)
					temp_net.history_node.push_back(node);
			}
			
			
			// Check via enclosure violation.
			if(this->kViaEncCostMode_)
			for(const TurnNode &turn_node :
					ripup_net.GetPathTurnNodes())
			{
				const ULL grid_index = Transform3dArrayTo1dIndex(turn_node);
				bool has_violation_on_grid = false;
				if(!kViaEncExpansion_[turn_node.z])
				{
					const int net_id = ripup_net.net_id;
					const int two_pin_net_id = ripup_net.two_pin_net_id;
					if(turn_node.z % 2 == kOddLayer)
					{
						if(turn_node.x-1 >= 0)
						{
							const ULL index = Transform3dArrayTo1dIndex(
									turn_node.x-1, turn_node.y, turn_node.z);
							const int grid_number = track_capacity_[index];
							if((grid_number != kNoBlockage) && 
								(grid_number != kBlockage) && 
								(net_id_table_[grid_number].net_id != net_id) &&
								(net_id_table_[grid_number].two_pin_net_id != 
								two_pin_net_id)
								)
							{
								has_violation_on_grid = true;
								violation_id_set.insert(two_pin_net_id);
								if(net_id_table_[grid_number].two_pin_net_id != -1)
									violation_id_set.insert(net_id_table_[grid_number].two_pin_net_id);
								else
									for(const int two_pin_net_id : layout_map_[index])
										violation_id_set.insert(two_pin_net_id);
								temp_net.via_enc_vio_nodes.push_back(
										Node(turn_node.x-1, turn_node.y, turn_node.z));
							}
						}
						if(turn_node.x+1 < layout_width_)
						{
							const ULL index = Transform3dArrayTo1dIndex(
									turn_node.x+1, turn_node.y, turn_node.z);
							const int grid_number = track_capacity_[index];
							if((grid_number != kNoBlockage) && 
								(grid_number != kBlockage) && 
								(net_id_table_[grid_number].net_id != net_id) &&
								(net_id_table_[grid_number].two_pin_net_id != 
								two_pin_net_id)
								)
							{
								has_violation_on_grid = true;
								violation_id_set.insert(two_pin_net_id);
								if(net_id_table_[grid_number].two_pin_net_id != -1)
									violation_id_set.insert(net_id_table_[grid_number].two_pin_net_id);
								else
									for(const int two_pin_net_id : layout_map_[index])
										violation_id_set.insert(two_pin_net_id);
								temp_net.via_enc_vio_nodes.push_back(
										Node(turn_node.x+1, turn_node.y, turn_node.z));
							}
						}
					}
					else
					{
						if(turn_node.y-1 >= 0)
						{
							const ULL index = Transform3dArrayTo1dIndex(
									turn_node.x, turn_node.y-1, turn_node.z);
							const int grid_number = track_capacity_[index];
							if((grid_number != kNoBlockage) && 
								(grid_number != kBlockage) && 
								(net_id_table_[grid_number].net_id != net_id) &&
								(net_id_table_[grid_number].two_pin_net_id != 
								two_pin_net_id)
								)
							{
								violation_id_set.insert(two_pin_net_id);
								has_violation_on_grid = true;
								if(net_id_table_[grid_number].two_pin_net_id != -1)
									violation_id_set.insert(net_id_table_[grid_number].two_pin_net_id);
								else
									for(const int two_pin_net_id : layout_map_[index])
										violation_id_set.insert(two_pin_net_id);
								temp_net.via_enc_vio_nodes.push_back(
										Node(turn_node.x, turn_node.y-1, turn_node.z));
							}
						}
						if(turn_node.y+1 < layout_height_)
						{
							const ULL index = Transform3dArrayTo1dIndex(
									turn_node.x, turn_node.y+1, turn_node.z);
							const int grid_number = track_capacity_[index];
							if((grid_number != kNoBlockage) && 
								(grid_number != kBlockage) && 
								(net_id_table_[grid_number].net_id != net_id) &&
								(net_id_table_[grid_number].two_pin_net_id != 
								two_pin_net_id)
								)
							{
								violation_id_set.insert(two_pin_net_id);
								has_violation_on_grid = true;
								if(net_id_table_[grid_number].two_pin_net_id != -1)
									violation_id_set.insert(net_id_table_[grid_number].two_pin_net_id);
								else
									for(const int two_pin_net_id : layout_map_[index])
										violation_id_set.insert(two_pin_net_id);
								temp_net.via_enc_vio_nodes.push_back(
										Node(turn_node.x, turn_node.y+1, turn_node.z));
							}
						}
					}
				}
				else
				{
					has_violation_on_grid = 
						(track_capacity_[grid_index] <= -2);
				}

				if(has_violation_on_grid)
					temp_net.history_node.push_back( 
							Node(turn_node.x, turn_node.y, turn_node.z));
			}

			// Check line-end violation.
			bool line_end_cut_violatin = false;
			if(this->kLineEndModeRouting_)
			{
				
				const vector<TurnNode> &turn_nodes = 
					ripup_net.GetPathTurnNodes();
				line_end_cut_violatin = 
					line_end_graph_.VerifyLineEndViolation(
							turn_nodes, temp_net.turn_node);
			}

			// bool line_end_cut_violatin = false;
			if(this->kLineEndModeRouting_)
			{
				// If the turn_nodes only contain pins, then there is no line-end violation.
				const Node &source = 
					*(ripup_net.GetDetailedGridPath().begin());
				const Node &target = 
					*(ripup_net.GetDetailedGridPath().rbegin());
					
				for(const TurnNode &turn_node : temp_net.turn_node)
				{
					const bool turn_node_is_not_source_pin = 
						(turn_node.x != source.x || 
						 turn_node.y != source.y || 
						 turn_node.z != source.z);
					const bool turn_node_is_not_target_pin = 
						(turn_node.x != target.x || 
						 turn_node.y != target.y || 
						 turn_node.z != target.z);
					
					const bool turn_node_is_not_pin = 
						!(turn_node_is_not_source_pin ^ turn_node_is_not_target_pin);
					line_end_cut_violatin = line_end_cut_violatin | turn_node_is_not_pin;
				}
			}

			const bool overlap_violation = (temp_net.history_node.size() > 0);
			if(overlap_violation || line_end_cut_violatin)
			{
				temp_net.violation_num = temp_net.history_node.size();
				temp_net.line_end_violation_num = temp_net.turn_node.size();
				reroute_two_pin_net_.push_back(temp_net);
				reroute_id_set.insert(temp_net.two_pin_net_id);
			}
		}
		else
		{
			TwoPinRerouteNet temp_net;
			temp_net.index = two_pin_net_index;
			temp_net.two_pin_net_id = ripup_net.two_pin_net_id;
			temp_net.rip_up_times = ripup_net.rip_up_times + 1;
			temp_net.violation_num = 0;
			temp_net.wire_length = 0;
			temp_net.net_score = 0.0;
			temp_net.straight_ratio = 
				ripup_net.GetConnection().GetStraightRatio();
			temp_net.is_straight = ripup_net.GetConnection().IsStraight();
			temp_net.history_node.clear();
			temp_net.turn_node.clear();
			temp_net.line_end_violation_num = 0;
			reroute_two_pin_net_.push_back(temp_net);
			reroute_id_set.insert(temp_net.two_pin_net_id);
		}
	}
	
	for(const int vio_id : violation_id_set)
	{
		const bool has_set_reroute = 
			(reroute_id_set.find(vio_id) != reroute_id_set.end());
		if(!has_set_reroute)
		{
			const int two_pin_net_index = 
				net_id_table_[vio_id].two_pin_net_index;
			TwoPinRipUpNet &ripup_net = 
				rip_up_two_pin_nets_[two_pin_net_index];
			TwoPinRerouteNet temp_net;
			temp_net.index = two_pin_net_index;
			temp_net.two_pin_net_id = ripup_net.two_pin_net_id;
			temp_net.rip_up_times = ripup_net.rip_up_times + 1;
			temp_net.violation_num = 0;
			temp_net.wire_length = 
				ripup_net.GetDetailedGridPath().size();
			temp_net.net_score = 0.0;
			temp_net.straight_ratio = 
				ripup_net.GetConnection().GetStraightRatio();
			temp_net.is_straight = 
				ripup_net.GetConnection().IsStraight();
			temp_net.history_node.clear();
			temp_net.turn_node.clear();
			temp_net.line_end_violation_num = 0;
			reroute_two_pin_net_.push_back(temp_net);
		}
	}
	
	return;
}


// Reroute part.
void SmallRouter::SetTwoPinNetPathToTrack(
		const int two_pin_net_index, 
		const int two_pin_net_id
		)
{
	for(const Node &path_node : 
			rip_up_two_pin_nets_[two_pin_net_index].GetDetailedGridPath())
		GetRightGridAfterReroute(path_node, two_pin_net_id);
	
	if(this->kViaEncCostMode_)
	{
		TwoPinRipUpNet &ripup_net = rip_up_two_pin_nets_[two_pin_net_index];
		for(const TurnNode &enc_node : 
				rip_up_two_pin_nets_[two_pin_net_index].GetPathTurnNodes())
		{
			const bool is_node_within_pin = ( 
				ripup_net.GetSource().IsNodeWithinPin(enc_node) ||
				ripup_net.GetTarget().IsNodeWithinPin(enc_node));
			if(!is_node_within_pin)
			{
				// cout << "\t" << enc_node << endl;
				GetRightGridAfterReroute(Node( \
						enc_node.x, enc_node.y, enc_node.z), two_pin_net_id);
			}
		}
	}
}


void SmallRouter::GetRightGridAfterReroute(
		const Node &path_node,
		const int two_pin_net_id
		)
{
	const int violation_magic_num = -1000;
	const int setting_two_pin_net_id = two_pin_net_id;
	
	const ULL node_index = 
		Transform3dArrayTo1dIndex(path_node);
	
	const int grid_id = track_capacity_[node_index];
	const int grid_two_pin_net_id = net_id_table_[grid_id].two_pin_net_id;
	
	const bool no_net_on_grid = 
		(track_capacity_[node_index] == kNoBlockage);
	const bool single_two_pin_net_on_grid = 
		(track_capacity_[node_index] > 0 && grid_two_pin_net_id != -1);
	const bool single_multi_pin_net_on_grid = 
		(track_capacity_[node_index] > 0 && grid_two_pin_net_id == -1);
	const bool over_two_illegal_net_on_grid = 
		(track_capacity_[node_index] != two_pin_net_id && 
		 track_capacity_[node_index] <= -2);
	if(no_net_on_grid)
	{
		track_capacity_[node_index] = setting_two_pin_net_id;
	}
	else if(single_two_pin_net_on_grid)
	{
		// first net id to compare
		const int checked_two_pin_net_id = setting_two_pin_net_id;
		const int checked_multi_net_id = 
			net_id_table_[checked_two_pin_net_id].net_id;
		// second net id to compare
		const int grid_two_pin_net_id = track_capacity_[node_index];
		const int grid_multi_net_id = 
			net_id_table_[grid_two_pin_net_id].net_id;
		
		// modify track capacity
		const bool the_same_two_pin_nets = 
			(checked_two_pin_net_id == grid_two_pin_net_id); // since the pin set to the track initially
		const bool not_belong_to_same_multi_pin_net = 
			(checked_multi_net_id != grid_multi_net_id);
		
		if(not_belong_to_same_multi_pin_net)
		{
			track_capacity_[node_index] = (-2);
		}
		else if(the_same_two_pin_nets)
		{
			track_capacity_[node_index] = setting_two_pin_net_id;
		}
		else
		{
			const int multi_pin_net_id = 
				net_id_table_[grid_two_pin_net_id].net_id;
			track_capacity_[node_index] = multi_pin_net_id;
		}
		
		const bool not_the_same_two_pin_net = (!the_same_two_pin_nets);
		if(not_the_same_two_pin_net)
		{
			// record the net wanted to add in
			const int first_searched_two_pin_net_id = checked_two_pin_net_id;
			const bool not_find_first_two_pin_net_id_in_layout_map = (
				( find(layout_map_[node_index].begin(), 
					   layout_map_[node_index].end(), 
					   first_searched_two_pin_net_id)) == 
				( layout_map_[node_index].end() ));
			if(not_find_first_two_pin_net_id_in_layout_map)
				layout_map_[node_index].push_back(checked_two_pin_net_id);
			
			// record the origin net on the grid
			const int second_searched_two_pin_net_id = grid_two_pin_net_id;
			const bool not_find_second_two_pin_net_id_in_layout_map = (
				( find(layout_map_[node_index].begin(), 
					   layout_map_[node_index].end(), 
					   second_searched_two_pin_net_id)) == 
				  ( layout_map_[node_index].end() ));
			if(not_find_second_two_pin_net_id_in_layout_map)
				layout_map_[node_index].push_back(grid_two_pin_net_id);
		}
		
		
	}
	else if(single_multi_pin_net_on_grid)
	{
		// first net id to compare
		const int checked_two_pin_net_id = setting_two_pin_net_id;
		const int checked_multi_net_id = 
			net_id_table_[checked_two_pin_net_id].net_id;
		// second multi net id to compare
		const int grid_multi_net_id = track_capacity_[node_index];
		
		// modify track capacity
		const bool not_belong_to_same_multi_pin_net = 
			(checked_multi_net_id != grid_multi_net_id);
		if(not_belong_to_same_multi_pin_net)
		{
			const int origin_two_pin_net_num_on_grid = 
				layout_map_[node_index].size();
			const int total_net_violation_num_on_grid = 
				origin_two_pin_net_num_on_grid + 1;
			track_capacity_[node_index] = (-1) * total_net_violation_num_on_grid;
		}
		else
		{
			const int multi_pin_net_id = grid_multi_net_id;
			track_capacity_[node_index] = multi_pin_net_id;
		}
		
		const int searched_two_pin_net_id = checked_two_pin_net_id;
		const bool not_find_two_pin_net_id_in_layout_map = (
			( find(layout_map_[node_index].begin(), 
				   layout_map_[node_index].end(), 
				   searched_two_pin_net_id)) == 
			( layout_map_[node_index].end() ));
		if(not_find_two_pin_net_id_in_layout_map)
			layout_map_[node_index].push_back(checked_two_pin_net_id);
	}
	else if(over_two_illegal_net_on_grid)
	{
		
		const int searched_two_pin_net_id = setting_two_pin_net_id;
		const bool not_find_two_pin_net_id_in_layout_map =(
			( find(layout_map_[node_index].begin(), 
				   layout_map_[node_index].end(), 
				   searched_two_pin_net_id)) == 
			( layout_map_[node_index].end() ));
		if(not_find_two_pin_net_id_in_layout_map)
		{
			track_capacity_[node_index] -= 1;
			layout_map_[node_index].push_back(setting_two_pin_net_id);
		}
	}
	else
	{
		
		cout << "Something goes wrong in SetTwoPinNetPathToTrack()" << endl;
		cout << "track capacity " << track_capacity_[node_index] << endl; 
		cout << "coordiante ";
		Transform1dTo3dIndex(node_index);
		cout << " two pin net id " << two_pin_net_id << endl;
		track_capacity_[node_index] = violation_magic_num;
		cout << "Reroute: magic!" <<  endl;
		exit(1);
	}
	
	if(track_capacity_[node_index] < -10)
	{
		cout << "Wrong node " << path_node << endl;
		cout << "track_capacity_ = " << track_capacity_[node_index] << endl;
		cout << "two_pin_net_id = ";
		for(int id : layout_map_[node_index])
			cout << id << " ";
		cout << endl;
		exit(1);
	}
	
	return;
}


// Rip-Up part.
// Rip up one two-pin net at a time.
void SmallRouter::RipUpTwoPinNetFromTrack(
		const int two_pin_net_index, 
		const int two_pin_net_id
		)
{
	TwoPinRipUpNet &ripup_net = rip_up_two_pin_nets_[two_pin_net_index];
	list<Node> &grid_path = ripup_net.GetDetailedGridPath();
	// Check the detailed path and rip up the path
	// Cet track_capacity_ to be right grid number rule after rip up the path
	// Not rip up the begin of and the end of the path.
	list<Node>::iterator iter_begin = ripup_net.GetDetailedGridPath().begin();
	iter_begin++;
	list<Node>::iterator iter_end = ripup_net.GetDetailedGridPath().end();
	iter_end--;
	// cout << "Net id = " << two_pin_net_id << endl;
	// cout << "net id = " << net_id_table_[two_pin_net_id].net_id << ", ";
	// cout << "2-pin id = " << net_id_table_[two_pin_net_id].two_pin_net_id << endl;
	// cout << "Rip-up Path Node = " << endl;
	for(list<Node>::iterator grid_iter = iter_begin;
		grid_iter != iter_end;
		grid_iter++)
	{
		const Node &node = (*grid_iter);
		const bool is_node_within_pin = (
			ripup_net.GetSource().IsNodeWithinPin(node) ||
			ripup_net.GetTarget().IsNodeWithinPin(node));
		if(!is_node_within_pin)
		{
		// cout << "\t" << node << endl;
		const ULL grid_index = Transform3dArrayTo1dIndex(node);
		const int grid_number = track_capacity_[grid_index];
		
		const int grid_number_after_rip_up = 
			GetRightGridNumberAfterRipUp(
					grid_index, 
					grid_number, 
					two_pin_net_id
					);
		track_capacity_[grid_index] = grid_number_after_rip_up;
		}
	}

	// cout << "Rip-up Via Enc Node = " << endl;
	if(this->kViaEncCostMode_)
	{
		for(const TurnNode &turn_node : ripup_net.GetPathTurnNodes())
		{
			const ULL grid_index = Transform3dArrayTo1dIndex(turn_node);
			const bool is_node_within_pin = (
				ripup_net.GetSource().IsNodeWithinPin(turn_node) ||
				ripup_net.GetTarget().IsNodeWithinPin(turn_node));
			if(!is_node_within_pin)
			{
				// cout << "\t" << turn_node << endl;
				const Node node(turn_node.x, turn_node.y, turn_node.z);
				if(find(grid_path.begin(), grid_path.end(), node) != 
						grid_path.end())
				{
					// cout << "\t" << turn_node << "Exist in path" << endl;
					continue;
				}
				// cout << "Rup Source " << ripup_net.GetConnection().source << endl;
				// cout << "Rup Target " << ripup_net.GetConnection().target << endl;
				// cout << "\tNet id = " << ripup_net.net_id << endl;
				// cout << "\t" << turn_node << endl;
				// haha
				const int grid_number = track_capacity_[grid_index];
				// cout << "\tgrid num = " << grid_number << endl;
				// cout << "\tgrid id = " << net_id_table_[grid_number].net_id << " ,";
				// cout << "2-pin id = " << net_id_table_[grid_number].two_pin_net_id << endl;
				const int grid_number_after_rip_up = 
				GetRightGridNumberAfterRipUp(
						grid_index, 
						grid_number, 
						two_pin_net_id
						);
				track_capacity_[grid_index] = grid_number_after_rip_up;
			}
		}
	}
}


int SmallRouter::GetRightGridNumberAfterRipUp(
		const ULL grid_index, 
		const int grid_number, 
		const int two_pin_net_id
		)
{
	const int violation_magic_num = -1000;
	const int grid_number_rule = grid_number;
	int revised_grid_number;
	
	const int grid_id = track_capacity_[grid_index];
	const int grid_two_pin_net_id = 
		net_id_table_[grid_id].two_pin_net_id;
	
	const bool single_two_pin_net_on_grid = 
		(grid_number_rule > 0 && grid_two_pin_net_id != -1);
	const bool single_multi_pin_net_on_grid = 
		(grid_number_rule > 0 && grid_two_pin_net_id == -1);
	const bool two_illegal_nets_on_grid = (grid_number_rule == -2);
	const bool over_two_illegal_nets_on_grid = (grid_number_rule < -2);
	
	const bool no_net_on_grid = (grid_number_rule == 0);
	
	if(single_two_pin_net_on_grid)
	{
		revised_grid_number = kNoBlockage;
	}
	else if(single_multi_pin_net_on_grid)
	{
		const int searched_two_pin_net_id = two_pin_net_id;
		const ULL transformed_grid_index =  grid_index;
		NetListIterator erase_iter = layout_map_[transformed_grid_index].end();
		
		for(NetListIterator net_iter = layout_map_[transformed_grid_index].begin();
			net_iter != layout_map_[transformed_grid_index].end();
			net_iter++)
		{
			const int two_pin_net_id_in_list = (*net_iter);
			const bool net_id_is_equal = 
				(two_pin_net_id_in_list == searched_two_pin_net_id);
			if(net_id_is_equal)
			{
				erase_iter = net_iter;
				break;
			}
		}
		
		const bool find_the_net_layout_map = 
			(erase_iter != layout_map_[transformed_grid_index].end());
		if(find_the_net_layout_map)
			layout_map_[transformed_grid_index].erase(erase_iter);
		
		const bool over_two_nets_on_grid_after_erase_net = 
			(layout_map_[transformed_grid_index].size() >= 2);
		const bool exactly_one_net_on_grid_after_erase_net = 
			(layout_map_[transformed_grid_index].size() == 1);
		const bool no_net_remained = 
			(layout_map_[transformed_grid_index].size() == 0);
		if(over_two_nets_on_grid_after_erase_net)
		{
			const int multi_net_id = grid_number_rule; // origin number on grid
			revised_grid_number = multi_net_id;
		}
		else if(exactly_one_net_on_grid_after_erase_net)
		{
			revised_grid_number = layout_map_[transformed_grid_index].front();
			layout_map_[transformed_grid_index].pop_front();
		}
		else if(no_net_remained)
		{
			revised_grid_number = kNoBlockage;
		}
		else
		{
			revised_grid_number = violation_magic_num;
			cout << "Something goes wrong in GetRigthGridNumberAfterRipUp()-0!!!" << endl;
			cout << "RipUp: magic1!" <<  endl;
			exit(1);
		}
		
	}
	else if(two_illegal_nets_on_grid)
	{
		const int searched_two_pin_net_id = two_pin_net_id;
		const ULL transformed_grid_index =  grid_index;
		NetListIterator erase_iter = layout_map_[transformed_grid_index].end();
		
		for(NetListIterator net_iter = layout_map_[transformed_grid_index].begin();
			net_iter != layout_map_[transformed_grid_index].end();
			net_iter++)
		{
			const int two_pin_net_id_in_list = (*net_iter);
			const bool net_id_is_equal = 
				(two_pin_net_id_in_list == searched_two_pin_net_id);
			if(net_id_is_equal)
			{
				erase_iter = net_iter;
				break;
			}
		}
		
		const bool find_the_net_layout_map = 
			(erase_iter != layout_map_[transformed_grid_index].end());
		if(find_the_net_layout_map)
			layout_map_[transformed_grid_index].erase(erase_iter);
		
		const bool one_net_remained_in_layout_map = 
			(layout_map_[transformed_grid_index].size() == 1);
		if(one_net_remained_in_layout_map)
		{
			revised_grid_number = layout_map_[transformed_grid_index].front();
			layout_map_[transformed_grid_index].pop_front();
		}
		else
		{
			revised_grid_number = -2;
		}
		
	}
	else if(over_two_illegal_nets_on_grid)
	{
		const int searched_two_pin_net_id = two_pin_net_id;
		const ULL transformed_grid_index = grid_index;
		NetListIterator erase_iter = layout_map_[transformed_grid_index].end();
		
		// find the net that equals to two_pin_net_id and erase it
		for(NetListIterator net_iter = layout_map_[transformed_grid_index].begin();
			net_iter != layout_map_[transformed_grid_index].end();
			net_iter++)
		{
			const int two_pin_net_id_in_list = (*net_iter);
			const bool net_id_is_equal = 
				(two_pin_net_id_in_list == searched_two_pin_net_id);
			if(net_id_is_equal)
			{
				erase_iter = net_iter;
				break;
			}
		}
		
		const bool find_the_net_layout_map = 
			(erase_iter != layout_map_[transformed_grid_index].end());
		if(find_the_net_layout_map)
		{
			layout_map_[transformed_grid_index].erase(erase_iter);
		}
		
		bool remaining_nets_in_grid_belong_to_same_multi_net = true;
		NetListIterator first_net_iter_in_list = 
			layout_map_[transformed_grid_index].begin();
		const int first_two_pin_net_id = (*first_net_iter_in_list);
		const int first_multi_net_id = net_id_table_[first_two_pin_net_id].net_id;
		
		NetListIterator second_net_iter_in_list = first_net_iter_in_list++;
		for(NetListIterator net_iter = second_net_iter_in_list;
			net_iter != layout_map_[transformed_grid_index].end();
			net_iter++)
		{
			const int remaining_two_pin_net_id = (*net_iter);
			const int remaining_multi_net_id = 
				net_id_table_[remaining_two_pin_net_id].net_id;
			const bool belong_to_same_multi_pin_net = 
				(first_multi_net_id == remaining_multi_net_id);
			if(!belong_to_same_multi_pin_net)
			{
				remaining_nets_in_grid_belong_to_same_multi_net = false;
				break;
			}
		}
		
		if(remaining_nets_in_grid_belong_to_same_multi_net)
			revised_grid_number = first_multi_net_id;
		else
			revised_grid_number = grid_number_rule + 1;
	}
	else if(no_net_on_grid)
	{
		revised_grid_number = 0;
	}
	else
	{
		revised_grid_number = violation_magic_num;
		cout << "Something goes wrong in GetRigthGridNumberAfterRipUp()-2!!!" << endl;
		cout << "RipUp: magic3!" <<  endl;
		cout << "track_capacity_ = " << track_capacity_[grid_index] << endl;
		cout << "two_pin_net_id = ";
		for(int id : layout_map_[grid_index])
			cout << id << " ";
		cout << endl;
		exit(1);
	}
	
	return revised_grid_number;
}


// Re-check whether these history nodes still have violations or not.
void SmallRouter::ReCheckAndUpdateHistoryGraph(const vector<Node>& history_node)
{
	vector<Node> revised_history_node;
	for(const Node &node : history_node)
	{
		const int node_index = Transform3dArrayTo1dIndex(node);
		const bool has_violation = (track_capacity_[node_index] <= -2); 
		if(has_violation)
			revised_history_node.push_back(node);
	}
	
	history_congestion_graph_.UpdateHistoryCongestion(revised_history_node);
	
	return;
}


// count the violations of the whole layout
inline int SmallRouter::CountLayoutViolationNumber()
{
	int violatin_num = 0;
	const ULL array_size = 
		layout_width_ * layout_height_ * layout_layer_;
	for(ULL index = 0; index < array_size; index++)
	{
		if(track_capacity_[index] < -1)
			violatin_num++;
	}
	
	return violatin_num;
}



// count the violations in the grid path of a net
inline int SmallRouter::CountViolationNumber(const int two_pin_net_index)
{
	const int index = two_pin_net_index;
	int violation_num = 0;
	
	for(const Node &node :
		rip_up_two_pin_nets_[index].two_pin_net_detailed_path.detail_grid_path)
	{
		const ULL index = Transform3dArrayTo1dIndex(node);
		const bool has_violation_on_grid = 
			(track_capacity_[index] <= -2);
		
		if(has_violation_on_grid)
			violation_num++;
	}
	
	return violation_num;
}


inline int SmallRouter::CountMultiNetNumber()
{
	int multi_net_num = 0;
	set<int> multi_net_id_set;
	for(const auto &two_pin_net : rip_up_two_pin_nets_)
	{
		const int &net_id = two_pin_net.net_id;
		multi_net_id_set.insert(net_id);
	}

	multi_net_num = multi_net_id_set.size();
	
	return multi_net_num;
}


// count the wire-length of a net
inline int SmallRouter::CountWireLength(const int two_pin_net_index)
{
	const int index = two_pin_net_index;
	int wire_length = 
		rip_up_two_pin_nets_[index].two_pin_net_detailed_path.detail_grid_path.size();
	
	return wire_length; 
}


// update the cost-related variables. e.g. wire-length...
void SmallRouter::UpdateRipUpTwoPinNetFromRerouteTwoPinNet()
{
	for(const TwoPinRerouteNet &reroute_net : reroute_two_pin_net_)
	{
		const int index = reroute_net.index;
		rip_up_two_pin_nets_[index].rip_up_times = reroute_net.rip_up_times;
		rip_up_two_pin_nets_[index].violation_num = CountViolationNumber(index);
		rip_up_two_pin_nets_[index].wire_length = CountWireLength(index);
	}
}


// track indexing usage
inline ULL SmallRouter::Transform3dArrayTo1dIndex(
		const int x, 
		const int y, 
		const int z)
{
	return ((z) * layout_width_ * layout_height_ + layout_width_ * (y) + (x) );
}

// track indexing usage
inline ULL SmallRouter::Transform3dArrayTo1dIndex(const Node &node)
{
	const int x = node.x;
	const int y = node.y;
	const int z = node.z;
	return (ULL)((z) * layout_width_ * layout_height_ + layout_width_ * (y) + (x) );
}

// track indexing usage
/*inline ULL SmallRouter::Transform3dArrayTo1dIndex(const MazeNode &maze_node)
{
	const int x = maze_node.x;
	const int y = maze_node.y;
	const int z = maze_node.z;
	return (ULL)((z) * layout_width_ * layout_height_ + layout_width_ * (y) + (x) );
}*/

// track indexing usage
inline ULL SmallRouter::Transform3dArrayTo1dIndex(const TurnNode &turn_node)
{
	const int x = turn_node.x;
	const int y = turn_node.y;
	const int z = turn_node.z;
	return (ULL)((z) * layout_width_ * layout_height_ + layout_width_ * (y) + (x) );
}

inline void SmallRouter::Transform1dTo3dIndex(ULL index)
{
	const int z = index / (layout_width_ * layout_height_);
	index -= z * (layout_width_ * layout_height_);
	const int y = index / layout_width_;
	index -= y * layout_width_;
	const int x = index;
	cout << x << " " << y << " " << z << endl;
}

// ImportFile() must be modified before using it.
// TODO...
/*
void SmallRouter::ImportFile(const char* input_file_name)
{
	char buffer[80];
	ifstream input;
	
	input.open(input_file_name, ios::in);
	if (!input) {
        printf("File %s Can Not Be Opened! Exit\n", input_file_name);
        //return false;
	} 
	else {
        printf("File %s Open Success.\n", input_file_name);
        printf("Parsing!\n");
		// p_rip_up_two_pin_net_ = new vector<TwoPinRipUpNet>;
	}

	int area_left = 0;
	int area_top = 0;
	int area_right = 0;
	int area_bottom = 0;
	input.getline(buffer, 80);
	sscanf(buffer, 
			"original layout %d %d %d %d", 
			&area_left,
			&area_top,
			&area_right,
			&area_bottom);
	
	input.getline(buffer, 80);
	sscanf(buffer, 
			"layout grid %d %d %d", 
			&layout_width_,
			&layout_height_,
			&layout_layer_);
	
	// get empty line 
	input.getline(buffer, 80);
	
	
	
	int h_track_1, h_track_2, h_track_3;
	int v_track_1, v_track_2, v_track_3;
	
	input.getline(buffer, 80);
	sscanf(buffer, "horizontal track each layer %d %d %d",
		&h_track_1, &h_track_2, &h_track_3);

	input.getline(buffer, 80);
	sscanf(buffer, "vertical track each layer %d %d %d",
		&v_track_1, &v_track_2, &v_track_3);
	
	// get empty line 
	input.getline(buffer, 80);
	
	int net_num;
	input.getline(buffer, 80);
	sscanf(buffer, "two-pin net num %d", &net_num);
	
	// *** Added part
	int counter = 0;
	map<int, int> map_id_table;
	// *** Added part
	const char *point = "point";
	const char *line = "line";
	const char *rect = "rect";

	int x1, y1, z1, x2, y2, z2;
	for(int i = 0; i < net_num; i++)
	{
		TwoPinRipUpNet temp_ripup_net;
		
		input.getline(buffer, 80);
		//sscanf(buffer, "net %d", &temp_ripup_net.two_pin_net_id);
		int net_index = -1;
		int two_pin_net_index = -1;
		sscanf(buffer, "net_index %d two_pin_net_index %d", 
				&net_index,
				&two_pin_net_index);
				// &temp_ripup_net.net_index,
				// &temp_ripup_net.two_pin_net_index);
				
		input.getline(buffer, 80);
		sscanf(buffer, "net_id %d two_pin_net_id %d", 
				&temp_ripup_net.net_id,
				&temp_ripup_net.two_pin_net_id);
		
		input.getline(buffer, 80);
		char pin1_type[8];
		sscanf(buffer, "%s", pin1_type);
		
		input.getline(buffer, 80);
		PinType type1 = kPinNone;
		if(!strcmp(pin1_type, point))
		{
			type1 = kPinPoint;
			sscanf(buffer, "from %d %d %d", &x1, &y1, &z1);
			temp_ripup_net.two_pin_net_connection.source.SetPinPoint(type1, x1, y1, z1);
			// cout << "pin1 point" << endl;
		}
		else if(!strcmp(pin1_type, line))
		{
			type1 = kPinLine;
			sscanf(buffer, "from %d %d %d %d %d %d", &x1, &y1, &z1, &x2, &y2, &z2);
			temp_ripup_net.two_pin_net_connection.source.SetPinLine(type1, x1, y1, z1, x2, y2, z2);
		}
		else if(!strcmp(pin1_type, rect))
		{
			type1 = kPinRectangle;
			sscanf(buffer, "from %d %d %d %d %d %d", &x1, &y1, &z1, &x2, &y2, &z2);
			temp_ripup_net.two_pin_net_connection.source.SetPinRectangle(type1, x1, y1, z1, x2, y2, z2);
		}
		else 
		{
		}
		
		
		input.getline(buffer, 80);
		char pin2_type[8];
		sscanf(buffer, "%s", pin2_type);

		input.getline(buffer, 80);
		PinType type2 = kPinNone;
		if(!strcmp(pin2_type, point))
		{
			type2 = kPinPoint;
			sscanf(buffer, "to %d %d %d", &x1, &y1, &z1);
			temp_ripup_net.two_pin_net_connection.target.SetPinPoint(type2, x1, y1, z1);
		}
		else if(!strcmp(pin2_type, line))
		{
			type2 = kPinLine;
			sscanf(buffer, "to %d %d %d %d %d %d", &x1, &y1, &z1, &x2, &y2, &z2);
			temp_ripup_net.two_pin_net_connection.target.SetPinLine(type2, x1, y1, z1, x2, y2, z2);
		}
		else if(!strcmp(pin2_type, rect))
		{
			type2 = kPinRectangle;
			sscanf(buffer, "to %d %d %d %d %d %d", &x1, &y1, &z1, &x2, &y2, &z2);
			temp_ripup_net.two_pin_net_connection.target.SetPinRectangle(type2, x1, y1, z1, x2, y2, z2);
		}
		else 
		{
		}
		
		temp_ripup_net.two_pin_net_connection.SetTargetTable(layout_width_, layout_height_, layout_layer_);
		
		// *** Added part
		if(map_id_table.find(temp_ripup_net.net_id) == map_id_table.end())
		{
			counter++;
			map_id_table[temp_ripup_net.net_id] = counter;
		}
		temp_ripup_net.net_id = map_id_table[temp_ripup_net.net_id];
		// *** Added part

		rip_up_two_pin_nets_.push_back(temp_ripup_net);		
	}
	
	
	
	// *** Added part
	sort(rip_up_two_pin_nets_.begin(), rip_up_two_pin_nets_.end());
	map<int, int> net_id_num_table;
	for(vector<TwoPinRipUpNet>::iterator net_iter = rip_up_two_pin_nets_.begin();
		net_iter != rip_up_two_pin_nets_.end();
		net_iter++)
	{
		TwoPinRipUpNet &temp_net = (*net_iter);
		if(net_id_num_table.find(temp_net.net_id) != net_id_num_table.end())
		{
			net_id_num_table[temp_net.net_id] += 1;
		}
		else
		{
			net_id_num_table[temp_net.net_id] = 1;
		}
	}

	vector<int> net_id_sum;
	int sum = 0;
	net_id_sum.push_back(sum);
	for(map<int, int>::iterator id_iter = net_id_num_table.begin();
		id_iter != net_id_num_table.end();
		id_iter++)
	{
		int tmp_net_num = id_iter->second;
		sum += (tmp_net_num);
		net_id_sum.push_back(sum);
	}

	int pre_id = 0;
	int id_counter = 0;
	for(vector<TwoPinRipUpNet>::iterator net_iter = rip_up_two_pin_nets_.begin();
		net_iter != rip_up_two_pin_nets_.end();
		net_iter++)
	{
		TwoPinRipUpNet &temp_net = (*net_iter);
		if(temp_net.net_id != pre_id)
		{
			pre_id = temp_net.net_id;
			id_counter = 0;
		}

		temp_net.net_id = temp_net.net_id + net_id_sum[temp_net.net_id - 1];

		id_counter++;
		temp_net.two_pin_net_id = temp_net.net_id + id_counter;
		
	}

	// *** Added part
	
	
	// get an empty line
	input.getline(buffer, 80);
	
	// get obstacle
	input.getline(buffer, 80);
	char* p_obstacle = strtok(buffer, " ");
	int obstacle_num = 0;
	if(!strcmp(p_obstacle, "obstacle"))
	{
		char* ob_num = strtok(NULL, " ");
		obstacle_num = atoi(ob_num);
	}

	if(obstacle_num)
	{
		cout << "obstacle_num " << obstacle_num << endl;
		for(int i = 0; i < obstacle_num; i++)
		{
			Node tmp_obstacle;
			input.getline(buffer, 80);
			sscanf(buffer, "%d %d %d", 
					&tmp_obstacle.x, &tmp_obstacle.y, &tmp_obstacle.z);
			obstacle_.push_back(tmp_obstacle);
		}
	
	}
	
	
	input.close();
	
	set_track_capacity();
	set_cost_array();
	set_back_track_array();
	set_id_table();
	
	SetNetPinsToTrack();
	SetObstacleToTrack();
	
}*/


// only for debug
void SmallRouter::PrintLayoutMap()
{
	printf("\n\nLayout Map\n");
	
	// build the corresponding index for every grid
	vector<pair<unsigned long, Node> > index_node;
	for(int z = 0; z < layout_layer_; z++)
	{
		for(int y = 0; y < layout_height_; y++)
		{
			for(int x = 0; x < layout_width_; x++)
			{
				Node temp_node;
				temp_node.x = x;
				temp_node.y = y;
				temp_node.z = z;
				
				int temp_index = Transform3dArrayTo1dIndex(x ,y, z);
				
				pair<unsigned long, Node> temp_pair;
				temp_pair = make_pair(temp_index, temp_node);
				
				index_node.push_back(temp_pair);
			}
		}
	}
	
	// print the layout_map_
	for(LayoutMap::iterator map_iter = layout_map_.begin();
		map_iter != layout_map_.end();
		map_iter++)
	{
		int index = map_iter->first;
		Node map_node;
		
		// search the node according to index
		for(vector<pair<unsigned long, Node> >::iterator vec_iter = index_node.begin();
			vec_iter != index_node.end();
			vec_iter++)
		{
			int temp_index = vec_iter->first;
			if(temp_index == index)
			{
				map_node.x = vec_iter->second.x;
				map_node.y = vec_iter->second.y;
				map_node.z = vec_iter->second.z;
				break;
			}
		}
		
		printf("index %d   grid (%d %d %d)   ", index, map_node.x, map_node.y, map_node.z);
		
		for(NetListIterator net_iter = map_iter->second.begin();
			net_iter != map_iter->second.end();
			net_iter++)
		{
			int net_id = (*net_iter);
			printf("%d ", net_id);	
		}
		printf("\n");
		
	}
}


// only for debug
void SmallRouter::PrintTrackGraph()
{
	const int kZero = 0; 
	printf("\n\nTrack Graph\n");
	for(int z = 0; z < layout_layer_; z++)
	{
		printf("LAYER %d\n", z);
		for(int y = layout_height_ - 1; y >= 0 ;y--)
		{
			for(int x = 0; x < layout_width_; x++)
			{
				unsigned long long grid_index = Transform3dArrayTo1dIndex(x, y, z);
				if(track_capacity_[grid_index] != kZero)
					printf("%5d", track_capacity_[grid_index]);
				else
					printf("    .");
			}
			printf("\n");
		}
		printf("\n\n");
	}

}


// only for debug
void SmallRouter::PrintTrackGraphViolation()
{
	int violatin_num = 0;
	printf("\n\nViolations\n");
	for(int z = 0; z < layout_layer_; z++)
	{
		for(int y = layout_height_ - 1; y >=0; y--)
		{
			for(int x = 0; x < layout_width_; x++)
			{
				int grid_index = Transform3dArrayTo1dIndex(x, y, z);
				if(track_capacity_[grid_index] < -1)
				{
					printf("(%d %d %d)\n", x, y, z);
					violatin_num++;
				}
			}
		}
	}
	
	printf("Total Violation %d\n", violatin_num);
}


// only for debug
void SmallRouter::PrintTrackNetIndex()
{
	set<int> net_id_set;
	for(vector<TwoPinRipUpNet>::iterator net_iter = rip_up_two_pin_nets_.begin();
		net_iter != rip_up_two_pin_nets_.end();
		net_iter++)
	{
		int net_id = net_iter->net_id;
		net_id_set.insert(net_id);
	}
	
	map<int, int> id_index_map;
	int net_id_count = 1;
	for(set<int>::iterator net_id_iter = net_id_set.begin();
		net_id_iter != net_id_set.end();
		net_id_iter++)
	{
		int net_id = (*net_id_iter);
		int net_index = net_id_count;
		id_index_map[net_id] = net_index;
		net_id_count++;
	}
	
	
	int wire_length = 0;
	printf("\n\nTrack Graph\n");
	for(int z = 0; z < layout_layer_; z++)
	{
		printf("LAYER %d\n", z);
		for(int y = layout_height_ - 1; y >= 0 ;y--)
		{
			for(int x = 0; x < layout_width_; x++)
			{
				unsigned long long grid_index = Transform3dArrayTo1dIndex(x, y, z);
				if(track_capacity_[grid_index] != 0)
				{
					int track_id = track_capacity_[grid_index];
					if(track_id <= -2)
					{
						printf("%5d", track_id);
						
						wire_length += (-track_id);
					}
					else
					{
						int net_id =  net_id_table_[track_id].net_id;
						int net_index = id_index_map[net_id];
						printf("%5d", net_index);
						
						wire_length++;
					}
					
					
				}
				else
				{
					printf("    .");
				}
			}
			printf("\n");
		}
		printf("\n\n");
	}
	
	printf("wire-length %d\n", wire_length);
}


// only for debug
void SmallRouter::PrintCostArray()
{ 
	printf("\n\nCost Array\n");
	for(int z = 0; z < layout_layer_; z++)
	{
		printf("LAYER %d\n", z);
		for(int y = layout_height_ - 1; y >= 0 ;y--)
		{
			for(int x = 0; x < layout_width_; x++)
			{
				const ULL grid_index = 
					Transform3dArrayTo1dIndex(x, y, z);
				if(cost_array_[grid_index] == FLT_MAX)
					printf("%8c", '*');
				else
					printf("%8.3f", cost_array_[grid_index]);
			}
			printf("\n");
		}
		printf("\n\n");
	}
}


// only for debug
void SmallRouter::PrintBackTrackArray()
{
	printf("\n\nBack Track Array\n");
	for(int z = 0; z < layout_layer_; z++)
	{
		printf("LAYER %d\n", z);
		for(int y = layout_height_ - 1; y >= 0 ;y--)
		{
			for(int x = 0; x < layout_width_; x++)
			{
				const ULL grid_index = 
					Transform3dArrayTo1dIndex(x, y, z);
				char dir = '.';
				switch(back_track_array_[grid_index])
				{
					case kLeft:
						dir = 'L';
						break;
					case kRight:
						dir = 'R';
						break;
					case kBottom:
						dir = 'B';
						break;
					case kTop:
						dir = 'T';
						break;
					case kUp:
						dir = '^';
						break;
					case kDown:
						dir = 'v';
						break;
					default:
						dir = '.';
				}
				printf("%5c", dir);
			}
			printf("\n");
		}
		printf("\n\n");
	}
}

void SmallRouter::PrintNetPins()
{
	cout << "========== Net Pin Grid Position ===========" << endl;
	for(TwoPinRipUpNet &ripup_net : this->rip_up_two_pin_nets_)
	{
		cout << "Net Id = " << ripup_net.net_id << ", ";
		cout << "2-Pin Id = " << ripup_net.two_pin_net_id << endl;
		cout << "\tSource " << ripup_net.GetConnection().source << endl;
		cout << "\tTarget " << ripup_net.GetConnection().target << endl;
	}
}


// anakyze the current layout result
void SmallRouter::AnalyzeTheResult(
		int &short_net_num, 
		int &open_net_num, 
		int &overlap_violation_num
		)
{
	cout << "Result : " << endl;
	int open_net_counter = 0;
	int short_net_counter = 0;
	int via_number_counter = 0;
	set<Node> via_node_set;
	for(TwoPinRipUpNet &net : rip_up_two_pin_nets_)
	{
		const bool has_net_path =
			(net.two_pin_net_detailed_path.detail_grid_path.size() > 0);
		if(has_net_path)
		{
			// Get via count.
			list<Node>::iterator second_node;
			for(list<Node>::iterator p_node = 
					net.GetDetailedGridPath().begin();
				p_node != (--net.GetDetailedGridPath().end());
				p_node++)
			{
				second_node = p_node;
				second_node++;
				const bool has_via = 
					((p_node->z != second_node->z) && 
					 (p_node->x == second_node->x) &&
					 (p_node->y == second_node->y));
				if(has_via)
					via_node_set.insert( \
							Node(p_node->x, p_node->y, min(p_node->z, second_node->z)));
			}

			// Get short violation num in net grid path.
			bool has_short_vio = false;
			for(const Node &node : net.GetDetailedGridPath())
			{
				const ULL index = Transform3dArrayTo1dIndex(node);
				const int grid_number = track_capacity_[index];
				if(grid_number <= -2)
				{
					#ifdef PrintDetailedResult
					cout << "net_short " << net.net_id << ":";
					cout << net.two_pin_net_id;
					cout << node << endl; 
					#endif
					short_net_counter++;
					has_short_vio = true;
					break;
				}
			}

			// Get short violation num in net via enclosures.
			if(has_short_vio) continue;
			for(const TurnNode &turn_node : net.GetPathTurnNodes())
			{
				const ULL index = Transform3dArrayTo1dIndex(turn_node);
				const int grid_number = track_capacity_[index];
				if(grid_number <= -2)
				{
					#ifdef PrintDetailedResult
					cout << "net_short " << net.net_id << ":";
					cout << net.two_pin_net_id;
					cout << turn_node << endl; 
					#endif
					short_net_counter++;
					has_short_vio = true;
					break;
				}
			}
		}
		else
		{
			if(!net.both_pins_are_overlapped && !net.has_only_one_pin)
			{
				#ifdef PrintDetailedResult
				cout << "net_open " << net.net_id << ":";
				cout << net.two_pin_net_id << endl;
				#endif
				open_net_counter++;
			}
		}
	}
	via_number_counter = (int)via_node_set.size();

	int wire_length = 0;
	int violation_counter = 0;

	for(int z = 0; z < layout_layer_; z++)
	{
		for(int y = 0; y < layout_height_; y++)
		{
			for(int x = 0; x < layout_width_; x++)
			{
				const ULL index = Transform3dArrayTo1dIndex(x,y,z);
				const int grid_number = track_capacity_[index];
				if(grid_number <= -2)
				{
					violation_counter++;
					wire_length += (-grid_number);
					#ifdef PrintDetailedResult
					cout << "overlapping_grid_num " << grid_number << " ";
					cout << "(" << x << " " << y << " " << z << ") ";
					for(int two_pin_net_id : layout_map_[index])
					{
						cout << net_id_table_[two_pin_net_id].net_id << ":";
						cout << net_id_table_[two_pin_net_id].two_pin_net_id;
						cout << " ";
					}
					cout << endl;
					#endif
				}
				else if(grid_number > 0)
				{
					wire_length++;
				}
				else
				{}
			}
		}
	}


	cout << " - Grid Wire-Length: " << wire_length << endl;
	cout << " - Grid Via: " << via_number_counter << endl;
	cout << " - Grid Short Net: " << short_net_counter << endl;
	cout << " - Grid Open Net: " << open_net_counter << endl;
	cout << " - Grid Overlap Vios: " << violation_counter << endl;

	short_net_num = short_net_counter;
	open_net_num = open_net_counter;
	overlap_violation_num = violation_counter;
}

void SmallRouter::ShowRerouteNetInfo(const int rip_up_times)
{
	cout << endl << "--------------------" << endl;
	cout << "R.R. Iteration : " << rip_up_times << endl;
	cout << "R.R. 2-pin Net Num: " << reroute_two_pin_net_.size() << endl;
	#ifdef PrintDetailedResult
	cout << "R.R. 2-pin Net Id: " << endl;
	int new_line_counter = 0;
	for(const TwoPinRerouteNet &reroute_net : reroute_two_pin_net_)
	{
		const int net_id = 
			rip_up_two_pin_nets_[reroute_net.index].net_id;
		cout << "(" << net_id << ":";
		cout << reroute_net.two_pin_net_id << ")";
		new_line_counter++;
		if(new_line_counter % 10 == 0) cout << endl;
	}
	if(new_line_counter % 10 != 0) cout << endl;
	#endif
}

// OutputGDSFile() must be modified before using it.
// Output the GDS File
/*
void SmallRouter::OutputGDSFile(const char* output_file_name)
{
	ofstream output_file(output_file_name, ios::out);
	if(output_file.is_open())
	{
		const float via_shifting = 0.2;
		const float wire_unit = 0.1;
		const float pin_unit = 0.3;
		const int text_layer = 255;
		const float text_vio_shifting = 0.2;
		
		output_file << "gds2{600" << endl;
		output_file << "m=2013-12-02 14:21:17 a=2013-12-02 14:21:17" << endl;
		output_file << "lib 'Flat' 0.001 1e-09" << endl;
		
		output_file << "cell{c=2013-12-02 14:21:17 m=2013-12-02 14:21:17 '" << output_file_name << "'" << endl;
		
		// Output the grid.
		const int track_layer = 63;
		const int coord_text_layer = 63;
		int track_count = 0;
		for(int h_track = 0; h_track < this->layout_height_; h_track++)
		{
			output_file << "p{";
			output_file << track_layer << " ";
			output_file << "xy(";
			output_file << 0 << " " << h_track << " ";
			output_file << (layout_width_ - 1) << " " << h_track;
			output_file << ")";
			output_file << "}" << endl;
			
			if(track_count % 5 == 0)
			{
				output_file << "t{" << coord_text_layer << " ";
				output_file << "xy(";
				output_file << 0 << " " << h_track;
				output_file << ") ";
				output_file << "\'" << h_track << "\'";
				output_file << "}" << endl;

				output_file << "t{" << coord_text_layer << " ";
				output_file << "xy(";
				output_file << (layout_width_ - 1) << " " << h_track;
				output_file << ") ";
				output_file << "\'" << h_track << "\'";
				output_file << "}" << endl;
			}
			track_count++;
		}

		track_count = 0;
		for(int v_track = 0; v_track < this->layout_width_; v_track++)
		{
			output_file << "p{";
			output_file << track_layer << " ";
			output_file << "xy(";
			output_file << v_track << " " << 0 << " ";
			output_file << v_track << " " << (layout_height_ - 1);
			output_file << ")";
			output_file << "}" << endl;
			
			if(track_count % 5 == 0)
			{
				output_file << "t{" << coord_text_layer << " ";
				output_file << "xy(";
				output_file << v_track << " " << 0 << " ";
				output_file << ") ";
				output_file << "\'" << v_track << "\'";
				output_file << "}" << endl;

				output_file << "t{" << coord_text_layer << " ";
				output_file << "xy(";
				output_file << v_track << " " << (layout_height_ - 1);
				output_file << ") ";
				output_file << "\'" << v_track << "\'";
				output_file << "}" << endl;
			}
			track_count++;
		}

		// output the path of each net
		for(vector<TwoPinRipUpNet>::iterator net_iter = rip_up_two_pin_nets_.begin();
			net_iter != rip_up_two_pin_nets_.end();
			net_iter++)
		{
			int net_id = net_iter->net_id;
			// int two_pin_net_id = net_iter->two_pin_net_id;
		
			// output the source pin
			const Pin &source_pin = net_iter->two_pin_net_connection.source;
			const PinType source_type = source_pin.pin_type;
			if(source_type == kPinPoint)
			{
				int x, y, z;
				source_pin.GetPinPointPosition(x, y, z);
				output_file << "t{" << text_layer << " ";
				output_file << "xy(" << x << " " << y << ")" << " ";
				output_file << "'" << net_id <<"'}" << endl;
				
				int source_layer = z + 1;
				output_file << "p{" << source_layer << " dt1 pt1 ";
				output_file << "w" << (pin_unit * source_layer) << " ";
				output_file << "xy(" << x << " " << y << " ";
				output_file << x << " " << y << ")";
				output_file << "}" << endl;

			}
			else if(source_type == kPinLine)
			{
				int x1, y1, z1, x2, y2, z2;
				source_pin.GetPinLinePosition(x1, y1, z1, x2, y2, z2);
				output_file << "t{" << text_layer << " ";
				output_file << "xy(" << (float)(x1 + x2)/2 << " " << (float)(y1 + y2)/2 << ")" << " ";
				output_file << "'" << net_id << "'}" << endl;
				
				int source_layer = z1 + 1;
				output_file << "p{";
				output_file << source_layer << " ";
				output_file << "w" << (wire_unit * source_layer) << " ";
				output_file << "xy(";
				output_file << x1 << " " << y1 << " ";
				output_file << x2 << " " << y2;
				output_file << ")";
				output_file << "}" << endl;
						

			}
			else if(source_type == kPinRectangle)
			{
				int left, bottom, z1, right, top, z2;
				source_pin.GetPinRectanglePosition(left, bottom, z1, right, top, z2);
				output_file << "t{" << text_layer << " ";
				output_file << "xy(" << (float)(left + right)/2 << " " << (float)(bottom + top)/2 << ")" << " ";
				output_file << "'" << net_id << "'}" << endl;
				
				int source_layer = z1 + 1;
				output_file << "b{" << source_layer << " dt1 ";
				output_file << "xy(" << left << " " << bottom << " ";
				output_file << left << " " << top << " ";
				output_file << right << " " << top << " ";
				output_file << right << " " << bottom << ")";
				output_file << "}" << endl;
			}
			else
			{}

			// output the target pin
			const Pin &target_pin = net_iter->two_pin_net_connection.target;
			const PinType target_type = target_pin.pin_type;
			if(target_type == kPinPoint)
			{
				int x, y, z;
				target_pin.GetPinPointPosition(x, y, z);
				output_file << "t{" << text_layer << " ";
				output_file << "xy(" << x << " " << y << ")" << " ";
				output_file << "'" << net_id << "'}" << endl;
				
				int target_layer = z + 1;
				output_file << "p{" << target_layer << " dt1 pt1 ";
				output_file << "w" << (pin_unit * target_layer) << " ";
				output_file << "xy(" << x << " " << y << " ";
				output_file << x << " " << y << ")";
				output_file << "}" << endl;

			}
			else if(target_type == kPinLine)
			{
				int x1, y1, z1, x2, y2, z2;
				target_pin.GetPinLinePosition(x1, y1, z1, x2, y2, z2);
				output_file << "t{" << text_layer << " ";
				output_file << "xy(" << (float)(x1 + x2)/2 << " " << (float)(y1 + y2)/2 << ")" << " ";
				output_file << "'" << net_id << "'}" << endl;
				
				int target_layer = z1 + 1;
				output_file << "p{";
				output_file << target_layer << " ";
				output_file << "w" << (wire_unit * target_layer) << " ";
				output_file << "xy(";
				output_file << x1 << " " << y1 << " ";
				output_file << x2 << " " << y2;
				output_file << ")";
				output_file << "}" << endl;
						

			}
			else if(target_type == kPinRectangle)
			{
				int left, bottom, z1, right, top, z2;
				target_pin.GetPinRectanglePosition(left, bottom, z1, right, top, z2);
				output_file << "t{" << text_layer << " ";
				output_file << "xy(" << (float)(left + right)/2 << " " << (float)(bottom + top)/2 << ")" << " ";
				output_file << "'" << net_id << "'}" << endl;
				
				int target_layer = z1 + 1;
				output_file << "b{" << target_layer << " dt1 ";
				output_file << "xy(" << left << " " << bottom << " ";
				output_file << left << " " << top << " ";
				output_file << right << " " << top << " ";
				output_file << right << " " << bottom << ")";
				output_file << "}" << endl;	
			}
			else
			{}


			list<Node>::iterator pre_path_node_iter = net_iter->two_pin_net_detailed_path.detail_grid_path.begin();
			list<Node>::iterator second_path_node_iter = net_iter->two_pin_net_detailed_path.detail_grid_path.begin();
			second_path_node_iter++;
			
			for(list<Node>::iterator path_node_iter = second_path_node_iter;
				path_node_iter != net_iter->two_pin_net_detailed_path.detail_grid_path.end();
				path_node_iter++)
			{
				bool same_x = (pre_path_node_iter->x == path_node_iter->x);
				bool same_y = (pre_path_node_iter->y == path_node_iter->y);
				bool same_z = (pre_path_node_iter->z == path_node_iter->z);
				
				bool same_xy = (same_x && same_y && !same_z); // via
				bool same_yz = (!same_x && same_y && same_z); // horizontal wire
				bool same_xz = (same_x && !same_y && same_z); // vertical wire
				bool same_segment = (same_xy || same_yz || same_xz);
				
				list<Node>::iterator tmp_path_node_iter = path_node_iter;
				tmp_path_node_iter++;
				bool last_node = (tmp_path_node_iter == net_iter->two_pin_net_detailed_path.detail_grid_path.end());
				
				if(!last_node && !same_segment)
				{
					
					Node from_path_node(pre_path_node_iter->x, pre_path_node_iter->y, pre_path_node_iter->z);
					
					list<Node>::iterator to_path_node_iter = path_node_iter;
					to_path_node_iter--;
					Node to_path_node(to_path_node_iter->x, to_path_node_iter->y, to_path_node_iter->z);
					
					bool not_via_path = (from_path_node.z == to_path_node.z);
					if(not_via_path)
					{
						float layer = from_path_node.z + 1;
						float left = from_path_node.x;
						float bottom = from_path_node.y;
						float right = to_path_node.x;
						float top = to_path_node.y;
						
						output_file << "p{" << layer << " dt1 ";
						output_file << "w" << (wire_unit * layer) << " ";
						output_file << "xy(" << left << " " << bottom << " ";
						output_file << right << " " << top << ")";
						output_file << "}" << endl;
					}
					else
					{
						float layer = min(from_path_node.z, to_path_node.z) + 1 + layout_layer_;
						float left = from_path_node.x - via_shifting;
						float bottom = from_path_node.y - via_shifting;
						float right = to_path_node.x + via_shifting;
						float top = to_path_node.y + via_shifting;
						
						if(left > right) swap(left, right);
						if(bottom > top) swap(bottom, top);
						
						output_file << "b{" << layer << " dt1 ";
						output_file << "xy(" << left << " " << bottom << " ";
						output_file << left << " " << top << " ";
						output_file << right << " " << top << " ";
						output_file << right << " " << bottom << ")";
						output_file << "}" << endl;
						
					}
					
					pre_path_node_iter = to_path_node_iter;
				}
				else if(last_node && same_segment)
				{
					
					Node from_path_node(pre_path_node_iter->x, pre_path_node_iter->y, pre_path_node_iter->z);
					
					list<Node>::iterator to_path_node_iter = path_node_iter;
					Node to_path_node(to_path_node_iter->x, to_path_node_iter->y, to_path_node_iter->z);
					
					bool not_via_path = (from_path_node.z == to_path_node.z);
					if(not_via_path)
					{
						float layer = from_path_node.z + 1;
						float left = from_path_node.x;
						float bottom = from_path_node.y;
						float right = to_path_node.x;
						float top = to_path_node.y;
						
						output_file << "p{" << layer << " dt1 ";
						output_file << "w" << (wire_unit * layer) << " ";
						output_file << "xy(" << left << " " << bottom << " ";
						output_file << right << " " << top << ")";
						output_file << "}" << endl;
					}
					else
					{
						float layer = min(from_path_node.z, to_path_node.z) + 1 + layout_layer_;
						float left = from_path_node.x - via_shifting;
						float bottom = from_path_node.y - via_shifting;
						float right = to_path_node.x + via_shifting;
						float top = to_path_node.y + via_shifting;
						
						if(left > right) swap(left, right);
						if(bottom > top) swap(bottom, top);
						
						output_file << "b{" << layer << " dt1 ";
						output_file << "xy(" << left << " " << bottom << " ";
						output_file << left << " " << top << " ";
						output_file << right << " " << top << " ";
						output_file << right << " " << bottom << ")";
						output_file << "}" << endl;
						
					}
					
				}
				else if(last_node && !same_segment)
				{
					
					Node from_path_node(pre_path_node_iter->x, pre_path_node_iter->y, pre_path_node_iter->z);
					
					list<Node>::iterator to_path_node_iter = path_node_iter;
					to_path_node_iter--;
					Node to_path_node(to_path_node_iter->x, to_path_node_iter->y, to_path_node_iter->z);
					
					bool not_via_path = (from_path_node.z == to_path_node.z);
					if(not_via_path)
					{
						float layer = from_path_node.z + 1;
						float left = from_path_node.x;
						float bottom = from_path_node.y;
						float right = to_path_node.x;
						float top = to_path_node.y;
						
						output_file << "p{" << layer << " dt1 ";
						output_file << "w" << (wire_unit * layer) << " ";
						output_file << "xy(" << left << " " << bottom << " ";
						output_file << right << " " << top << ")";
						output_file << "}" << endl;
					}
					else
					{
						float layer = min(from_path_node.z, to_path_node.z) + 1 + layout_layer_;
						float left = from_path_node.x - via_shifting;
						float bottom = from_path_node.y - via_shifting;
						float right = to_path_node.x + via_shifting;
						float top = to_path_node.y + via_shifting;
						
						if(left > right) swap(left, right);
						if(bottom > top) swap(bottom, top);
						
						output_file << "b{" << layer << " dt1 ";
						output_file << "xy(" << left << " " << bottom << " ";
						output_file << left << " " << top << " ";
						output_file << right << " " << top << " ";
						output_file << right << " " << bottom << ")";
						output_file << "}" << endl;
						
					}
					
					
					// output the last segment
					Node last_from_path_node = (*to_path_node_iter);

					Node last_to_path_node = (*path_node_iter);
					
					bool not_last_via_path = (last_from_path_node.z == last_to_path_node.z);
					if(not_last_via_path)
					{
						float layer = last_from_path_node.z + 1;
						float left = last_from_path_node.x;
						float bottom = last_from_path_node.y;
						float right = last_to_path_node.x;
						float top = last_to_path_node.y;
						
						output_file << "p{" << layer << " dt1 ";
						output_file << "w" << (wire_unit * layer) << " ";
						output_file << "xy(" << left << " " << bottom << " ";
						output_file << right << " " << top << ")";
						output_file << "}" << endl;
					}
					else
					{
						float layer = min(last_from_path_node.z, last_to_path_node.z) + 1 + layout_layer_;
						float left = last_from_path_node.x - via_shifting;
						float bottom = last_from_path_node.y - via_shifting;
						float right = last_to_path_node.x + via_shifting;
						float top = last_to_path_node.y + via_shifting;;
						
						if(left > right) swap(left, right);
						if(bottom > top) swap(bottom, top);
						
						output_file << "b{" << layer << " dt1 ";
						output_file << "xy(" << left << " " << bottom << " ";
						output_file << left << " " << top << " ";
						output_file << right << " " << top << " ";
						output_file << right << " " << bottom << ")";
						output_file << "}" << endl;
						
					}

					
				}
				else
				{
					// do nothing
					// cout << "something goes wrong when output the SILK path file" << endl;
				}
				
				
			}
		}
		
		// output obstacles
		// collect obstacles
		set<Node> obstacle_node;
		for(int z = 0; z < layout_layer_; z++)
		{
			for(int y = 0; y < layout_height_; y++)
			{
				for(int x = 0; x < layout_width_; x++)
				{
					unsigned long long node_index = Transform3dArrayTo1dIndex(x, y, z);
					bool has_violation = (track_capacity_[node_index] <= -2);
					if(has_violation)
					{
						Node violation_node(x, y, z);
						obstacle_node.insert(violation_node);
					}
				}
			}
		}

		for(set<Node>::iterator vio_node_iter = obstacle_node.begin();
			vio_node_iter != obstacle_node.end();
			vio_node_iter++)
		{
			Node tmp_vio_node(vio_node_iter->x, vio_node_iter->y, vio_node_iter->z);
			int vio_layer = tmp_vio_node.z + 1;
			
			output_file << "t{" << vio_layer << " ";
			output_file << "xy(" << (tmp_vio_node.x + text_vio_shifting) << " " << (tmp_vio_node.y + text_vio_shifting) << ")" << " ";
			output_file << "'QQ'}" << endl;
			
		}
		
	
		if(this->kLineEndModeRouting_)
		{
			line_end_graph_.OutputGDSFile(output_file);
		}

		output_file << "}" << endl;
		output_file << "}" << endl;
		
	}
	else
	{
		cout << "Unable to Output GDS File: " << output_file_name << endl;
		output_file.close();
		return;
	}
	
	output_file.close();
	return;
}*/

/*
void SmallRouter::OutputPinGDT(const char *top_cell_name)
{
	string output_file_name = string(top_cell_name) + "_grid_pins.gdt";
	ofstream output(output_file_name);
	if(!output.is_open())
	{
		cout << "Unable to output the file: ";
		cout << output_file_name << "." << endl;
		output.close();
		return;
	}

	// Output the gds hearder.
	string lib_name(top_cell_name);
	output << "gds2{600 " << endl; // gds version
	output::OutputHeaderGDT(output, 1e-04, 1e-10, lib_name);

	// Output the cell title.
	string top_cell_str(top_cell_name);
	output << "cell{";
	output::OutputCellHeaderGDT(output, top_cell_str);

	// Output the vert/hori tracks.
	const oaLayerNum track_layer = 63; 
	const int text_dt = 1;
	const int path_dt = 0;
	const float DBU = 1.0;
	// Horizontal tracks
	int track_count = 0;
	for(int h_track = 0; h_track < layout_height_; h_track++)
	{
		output::OutputPathGDT(output, 
				0, h_track, (layout_width_-1), h_track, 
				track_layer, path_dt, text_dt, DBU
			);
		if(track_count % 5 == 0)
		{
			string str1 = to_string(track_count);
			output::OutputTextGDT(
					output, -1, h_track, str1, 
					track_layer, text_dt, DBU);
			string str2 = to_string(track_count);
			output::OutputTextGDT(
					output, layout_width_, h_track, str2, 
					track_layer, text_dt, DBU);
		}
		track_count++;
	}
	// Vertical tracks
	track_count = 0;
	for(int v_track = 0; v_track < layout_width_; v_track++)
	{
		output::OutputPathGDT(output, 
				v_track, 0, v_track, layout_height_, 
				track_layer, path_dt, text_dt, DBU
			);
		if(track_count % 5 == 0)
		{
			string str1 = to_string(track_count);
			output::OutputTextGDT(
					output, v_track, -1, str1, 
					track_layer, text_dt, DBU);
			string str2 = to_string(track_count);
			output::OutputTextGDT(
					output, v_track, layout_height_, str2, 
					track_layer, text_dt, DBU);
		}
		track_count++;
	}
	
	// Output the pins of each net.
	const float wire_unit = 0.1;
	const float pin_unit = 0.4;
	const int pin_point_dt = 1;
	const int pin_line_dt = 2;
	const int pin_rect_dt = 3;
	const int net_text_dt = 4;
	for(TwoPinRipUpNet &ripup_net : rip_up_two_pin_nets_)
	{
		string net_id = to_string(ripup_net.net_id);
		// output the source pin
		const Pin &source_pin = 
			ripup_net.two_pin_net_connection.source;
		const PinType source_type = source_pin.pin_type;
		if(source_type == kPinPoint)
		{
			int x, y, z;
			source_pin.GetPinPointPosition(x, y, z);
			output::OutputTextGDT(output, x, y, net_id, z, net_text_dt, DBU);
			output::OutputWidthPathGDT(
					output, x, y, x, y, z, 
					(pin_unit*(z+1)), 1, pin_point_dt, DBU);
		}
		else if(source_type == kPinLine)
		{
			int x1, y1, z1, x2, y2, z2;
			source_pin.GetPinLinePosition(x1, y1, z1, x2, y2, z2);
			output::OutputTextGDT(
					output, (x1+y2)/2, (y1+y2)/2, 
					net_id, z1, net_text_dt, DBU);
			output::OutputWidthPathGDT(
					output, x1, y1, x2, y2, z1, 
					(wire_unit*(z1+1)), 1, pin_line_dt, DBU);
		}
		else if(source_type == kPinRectangle)
		{
			int left, bottom, z1, right, top, z2;
			source_pin.GetPinRectanglePosition(left, bottom, z1, right, top, z2);
			output::OutputTextGDT(
					output, (left+right)/2, (bottom+top)/2, 
					net_id, z1, net_text_dt, DBU);
			oaBox box(left, bottom, right, top);
			output::OutputBoxGDT(output, box, z1, pin_rect_dt, DBU);
		}
		else
		{}

		// output the target pin
		const Pin &target_pin = 
			ripup_net.two_pin_net_connection.target;
		const PinType target_type = target_pin.pin_type;
		if(target_type == kPinPoint)
		{
			int x, y, z;
			target_pin.GetPinPointPosition(x, y, z);
			output::OutputTextGDT(output, x, y, net_id, z, net_text_dt, DBU);
			output::OutputWidthPathGDT(
					output, x, y, x, y, z, 
					(pin_unit*(z+1)), 1, pin_point_dt, DBU);
		}
		else if(target_type == kPinLine)
		{
			int x1, y1, z1, x2, y2, z2;
			target_pin.GetPinLinePosition(x1, y1, z1, x2, y2, z2);
			output::OutputTextGDT(
					output, (x1+y2)/2, (y1+y2)/2, 
					net_id, z1, net_text_dt, DBU);
			output::OutputWidthPathGDT(
					output, x1, y1, x2, y2, z1, 
					(wire_unit*(z1+1)), 1, pin_line_dt, DBU);
		}
		else if(target_type == kPinRectangle)
		{
			int left, bottom, z1, right, top, z2;
			target_pin.GetPinRectanglePosition(left, bottom, z1, right, top, z2);
			output::OutputTextGDT(
					output, (left+right)/2, (bottom+top)/2, 
					net_id, z1, net_text_dt, DBU);
			oaBox box(left, bottom, right, top);
			output::OutputBoxGDT(output, box, z1, pin_rect_dt, DBU);
		}
		else
		{}
	}

	const int ob_text_dt = 5;
	string ob_str("X");
	// Output the obstacles.
	for(Node &ob_node : obstacle_)
	{
		output::OutputTextGDT(
				output, (ob_node.x), (ob_node.y), 
				ob_str, (ob_node.z), ob_text_dt, DBU);
	}

	output << "}" << endl; // The end of the cell.
	output << "}" << endl; // The end of the gds.

	return; 
}*/


