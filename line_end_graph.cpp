#include "line_end_graph.h"

LineEndGraph::LineEndGraph() :
layout_width_(INT_MIN), layout_height_(INT_MIN), layout_layer_(INT_MIN),
p_track_capacity_(NULL),
p_net_id_table_(NULL),
line_end_graph_(NULL),
line_end_history_graph_(NULL),
graph_is_setup_(false)
{
	
}

LineEndGraph::~LineEndGraph()
{
	if(this->line_end_graph_ != NULL)
		delete [] this->line_end_graph_;

	if(this->line_end_history_graph_ != NULL)
		delete [] this->line_end_history_graph_;
}

// Set the preferred direction
void LineEndGraph::SetPreferredDirection(
		const int odd_layer, 
		const int even_layer
		)
{
	this->kOddLayer = odd_layer;
	this->kEvenLayer = even_layer;
	return;
}


void LineEndGraph::SetLineEndSpacing(
		RoutingLayerInfos &routing_layer_infos, 
		const int low_layer, 
		const int high_layer)
{
	// 1. Search the pseudo M1 layer.
	const int m1_layer_num = low_layer;
	RoutingLayerInfo *p_m1_info = NULL;
	for (RoutingLayerInfo &info : routing_layer_infos.GetLayerInfos())
	{
		const int layer_num = (int)info.GetLayer();
		if(layer_num == m1_layer_num)
		{
			p_m1_info = &info;
			break;
		}
	}
	// 2. Search the pseudo M2 layer.
	const int m2_layer_num = m1_layer_num + 2;
	RoutingLayerInfo *p_m2_info = NULL;
	for (RoutingLayerInfo &info : routing_layer_infos.GetLayerInfos())
	{
		const int layer_num = (int)info.GetLayer();
		if(layer_num == m2_layer_num)
		{
			p_m2_info = &info;
			break;
		}
	}

	if(p_m1_info == NULL || p_m2_info == NULL)
	{
		cout << "LineEndGraph: Can't get the M1 or M2 info." << endl;
		exit(1);
	}

	// Compute the line-end parity spacing based on M1 & M2 pitch.
	const bool is_m1_vertical = p_m1_info->is_vertical;
	const int layer_size = (high_layer - low_layer) / 2 + 1;
	this->kLineEndSpacing_.resize(layer_size);
	this->kSafeSpacing_.resize(layer_size);
	this->kMaskNum_.resize(layer_size);
	int index = 0;
	for (RoutingLayerInfo &info : routing_layer_infos.GetLayerInfos())
	{
		const int layer_num = (int)info.GetLayer();
		const bool outside_layer =
			((layer_num < low_layer) || (layer_num > high_layer));
		if(outside_layer)
			continue;
		
		if(!(info.has_line_end_rule))
		{
			this->kLineEndSpacing_[index] = 0;
			this->kSafeSpacing_[index] = 0;
			this->kMaskNum_[index] = 0;
		}
		else
		{
			// Parity track part.
			int pitch = -1;
			if(info.is_vertical)
				pitch = (int)((is_m1_vertical) ? \
						(p_m2_info->h_track_spacing) : \
						(p_m1_info->v_track_spacing));
			else
				pitch = (int)((is_m1_vertical) ? \
						(p_m1_info->v_track_spacing) : \
						(p_m2_info->h_track_spacing));
			
			// End-End spacing part.
			int grid_spacing_e2e = 0;
			{
				const int half_enc = (int)((info.is_vertical) ?
					(info.upper_enc1_height / 2) : (info.upper_enc2_width / 2));
				const int spacing_e2e = (int)info.spacing_e2e;
				const int base_grid = (spacing_e2e / pitch) + 1;
				const int remained_spacing = (spacing_e2e % pitch);
				const int pitch_enc_diff = (half_enc >= pitch) ? 
					(pitch - (half_enc % pitch)) : (pitch - half_enc);
				const bool has_enough_spacing = 
					((pitch_enc_diff * 2) >= remained_spacing); 
				grid_spacing_e2e = (has_enough_spacing) ? 
					(base_grid + 1) : (base_grid + 2);
			}
			this->kSafeSpacing_[index] = grid_spacing_e2e;

			// Parity spacing part.
			int grid_parity_spacing_size = 0;
			{
				const int parity_spacing = (int)info.parity_track_spacing_e2e;
				grid_parity_spacing_size = (parity_spacing / pitch) + 1;
			}
			this->kLineEndSpacing_[index] = grid_parity_spacing_size;


			// Mask num part.
			const int mask_num = (int)info.mask_num;
			this->kMaskNum_[index] = mask_num;
			
		}
		index++;
	}
	
	cout << "========== Parity Spacing in Grid ==========" << endl;
	int layer = 0;
	for(const int spacing : this->kLineEndSpacing_)
	{
		cout << "Layer " << layer << ", Parity Grid Spacing = " << spacing << endl; 
		layer++;
	}
	cout << endl;
		
	cout << "========== End-End Spacing in Grid ==========" << endl;
	layer = 0;
	for(const int spacing : this->kSafeSpacing_)
	{
		cout << "Layer " << layer << ", End-End Spacing = " << spacing << endl; 
		layer++;
	}
	cout << endl;

	cout << "========== Mask Number ==========" << endl;
	layer = 0;
	for(const int mask_num : this->kMaskNum_)
	{
		cout << "Layer " << layer << ", Mask Num = " << mask_num << endl; 
		layer++;
	}
	cout << endl << endl;

	return;
}

void LineEndGraph::SetMaskNum(
		const int grid_layer,
		const int mask_num
		)
{
	// Error-proofing.
	if(grid_layer < 0 || grid_layer >= layout_layer_)
		return;
	if(mask_num <= 0)	
		return;

	// Set the mask num of the specified layer.
	kMaskNum_[grid_layer] = mask_num;
	return;
}

void LineEndGraph::SetViaEncCostMode(const bool is_via_end_mode)
{
	this->kViaEncCostMode_ = is_via_end_mode;
}

void LineEndGraph::SetUpGraph(
		const int layout_width, 
		const int layout_height, 
		const int layout_layer,
		int* const track_capacity, 
		NetIdTable* const net_id_table
		)
{
	if(this->line_end_graph_ != NULL)
	{
		delete [] this->line_end_graph_;
	}
	
	this->layout_width_ = layout_width;
	this->layout_height_ = layout_height;
	this->layout_layer_ = layout_layer;
	
	
	this->p_track_capacity_ = track_capacity;
	this->p_net_id_table_ = net_id_table;
	
	
	const ULL array_size = layout_width_ * layout_height_ * layout_layer_;
	this->line_end_graph_ = new int[array_size];
	this->line_end_history_graph_ = new int[array_size];
	
	for(ULL index = 0; index < array_size; index++)
	{
		this->line_end_graph_[index] = kNone;
		this->line_end_history_graph_[index] = 0;
	}
		
	this->graph_is_setup_ = true;
}



// Init the location of pins int the line-end graph according to their pin types
void LineEndGraph::InitPartialPinLocation(
		const PinType pin_type, 
		const vector<Node> &pin_nodes
		)
{
	switch(pin_type)
	{
		case kPinPoint:
		case kPinLine:
		{
			for(const Node &pin_node : pin_nodes)
			{
				const ULL node_index = 
					Transform3dArrayTo1dIndex(pin_node);
				line_end_graph_[node_index] = kPinLocation;
			}
		}
		break;
		case kPinRectangle:
		{
			for(const Node &pin_node : pin_nodes)
			{
				const ULL node_index = 
					Transform3dArrayTo1dIndex(pin_node);
				line_end_graph_[node_index] = kPinLocationRectangle;
			}
		}
		break;
		default:
			// do nothing
			;
	}

	return;
}


// Insert the Pseudo Blockages by location of pins.
void LineEndGraph::InsertBlockageForPin(const vector<Node> &pin_nodes)
{
	 // Check whether the line-end violation tend to be around the pin
	 // Insert the Pseudo Blockage to the grid tended to have a line-end violation.
	for(const Node &pin_node : pin_nodes)
	{
		const bool has_line_end_rule = (kLineEndSpacing_[pin_node.z] != 0);
		if(!has_line_end_rule)
			continue;
		
		InsertBlockageForPin(pin_node);
		// const bool node_in_odd_layer = (pin_node.z == kOddLayer);
		// if(node_in_odd_layer)
			// InsertBlockageForPinHorizontal(pin_node);
		// else
			// InsertBlockageForPinVertical(pin_node);
	}	
	return;
}

// Veriry the line-end violation caused by pins and Insert Pseudo Blcokage.
void LineEndGraph::InsertBlockageForPin(const Node &pin)
{
	const int pin_x = pin.x;
	const int pin_y = pin.y;
	const int pin_z = pin.z;
	const bool is_horizontal = (pin.z == kOddLayer);

	const int kSafeSpacing = kSafeSpacing_[pin_z];
	const int start = (is_horizontal) ? (pin_x) : (pin_y);
	const int end = (is_horizontal) ? 
		(((pin_x + kSafeSpacing) < layout_width_) ? 
		(pin_x + kSafeSpacing) : (layout_width_ - 1)) :
		(((pin_y + kSafeSpacing) < layout_height_) ? 
		(pin_y + kSafeSpacing) : (layout_height_ - 1));
	
	bool line_end_pair_exist = false;
	int line_end_start = start;
	int line_end_end = end;
	const int pre_grid = start;
	const ULL pre_grid_index = (is_horizontal) ? 
		Transform3dArrayTo1dIndex(pre_grid, pin_y, pin_z) :
		Transform3dArrayTo1dIndex(pin_x, pre_grid, pin_z);
	const bool pre_grid_has_a_pin = 
		(line_end_graph_[pre_grid_index] == kPinLocation);
	for(int idx = start + 1; idx < end; idx++)
	{
		const int post_grid = idx;
		const ULL post_grid_index = (is_horizontal) ?
			Transform3dArrayTo1dIndex(post_grid, pin_y, pin_z) :
			Transform3dArrayTo1dIndex(pin_x, post_grid, pin_z);
		const bool post_grid_has_a_pin = 
			(line_end_graph_[post_grid_index] == kPinLocation);
		
		const bool has_pin_pair = 
			(pre_grid_has_a_pin && post_grid_has_a_pin);
		if(has_pin_pair)
		{
			const int pre_id = *(p_track_capacity_ + pre_grid_index);
			const int pre_two_pin_net_id = 
				(*p_net_id_table_)[pre_id].two_pin_net_id;
			const int post_id = *(p_track_capacity_ + post_grid_index);
			const int post_two_pin_net_id = 
				(*p_net_id_table_)[post_id].two_pin_net_id;
			const bool the_same_two_pin_net = 
				(pre_two_pin_net_id == post_two_pin_net_id);
			if(!the_same_two_pin_net)
			{
				line_end_pair_exist = true;
				line_end_start = pre_grid;
				line_end_end = post_grid;
			}
			break;
		}
	}
	if(!line_end_pair_exist)
		return;

	// Insert Blockage between line-end pair to avoid violaton.
	const int start2 = line_end_start;
	const int end2 = line_end_end;
	const int track = (is_horizontal) ? (pin_y) : (pin_x);
	const int layer_z = pin_z;
	for(int idx = start2 + 1; idx < end2; idx++)
	{
		const ULL grid_index = (is_horizontal) ? 
			Transform3dArrayTo1dIndex(idx, track, layer_z) : 
			Transform3dArrayTo1dIndex(track, idx, layer_z);
		line_end_graph_[grid_index] = kPinBlockage;
	}

	const int parity_track = track + 2/*kMaskNum_[layer_z]*/;
	const bool exceed_upper_boundary = (is_horizontal) ?
		(parity_track >= layout_height_):
		(parity_track >= layout_width_);
	if(exceed_upper_boundary)
		return;

	// Insert Blockage to Parity Pin if it intend to have violation.
	const ULL pre_grid_parity_index = (is_horizontal) ? 
		Transform3dArrayTo1dIndex(start2, parity_track, layer_z) :
		Transform3dArrayTo1dIndex(parity_track, start2, layer_z);
	const bool pre_grid_parity_has_a_pin = 
		(line_end_graph_[pre_grid_parity_index] == kPinLocation);

	const ULL post_grid_parity_index = (is_horizontal) ? 
		Transform3dArrayTo1dIndex(end2, parity_track, layer_z) :
		Transform3dArrayTo1dIndex(parity_track, end2, layer_z);
	const bool post_grid_parity_has_a_pin = 
		(line_end_graph_[post_grid_parity_index] == kPinLocation);

	if(!pre_grid_parity_has_a_pin && !post_grid_parity_has_a_pin)
		return;
	for(int idx = start2 + 1; idx < end2; idx++)
	{
		const ULL grid_index = (is_horizontal) ? 
			Transform3dArrayTo1dIndex(idx, parity_track, layer_z):
			Transform3dArrayTo1dIndex(parity_track, idx, layer_z);
		const bool grid_has_nothing = 
			(line_end_graph_[grid_index] == kNone);
		if(grid_has_nothing)
			line_end_graph_[grid_index] = kPinBlockage;
	}
		
	return;
}

// Normal blockage insertion
void LineEndGraph::InsertBlockage(const vector<TurnNode> &turn_nodes)
{
	for(const TurnNode &turn_node : turn_nodes)
	{
		const bool has_line_end_rule = (kLineEndSpacing_[turn_node.z] != 0);
		if(!has_line_end_rule)
			continue;
		if(IsPinOnGrid(turn_node) || IsBlockageOnGrid(turn_node))
			continue;
		const ULL node_index = 
			Transform3dArrayTo1dIndex(turn_node);
		const LineEndType le_type = turn_node.line_end_type;
		switch(le_type)
		{
			case kSegLeft:
				this->line_end_graph_[node_index] = kSegLeftEnd;
				break;
			case kSegRight:
				this->line_end_graph_[node_index] = kSegRightEnd;
				break;
			case kSegTop:
				this->line_end_graph_[node_index] = kSegTopEnd;
				break;
			case kSegBottom:
				this->line_end_graph_[node_index] = kSegBottomEnd;
				break;
			case kSegNone:
				this->line_end_graph_[node_index] = kPinPseudoLocation;
				break;
			default:
				// do nothing
				;
		}

		const bool is_odd_layer = (turn_node.z % 2 == kOddLayer);
		if(kSegNone != le_type)
		{
			InsertBlockage(turn_node);
		}
		else
		{
			TurnNode temp_pin_turn_node = turn_node;
			temp_pin_turn_node.line_end_type = (is_odd_layer) ? 
				kSegRight : kSegTop;
			InsertBlockage(temp_pin_turn_node);
			temp_pin_turn_node.line_end_type = (is_odd_layer) ? 
				kSegLeft : kSegBottom;
			InsertBlockage(temp_pin_turn_node);
		}
	}
	return;
}

void LineEndGraph::InsertBlockage(const TurnNode &turn_node)
{
	// 1. Check if has a end-end violation. 
	int line_end_start = -1;
	int line_end_end = -1;
	const bool line_end_pair_exist = 
		HasLineEndPair(turn_node, line_end_start, line_end_end);
	if(!line_end_pair_exist)
		return;
	
	// 2. Check if need to insert blockages. If yes, then insert them.
	const bool is_odd_layer = (turn_node.z % 2 == kOddLayer);
	const int track = (is_odd_layer) ? (turn_node.y) : (turn_node.x);
	const int layer = turn_node.z;
	// const int spacing = line_end_end - line_end_start;
	
	// const int kMaskNum = kMaskNum_[layer];
	// const int kLineEndSpacing = kLineEndSpacing_[layer];
	// const int check_parity_spacing = (kLineEndSpacing - spacing);

	int check_times = 4;
	bool insert_blockage_between_line_end_pair = false;
	while(check_times)
	{
		int parity_track = -1;
		int start = -1, end = -1;
		const bool is_position_okay =
			SetLineEndCheckPosition(
					check_times, 
					turn_node, 
					line_end_start, 
					line_end_end,
					parity_track,
					start,
					end
					);
		if(!is_position_okay)
		{
			check_times--;
			continue;
		}

		// check the pre part or post part of the parity track. 
		bool insert_blockage_flag = false; // pattern-1
		bool insert_exceed_blockage_flag = false; // pattern-2
		for(int coord = start; coord <= end; coord++)
		{
			const ULL node_index = (is_odd_layer) ? 
				Transform3dArrayTo1dIndex(coord, parity_track, layer) :
				Transform3dArrayTo1dIndex(parity_track, coord, layer);
			const int line_end_map_status = line_end_graph_[node_index];
			if(kCheckPreviousTrackPre == check_times || kCheckNextTrackPre == check_times)
			{
				const bool segment_pre_end = (is_odd_layer) ?
					(line_end_map_status == kSegRightEnd) :
					(line_end_map_status == kSegTopEnd);
				const bool segment_post_end = (is_odd_layer) ? 
					(line_end_map_status == kSegLeftEnd) :
					(line_end_map_status == kSegBottomEnd);
				const bool pin_pre_end = (
						(line_end_map_status == kPinLocation) ||
						(line_end_map_status == kPinPseudoLocation));
				if(segment_pre_end || pin_pre_end)
					insert_blockage_flag = true;
				else if(segment_post_end)
					insert_exceed_blockage_flag = true;
				else
					;
			}
			else if(kCheckPreviousTrackPost == check_times || kCheckNextTrackPost == check_times)
			{
				const bool segment_pre_end = (is_odd_layer) ? 
					(line_end_map_status == kSegRightEnd) :
					(line_end_map_status == kSegTopEnd);
				const bool segment_post_end = (is_odd_layer) ?
					(line_end_map_status == kSegLeftEnd) :
					(line_end_map_status == kSegBottomEnd);
				const bool pin_post_end = (
						(line_end_map_status == kPinLocation) ||
						(line_end_map_status == kPinPseudoLocation));
				if(segment_post_end || pin_post_end)
					insert_blockage_flag = true;
				else if(segment_pre_end)
					insert_exceed_blockage_flag = true; // today
				else
					;
			}
			else
			{}

			// Insert pattern-1 blockage.
			if(insert_blockage_flag)
			{
				int blockage_start = coord;
				int blockage_end = coord;
				if(kCheckPreviousTrackPre == check_times || kCheckNextTrackPre == check_times)
				{
					blockage_start = coord + 1;
					blockage_end = line_end_end;
				}
				else if(kCheckPreviousTrackPost == check_times || kCheckNextTrackPost == check_times)
				{
					blockage_start = line_end_start + 1;
					blockage_end = coord;
				}
				else
				{}
				
				for(int blockage_coord = blockage_start; blockage_coord < blockage_end; blockage_coord++)
				{
					const ULL blockage_index = (is_odd_layer) ?
						Transform3dArrayTo1dIndex(blockage_coord, parity_track, layer) :
						Transform3dArrayTo1dIndex(parity_track, blockage_coord, layer);
					int &line_end_map_status = line_end_graph_[blockage_index];
					const bool no_pin_blockage = (
							// (line_end_map_status == kPinPseudoLocation) &&
							(line_end_map_status != kPinLocation) &&
							(line_end_map_status != kPinBlockage) &&
							(line_end_map_status != kPinLocationRectangle) &&
							(line_end_map_status != kSegRightEnd) &&
							(line_end_map_status != kSegLeftEnd) &&
							(line_end_map_status != kSegTopEnd) &&
							(line_end_map_status != kSegBottomEnd));
					if(no_pin_blockage)
						line_end_map_status = kNormalBlockage;
					else 
						break;
				}
				// break;
			}

			// Insert pattern-2 blockage.
			else if(insert_exceed_blockage_flag)
			{
				int blockage_start = coord;
				int blockage_end = coord;
				if(kCheckPreviousTrackPre == check_times || kCheckNextTrackPre == check_times)
				{
					blockage_start = start;
					blockage_end = coord;
				}
				else if(kCheckPreviousTrackPost == check_times || kCheckNextTrackPost == check_times)
				{
					blockage_start = coord + 1;
					blockage_end = end + 1;
				}
				else
				{}

				for(int blockage_coord = blockage_start; blockage_coord < blockage_end; blockage_coord++)
				{
					const ULL blockage_index = (is_odd_layer) ?
						Transform3dArrayTo1dIndex(blockage_coord, parity_track, layer) :
						Transform3dArrayTo1dIndex(parity_track, blockage_coord, layer);
					int &line_end_map_status = line_end_graph_[blockage_index];
					const bool no_pin_blockage = (
							// (line_end_map_status == kPinPseudoLocation) &&
							(line_end_map_status != kPinLocation) &&
							(line_end_map_status != kPinBlockage) &&
							(line_end_map_status != kPinLocationRectangle) &&
							(line_end_map_status != kSegRightEnd) &&
							(line_end_map_status != kSegLeftEnd) &&
							(line_end_map_status != kSegTopEnd) &&
							(line_end_map_status != kSegBottomEnd));
					if(no_pin_blockage)
						line_end_map_status = kNormalBlockage;
					else
						break;
				}
				// break;
			}
			else
			{}

			if(insert_blockage_flag || insert_exceed_blockage_flag)
				insert_blockage_between_line_end_pair = true;
		}
		check_times--;
	}
	
	// insert the blockage between the line-end pair
	if(insert_blockage_between_line_end_pair)
	{
		for(int blockage_coord = line_end_start + 1; 
				blockage_coord < line_end_end; blockage_coord++)
		{
			const ULL index = (is_odd_layer) ?
				Transform3dArrayTo1dIndex(blockage_coord, track, layer) :
				Transform3dArrayTo1dIndex(track, blockage_coord, layer);
			int &line_end_map_status = line_end_graph_[index]; 
			const bool no_pin_blockage = (
					// (line_end_map_status == kPinPseudoLocation) &&
					(line_end_map_status != kPinLocation) &&
					(line_end_map_status != kPinBlockage) &&
					(line_end_map_status != kPinLocationRectangle) );
			if(no_pin_blockage)
				line_end_map_status = kNormalBlockage;
		}
	}
}

// Normal Blockage clean
void LineEndGraph::CleanBlockage(const vector<TurnNode> &turn_nodes)
{
	for(const TurnNode &turn_node : turn_nodes)
	{
		const bool has_line_end_rule = (kLineEndSpacing_[turn_node.z] != 0);
		if(!has_line_end_rule)
			return;
		if(IsPinRectOnGrid(turn_node) || IsPinBlockageOnGrid(turn_node))
			continue;

		const bool pin_connect_via = (turn_node.line_end_type == kSegNone);
		if(!pin_connect_via)
		{
			CleanBlockage(turn_node);
		}
		else
		{
			const bool is_odd_layer = (turn_node.z % 2 == 0);
			TurnNode temp_pin_turn_node = turn_node;
			temp_pin_turn_node.line_end_type = (is_odd_layer) ? 
				kSegRight : kSegTop;
			CleanBlockage(temp_pin_turn_node);
			temp_pin_turn_node.line_end_type = (is_odd_layer) ?
				kSegLeft : kSegBottom;
			CleanBlockage(temp_pin_turn_node);
		}
		const ULL node_index = 
		Transform3dArrayTo1dIndex(turn_node);
		const bool not_pin_location = (this->line_end_graph_[node_index] != kPinLocation);
		const bool not_pin_blockage = (this->line_end_graph_[node_index] != kPinBlockage);
		// const bool not_none_le_type = (le_type != kSegNone);
		if(not_pin_location && not_pin_blockage 
				// && not_none_le_type 
				)
		{
			const TurnNode &tmp_turn_node = turn_node;
			const bool potential_line_end_violation_grid = VerifyLineEndViolationVer2(tmp_turn_node);
			this->line_end_graph_[node_index] = (potential_line_end_violation_grid ? kPotentialBlockage : kNone);
		}
	}
	return;
}

void LineEndGraph::CleanBlockage(const TurnNode &turn_node)
{
	// 1. Check if has a end-end violation. 
	int line_end_start = -1;
	int line_end_end = -1;
	const bool line_end_pair_exist = 
		HasLineEndPair(turn_node, line_end_start, line_end_end);
	
	// 2. Check if need to clean blockages. If yes, then clean them.
	const bool is_odd_layer = (turn_node.z % 2 == kOddLayer);
	const int track = (is_odd_layer) ? (turn_node.y) : (turn_node.x);
	const int layer = turn_node.z;
	// const int spacing = line_end_end - line_end_start;
	
	// const int kMaskNum = kMaskNum_[layer];
	// const int kSafeSpacing = kSafeSpacing_[layer];
	const int kLineEndSpacing = kLineEndSpacing_[layer];
	// const int check_parity_spacing = (kLineEndSpacing - spacing);


	// Check whether need to clean blockage into line-end pair.
	if(line_end_pair_exist)
	{
		int check_times = 4;
		bool clean_blockage_between_line_end_pair = false; 
		while(check_times)
		{
			int parity_track = -1;
			int start = -1, end = -1;
			const bool is_position_okay =
				SetLineEndCheckPosition(
						check_times, 
						turn_node, 
						line_end_start, 
						line_end_end,
						parity_track,
						start,
						end
						);
			if(!is_position_okay)
			{
				check_times--;
				continue;
			}
			// check the pre part or post part of the parity track. 
			bool insert_blockage_flag = false; // pattern-1
			for(int coord = start; coord <= end; coord++)
			{
				const ULL node_index = (is_odd_layer) ? 
					Transform3dArrayTo1dIndex(coord, parity_track, layer) :
					Transform3dArrayTo1dIndex(parity_track, coord, layer);
				const int line_end_map_status = line_end_graph_[node_index];
				if(kCheckPreviousTrackPre == check_times || kCheckNextTrackPre == check_times)
				{
					const bool segment_pre_end = (is_odd_layer) ?
						(line_end_map_status == kSegRightEnd) :
						(line_end_map_status == kSegTopEnd);
					// const bool segment_post_end = (is_odd_layer) ? 
						// (line_end_map_status == kSegLeftEnd) :
						// (line_end_map_status == kSegBottomEnd);
					const bool pin_pre_end = (
							(line_end_map_status == kPinLocation) ||
							(line_end_map_status == kPinPseudoLocation));
					if(segment_pre_end || pin_pre_end)
						insert_blockage_flag = true;
					// else if(segment_post_end)
						// insert_exceed_blockage_flag = true;
					// else
						// ;
				}
				else if(kCheckPreviousTrackPost == check_times || kCheckNextTrackPost == check_times)
				{
					// const bool segment_pre_end = (line_end_map_status == kSegRightEnd);
					const bool segment_post_end = (line_end_map_status == kSegLeftEnd);
					const bool pin_post_end = (
							(line_end_map_status == kPinLocation) ||
							(line_end_map_status == kPinPseudoLocation));
					if(segment_post_end || pin_post_end)
						insert_blockage_flag = true;
					// else if(segment_pre_end)
						// insert_exceed_blockage_flag = true; // today
					// else
						// ;
				}
				else
				{}
				
				// Insert pattern-1 blockage.
				if(insert_blockage_flag)
				{
					int blockage_start = coord;
					int blockage_end = coord;
					if(kCheckPreviousTrackPre == check_times || kCheckNextTrackPre == check_times)
					{
						blockage_start = coord + 1;
						blockage_end = line_end_end;
					}
					else if(kCheckPreviousTrackPost == check_times || kCheckNextTrackPost == check_times)
					{
						blockage_start = line_end_start + 1;
						blockage_end = coord;
					}
					else
					{}
					
					for(int blockage_coord = blockage_start; blockage_coord < blockage_end; blockage_coord++)
					{
						const ULL blockage_index = (is_odd_layer) ?
							Transform3dArrayTo1dIndex(blockage_coord, parity_track, layer) :
							Transform3dArrayTo1dIndex(parity_track, blockage_coord, layer);
						int &line_end_map_status = line_end_graph_[blockage_index];
						const bool no_pin_blockage = (
								// (line_end_map_status != kPinPseudoLocation) &&
								(line_end_map_status != kPinLocation) &&
								(line_end_map_status != kPinBlockage) &&
								(line_end_map_status != kPinLocationRectangle));
						if(no_pin_blockage)
							line_end_map_status = kNone;
					}
					clean_blockage_between_line_end_pair = true;
					break;
				}
			}

			check_times--;
		}
		// clean the blockage between the line-end pair
		if(clean_blockage_between_line_end_pair)
		{
			for(int blockage_coord = line_end_start + 1; 
					blockage_coord < line_end_end; blockage_coord++)
			{
				const ULL index = (is_odd_layer) ?
					Transform3dArrayTo1dIndex(blockage_coord, track, layer) :
					Transform3dArrayTo1dIndex(track, blockage_coord, layer);
				int &line_end_map_status = line_end_graph_[index]; 
				const bool no_pin_blockage = (
						// (line_end_map_status != kPinPseudoLocation) &&
						(line_end_map_status != kPinLocation) &&
						(line_end_map_status != kPinBlockage) &&
						(line_end_map_status != kPinLocationRectangle) );
				if(no_pin_blockage)
					line_end_map_status = kNone;
			}
		}
	}
	else
	{
		// bool need_to_clean_blockage = true;
		// const LineEndType le_type = turn_node.line_end_type;
		const bool segment_pre_line_end = (
				(turn_node.line_end_type == kSegRight) ||
				(turn_node.line_end_type == kSegTop));
		const bool segment_post_line_end = (
				(turn_node.line_end_type == kSegLeft) ||
				(turn_node.line_end_type == kSegBottom));
		
		int start = -1, end = -1;
		if(segment_pre_line_end)
		{
			start = (is_odd_layer) ? (
					((turn_node.x + 1) < layout_width_) ? 
					(turn_node.x + 1) : (layout_width_ - 1)) : (
					((turn_node.y + 1) < layout_height_) ?
					(turn_node.y + 1) : (layout_height_ - 1));
			end = (is_odd_layer) ? (
				((turn_node.x + kLineEndSpacing) < layout_width_) ? 
				(turn_node.x + kLineEndSpacing) : (layout_width_ - 1)) : (
				((turn_node.y + kLineEndSpacing) < layout_height_) ? 
				(turn_node.y + kLineEndSpacing) : (layout_height_ - 1));
			
			for(int coord = start; coord < end; coord++)
			{
				const ULL node_index = (is_odd_layer) ?
					Transform3dArrayTo1dIndex(coord, track, layer) :
					Transform3dArrayTo1dIndex(track, coord, layer);
				int &line_end_map_status = line_end_graph_[node_index]; 
				const bool node_is_normal_blockage = 
					( line_end_map_status == kNormalBlockage);
				// const bool node_is_line_end = (
						// (line_end_map_status == kSegLeftEnd)
						// (line_end_map_status == kSegBottomEnd) || 
						// (line_end_map_status == kPinPseudoLocation) ||
						// (line_end_map_status == kPinLocation));

				if(node_is_normal_blockage)
					line_end_map_status = kNone;
				else 
					break;
				/*if(node_is_normal_blockage)
				{
					line_end_map_status = kNone;
				}
				else if(node_is_line_end)
				{
					need_to_clean_blockage = false;
					break;
				}
				else
				{
					break;
				}*/
			}
		}
		else if(segment_post_line_end)
		{
			start = (is_odd_layer) ? (
				((turn_node.x - 1) > 0) ? (turn_node.x - 1) : 0) : (
				((turn_node.y - 1) > 0) ? (turn_node.y - 1) : 0);
			end = (is_odd_layer) ? (
				((turn_node.x - kLineEndSpacing) > 0) ? (turn_node.x - kLineEndSpacing) : 0) : (
				((turn_node.y - kLineEndSpacing) > 0) ? (turn_node.y - kLineEndSpacing) : 0);

			for(int coord = start; coord > end; coord--)
			{
				const ULL node_index = (is_odd_layer) ?
					Transform3dArrayTo1dIndex(coord, track, layer) :
					Transform3dArrayTo1dIndex(track, coord, layer);
				int &line_end_map_status = line_end_graph_[node_index]; 
				const bool node_is_normal_blockage = 
					( line_end_map_status == kNormalBlockage);
				// const bool node_is_line_end = (
						// (line_end_map_status == kSegRightEnd) || 
						// (line_end_map_status == kSegTopEnd) || 
						// (line_end_map_status == kPinPseudoLocation) ||
						// (line_end_map_status == kPinLocation));

				if(node_is_normal_blockage)
					line_end_map_status = kNone;
				else
					break;
				/*if(node_is_normal_blockage)
				{
					line_end_map_status = kNone;
				}
				else if(node_is_line_end)
				{
					need_to_clean_blockage = false;
					break;
				}
				else
				{
					break;
				}*/
			}
		}
		else
		{}
		// clean the nodes
		/*if(need_to_clean_blockage)
		{
			for(vector<ULL>::iterator index_iter = clean_node_indexes.begin(); 
				index_iter != clean_node_indexes.end(); 
				index_iter++)
			{
				const ULL node_index = (*index_iter);
				this->line_end_graph_[node_index] = kNone;
			}
		}*/
	}
}

void LineEndGraph::CleanBlockageWholeGraph()
{
	const ULL graph_array_size = (ULL)layout_width_ * layout_height_ * layout_layer_;
	for(ULL index = 0; index < graph_array_size; index++)
	{
		int &line_end_map_status = line_end_graph_[index];
		const bool not_pin = 
			(
				(line_end_map_status != kPinLocation) &&
				(line_end_map_status != kPinLocationRectangle) &&
				(line_end_map_status != kPinBlockage)
			);
		if(not_pin)
		{
			line_end_map_status = kNone;
		}
	}
	return;
}


// 2016-11-22
bool LineEndGraph::VerifyLineEndViolation(
		const vector<TurnNode> &turn_nodes,
		vector<TurnNode> &vio_turn_nodes
		)
{
	bool has_line_end_violation = false;
	for(const TurnNode &turn_node : turn_nodes)
	{
		const LineEndType le_type = turn_node.line_end_type;
		if(kSegNone != le_type)
		{
			const bool has_violation = 
				VerifyLineEndViolationVer2(turn_node);
			if(has_violation)
				vio_turn_nodes.push_back(turn_node);
			
		}
		else
		{
			const bool is_odd_layer = (turn_node.z % 2 == 0);
			TurnNode temp_pin_turn_node = turn_node;
			temp_pin_turn_node.line_end_type = (is_odd_layer) ? 
				kSegRight : kSegTop;
			const bool has_violation = 
				VerifyLineEndViolationVer2(temp_pin_turn_node);
			if(has_violation)
				vio_turn_nodes.push_back(turn_node);
			

			if(!has_violation)
			{
				temp_pin_turn_node.line_end_type = (is_odd_layer) ? 
					kSegLeft : kSegBottom;
				const bool has_other_violation = 
					VerifyLineEndViolationVer2(temp_pin_turn_node);
				if(has_other_violation)
					vio_turn_nodes.push_back(turn_node);
			}
		}
	}

	has_line_end_violation = (vio_turn_nodes.size() > 0);
	return has_line_end_violation;
}

// Check if line-end violation around the node(Horizontal)
bool LineEndGraph::VerifyLineEndViolationVer2(const TurnNode &turn_node)
{
	bool has_line_end_violation = false;

	bool line_end_pair_exist = false;
	int line_end_start = -1;
	int line_end_end = -1;
	line_end_pair_exist = 
		HasLineEndPair(turn_node, line_end_start, line_end_end);
	if(!line_end_pair_exist)
		return has_line_end_violation;
	
	const bool is_odd_layer = (turn_node.z % 2 == 0);
	const int track = (is_odd_layer) ? (turn_node.y) : (turn_node.x);
	const int layer = turn_node.z;

	// check whether the line-end pair is a pin pair
	const ULL pre_end_node_index = (is_odd_layer) ?
		Transform3dArrayTo1dIndex(line_end_start, track, layer) :
		Transform3dArrayTo1dIndex(track, line_end_start, layer);
	const bool pre_end_is_a_pin = 
		(line_end_graph_[pre_end_node_index] == kPinLocation);
	const ULL post_end_node_index = (is_odd_layer) ?
		Transform3dArrayTo1dIndex(line_end_end, track, layer) :
		Transform3dArrayTo1dIndex(track, line_end_end, layer);
	const bool post_end_is_a_pin = 
		(line_end_graph_[post_end_node_index] == kPinLocation);
	const bool checked_line_end_pair_is_pin_pair = 
		(pre_end_is_a_pin && post_end_is_a_pin);

	// check the violation in parity track
	int check_times = 4;
	bool all_parity_line_end_pairs_are_pin_pairs = true;
	while(check_times)
	{
		int start = -1;
		int end = -1;
		int parity_track = -1;
		const bool is_position_okay = 
			SetLineEndCheckPosition(
					check_times, 
					turn_node, 
					line_end_start, 
					line_end_end,
					parity_track,
					start,
					end
					);
		if(!is_position_okay)
		{
			check_times--;
			continue;
		}

		// check the pre part or post part of the parity track. 
		int pre_coord = start;
		for(int coord = start + 1; coord <= end; coord++)
		{
			const ULL pre_end_grid_index = (is_odd_layer) ?
				Transform3dArrayTo1dIndex(pre_coord, parity_track, layer) :
				Transform3dArrayTo1dIndex(parity_track, pre_coord, layer);
			const bool pre_grid_is_segment_pre_end = (
					(this->line_end_graph_[pre_end_grid_index] == kSegRightEnd) ||
					(this->line_end_graph_[pre_end_grid_index] == kSegTopEnd) ||
					(this->line_end_graph_[pre_end_grid_index] == kPinLocation));
			const ULL post_end_grid_index = (is_odd_layer) ?
				Transform3dArrayTo1dIndex(coord, parity_track, layer) :
				Transform3dArrayTo1dIndex(parity_track, coord, layer);
			const bool post_grid_is_segment_post_end = (
					(this->line_end_graph_[post_end_grid_index] == kSegLeftEnd) ||
					(this->line_end_graph_[post_end_grid_index] == kSegBottomEnd) ||
					(this->line_end_graph_[post_end_grid_index] == kPinLocation));
			const bool has_line_end_pair = (
					pre_grid_is_segment_pre_end && 
					post_grid_is_segment_post_end);
			
			if(!pre_grid_is_segment_pre_end)
			{
				pre_coord = coord;
			}
			else if(has_line_end_pair)
			{
				// check whether the line-end pair is a pin pair
				all_parity_line_end_pairs_are_pin_pairs = false;
				has_line_end_violation = true;
				line_end_history_graph_[pre_end_grid_index] += 1;
				line_end_history_graph_[post_end_grid_index] += 1;
				pre_coord = coord + 1;
			}
			else
			{}
		}
		check_times--;
	}
	
	// If all the violated line-end are all pin-ends,
	// Then, the violation doesn't count.
	if(has_line_end_violation && 
			checked_line_end_pair_is_pin_pair && 
			all_parity_line_end_pairs_are_pin_pairs) 
		has_line_end_violation = false;

	return has_line_end_violation;
}


// Check If the node has line-end pair and return the line-end positions.
// The pair is within the safe distance.
bool LineEndGraph::HasLineEndPair(
		const TurnNode &turn_node, 
		int &line_end_head, 
		int &line_end_tail
		)
{
	bool line_end_pair_exist = false;

	const int turn_node_x = turn_node.x;
	const int turn_node_y = turn_node.y;
	const int turn_node_z = turn_node.z;
	const LineEndType le_type = (turn_node.line_end_type);

	// Check if has the line-end pair of the TurnNode.
	// Set the position of the pair.
	const int kSafeSpacing = kSafeSpacing_[turn_node_z];
	// const int kSafeSpacing = kLineEndSpacing_[turn_node_z];
	line_end_head = -1;
	line_end_tail = -1;
	switch(le_type)
	{
		case kSegRight:
		{
			const int start_x = turn_node_x;
			const int end_x = 
				((turn_node_x + kSafeSpacing) < layout_width_) ? 
				(turn_node_x + kSafeSpacing) : (layout_width_ - 1);
			const ULL pre_grid_index = 
				Transform3dArrayTo1dIndex(start_x, turn_node_y, turn_node_z);
			const bool pre_grid_is_segment_right_end = (
				( line_end_graph_[pre_grid_index] == kSegRightEnd) ||
				( line_end_graph_[pre_grid_index] == kPinLocation) ||
				( line_end_graph_[pre_grid_index] == kPinPseudoLocation) );
			if(pre_grid_is_segment_right_end)
				for(int x = start_x + 1; x < end_x; x++)
				{
					const ULL node_index = 
						Transform3dArrayTo1dIndex(x, turn_node_y, turn_node_z);
					const bool node_is_segment_left_line_end = (
						( line_end_graph_[node_index] == kSegLeftEnd) ||
						( line_end_graph_[node_index] == kPinLocation) ||
						( line_end_graph_[node_index] == kPinPseudoLocation) );
					if(node_is_segment_left_line_end)
					{
						line_end_pair_exist = true;
						line_end_head = start_x;
						line_end_tail = x;
						break;
					}
				}
		}
		break;
		case kSegLeft:
		{
			const int start_x = 
				((turn_node_x - kSafeSpacing) < 0) ? 
				(0) : (turn_node_x - kSafeSpacing);
			const int end_x = turn_node_x;
			const ULL post_grid_index = 
				Transform3dArrayTo1dIndex(end_x, turn_node_y, turn_node_z);
			const bool post_grid_is_segment_left_end = (
				( line_end_graph_[post_grid_index] == kSegLeftEnd) ||
				( line_end_graph_[post_grid_index] == kPinLocation) ||
				( line_end_graph_[post_grid_index] == kPinPseudoLocation));
			if(post_grid_is_segment_left_end)
				for(int x = end_x - 1; x > start_x ; x--)
				{
					const ULL node_index = 
						Transform3dArrayTo1dIndex(x, turn_node_y, turn_node_z);
					const bool node_is_segment_right_line_end = (
						( line_end_graph_[node_index] == kSegRightEnd) ||
						( line_end_graph_[node_index] == kPinLocation) ||
						( line_end_graph_[node_index] == kPinPseudoLocation) );
					if(node_is_segment_right_line_end)
					{
						line_end_pair_exist = true;
						line_end_head = x;
						line_end_tail = end_x;
						break;
					}
				}
		}
		break;
		case kSegTop:
		{
			const int start_y = turn_node_y;
			const int end_y = 
				((turn_node_y + kSafeSpacing) < layout_height_) ? 
				(turn_node_y + kSafeSpacing) : (layout_height_ - 1);
			const ULL pre_grid_index = 
				Transform3dArrayTo1dIndex(turn_node_x, start_y, turn_node_z);
			const bool pre_grid_is_segment_top_end = (
					( line_end_graph_[pre_grid_index] == kSegTopEnd) ||
					( line_end_graph_[pre_grid_index] == kPinLocation) || 
					( line_end_graph_[pre_grid_index] == kPinPseudoLocation) );
			if(pre_grid_is_segment_top_end)
				for(int y = start_y + 1; y < end_y; y++)
				{
					const ULL node_index = 
						Transform3dArrayTo1dIndex(turn_node_x, y, turn_node_z);
					const bool node_is_segment_bottom_line_end = (
						(line_end_graph_[node_index] == kSegBottomEnd) ||
						( line_end_graph_[node_index] == kPinLocation) ||
						( line_end_graph_[node_index] == kPinPseudoLocation) );
					if(node_is_segment_bottom_line_end)
					{
						line_end_pair_exist = true;
						line_end_head = start_y;
						line_end_tail = y;
						break;
					}
				}
		}
		break;
		case kSegBottom:
		{
			const int start_y = 
				((turn_node_y - kSafeSpacing) < 0) ? 
				(0) : (turn_node_y - kSafeSpacing);
			const int end_y = turn_node_y;
			const ULL post_grid_index = 
				Transform3dArrayTo1dIndex(turn_node_x, end_y, turn_node_z);
			const bool post_grid_is_segment_bottom_end = (
				( line_end_graph_[post_grid_index] == kSegBottomEnd) ||
				( line_end_graph_[post_grid_index] == kPinLocation) ||
				( line_end_graph_[post_grid_index] == kPinPseudoLocation) );
			if(post_grid_is_segment_bottom_end)
				for(int y = end_y - 1; y > start_y; y--)
				{
					const ULL node_index = 
						Transform3dArrayTo1dIndex(turn_node_x, y, turn_node_z);
					const bool node_is_segment_top_line_end = (
						(line_end_graph_[node_index] == kSegTopEnd) ||
						( line_end_graph_[node_index] == kPinLocation) ||
						( line_end_graph_[node_index] == kPinPseudoLocation) );
					if(node_is_segment_top_line_end)
					{
						line_end_pair_exist = true;
						line_end_head = y;
						line_end_tail = end_y;
						break;
					}
				}
		}
		break;
		default:
			;
	}
	return line_end_pair_exist;
}

inline bool LineEndGraph::SetLineEndCheckPosition(
		const int check_id,
		const TurnNode &turn_node, 
		const int line_end_head,
		const int line_end_tail,
		int &parity_track, 
		int &start, 
		int &end
		)
{
	bool is_position_okay = true;
	
	const int spacing = line_end_tail - line_end_head;
	const int check_parity_spacing = kLineEndSpacing_[turn_node.z] - spacing;
	const int kMaskNum = kMaskNum_[turn_node.z];
	const bool is_odd_layer = (turn_node.z % 2 == kOddLayer);
	
	// 1. Set parity track.
	if(kCheckNextTrackPre == check_id || 
			kCheckNextTrackPost == check_id)
	{
		parity_track = (is_odd_layer) ?
			(turn_node.y + kMaskNum) : (turn_node.x + kMaskNum);
		const bool exceed_boundary = (is_odd_layer) ?
			(parity_track >= layout_height_) : (parity_track >= layout_width_);
		if(exceed_boundary)
			is_position_okay = false;
	}
	else if(kCheckPreviousTrackPre == check_id || 
			kCheckPreviousTrackPost == check_id)
	{
		parity_track = (is_odd_layer) ?
			(turn_node.y - kMaskNum) : (turn_node.x - kMaskNum);
		const bool exceed_boundary = (is_odd_layer) ?
			(parity_track < 0) : (parity_track < 0);
		if(exceed_boundary)
			is_position_okay = false;
	}
	else
	{
		is_position_okay = false;
	}

	if(!is_position_okay) 
		return is_position_okay;

	// 2. Set line-end check position.
	if(kCheckNextTrackPre == check_id || 
			kCheckPreviousTrackPre == check_id)
	{
		start = 
			(line_end_head - check_parity_spacing > 0) ? 
			(line_end_head - check_parity_spacing) : (0);
		if(this->kViaEncCostMode_)
		{
			end = (is_odd_layer) ? (
					(line_end_head + 1 < layout_width_) ? 
					(line_end_head + 1) : (layout_width_ - 1)) : ( 
					(line_end_head + 1 < layout_height_) ? 
					(line_end_head + 1) : (layout_height_ - 1) );
		}
		else
		{
			end = line_end_head;
		}
	}
	else if(kCheckNextTrackPost == check_id ||
			kCheckPreviousTrackPost == check_id)
	{
		if(this->kViaEncCostMode_)
		{
			start = 
				(line_end_tail - 1 > 0) ? 
				(line_end_tail - 1) : (0);
		}
		else
		{
			start = line_end_tail;
		}
		
		end = (is_odd_layer) ? (
				(line_end_tail + check_parity_spacing < layout_width_) ? 
				(line_end_tail + check_parity_spacing) : (layout_width_ - 1)) : (
				(line_end_tail + check_parity_spacing < layout_height_) ? 
				(line_end_tail + check_parity_spacing) : (layout_height_ - 1));
	
	}
	else
	{
		is_position_okay = false;
	}
	
	return is_position_okay;
}


// Indentify the line-end violation of the whole graph.
int LineEndGraph::AnalyzeLineEndViolation()
{
	// real cut number(good + bad)
	// int cut_num = 0; 
	// bad cut number(1 < spacing < line_end_spacing)
	int vio_cut_num = 0; 
	// bad but number(violates the parity track number)
	int parity_vio_cut_num = 0;
	// the line-end violations caused by the original location of pin pairs.
	// int pin_pair_line_end_violation_number = 0; 

	for(int z = 0; z < layout_layer_; z++)
	{
		// If has no line-end rule in the layer,
		// then, skip the check.
		const bool has_line_end_rule = (kSafeSpacing_[z] > 0);
		if(!has_line_end_rule) 
			continue;
		
		const bool is_odd_layer = (z % 2 == kOddLayer);
		const int kMaskNum = kMaskNum_[z];
		const int kSafeSpacing = kSafeSpacing_[z];
		const int track_boundary = (is_odd_layer) ? 
			(layout_height_ - kMaskNum) : (layout_width_ - kMaskNum);
		const int coord_boundary = (is_odd_layer) ? 
			(layout_width_ - 1) : (layout_height_ - 1);
		for(int track = 0; track < track_boundary; track++)
		{
			// int spacing_count = 1;
			int pre_coord = 0;
			for(int coord = 1; coord < coord_boundary; coord++)
			{
				const int spacing_count = coord - pre_coord;
				const ULL pre_end_grid_index = (is_odd_layer) ?
					Transform3dArrayTo1dIndex(pre_coord, track, z) :
					Transform3dArrayTo1dIndex(track, pre_coord, z);
				const int pre_grid_number = 
					(*p_net_id_table_)[p_track_capacity_[pre_end_grid_index]].net_id;
				const bool pre_grid_number_is_a_net = (pre_grid_number > 0);
				
				const ULL post_end_grid_index = (is_odd_layer) ?
					Transform3dArrayTo1dIndex(coord, track, z) : 
					Transform3dArrayTo1dIndex(track, coord, z);
				const int post_grid_number = 
					(*p_net_id_table_)[p_track_capacity_[post_end_grid_index]].net_id;
				const bool post_grid_number_is_a_net = (post_grid_number > 0);
				
				const bool exceed_spacing = (spacing_count >= kSafeSpacing);
				const bool one_unit_spacing_width = (spacing_count == 1);
				const bool the_same_net = 
					((pre_grid_number_is_a_net && post_grid_number_is_a_net) && 
					 (pre_grid_number == post_grid_number));
				const bool not_the_same_net = 
					((pre_grid_number_is_a_net && post_grid_number_is_a_net) && 
					 (pre_grid_number != post_grid_number));
				
				const bool pre_grid_is_empty_or_blockage = 
					(pre_grid_number == 0 || pre_grid_number == kBlockage);
				const bool post_grid_is_empty_or_blockage = 
					(post_grid_number == 0 || post_grid_number == kBlockage);
				
				const bool has_line_end_pair = 
					((the_same_net && !one_unit_spacing_width) ||
					 (not_the_same_net));
				const bool adjacent_metal = 
					(the_same_net && one_unit_spacing_width);
				const bool over_spacing_width_checking = 
					(!pre_grid_is_empty_or_blockage && 
					 post_grid_is_empty_or_blockage && 
					 exceed_spacing);
				if(pre_grid_is_empty_or_blockage || 
						adjacent_metal || 
						over_spacing_width_checking)
				{
					pre_coord = coord;
					// spacing_count = 0;
				}
				else if(has_line_end_pair)
				{
					const int start = pre_coord;
					const int end = coord;
					
					parity_vio_cut_num += 
						AnalyzeParityLineEndViolation(start, end, track, z);
					const bool exceed_one_spacing = 
						(spacing_count > 1 && spacing_count < kSafeSpacing);
					if(exceed_one_spacing)
						vio_cut_num += 1;
					
					/*const bool line_end_is_pin_pair = 
							((this->line_end_graph_[pre_end_grid_index] == kPinLocation) && 
								(this->line_end_graph_[post_end_grid_index] == kPinLocation));
					if(line_end_is_pin_pair)
					{
						pin_pair_line_end_violation_number += 
							AnalyzeLineEndPinPairViolationParityHorizontalTrack(
									start_x, end_x, track_y, layer_z);
					}*/

					pre_coord = coord;
					// spacing_count = 0;
				}
				else
				{
					// do nothing
				}
				// spacing_count++;
			}
		}
	}
	
	
	// cout << "--------------------------" << endl;
	// cout << " - Grid End-End Spacing Vios: ";
	// cout << vio_cut_num << endl;
	// cout << " - Grid Parity Vios:  "; 
	// cout << parity_vio_cut_num << endl;
	

	return parity_vio_cut_num;

}


// Find if the input line-end has parity line-end violation.
// Only find violaton on the next higher track.
int LineEndGraph::AnalyzeParityLineEndViolation(
		const int line_end_start, 
		const int line_end_end,
		const int track,
		const int layer
		)
{
	int line_end_vios_num = 0;
	
	const int kSafeSpacing = kSafeSpacing_[layer];
	const int kLineEndSpacing = kLineEndSpacing_[layer];
	const int parity_track = track + kMaskNum_[layer];
	const int spacing = line_end_end - line_end_start;
	const bool is_odd_layer = (layer % 2 == kOddLayer);

	// No need to analyze the line-end violation.
	const bool exceed_boundary = (is_odd_layer) ? 
		(parity_track >= layout_width_) : 
		(parity_track >= layout_height_);
	const bool exceed_safe_spacing = 
		(spacing >= kSafeSpacing);
	if(exceed_boundary || exceed_safe_spacing)
		return line_end_vios_num;

	// Check if has the parity line-end violation in the parity track.
	int check_times = 2;
	const int kCheckPre = 2;
	const int kCheckPost = 1;
	const int check_parity_spacing = (kLineEndSpacing - spacing);
	while(check_times)
	{
		int start = 0;
		int end = 0;
		if(kCheckPre == check_times)
		{
			// start = (line_end_start - check_parity_spacing > 0) ? 
				// (line_end_start - check_parity_spacing) : (0);
			// end = line_end_start;
			start = 
				(line_end_start - check_parity_spacing > 0) ? 
				(line_end_start - check_parity_spacing) : (0);
			if(this->kViaEncCostMode_)
			{
				end = (is_odd_layer) ? (
						(line_end_start + 1 < layout_width_) ? 
						(line_end_start + 1) : (layout_width_ - 1)) : ( 
						(line_end_start + 1 < layout_height_) ? 
						(line_end_start + 1) : (layout_height_ - 1) );
			}
			else
			{
				end = line_end_start;
			}
		}
		else if(kCheckPost == check_times)
		{
			// start = line_end_end;
			// end = (is_odd_layer) ? (
					// (line_end_end + check_parity_spacing < layout_width_) ? 
					// (line_end_end + check_parity_spacing) : (layout_width_ - 1)) : (
					// (line_end_end + check_parity_spacing < layout_height_) ?
					// (line_end_end + check_parity_spacing) : (layout_height_ - 1));
			if(this->kViaEncCostMode_)
			{
				start = 
					(line_end_end - 1 > 0) ? 
					(line_end_end - 1) : (0);
			}
			else
			{
				start = line_end_end;
			}
			
			end = (is_odd_layer) ? (
					(line_end_end + check_parity_spacing < layout_width_) ? 
					(line_end_end + check_parity_spacing) : (layout_width_ - 1)) : (
					(line_end_end + check_parity_spacing < layout_height_) ? 
					(line_end_end + check_parity_spacing) : (layout_height_ - 1));
		}
		else
		{
			cout << "Something goes wrong in AnalyzeParityLineEndViolation()." << endl;
			continue;
		}

		int pre_coord = start + 1;
		for(int coord = start + 2; coord <= end; coord++)
		{
			const ULL pre_end_grid_index = (is_odd_layer) ? 
				Transform3dArrayTo1dIndex(pre_coord, parity_track, layer) :
				Transform3dArrayTo1dIndex(parity_track, pre_coord, layer);
			const int pre_grid_number = 
				(*p_net_id_table_)[p_track_capacity_[pre_end_grid_index]].net_id;
			const bool pre_grid_number_is_a_net = (pre_grid_number > 0);
			
			const ULL post_end_grid_index = (is_odd_layer) ?
				Transform3dArrayTo1dIndex(coord, parity_track, layer) :
				Transform3dArrayTo1dIndex(parity_track, coord, layer);
			const int post_grid_number = 
				(*p_net_id_table_)[p_track_capacity_[post_end_grid_index]].net_id;
			const bool post_grid_number_is_a_net = (post_grid_number > 0);
			const bool one_unit_spacing_width = ((coord - pre_coord) == 1);
			
			const bool the_same_net = 
				((pre_grid_number_is_a_net && post_grid_number_is_a_net) && 
				 (pre_grid_number == post_grid_number));
			const bool not_the_same_net = 
				((pre_grid_number_is_a_net && post_grid_number_is_a_net) && 
				 (pre_grid_number != post_grid_number));
			const bool pre_grid_is_empty_or_blockage = 
				(pre_grid_number == 0 || pre_grid_number == kBlockage);
			const bool has_line_end_pair = 
				((the_same_net && !one_unit_spacing_width) ||
				 (not_the_same_net));
			const bool adjacent_metal = (the_same_net && one_unit_spacing_width);
			if(pre_grid_is_empty_or_blockage || adjacent_metal)
			{
				pre_coord = coord;
			}
			else if(has_line_end_pair)
			{
				line_end_vios_num++;
				pre_coord = coord;
			}
			else
			{
				// do nothing.
			}
		}
		check_times--;
	}

	return line_end_vios_num;
}

// debug
void LineEndGraph::PrintLineEndGraph()
{
	printf("\n\nLine End Graph\n");
	for(int z = 0; z < layout_layer_; z++)
	{
		printf("LAYER %d\n", z);
		for(int y = layout_height_ - 1; y >= 0 ;y--)
		{
			for(int x = 0; x < layout_width_; x++)
			{
				const ULL grid_index = 
					Transform3dArrayTo1dIndex(x, y, z);
				switch(line_end_graph_[grid_index])
				{
					case kPinLocation:
						printf("P ");
						break;
					case kPinLocationRectangle:
						printf("Q ");
						break;
					case kSegLeftEnd:
						printf("L ");
						break;
					case kSegRightEnd:
						printf("R ");
						break;
					case kSegBottomEnd:
						printf("B ");
						break;
					case kSegTopEnd:
						printf("T ");
						break;
					case kPinPseudoLocation:
						printf("S ");
						break;
					case kPinBlockage:
						printf("- ");
						break;
					case kNormalBlockage:
						printf("* ");
						break;
					case kPotentialBlockage:
						printf("$ ");
						break;
					default:
						printf(". ");
						break;
				}
			}
			printf("\n");
		}
		printf("\n\n");
	}
}

