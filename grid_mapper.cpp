#include "grid_mapper.h"

PseudoNet::Pin::Pin(Box &box, UInt layer)
{
	this->pin_box.set(box.lowerLeft(), box.upperRight());
	this->pin_layer = layer;
}

PseudoNet::Pin::Pin(Pin &p_pin)
{
	this->pin_box.set(p_pin.pin_box.lowerLeft(), p_pin.pin_box.upperRight());
	this->pin_layer = p_pin.pin_layer;
}

float GetEdgeCost(PseudoNet::Pin &pin1, PseudoNet::Pin &pin2)
{
	float cost = 1.0;
	if (pin1.pin_box.overlaps(pin2.pin_box))
	{
		cost = 1.0;
	}
	else
	{
		const float layer_dist =
			(abs((int)pin1.pin_layer - (int)pin2.pin_layer) > 0) ?
			(abs((int)pin1.pin_layer - (int)pin2.pin_layer)) : (1.0);
		Point pt1, pt2;
		pin1.pin_box.getCenter(pt1);
		pin2.pin_box.getCenter(pt2);
		const float area_dist =
			abs(pt1.x() - pt2.x()) + abs(pt1.y() - pt2.y());
		cost = layer_dist + area_dist;
		if (pt1.x() != pt2.x() && pt1.y() != pt2.y())
			cost *= 1.5;
	}

	return cost;
}

GridMapper::GridMapper(
		RoutingLayerInfos *p_infos, oaBlock *top_block,
		const int h_track_start, const int v_track_start, 
		const int h_track_spacing, const int v_track_spacing,
		const int h_track_num, const int v_track_num,
		const int low_layer, const int high_layer
		)
{
	this->top_block_ = top_block;
	if(!top_block_)
	{
		cout << "GridMapper: Can't get TopBlock!" << endl;
		exit(1);
	}
	this->p_infos_ = p_infos;
	if(!p_infos_)
	{
		cout << "GridMapper: Can't routing layer infos." << endl;
		exit(1);
	}
	
	this->h_track_start_ = h_track_start;
	this->v_track_start_ = v_track_start;
	this->h_track_spacing_ = h_track_spacing;
	this->v_track_spacing_ = v_track_spacing;
	this->h_track_num_ = h_track_num;
	this->v_track_num_ = v_track_num;
	this->low_layer_ = low_layer;
	this->high_layer_ = high_layer;

	int layer_count = 0;
	for(int index = low_layer; index <= high_layer; index += 2)
	{
		this->layer_map_[index] = layer_count;
		this->reverse_layer_map_[layer_count] = index;
		layer_count ++;
		for(int info_index = 0; info_index < p_infos_->GetSize(); info_index++)
		{
			if((int)(*p_infos_)[info_index].layer == index)
			{
				this->infos_layer_map_[index] = info_index;
				this->reverse_infos_layer_map_[info_index] = index;
				break;
			}
		}
	}

	this->grid_width_ = v_track_num_;
	this->grid_height_ = h_track_num_;
	this->grid_layer_ = layer_count;

	for(LayerInfo &info : p_infos_->GetLayerInfos())
	{
		if((int)info.layer == low_layer)
		{
			is_first_layer_vertical_ = info.is_vertical;
			break;
		}
	}
	
	// Set via enclosure expansion.
	this->kViaEncExpansion_.resize(grid_layer_);
	for(unsigned int idx = 0; idx < kViaEncExpansion_.size(); idx++)
		kViaEncExpansion_[idx] = false;
	
	const int v_pitch = h_track_spacing_;
	const int h_pitch = v_track_spacing_;
	int idx = 0;
	for(int layer_num = low_layer_; layer_num < high_layer_; layer_num += 2)
	{
		if(layer_num == low_layer)
		{
			LayerInfo &info = (*p_infos_)[(layer_num / 2) - 1];
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
			LayerInfo &next_info = (*p_infos_)[(layer_num / 2)];
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
			LayerInfo &info = (*p_infos_)[(layer_num / 2)];
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
}


void GridMapper::MapTwoPinNet(
		const PseudoNet &pseudo_net, 
		const IndexPairs &pairs, 
		vector<TwoPinRipUpNet> &ripup_nets
		)
{
	if(pseudo_net.pins.size() < 2)
	{
		oaNativeNS ns;
		oaString net_name;
		pseudo_net.net->getName(ns, net_name);
		cout << "GridMapper: (Pin)Net " << net_name;
		cout << " has error pin number = " << pseudo_net.pins.size() << endl;
		exit(1);
	}

	int two_pin_net_count = 1;
	TwoPinRipUpNet tmp_net;
	int left, bottom, right, top;
	for(const IndexPair &index_pair : pairs)
	{
		tmp_net.p_net = pseudo_net.net;
		tmp_net.net_id = net_id_;
		tmp_net.two_pin_net_id = net_id_ + two_pin_net_count;

		// source pin
		left = -1; bottom = -1; right = -1; top = -1;
		const PseudoNet::Pin &source = pseudo_net.pins[index_pair.first];
		MapToGrid(source, left, bottom, right, top);
		oaFig *p_source_fig = source.pin_fig;
		tmp_net.two_pin_net_connection.source_box.set(
				source.pin_box.lowerLeft(), source.pin_box.upperRight());
		PinType source_type = 
			CheckPinType(p_source_fig, left, bottom, right, top);
		const int source_layer = 
			(layer_map_.find(source.pin_layer) != layer_map_.end()) ? 
			(layer_map_[source.pin_layer]) : (-1);
		switch(source_type)
		{
			case kPinPoint:
			{
				tmp_net.two_pin_net_connection.source.SetPinPoint(
						kPinPoint, left, bottom, source_layer);
			}
			break;
			case kPinLine:
			{
				tmp_net.two_pin_net_connection.source.SetPinLine(
						kPinLine, left, bottom, source_layer, 
						right, top, source_layer);
			}
			break;
			case kPinRectangle:
			{
				tmp_net.two_pin_net_connection.source.SetPinRectangle(
						kPinRectangle, left, bottom, source_layer, 
						right, top, source_layer);
			}
			break;
			default:
				;
		}

		// target pin
		left = -1; bottom = -1; right = -1; top = -1;
		const PseudoNet::Pin &target = pseudo_net.pins[index_pair.second];
		MapToGrid(target, left, bottom, right, top);
		oaFig *p_target_fig = target.pin_fig;
		tmp_net.two_pin_net_connection.target_box.set(
				target.pin_box.lowerLeft(), target.pin_box.upperRight());
		PinType target_type = 
			CheckPinType(p_target_fig, left, bottom, right, top);
		const int target_layer = 
			(layer_map_.find(target.pin_layer) != layer_map_.end()) ? 
			(layer_map_[target.pin_layer]) : (-1);
		switch(target_type)
		{
			case kPinPoint:
			{
				tmp_net.two_pin_net_connection.target.SetPinPoint(
						kPinPoint, left, bottom, target_layer);
			}
			break;
			case kPinLine:
			{
				tmp_net.two_pin_net_connection.target.SetPinLine(
						kPinLine, left, bottom, target_layer, 
						right, top, target_layer);
			}
			break;
			case kPinRectangle:
			{
				tmp_net.two_pin_net_connection.target.SetPinRectangle(
						kPinRectangle, left, bottom, target_layer, 
						right, top, target_layer);
			}
			break;
			default:
				;
		}
		ripup_nets.push_back(tmp_net);
		two_pin_net_count++;
	}

	net_id_ += two_pin_net_count;

}


void GridMapper::MapBlockage(
		const PseudoNet &pseudo_net, 
		vector<Node> &blockage_nodes
		)
{
	oaName name;
	oaString name_str;
	pseudo_net.net->getName(name);
	name.get(name_str);
	const bool is_global_net = 
		(pseudo_net.net->isGlobal() || 
		 name_str == oaString("VDD") || 
		 name_str == oaString("VSS") ||
		 name_str == oaString("CLOCK"));

	if(pseudo_net.pins.size() != 1 && !is_global_net)
	{
		oaNativeNS ns;
		oaString net_name;
		pseudo_net.net->getName(ns, net_name);
		cout << "GridMapper: (ObstacleMap)Net " << net_name;
		cout << " has error pin number = " << pseudo_net.pins.size() << endl;
		exit(1);
	}

	for(const PseudoNet::Pin &obs_pin : pseudo_net.pins)
	{
		int left = -1, bottom = -1, right = -1, top = -1;
		MapToGrid(obs_pin, left, bottom, right, top);
		const int obs_layer = 
			(layer_map_.find(obs_pin.pin_layer) != layer_map_.end()) ? 
			(layer_map_[obs_pin.pin_layer]) : (-1);

		for(int x = left; x <= right; x++)
		{
			for(int y = bottom; y <= top; y++)
				blockage_nodes.push_back(Node(x, y, obs_layer));
		}
	}

	return;
}

void GridMapper::MapToGrid(
		const PseudoNet::Pin &pin, 
		int &left, 
		int &bottom, 
		int &right, 
		int &top
		)
{
	const int l_offset = 
		(pin.pin_box.left() - v_track_start_) % v_track_spacing_; 
	const float l_offset_precision = 
		(float)l_offset / v_track_spacing_;
	// int l_track = 
		// (pin.pin_box.left() - v_track_start_) / v_track_spacing_;
	// left = l_track;
	left = (pin.pin_box.left() - v_track_start_ <= 0) ?
		(0) : ((pin.pin_box.left() - v_track_start_) / v_track_spacing_);
	if(l_offset_precision > 0.5)
		left += 1;

	const int b_offset = 
		(pin.pin_box.bottom() - h_track_start_) % h_track_spacing_; 
	const float b_offset_precision = 
		(float)b_offset / h_track_spacing_;
	// int b_track = 
		// (pin.pin_box.bottom() - h_track_start_) / h_track_spacing_;
	// bottom = b_track;
	bottom = (pin.pin_box.bottom() - h_track_start_ <= 0) ?
		(0) : (pin.pin_box.bottom() - h_track_start_) / h_track_spacing_;
	if(b_offset_precision > 0.5)
		bottom += 1;

	const int r_offset = 
		(pin.pin_box.right() - v_track_start_) % v_track_spacing_; 
	const float r_offset_precision = 
		(float)r_offset / v_track_spacing_;
	// int r_track = 
		// (pin.pin_box.right() - v_track_start_) / v_track_spacing_;
	// right = r_track;
	right = (pin.pin_box.right() - v_track_start_ <= 0) ?
		(0) : (pin.pin_box.right() - v_track_start_ ) / v_track_spacing_;
	if(r_offset_precision > 0.5)
		right += 1;

	const int t_offset = 
		(pin.pin_box.top() - h_track_start_) % h_track_spacing_; 
	const float t_offset_precision = 
		(float)t_offset / h_track_spacing_;
	// int t_track = 
		// (pin.pin_box.top() - h_track_start_) / h_track_spacing_;
	// top = t_track;
	top = (pin.pin_box.top() - h_track_start_ <= 0) ? 
		(0) : (pin.pin_box.top() - h_track_start_) / h_track_spacing_;
	if(t_offset_precision > 0.5)
		top += 1;

	// Error-proofing.
	if(left > right)
		swap(left, right);
	if(bottom > top)
		swap(bottom, top);

	LayerInfo *p_info = NULL;
	for(LayerInfo &info : p_infos_->GetLayerInfos())
		if(info.layer == pin.pin_layer)
			p_info = &info;

	const oaUInt4 width = pin.pin_box.right() - pin.pin_box.left();
	const oaUInt4 height = pin.pin_box.top() - pin.pin_box.bottom();
	const bool is_vert_seg_within_width = (p_info != NULL) ?
		(width <= p_info->width) : (false);
	const bool is_hori_seg_within_width = (p_info != NULL) ?
		(height <= p_info->width) : (false);
	if(is_vert_seg_within_width || is_hori_seg_within_width)
	{
		if(is_vert_seg_within_width)
			right = left;
		if(is_hori_seg_within_width)
			top = bottom;
	}

	// Check via;
	if(pin.pin_fig->getType() == oacCustomViaType)
	{
		const int grid_layer = 
			(layer_map_.find(pin.pin_layer) != layer_map_.end()) ? 
			(layer_map_[pin.pin_layer]) : (-1);
		if(grid_layer != -1 && !kViaEncExpansion_[grid_layer])
		{
			const int mid_x = (left + right) / 2;
			left = mid_x;
			right = mid_x;
			const int mid_y = (bottom + top) / 2;
			bottom = mid_y;
			top = mid_y;
		}
	}


	// Out-off-grid prevention.
	if(left < 0)
		left = 0;
	else if(left >= v_track_num_)
		left = v_track_num_ - 1;
	else
		;

	if(bottom < 0)
		bottom = 0;
	else if(bottom >= h_track_num_)
		bottom = h_track_num_ - 1;
	else
		;

	if(right < 0)
		right = 0;
	else if(right >= v_track_num_)
		right = v_track_num_ - 1;
	else
		;

	if(top < 0)
		top = 0;
	else if(top >= h_track_num_)
		top = h_track_num_ - 1;
	else
		;

	return;
}

PinType GridMapper::CheckPinType(
		const oaFig *p_fig,
		const int &left, 
		const int &bottom, 
		const int &right, 
		const int &top
		)
{
	PinType pin_type = kPinNone;

	const bool lr_equal = (left == right);
	const bool bt_equal = (bottom == top);
	if(lr_equal && bt_equal)
		pin_type = kPinPoint;
	else if((lr_equal && !bt_equal) || (!lr_equal && bt_equal))
		pin_type = kPinLine;
	else if(!lr_equal && !bt_equal)
		pin_type = kPinRectangle;
	else
		;
	return pin_type;
}


// Generate Net Path to Layout.
void GridMapper::GenerateNetPathToLayout(
		vector<TwoPinRipUpNet> &two_pin_rip_up_net
		)
{
	vector<oaPathSeg*> path_segs;
	vector<oaVia*> vias;
	int net_id = 1;
	for(TwoPinRipUpNet &ripup_net : two_pin_rip_up_net)
	{
		if(ripup_net.net_id != net_id)
		{
			// Smooth and prune the path segments and vias.
			MergePathSegments(path_segs);
			PruneRedundantVias(vias);
			path_segs.clear();
			vias.clear();
			net_id = ripup_net.net_id;
		}
		list<Node> &path = ripup_net.GetDetailedGridPath();
		vector< pair<Node*, Node*> > segments;
		GetGridPathSegments(path, segments);
		// If the net has no grid path, then
		// check if the source and the target are overlapped by each other.
		// If not, create a path segment to connect them.
		oaNet *net = ripup_net.p_net;
		if(segments.size() == 0)
		{
			const oaBox &source_box = 
				ripup_net.two_pin_net_connection.source_box;
			const oaBox &target_box =
				ripup_net.two_pin_net_connection.target_box;
			// If source and target are overlapped by each other,
			// then no need to create a segment to connect them.
			if(source_box.overlaps(target_box))
				continue;
			const int grid_layer = 
				ripup_net.two_pin_net_connection.source.GetLayer();
			oaPathSeg *path_seg = 
				CreatePathSeg(source_box, target_box, grid_layer);
			if(path_seg != NULL)
			{
				((oaConnFig*)path_seg)->addToNet(net);
				path_segs.push_back(path_seg);
			}
			continue;
		}
		
		// Create the path segments for the net.
		int seg_count = 0;
		oaPathSeg *first_seg = NULL;
		oaPathSeg *last_seg = NULL;
		const int segments_size = (int)segments.size();
		for(auto &seg : segments)
		{
			seg_count++;
			const Node &from_node = *(seg.first);
			const Node &to_node = *(seg.second);
			const bool is_via = (from_node.z != to_node.z);
			if(!is_via)
			{
				// Generate a segment.
				oaPathSeg *path_seg = 
					CreatePathSeg(from_node, to_node);
				((oaConnFig*)path_seg)->addToNet(net);
				if(seg_count == 1)
					first_seg = path_seg;
				if(seg_count == segments_size)
					last_seg = path_seg;
				path_segs.push_back(path_seg);
			}
			else
			{
				// Generate a via.
				vector<oaVia*> p_vias;
				CreateVia(from_node, to_node, p_vias);
				for(oaVia* p_via : p_vias)
					if(p_via != NULL)
					{
						((oaConnFig*)p_via)->addToNet(net);
						vias.push_back(p_via);
					}
			}
		}

		// Check if the source/target overlaps the first/last path segment.
		// If not, do path segment extension to connect them.
		// Source
		if(first_seg != NULL)
		{
			EntendPathSegment(first_seg, 
					ripup_net.two_pin_net_connection.source_box); 
		}
		// Target
		if(last_seg != NULL)
		{
			EntendPathSegment(last_seg, 
					ripup_net.two_pin_net_connection.target_box); 
		}
	}
	// Smooth and prune the path segments and vias.
	MergePathSegments(path_segs);
	PruneRedundantVias(vias);
	return;
}

oaPathSeg* GridMapper::CreatePathSeg(
		const Node &from_node, 
		const Node &to_node
		)
{
	oaBlock *block = top_block_;
	oaLayerNum layer_num = 
		(oaLayerNum)reverse_layer_map_[from_node.z]; 
	oaPurposeNum purpose_num = oavPurposeNumberDrawing;
	
	oaCoord x1, y1;
	oaCoord x2, y2;
	x1 = v_track_start_ + from_node.x * v_track_spacing_;
	y1 = h_track_start_ + from_node.y * h_track_spacing_; 
	x2 = v_track_start_ + to_node.x * v_track_spacing_;
	y2 = h_track_start_ + to_node.y * h_track_spacing_; 
	oaPoint begin_pt(x1, y1);
	oaPoint end_pt(x2, y2);

	oaEndStyle begin_style(oacTruncateEndStyle);
	oaEndStyle end_style(oacTruncateEndStyle);
	const int info_index = infos_layer_map_[((int)layer_num)];
	oaDist width = (*p_infos_)[info_index].width;
	oaSegStyle style(width, begin_style, end_style);
	oaPathSeg *path_seg;
	path_seg = oaPathSeg::create(
			block, layer_num, purpose_num, begin_pt, end_pt, style);
	return path_seg;
}


// Create a path segment to connect two boxes.
oaPathSeg* GridMapper::CreatePathSeg(
		const oaBox &source_box,
		const oaBox &target_box,
		const int grid_layer
		)
{
	oaPathSeg *path_seg = NULL;

	oaBlock *block = top_block_;
	oaLayerNum layer_num = 
		(oaLayerNum)reverse_layer_map_[grid_layer]; 
	oaPurposeNum purpose_num = oavPurposeNumberDrawing;

	oaPoint source_pt;
	source_box.getCenter(source_pt);
	oaPoint target_pt;
	target_box.getCenter(target_pt);
	oaPoint begin_pt(source_pt.x(), source_pt.y());
	oaPoint end_pt(target_pt.x(), target_pt.y());
	
	// If the two pins are not on the same track, 
	// don't create a segment for them now. 
	if(begin_pt.x() != end_pt.x() && 
			begin_pt.y() != end_pt.y())
		return path_seg;

	oaEndStyle begin_style(oacTruncateEndStyle);
	oaEndStyle end_style(oacTruncateEndStyle);
	const int info_index = infos_layer_map_[((int)layer_num)];
	oaDist width = (*p_infos_)[info_index].width;
	oaSegStyle style(width, begin_style, end_style);
	path_seg = oaPathSeg::create(
			block, layer_num, purpose_num, begin_pt, end_pt, style);
	// cout << "Layer " << layer_num << endl;
	// cout << "Source " << source_pt.x() << " " << source_pt.y() << endl;
	// cout << "Target " << target_pt.x() << " " << target_pt.y() << endl;
	return path_seg;
}

// Only apply to 1-layer difference.
oaVia* GridMapper::CreateVia(
		const Node &from_node, 
		const Node &to_node
		)
{
	oaBlock *block = top_block_;
	oaLayerNum layer1_num = 
		(oaLayerNum)reverse_layer_map_[from_node.z]; 
	oaLayerNum layer2_num = 
		(oaLayerNum)reverse_layer_map_[to_node.z]; 
	
	oaOffset x_offset = v_track_start_ + from_node.x * v_track_spacing_;
	oaOffset y_offset = h_track_start_ + from_node.y * h_track_spacing_;
	oaTransform form(x_offset, y_offset);
	oaViaDef *p_def = NULL;
	const int info_index = infos_layer_map_[(int)layer1_num];
	if(layer1_num > layer2_num)
		p_def = (*p_infos_)[info_index].lower_via_def;
	else
		p_def = (*p_infos_)[info_index].upper_via_def;

	oaVia *p_via = NULL;
	if(p_def != NULL)
	{
		switch(p_def->getType())
		{
			case oacStdViaDefType:
			{
				p_via = (oaVia*)oaStdVia::create(
							block, (oaStdViaDef*)p_def, form);
			}
			break;
			case oacCustomViaDefType:
			{
				p_via = (oaVia*)oaCustomVia::create(
							block, (oaCustomViaDef*)p_def, form);
			}
			break;
			default:
				;
		}
	}
	return p_via;
}

void GridMapper::CreateVia(
		const Node &from_node,
		const Node &to_node,
		vector<oaVia*> &p_vias
		)
{
	int layer_diff = 0;
	layer_diff = (from_node.z > to_node.z) ?
		(from_node.z - to_node.z) : (to_node.z - from_node.z);

	if(layer_diff == 1)
	{
		oaVia *p_via = CreateVia(from_node, to_node);
		if(p_via != NULL)
			p_vias.push_back(p_via);
	}
	else if(layer_diff >= 2)
	{
		oaBlock *block = top_block_;
		oaOffset x_offset = v_track_start_ + from_node.x * v_track_spacing_;
		oaOffset y_offset = h_track_start_ + from_node.y * h_track_spacing_;
		oaTransform form(x_offset, y_offset);
		
		const oaLayerNum from_layer_num = 
			(oaLayerNum)reverse_layer_map_[from_node.z]; 
		const oaLayerNum to_layer_num = 
			(oaLayerNum)reverse_layer_map_[to_node.z]; 
		const oaLayerNum start_layer = (from_layer_num > to_layer_num) ? 
			(to_layer_num) : (from_layer_num);
		const oaLayerNum end_layer = (from_layer_num > to_layer_num) ?
			(from_layer_num) : (to_layer_num);
		for(oaLayerNum layer = start_layer; layer < end_layer; layer+=2)
		{
			const int info_index = infos_layer_map_[(int)layer];
			oaViaDef *p_def = NULL;
			p_def = (*p_infos_)[info_index].upper_via_def;
			if(p_def != NULL)
			{
				oaVia *p_via = NULL;
				switch(p_def->getType())
				{
					case oacStdViaDefType:
					{
						p_via = (oaVia*)oaStdVia::create(
									block, (oaStdViaDef*)p_def, form);
					}
					break;
					case oacCustomViaDefType:
					{
						p_via = (oaVia*)oaCustomVia::create(
									block, (oaCustomViaDef*)p_def, form);
					}
					break;
					default:
						;
				}
				if(p_via != NULL)
					p_vias.push_back(p_via);
			}
		}
	}
	else
	{}
}

void GridMapper::EntendPathSegment(
		oaPathSeg *path_seg,
		oaBox &pin_box
		)
{
	// Check if the path segment touches the fig.
	oaBox path_box;
	path_seg->getBBox(path_box);
	

	// If overlapping, the path segment is not required to be extended.
	if(path_box.overlaps(pin_box))
		return;

	// cout << "Extend !" << endl;
	// Extend the path segment.
	oaPoint begin_pt, end_pt;
	path_seg->getPoints(begin_pt, end_pt);
	const int path_layer = (int)path_seg->getLayerNum();
	const int info_index = infos_layer_map_[path_layer];
	const bool is_vertical = (*p_infos_)[info_index].is_vertical;
	if(is_vertical)
	{
		if(pin_box.bottom() > path_box.top())
		{
			if(begin_pt.y() > end_pt.y())
				begin_pt.y() += h_track_spacing_;
			else
				end_pt.y() += h_track_spacing_;
			
		}
		else if(path_box.bottom() > pin_box.top())
		{
			if(begin_pt.y() > end_pt.y())
				end_pt.y() -= h_track_spacing_;
			else
				begin_pt.y() -= h_track_spacing_;
		}
		else
		{}
	}
	else
	{
		if(pin_box.left() > path_box.right())
		{
			if(begin_pt.x() > end_pt.x())
				begin_pt.x() = begin_pt.x() + v_track_spacing_;
			else
				end_pt.x() = end_pt.x() + v_track_spacing_;
		}
		else if(path_box.left() > pin_box.right())
		{
			if(begin_pt.x() > end_pt.x())
				end_pt.x() = end_pt.x() - v_track_spacing_;
			else
				begin_pt.x() = begin_pt.x() - v_track_spacing_;
		}
		else
		{}
	}
	path_seg->setPoints(begin_pt, end_pt);
	path_seg->getBBox(path_box);
	const bool is_ov2 = (path_box.overlaps(pin_box));
	if(!is_ov2)
		cout << "Not Enough Extension!" << endl;
	return;
}
	
void GridMapper::GetGridPathSegments(
		list<Node> &grid_path,
		vector< pair<Node*, Node*> > &segments
		)
{
	segments.clear();
	if(grid_path.size() == 0)
		return;

	// list<Node> &path = net.GetDetailedGridPath();
	// if(path.size() == 0)
	// {
		// cout << "No grid path." << endl;
		// return;
	// }
	// cout << "Begin = " << *(path.begin()) << endl;
	// cout << "End = " << *(path.rbegin()) << endl;
	
	list<Node>::iterator p_pre_node = grid_path.begin();
	list<Node>::iterator p_second_node = grid_path.begin();
	p_second_node++;
	
	// cout << "Node = " << endl;
	// cout << "\t" << *(p_pre_node) << endl;
	for(list<Node>::iterator p_node = p_second_node;
		p_node != grid_path.end(); p_node++)
	{
		// cout << "\t" << *(p_node) << endl;
		const bool same_x = (p_pre_node->x == p_node->x);
		const bool same_y = (p_pre_node->y == p_node->y);
		const bool same_z = (p_pre_node->z == p_node->z);

		// Via, Horizontal segment and Vertical segment.
		const bool same_xy = (same_x && same_y && !same_z);
		const bool same_yz = (!same_x && same_y && same_z);
		const bool same_xz = (same_x && !same_y && same_z);
		const bool same_segment = (same_xy || same_yz || same_xz);

		list<Node>::iterator p_tmp_node = p_node;
		p_tmp_node++;
		const bool last_node = (p_tmp_node == grid_path.end());
		
		if(!same_segment && !last_node)
		{
			list<Node>::iterator p_from_node = p_pre_node;
			list<Node>::iterator p_to_node = p_node;
			p_to_node--;
			segments.push_back(make_pair(&(*p_from_node), &(*p_to_node)));
			p_pre_node = p_to_node;
		}
		else if(same_segment && last_node)
		{
			list<Node>::iterator p_from_node = p_pre_node;
			list<Node>::iterator p_to_node = p_node;
			segments.push_back(make_pair(&(*p_from_node), &(*p_to_node)));
		}
		else if(!same_segment && last_node)
		{
			list<Node>::iterator p_from_node = p_pre_node;
			list<Node>::iterator p_to_node = p_node;
			p_to_node--;
			segments.push_back(make_pair(&(*p_from_node), &(*p_to_node)));

			// The last segment or via.
			p_from_node = p_to_node;
			p_to_node = p_node;
			segments.push_back(make_pair(&(*p_from_node), &(*p_to_node)));
		}
		else
		{}
	}

	// int count = 0;
	// for(auto &segment : segments)
	// {
		// count++;
		// cout << "Segment " << count << " = " << endl;
		// cout << "\t" << *(segment.first) << endl;
		// cout << "\t" << *(segment.second) << endl;
	// }

	return;
}

void GridMapper::MergePathSegments(
		vector<oaPathSeg*> path_segs
		)
{
	if(path_segs.size() < 2) 
		return;

	// Build the relationship if two segments are overlapped.
	oaBox box1, box2;
	map<int, vector<int>> map_id;
	for(unsigned int idx1 = 0; idx1 < path_segs.size(); idx1++)
	{
		oaPathSeg *seg1 = path_segs[idx1];
		oaLayerNum layer1 = seg1->getLayerNum();
		seg1->getBBox(box1);
		for(unsigned int idx2 = idx1 + 1; idx2 < path_segs.size(); idx2++)
		{
			oaPathSeg *seg2 = path_segs[idx2];
			oaLayerNum layer2 = seg2->getLayerNum();
			seg2->getBBox(box2);
			if((layer1 == layer2) && (box1.overlaps(box2)))
				map_id[idx1].push_back(idx2);
		}
	}

	// Group overlapped segments and Merge them.
	vector<oaPathSeg*> overlapped_segs;
	vector<bool> visited;
	visited.resize(path_segs.size());
	for(unsigned int idx = 0; idx < visited.size(); idx++)
		visited[idx] = false;
	queue<int> id_queue;
	
	id_queue.push(0);
	visited[0] = true;
	overlapped_segs.push_back(path_segs[0]);
	
	while(!id_queue.empty())
	{
		const int id = id_queue.front();
		id_queue.pop();
		// Get Neighbors
		for(int neighbor_id : map_id[id])
		{
			if(!visited[neighbor_id])
			{
				visited[neighbor_id] = true;
				id_queue.push(neighbor_id);
				overlapped_segs.push_back(path_segs[neighbor_id]);
			}
		}

		if(id_queue.empty())
		{
			// Merge
			MergeOverlappedPathSegments(overlapped_segs);
			overlapped_segs.clear();
			for(unsigned int idx = 1; idx < visited.size(); idx++)
			{
				if(!visited[idx])
				{
					visited[idx] = true;
					id_queue.push(idx);
					overlapped_segs.push_back(path_segs[idx]);
					break;
				}
			}
		}
	}
	return;
}

void GridMapper::MergeOverlappedPathSegments(
		vector<oaPathSeg*> overlapped_segs
		)
{
	if(overlapped_segs.size() < 2) 
		return;

	// Init the begin point and end point.
	oaPathSeg *first_seg = overlapped_segs[0];
	oaPoint begin_pt, end_pt;
	first_seg->getPoints(begin_pt, end_pt);
	const bool is_vertical = 
		(begin_pt.x() == end_pt.x());
	int start, end;
	if(is_vertical)
	{
		start = (begin_pt.y() > end_pt.y()) ? 
			(end_pt.y()) : (begin_pt.y());
		end = (begin_pt.y() > end_pt.y()) ? 
			(begin_pt.y()) : (end_pt.y());
	}
	else
	{
		start = (begin_pt.x() > end_pt.x()) ? 
			(end_pt.x()) : (begin_pt.x());
		end = (begin_pt.x() > end_pt.x()) ? 
			(begin_pt.x()) : (end_pt.x());
	}

	// Find the boundary position from the segments.
	if(is_vertical)
	{
		oaPoint pt1, pt2;
		for(unsigned int idx = 1; idx < overlapped_segs.size(); idx++)
		{
			overlapped_segs[idx]->getPoints(pt1, pt2);
			if(pt1.y() > pt2.y())
			{
				if(pt1.y() > end)
					end = pt1.y();
				if(pt2.y() < start)
					start = pt2.y();
			}
			else
			{
				if(pt2.y() > end)
					end = pt2.y();
				if(pt1.y() < start)
					start = pt1.y();
			}
		}
	}
	else
	{
		oaPoint pt1, pt2;
		for(unsigned int idx = 1; idx < overlapped_segs.size(); idx++)
		{
			overlapped_segs[idx]->getPoints(pt1, pt2);
			if(pt1.x() > pt2.x())
			{
				if(pt1.x() > end)
					end = pt1.x();
				if(pt2.x() < start)
					start = pt2.x();
			}
			else
			{
				if(pt2.x() > end)
					end = pt2.x();
				if(pt1.x() < start)
					start = pt1.x();
			}
		}
	}

	// Set the points of the first segment to the boundary position.
	if(is_vertical)
	{
		begin_pt.y() = start;
		end_pt.y() = end;
	}
	else
	{
		begin_pt.x() = start;
		end_pt.x() = end;
	}
	first_seg->setPoints(begin_pt, end_pt);
	
	// Destroy useless path segments.
	for(unsigned int idx = 1; idx < overlapped_segs.size(); idx++)
		overlapped_segs[idx]->destroy();

	return;
}

void GridMapper::PruneRedundantVias(
		vector<oaVia*> vias
		)
{
	if(vias.size() < 2) 
		return;

	oaTransform form1, form2;
	map<int, vector<int>> map_id;
	for(unsigned int idx1 = 0; idx1 < vias.size(); idx1++)
	{
		oaVia *via1 = vias[idx1];
		via1->getTransform(form1);
		const oaLayerNum layer1_num1 = 
			via1->getViaDef()->getLayer1Num();
		for(unsigned int idx2 = idx1 + 1; idx2 < vias.size(); idx2++)
		{
			oaVia *via2 = vias[idx2];
			via2->getTransform(form2);
			const oaLayerNum layer1_num2 = 
				via2->getViaDef()->getLayer1Num();
			const bool same_layer = (layer1_num1 == layer1_num2);
			const bool same_position = (form1 == form2);
			if(same_layer && same_position )
				map_id[idx1].push_back(idx2);
		}
	}

	vector<oaVia*> overlapped_vias;
	vector<bool> visited;
	visited.resize(vias.size());
	for(unsigned int idx = 0; idx < visited.size(); idx++)
		visited[idx] = false;
	queue<int> id_queue;
	
	id_queue.push(0);
	visited[0] = true;
	overlapped_vias.push_back(vias[0]);
	while(!id_queue.empty())
	{
		const int id = id_queue.front();
		id_queue.pop();
		// Get Neighbors
		for(int neighbor_id : map_id[id])
		{
			if(!visited[neighbor_id])
			{
				visited[neighbor_id] = true;
				id_queue.push(neighbor_id);
				overlapped_vias.push_back(vias[neighbor_id]);
			}
		}

		if(id_queue.empty())
		{
			// Prune
			if(overlapped_vias.size() > 1)
			{
				for(unsigned int idx = 1; idx < overlapped_vias.size(); idx++)
					overlapped_vias[idx]->destroy();
			}
			overlapped_vias.clear();
			for(unsigned int idx = 1; idx < visited.size(); idx++)
			{
				if(!visited[idx])
				{
					visited[idx] = true;
					id_queue.push(idx);
					overlapped_vias.push_back(vias[idx]);
					break;
				}
			}
		}
	}
	return;
}

