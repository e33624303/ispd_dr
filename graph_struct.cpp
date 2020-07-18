#include <tuple>
#include "graph_struct.h"

Node::Node(
	const int x_val,
	const int y_val,
	const int8_t z_val,
	float cost_val)
{
	x = x_val;
	y = y_val;
	z = z_val;
	cost = cost_val;
}

Node::Node(const Node &node)
{
	x = node.x;
	y = node.y;
	z = node.z;
	cost = node.cost;
}

Node::Node(const Node &node, const float cost_)
{
	x = node.x;
	y = node.y;
	z = node.z;
	this->cost = cost_;
}

void Node::SetNode(
	const int x_val,
	const int y_val,
	const int8_t z_val)
{
	this->x = x_val;
	this->y = y_val;
	this->z = z_val;
}

void Node::SetNode(const Node &node_val)
{
	this->x = node_val.x;
	this->y = node_val.y;
	this->z = node_val.z;
	this->cost = node_val.cost;
}

bool operator >(const Node &lhs, const Node &rhs)
{
	return lhs.cost > rhs.cost;
}

bool operator < (const Node &lhs, const Node &rhs) {
	return lhs.cost < rhs.cost;	
}

bool operator == (const Node &lhs, const Node &rhs)
{
	return (lhs.x == rhs.x) & (lhs.y == rhs.y) & (lhs.z == rhs.z);
}

ostream &
operator<<(ostream &os, const Node &node)
{
	os << "(";
	os << node.x << " ";
	os << node.y << " ";
	os << node.z << ") ";
	return os;
}


MazeNode::MazeNode(
		bool InPlane,
		bool raise_able,
		bool fall_able
		)
{

	this->RaiseAble = raise_able;
	this->FallAble = fall_able;
	this->InPlane = InPlane;
}

MazeNode::MazeNode(const MazeNode &maze_node)
{
	this->RaiseAble = maze_node.RaiseAble;
	this->FallAble = maze_node.FallAble;
	this->InPlane = maze_node.InPlane;
}


TurnNode::TurnNode(
	const int x_val, 
	const int y_val, 
	const int z_val, 
	const LineEndType line_end_type_val
	)
{
	this->x = x_val;
	this->y = y_val;
	this->z = z_val;
	this->line_end_type = line_end_type_val;
}

TurnNode::TurnNode(
	const Node &node_val, 
	const LineEndType line_end_type_val
	)
{
	this->x = node_val.x;
	this->y = node_val.y;
	this->z = node_val.z;
	this->line_end_type = line_end_type_val;
}

ostream &operator<<(ostream &os, const TurnNode &turn_node)
{
	os << "(";
	os << turn_node.x << " ";
	os << turn_node.y << " ";
	os << turn_node.z << ", ";
	const LineEndType le_type = turn_node.line_end_type;
	if(le_type == kSegLeft)
		os << "L";
	else if(le_type == kSegRight)
		os << "R";
	else if(le_type == kSegBottom)
		os << "B";
	else if(le_type == kSegTop)
		os << "T";
	else
		os << "No";
	os << ")";
	return os;
}

void Pin::PinPoint::SetPinPoint(
		const int x_val, 
		const int y_val, 
		const int z_val
		)
{
	this->x = x_val;
	this->y = y_val;
	this->z = z_val;
}

void Pin::PinLine::SetPinLine(
		const int x_val1, const int y_val1, const int z_val1,
		const int x_val2, const int y_val2, const int z_val2
)
{
	this->x1 = x_val1;
	this->y1 = y_val1;
	this->z1 = z_val1;
	
	this->x2 = x_val2;
	this->y2 = y_val2;
	this->z2 = z_val2;
}

void Pin::PinRectangle::SetPinRectangle(
		const int left_x_val, const int bottom_y_val, const int z_val1,
		const int right_x_val, const int top_y_val, const int z_val2
)
{
	this->left_x = left_x_val;
	this->bottom_y = bottom_y_val;
	this->z1 = z_val1;

	this->right_x = right_x_val;
	this->top_y = top_y_val;
	this->z2 = z_val2;
}

Pin::Pin()
{
	this->pin_type = kPinNone;
	this->pin_position.pin_point.SetPinPoint(0,0,0);
}


Pin::Pin(const Pin &pin)
{
	this->pin_type = pin.pin_type;
	switch(pin.pin_type)
	{
		case kPinPoint:
		{
			const PinPoint &point = pin.pin_position.pin_point;
			this->pin_position.pin_point.SetPinPoint(point.x, point.y, point.z);
		}
		break;

		case kPinLine:
		{
			const PinLine &line = pin.pin_position.pin_line;
			this->pin_position.pin_line.SetPinLine(
					line.x1, line.y1, line.z1,
					line.x2, line.y2, line.z2);
		}
		break;

		case kPinRectangle:
		{
			const PinRectangle &rect = pin.pin_position.pin_rect;
			this->pin_position.pin_rect.SetPinRectangle(
					rect.left_x, rect.bottom_y, rect.z1,
					rect.right_x, rect.top_y, rect.z2);
		}
		break;

		default:
			;
	}
}

/* Pin& */ void Pin::operator=(const Pin &pin)
{
	this->pin_type = pin.pin_type;
	switch(pin.pin_type)
	{
		case kPinPoint:
		{
			const PinPoint &point = pin.pin_position.pin_point;
			this->pin_position.pin_point.SetPinPoint(point.x, point.y, point.z);
		}
		break;

		case kPinLine:
		{
			const PinLine &line = pin.pin_position.pin_line;
			this->pin_position.pin_line.SetPinLine(
					line.x1, line.y1, line.z1,
					line.x2, line.y2, line.z2);
		}
		break;

		case kPinRectangle:
		{
			const PinRectangle &rect = pin.pin_position.pin_rect;
			this->pin_position.pin_rect.SetPinRectangle(
					rect.left_x, rect.bottom_y, rect.z1,
					rect.right_x, rect.top_y, rect.z2);
		}
		break;

		default:
			;
	}
}


void Pin::SetPinPoint(
		const PinType pin_type_val, 
		const int x, const int y, const int z
		)
{
	this->pin_type = pin_type_val;
	this->pin_position.pin_point.SetPinPoint(x, y ,z);
}

void Pin::SetPinLine(
		const PinType pin_type_val,
		const int x1, const int y1, const int z1,
		const int x2, const int y2, const int z2
	)
{
	this->pin_type = pin_type_val;
	this->pin_position.pin_line.SetPinLine(x1, y1, z1, x2, y2, z2);
}

void Pin::SetPinRectangle(
		const PinType pin_type_val,
		const int left_x, const int bottom_y, const int z_val1,
		const int right_x, const int top_y, const int z_val2	
	)
{
	this->pin_type = pin_type_val;
	this->pin_position.pin_rect.SetPinRectangle(
			left_x, bottom_y, z_val1, 
			right_x, top_y, z_val2
			);
}

void Pin::GetPinPointPosition(int &x, int &y, int &z) const
{
	x = this->pin_position.pin_point.x;
	y = this->pin_position.pin_point.y;
	z = this->pin_position.pin_point.z;
}

void Pin::GetPinLinePosition(
		int &x1, int &y1, int &z1, 
		int &x2, int &y2, int &z2
		) const 
{
	x1 = this->pin_position.pin_line.x1;
	y1 = this->pin_position.pin_line.y1;
	z1 = this->pin_position.pin_line.z1;
	x2 = this->pin_position.pin_line.x2;
	y2 = this->pin_position.pin_line.y2;
	z2 = this->pin_position.pin_line.z2;
}

void Pin::GetPinRectanglePosition(
		int &left, int &bottom, int &z1, 
		int &right, int &top, int &z2
		) const
{
	left = this->pin_position.pin_rect.left_x;
	bottom = this->pin_position.pin_rect.bottom_y;
	z1 = this->pin_position.pin_rect.z1;
	right = this->pin_position.pin_rect.right_x;
	top = this->pin_position.pin_rect.top_y;
	z2 = this->pin_position.pin_rect.z2;
}

void Pin::GetPinNodes(vector<Node> &nodes) const
{
	if(this->pin_type == kPinPoint)
	{
		Node tmp_node = 
			Node(pin_position.pin_point.x, 
					pin_position.pin_point.y, 
					pin_position.pin_point.z
					);
		nodes.push_back(tmp_node);
	}
	else if(this->pin_type == kPinLine)
	{
		int x1, y1, z1;
		int x2, y2, z2;
		this->GetPinLinePosition(x1, y1, z1, x2, y2, z2);
		const bool horizontal_line = (y1 == y2);
		if(horizontal_line)
		{
			int y = y1;
			int z = z1;
			for(int x = x1; x <= x2; x++)
			{
				Node tmp_node = Node(x, y, z);
				nodes.push_back(tmp_node);
			}
		}
		else
		{
			int x = x1;
			int z = z1;
			for(int y = y1; y <= y2; y++)
			{
				Node tmp_node = Node(x, y, z);
				nodes.push_back(tmp_node);
			}

		}
	}
	else if(this->pin_type == kPinRectangle)
	{
		int left, bottom, z1;
		int right, top, z2;
		this->GetPinRectanglePosition(left, bottom, z1, right, top, z2);
		int z = z1;
		// horizontal lines
		for(int x = left; x <= right; x++)
		{
			Node tmp_node_bottom = Node(x, bottom, z);
			nodes.push_back(tmp_node_bottom);
			Node tmp_node_top = Node(x, top, z);
			nodes.push_back(tmp_node_top);
		}
		// vertical lines
		for(int y = bottom + 1; y <= top -1; y++)
		{
			Node tmp_node_left = Node(left, y, z);
			nodes.push_back(tmp_node_left);
			Node tmp_node_right = Node(right, y, z);
			nodes.push_back(tmp_node_right);
		}
	}
	else
	{}
}

void Pin::GetWholePinNodes(vector<Node> &nodes) const
{
	if(this->pin_type == kPinPoint)
	{
		Node tmp_node = 
			Node(pin_position.pin_point.x, 
					pin_position.pin_point.y, 
					pin_position.pin_point.z);
		nodes.push_back(tmp_node);
	}
	else if(this->pin_type == kPinLine)
	{
		int x1, y1, z1;
		int x2, y2, z2;
		this->GetPinLinePosition(x1, y1, z1, x2, y2, z2);
		const bool horizontal_line = (y1 == y2);
		if(horizontal_line)
		{
			int y = y1;
			int z = z1;
			for(int x = x1; x <= x2; x++)
			{
				Node tmp_node = Node(x, y, z);
				nodes.push_back(tmp_node);
			}
		}
		else
		{
			int x = x1;
			int z = z1;
			for(int y = y1; y <= y2; y++)
			{
				Node tmp_node = Node(x, y, z);
				nodes.push_back(tmp_node);
			}

		}
	}
	else if(this->pin_type == kPinRectangle)
	{
		int left, bottom, z1;
		int right, top, z2;
		this->GetPinRectanglePosition(left, bottom, z1, right, top, z2);
		int z = z1;
		for(int y = bottom; y <= top; y++)
		{
			for(int x = left; x <= right; x++)
			{
				Node tmp_node = Node(x, y, z);
				nodes.push_back(tmp_node);
			}
		}
	}
	else
	{
		nodes.clear();
	}

	return;
}

void Pin::GetPinLineEndPoints(vector<Node> &line_end_nodes) const
{
	if(this->pin_type == kPinPoint)
	{
		Node tmp_node = 
			Node(pin_position.pin_point.x, 
					pin_position.pin_point.y, 
					pin_position.pin_point.z);
		line_end_nodes.push_back(tmp_node);
	}
	else if(this->pin_type == kPinLine)
	{
		int x1, y1, z1;
		int x2, y2, z2;
		this->GetPinLinePosition(x1, y1, z1, x2, y2, z2);
		const bool horizontal_line = (y1 == y2);
		if(horizontal_line)
		{
			int y = y1;
			int z = z1;
			
			Node tmp_left_node = Node(x1, y, z);
			line_end_nodes.push_back(tmp_left_node);
			Node tmp_right_node = Node(x2, y, z);
			line_end_nodes.push_back(tmp_right_node);

		}
		else
		{
			int x = x1;
			int z = z1;
			
			Node tmp_bottom_node = Node(x, y1, z);
			line_end_nodes.push_back(tmp_bottom_node);
			Node tmp_top_node = Node(x, y2, z);
			line_end_nodes.push_back(tmp_top_node);
		}

	}
	else if(this->pin_type == kPinRectangle)
	{
		line_end_nodes.clear();
	}
	else
	{
		line_end_nodes.clear();
	}
	
	return;
}

void Pin::GetCenterNode(Node &center_node) const
{
	if(this->pin_type == kPinPoint)
	{
		const Pin::PinPoint &pin = pin_position.pin_point;
		center_node.SetNode(pin.x, pin.y, pin.z);
	}
	else if(this->pin_type == kPinLine)
	{
		const Pin::PinLine &pin = pin_position.pin_line;
		center_node.SetNode((pin.x1+pin.x2)/2, (pin.y1+pin.y2)/2, (pin.z1));
	}
	else if(this->pin_type == kPinRectangle)
	{
		const Pin::PinRectangle &pin = pin_position.pin_rect;
		center_node.SetNode(
				(pin.left_x+pin.right_x)/2, 
				(pin.bottom_y+pin.top_y)/2, 
				(pin.z1)
				);
	}
	else
	{
		// do nothing
	}
}

bool Pin::IsNodeWithinPin(const TurnNode &node) const
{
	bool is_node_within_pin = false;
	const PinType pin_type = this->pin_type;
	switch(pin_type)
	{
		case kPinPoint:
		{
			const Pin::PinPoint &pin = pin_position.pin_point;
			is_node_within_pin = (
					(pin.z == node.z) && 
					(pin.x == node.x) && 
					(pin.y == node.y)
					);
		}
		break;

		case kPinLine:
		{
			const Pin::PinLine &pin = pin_position.pin_line;
			is_node_within_pin = (
					(pin.z1 == node.z) &&
					(pin.x1 <= node.x && pin.x2 >= node.x) &&
					(pin.y1 <= node.y && pin.y2 >= node.y)
					);
		}
		break;

		case kPinRectangle:
		{
			const Pin::PinRectangle &pin = pin_position.pin_rect;
			is_node_within_pin = (
					(pin.z1 == node.z) &&
					(pin.left_x <= node.x && pin.right_x >= node.x) &&
					(pin.bottom_y <= node.y && pin.top_y >= node.y)
					);
		}
		break;

		default:
			;
	}

	return is_node_within_pin;
}

bool Pin::IsNodeWithinPin(const Node &node) const
{
	bool is_node_within_pin = false;
	const PinType pin_type = this->pin_type;
	switch(pin_type)
	{
		case kPinPoint:
		{
			const Pin::PinPoint &pin = pin_position.pin_point;
			is_node_within_pin = (
					(pin.z == node.z) && 
					(pin.x == node.x) && 
					(pin.y == node.y)
					);
		}
		break;

		case kPinLine:
		{
			const Pin::PinLine &pin = pin_position.pin_line;
			is_node_within_pin = (
					(pin.z1 == node.z) &&
					(pin.x1 <= node.x && pin.x2 >= node.x) &&
					(pin.y1 <= node.y && pin.y2 >= node.y)
					);
		}
		break;

		case kPinRectangle:
		{
			const Pin::PinRectangle &pin = pin_position.pin_rect;
			is_node_within_pin = (
					(pin.z1 == node.z) &&
					(pin.left_x <= node.x && pin.right_x >= node.x) &&
					(pin.bottom_y <= node.y && pin.top_y >= node.y)
					);
		}
		break;

		default:
			;
	}

	return is_node_within_pin;
}

int Pin::GetLayer() const
{
	int layer = -1;
	if(this->pin_type == kPinPoint)
	{
		const Pin::PinPoint &pin = pin_position.pin_point;
		layer = pin.z;
	}
	else if(this->pin_type == kPinLine)
	{
		const Pin::PinLine &pin = pin_position.pin_line;
		layer = pin.z1;
	}
	else if(this->pin_type == kPinRectangle)
	{
		const Pin::PinRectangle &pin = pin_position.pin_rect;
		layer = pin.z1;
	}
	else
	{
		// do nothing
	}
	return layer;
}

ostream &operator<<(ostream &os, const Pin &pin)
{
	os << "(";
	switch(pin.pin_type)
	{
		case kPinPoint:
			os << "Point, (";
			os << pin.pin_position.pin_point.x << " ";
			os << pin.pin_position.pin_point.y << " ";
			os << pin.pin_position.pin_point.z;
			os << ")";
			break;
		case kPinLine:
			os << "Line, (";
			os << "(" << pin.pin_position.pin_line.x1 << "/" << pin.pin_position.pin_line.x2 << ") ";
			os << "(" << pin.pin_position.pin_line.y1 << "/" << pin.pin_position.pin_line.y2 << ") ";
			os << "(" << pin.pin_position.pin_line.z1 << ")";
			os << ")";
			break;
		case kPinRectangle:
			os << "Rect, (";
			os << "(" << pin.pin_position.pin_rect.left_x << "/" << pin.pin_position.pin_rect.right_x << ") ";
			os << "(" << pin.pin_position.pin_rect.bottom_y << "/" << pin.pin_position.pin_rect.top_y << ") ";
			os << "(" << pin.pin_position.pin_rect.z1;
			os << ")";
			break;
		default:
			os << "N/A";
	}

	os << ")";
	return os;
}

TwoPinNetConnection::TwoPinNetConnection(){
	;
}

TwoPinNetConnection::TwoPinNetConnection(
		TwoPinNetConnection &connection
		)
{
	source_box.set(
			connection.source_box.lowerLeft(), 
			connection.source_box.upperRight());
	target_box.set(
			connection.target_box.lowerLeft(), 
			connection.target_box.upperRight());
	source = connection.source;
	target = connection.target;
}

void TwoPinNetConnection::SetTargetTable(
		const int layout_width, 
		const int layout_height, 
		const int layout_layer
		)
{
	const bool source_is_point = (this->source.pin_type == kPinPoint);
	const bool target_is_point = (this->target.pin_type == kPinPoint);

	// make the table as small as possible for efficiency
	// const bool need_to_swap_node = (source_is_point && !target_is_point);
	// if(need_to_swap_node)
	// {
		// Pin tmp = this->source;
		// this->source = this->target;
		// this->target = tmp;
	// }

	if(source_is_point && target_is_point)
	{
		int source_x, source_y, source_z;
		this->source.GetPinPointPosition(source_x, source_y, source_z);
		// const int source_distance = 
			// abs((layout_width - source_x) + (layout_height - source_y));
		
		int target_x, target_y, target_z;
		this->target.GetPinPointPosition(target_x, target_y, target_z);
		// const int target_distance = 
			// abs((layout_width - target_x) + (layout_height - target_y));
		
		// const bool target_is_closer_to_boundary = 
			// (target_distance < source_distance);
		// if(target_is_closer_to_boundary)
		// {
			// Pin tmp = this->source;
			// this->source = this->target;
			// this->target = tmp;
			
		// }
	}

	Node pin_center;
	target.GetCenterNode(pin_center);
	const ULL index = 
		layout_width * (layout_height * pin_center.z + pin_center.y) 
		+ pin_center.x;
	this->target_table.insert(index);
	/*if(this->target.pin_type == kPinPoint)
	{
		
		int target_x, target_y, target_z;
		this->target.GetPinPointPosition(target_x, target_y, target_z);
		ULL index = 
			layout_width * (layout_height * target_z + target_y) + target_x;
		this->target_table.insert(index);
	}
	else if(this->target.pin_type == kPinLine)
	{
		int target_x1, target_y1, target_z1;
		int target_x2, target_y2, target_z2;
		this->target.GetPinLinePosition(
				target_x1, target_y1, target_z1, 
				target_x2, target_y2, target_z2
				);
		if(target_x1 == target_x2)
		{
			const int x = target_x1;
			const int z = target_z1;
			for(int y = target_y1; y <= target_y2; y++)
			{
				ULL index = layout_width * (layout_height * z + y) + x;
				this->target_table.insert(index);
			}
		}
		else
		{
			const int y = target_y1;
			const int z = target_z1;
			for(int x = target_x1; x <= target_x2; x++)
			{
				ULL index = layout_width * (layout_height * z + y) + x;
				this->target_table.insert(index);
			}
		}
	}
	else if(this->target.pin_type == kPinRectangle)
	{
		int left_x, bottom_y, z1;
		int right_x, top_y, z2;
		this->target.GetPinRectanglePosition(
				left_x, bottom_y, z1, 
				right_x, top_y, z2
				);
		for(int x = left_x; x <= right_x; x++)
		{
			ULL index1 = layout_width * (layout_height * z1 + bottom_y) + x;
			this->target_table.insert(index1);
			ULL index2 = layout_width * (layout_height * z1 + top_y) + x;
			this->target_table.insert(index2);
		}

		for(int y = bottom_y + 1; y <= top_y - 1; y++)
		{
			ULL index1 = layout_width * (layout_height * z1 + y) + right_x;
			this->target_table.insert(index1);
			ULL index2 = layout_width * (layout_height * z1 + y) + left_x;
			this->target_table.insert(index2);
			
		}
	}
	else
	{
		// do nothing
	}*/
}

void TwoPinNetConnection::EraseTargetKey(const ULL target_key)
{
	/*cout << "key num. " << target_key << endl; 
	cout << "a. " << target_table.size() << endl;
	for(auto key : target_table)
		cout << key << " ";
	cout << endl;*/
	target_table.erase(target_key);
	/*cout << "b. " << target_table.size() << endl;
	for(auto key : target_table)
		cout << key << " ";
	cout << endl;*/
}

/*
void TwoPinNetConnection::SetTargetTable(
		const int layout_width, 
		const int layout_height, 
		const int layout_layer
		)
{
	const bool source_is_point = (this->source.pin_type == kPinPoint);
	const bool target_is_point = (this->target.pin_type == kPinPoint);

	// make the table as small as possible for efficiency
	const bool need_to_swap_node = (source_is_point && !target_is_point);
	if(need_to_swap_node)
	{
		Pin tmp = this->source;
		this->source = this->target;
		this->target = tmp;
	}

	if(source_is_point && target_is_point)
	{
		int source_x, source_y, source_z;
		this->source.GetPinPointPosition(source_x, source_y, source_z);
		const int source_distance = 
			abs((layout_width - source_x) + (layout_height - source_y));
		
		int target_x, target_y, target_z;
		this->target.GetPinPointPosition(target_x, target_y, target_z);
		const int target_distance = 
			abs((layout_width - target_x) + (layout_height - target_y));
		
		const bool target_is_closer_to_boundary = 
			(target_distance < source_distance);
		if(target_is_closer_to_boundary)
		{
			Pin tmp = this->source;
			this->source = this->target;
			this->target = tmp;
		}
	}

	if(this->target.pin_type == kPinPoint)
	{
		
		int target_x, target_y, target_z;
		this->target.GetPinPointPosition(target_x, target_y, target_z);
		const ULL index = 
			layout_width * (layout_height * target_z + target_y) + target_x;
		this->target_table.insert(index);
	}
	else if(this->target.pin_type == kPinLine)
	{
		int target_x1, target_y1, target_z1;
		int target_x2, target_y2, target_z2;
		this->target.GetPinLinePosition(
				target_x1, target_y1, target_z1, 
				target_x2, target_y2, target_z2
				);
		const int mid_x = (target_x1 + target_x2) / 2;
		const int mid_y = (target_y1 + target_y2) / 2;
		const int mid_z = (target_z1 + target_z2) / 2;
		const ULL index = 
			layout_width * (layout_height * mid_z + mid_y) + mid_x;
		this->target_table.insert(index);
	}
	else if(this->target.pin_type == kPinRectangle)
	{
		int left_x, bottom_y, z1;
		int right_x, top_y, z2;
		this->target.GetPinRectanglePosition(
				left_x, bottom_y, z1, 
				right_x, top_y, z2
				);
		const int mid_x = (left_x + right_x) / 2;
		const int mid_y = (bottom_y + top_y) / 2;
		const int mid_z = (z1 + z2) / 2;
		const ULL index = 
			layout_width * (layout_height * mid_z + mid_y) + mid_x;
		this->target_table.insert(index);

	}
	else
	{
		// do nothing
	}
}*/


int TwoPinNetConnection::GetEstimatedWireLength() const
{
	int estimated_wire_length = 0;
	
	// compute the distance from the source to target
	// TODO...
	int source_x = -1, source_y = -1, source_z = -1;
	int target_x = -1, target_y = -1, target_z = -1;

	// get the center of the source
	PinType source_pin_type = this->source.pin_type;
	if(source_pin_type == kPinPoint)
	{
		this->source.GetPinPointPosition(source_x, source_y, source_z);
	}
	else if(source_pin_type == kPinLine)
	{
		int x1, y1, z1;
		int x2, y2, z2;
		this->source.GetPinLinePosition(x1, y1, z1, x2, y2, z2);
		
		float center_x = fabs((float)(x2 - x1) / 2);
		source_x = (int) ceil(center_x);
		float center_y = fabs((float)(y2 - y1) / 2);
		source_y = (int) ceil(center_y);
		source_z = z1;
	}
	else if(source_pin_type == kPinRectangle)
	{
		int left, bottom, z1;
		int right, top, z2;
		this->source.GetPinRectanglePosition(left, bottom, z1, right, top, z2);
		
		float center_x = fabs((float)(right - left) / 2);
		source_x = (int) ceil(center_x);
		float center_y = fabs((float)(top - bottom) / 2);
		source_y = (int) ceil(center_y);
		source_z = z1;
	}
	else
	{
	}

	// get the center of the target
	PinType target_pin_type = this->source.pin_type;
	if(target_pin_type == kPinPoint)
	{
		this->target.GetPinPointPosition(target_x, target_y, target_z);
	}
	else if(target_pin_type == kPinLine)
	{
		int x1, y1, z1;
		int x2, y2, z2;
		this->target.GetPinLinePosition(x1, y1, z1, x2, y2, z2);
		
		float center_x = fabs((float)(x2 - x1) / 2);
		target_x = (int) ceil(center_x);
		float center_y = fabs((float)(y2 - y1) / 2);
		target_y = (int) ceil(center_y);
		target_z = z1;
	}
	else if(target_pin_type == kPinRectangle)
	{
		int left, bottom, z1;
		int right, top, z2;
		this->target.GetPinRectanglePosition(left, bottom, z1, right, top, z2);
		
		float center_x = fabs((float)(right - left) / 2);
		target_x = (int) ceil(center_x);
		float center_y = fabs((float)(top - bottom) / 2);
		target_y = (int) ceil(center_y);
		target_z = z1;
	}
	else
	{
	}

	int dist_x = abs(target_x - source_x);
	int dist_y = abs(target_y - source_y);
	int dist_z = abs(target_z - source_z);
	estimated_wire_length = dist_x + dist_y + dist_z;

	return estimated_wire_length;
}

bool TwoPinNetConnection::DoesSourceOverlapTarget() const
{
	bool pins_are_overlapped = true;
	if(source.GetLayer() != target.GetLayer())
	{
		pins_are_overlapped = false;
	}
	else
	{
		const PinType &source_type = source.pin_type;
		//oaBox source_box;
		Box source_box;
		if(source_type == kPinPoint)
		{
			int x, y, z;
			source.GetPinPointPosition(x, y, z);
			source_box.set(x, y, x, y);
		}
		else if(source_type == kPinLine)
		{
			int x1, y1, z1;
			int x2, y2, z2;
			this->source.GetPinLinePosition(x1, y1, z1, x2, y2, z2);
			source_box.set(x1, y1, x2, y2);
		}
		else if(source_type == kPinRectangle)
		{
			int left, bottom, z1;
			int right, top, z2;
			this->source.GetPinRectanglePosition(left, bottom, z1, right, top, z2);
			source_box.set(left, bottom, right, top);
		}
		else
		{}

		const PinType &target_type = target.pin_type;
		//oaBox target_box;
		Box target_box;
		if(target_type == kPinPoint)
		{
			int x, y, z;
			target.GetPinPointPosition(x, y, z);
			target_box.set(x, y, x, y);
		}
		else if(target_type == kPinLine)
		{
			int x1, y1, z1;
			int x2, y2, z2;
			this->target.GetPinLinePosition(x1, y1, z1, x2, y2, z2);
			target_box.set(x1, y1, x2, y2);
		}
		else if(target_type == kPinRectangle)
		{
			int left, bottom, z1;
			int right, top, z2;
			this->target.GetPinRectanglePosition(left, bottom, z1, right, top, z2);
			target_box.set(left, bottom, right, top);
		}
		else
		{}

		pins_are_overlapped = source_box.overlaps(target_box) ? true : false;
	
	}

	return pins_are_overlapped;
}

bool TwoPinNetConnection::IsSourceEqualToTarget() const
{
	bool only_one_pin = false;
	
	if(source.GetLayer() != target.GetLayer())
	{
		only_one_pin = false;
	}
	else
	{
		const PinType &source_type = source.pin_type;
		//oaBox source_box;
		Box source_box;
		if(source_type == kPinPoint)
		{
			int x, y, z;
			source.GetPinPointPosition(x, y, z);
			source_box.set(x, y, x, y);
		}
		else if(source_type == kPinLine)
		{
			int x1, y1, z1;
			int x2, y2, z2;
			this->source.GetPinLinePosition(x1, y1, z1, x2, y2, z2);
			source_box.set(x1, y1, x2, y2);
		}
		else if(source_type == kPinRectangle)
		{
			int left, bottom, z1;
			int right, top, z2;
			this->source.GetPinRectanglePosition(left, bottom, z1, right, top, z2);
			source_box.set(left, bottom, right, top);
		}
		else
		{}

		const PinType &target_type = target.pin_type;
		//oaBox target_box;
		Box target_box;
		if(target_type == kPinPoint)
		{
			int x, y, z;
			target.GetPinPointPosition(x, y, z);
			target_box.set(x, y, x, y);
		}
		else if(target_type == kPinLine)
		{
			int x1, y1, z1;
			int x2, y2, z2;
			this->target.GetPinLinePosition(x1, y1, z1, x2, y2, z2);
			target_box.set(x1, y1, x2, y2);
		}
		else if(target_type == kPinRectangle)
		{
			int left, bottom, z1;
			int right, top, z2;
			this->target.GetPinRectanglePosition(left, bottom, z1, right, top, z2);
			target_box.set(left, bottom, right, top);
		}
		else
		{}
		only_one_pin = (source_box == target_box);
	
	}
	return only_one_pin;
}

bool TwoPinNetConnection::IsStraight() const
{
	bool is_straight = false;
	Node source_node, target_node;
	source.GetCenterNode(source_node);
	target.GetCenterNode(target_node);
	if(source_node.x == target_node.x || 
			source_node.y == target_node.y)
		is_straight = true;

	return is_straight;
}

float TwoPinNetConnection::GetStraightRatio() const
{
	float straight_ratio = 0.0;
	Node source_node, target_node;
	source.GetCenterNode(source_node);
	target.GetCenterNode(target_node);
	
	const int diff_x = (source_node.x > target_node.x) ?
		(source_node.x - target_node.x) : (target_node.x - source_node.x);
	const int diff_y = (source_node.y > target_node.y) ?
		(source_node.y - target_node.y) : (target_node.y - source_node.y);

	if(source_node.x == target_node.x || 
		source_node.y == target_node.y)
	{
		straight_ratio = FLT_MAX;
	}
	else
	{
		straight_ratio = (diff_x > diff_y) ? 
			( ((float)diff_x) / diff_y ) : ( ((float)diff_y) / diff_x );
	}
	
	return straight_ratio;
}

void TwoPinNetConnection::PrintTargetTable()
{
	for(auto key : target_table)
		cout << key << " ";
	cout << endl;
}

TwoPinRipUpNet::TwoPinRipUpNet() :
p_net(NULL),
net_id(-1),
two_pin_net_id(-1),
rip_up_times(0),
violation_num(0),
wire_length(0),
path_is_found(false),
both_pins_are_overlapped(false),
has_only_one_pin(false),
src_need_refine(false),
tar_need_refine(false)
{}

TwoPinRipUpNet::TwoPinRipUpNet(const TwoPinRipUpNet &net)
{
	this->p_net = net.p_net;
	this->net_id = net.net_id;
	this->two_pin_net_id = net.two_pin_net_id;
	this->rip_up_times = 0;
	this->violation_num = 0;
	this->wire_length = 0;
	this->two_pin_net_connection = net.two_pin_net_connection;
	this->Global_guide = net.Global_guide;
	this->both_pins_are_overlapped = false;
	this->has_only_one_pin = false;
	this->src_need_refine = net.src_need_refine;
	this->tar_need_refine = net.tar_need_refine;
	this->DesignPinList = net.DesignPinList;
}

bool TwoPinRipUpNet::operator < ( const TwoPinRipUpNet &a ) const
{
	if(this->net_id < a.net_id)
		return true;
	else
		return false;
}

void TwoPinRipUpNet::SetPinsToConnection()
{
	bool debug_mode = false;
	bool stop = false;
	bool CacheFirstTime = false;
	bool CacheUsing = false;
	
	map<tuple<int, int, int>, bool> SrcPinTable;
	map<tuple<int, int, int>, bool> TarPinTable;

	if (this->netname.compare("net1236dddp") == 0)
	{
		debug_mode = true;
		printf("DEBUG Print::%s\n", this->netname.c_str());
	}
	
	PinToBitwise.clear();

	if(this->cache_pin_path_list->size() != 0)
	{
		if (debug_mode)
			printf("Cache clear\n");
		CacheUsing = true;
	}
	//
	if (this->temp_target_path_list->empty() && !PreNetPath.empty())
	{
		for (auto &newpin : PreNetPath)
		{
			PinToBitwise.push_back(newpin);
		}
	}
	else 
	{
		if (!PreNetPath.empty())
			PreNetPath.clear();

		for (auto &newpin : *(this->temp_target_path_list))
		{
			//cout << "Add pin to bitwise\n" ;
			PinToBitwise.push_back(newpin);
			PreNetPath.push_back(newpin);
		}
	}

	//
	this->temp_target_path_list->clear();
	this->tempTargetTable->clear();
	// add origin
	this->two_pin_net_connection.Source_pin_path_list.clear();
	this->two_pin_net_connection.Target_pin_path_list.clear();
	for (auto &srcnode : this->two_pin_net_connection.source_pin_list)
	{
		// only use pin point
		int x, y, z;
		srcnode.GetPinPointPosition(x, y, z);
		Pin p;
		p.SetPinPoint(kPinPoint, x, y, z);
		auto FindPin = SrcPinTable.find(make_tuple(x, y, z));
		if (FindPin == SrcPinTable.end())
		{
			SrcPinTable.emplace(make_tuple(x, y, z), true);
			this->two_pin_net_connection.Source_pin_path_list.push_back(p);
			if (debug_mode)
				printf("source_pin_list::(%d,%d,%d)\n", x, y, z);
		}
		// save the source, it is the target of the next rnd
		auto FindTempPin = this->tempTargetTable->find(make_tuple(x, y, z));
		if (FindTempPin == this->tempTargetTable->end())
		{
			this->tempTargetTable->emplace(make_tuple(x, y, z), true);
			this->temp_target_path_list->push_back(p);
			if (debug_mode)
			{
				printf("TEMP::source_pin_list::(%d,%d,%d)\n", x, y, z);
				stop = true;
			}
		}

		// if sub2sub == true && sub is pin == true
		// add cache
		if ((*this->TwoPinNetData).subtree2subtree == true && (*this->TwoPinNetData).subtree_is_pin == true)
		{
			auto FindCachePin = this->cachePinTable->find(make_tuple(x, y, z));
			if (FindCachePin == this->cachePinTable->end())
			{
				if(this->cache_pin_path_list->size() == 0)
				{
					CacheFirstTime = true;
					CacheUsing = true;
				}

				this->cachePinTable->emplace(make_tuple(x, y, z), true);
				this->cache_pin_path_list->push_back(p);
				if (debug_mode)
				{
					printf("Cache::source_pin_list::(%d,%d,%d)\n", x, y, z);
					stop = true ;
				}
			}
		}
		
	}

	for (auto &tarnode : this->two_pin_net_connection.target_pin_list)
	{
		// only use pin point
		int x, y, z;
		tarnode.GetPinPointPosition(x, y, z);
		Pin p;
		p.SetPinPoint(kPinPoint, x, y, z);
		auto FindPin = TarPinTable.find(make_tuple(x, y, z));
		if (FindPin == TarPinTable.end())
		{
			TarPinTable.emplace(make_tuple(x, y, z), true);
			this->two_pin_net_connection.Target_pin_path_list.push_back(p);
			//PinToBitwise.push_back(p);
			if (debug_mode)
				printf("target_pin_list::(%d,%d,%d)\n", x, y, z);
		}
	}

	// add PreRoutedPath : source
	if( this->TwoPinNetData == NULL ) 
	{
		cout << "ERROR TwoPinNetData is NULL\n";
		exit(-1);
	}

	// remove this if update version
	//printf("first = %d, second = %d\n", (*this->TwoPinNetData).TwoPinIndex.first, (*this->TwoPinNetData).TwoPinIndex.second);
	if((*this->TwoPinNetData).TwoPinIndex.first != UndefineValue)
	{
		int PinIndex = (*this->TwoPinNetData).TwoPinIndex.first;

		if(this->DesignPinList == NULL)
		{
			cout << "ERROR DesignPinList is NULL\n";
			exit(-1);
		}

		Parser::IPSD_Routing_PIN &tempPin = (*this->DesignPinList).at(PinIndex);
		for( auto &tempIRECT : tempPin.IRect_list)
		{
			this->two_pin_net_connection.Src_Pin_Shape.push_back(tempIRECT);
		}
	}
	if ((*this->TwoPinNetData).TwoPinIndex.second != UndefineValue)
	{
		int PinIndex = (*this->TwoPinNetData).TwoPinIndex.second;
		Parser::IPSD_Routing_PIN &tempPin = (*this->DesignPinList).at(PinIndex);
		for (auto &tempIRECT : tempPin.IRect_list)
		{
			this->two_pin_net_connection.Tar_Pin_Shape.push_back(tempIRECT);
		}
	}
	//===============

	if ((*this->TwoPinNetData).subtree2subtree == true && (*this->TwoPinNetData).subtree_is_pin == false)
	{
		if (debug_mode)
			printf("CASE::SUB2SUB\n"); 
		for (int index = 0; index < (int)(*this->TwoPinNetData).SourceSubtreeNet.size(); index++)
		{
			// add to cache



			Parser::ISPD_2PinNet &temp2PinNet = (*this->TwoPinNetList).at((*this->TwoPinNetData).SourceSubtreeNet.at(index));

			//cout << "SRCNODES : " << temp2PinNet.SrcNodes.size() << " index = " << (*this->TwoPinNetData).SourceSubtreeNet.at(index) << endl;
			for (auto &srcnode : temp2PinNet.SrcNodes)
			{
				// only use pin point
				Pin p;
				p.SetPinPoint(kPinPoint, srcnode.x, srcnode.y, srcnode.z);
				auto FindPin = SrcPinTable.find(make_tuple(srcnode.x, srcnode.y, srcnode.z));
				if (FindPin == SrcPinTable.end())
				{
					SrcPinTable.emplace(make_tuple(srcnode.x, srcnode.y, srcnode.z), true);
					this->two_pin_net_connection.Source_pin_path_list.push_back(p);
					if (debug_mode)
						printf("Subtree SRC Pin::(%d,%d,%d)\n", srcnode.x, srcnode.y, srcnode.z);
				}

				// save the source, it is the target of the next rnd
				auto FindTempPin = this->tempTargetTable->find(make_tuple(srcnode.x, srcnode.y, srcnode.z));
				if (FindTempPin == this->tempTargetTable->end())
				{
					this->tempTargetTable->emplace(make_tuple(srcnode.x, srcnode.y, srcnode.z), true);
					this->temp_target_path_list->push_back(p);
					if (debug_mode)
					{
						printf("TEMP::source::(%d,%d,%d)\n", srcnode.x, srcnode.y, srcnode.z);
						stop = true;
					}
				}

				auto FindCachePin = this->cachePinTable->find(make_tuple(srcnode.x, srcnode.y, srcnode.z));
				if (FindCachePin == this->cachePinTable->end())
				{
					if (this->cache_pin_path_list->size() == 0)
					{
						CacheUsing = true;
						CacheFirstTime = true;
					}

					this->cachePinTable->emplace(make_tuple(srcnode.x, srcnode.y, srcnode.z), true);
					this->cache_pin_path_list->push_back(p);
					if (debug_mode)
					{
						printf("Cache::Subtree SRC Pin::(%d,%d,%d)\n", srcnode.x, srcnode.y, srcnode.z);
						stop = true;
					}
				}
			} // add its src tar node, if need

			for (int index2 = 0; index2 < (int)temp2PinNet.RipUp_path.size(); index2++)
			{
				int _x = temp2PinNet.RipUp_path.at(index2).x;
				int _y = temp2PinNet.RipUp_path.at(index2).y;
				int _z = temp2PinNet.RipUp_path.at(index2).z;
				Pin p;
				p.SetPinPoint(kPinPoint, _x, _y, _z);
				auto FindPin = SrcPinTable.find(make_tuple(_x, _y, _z));
				if (FindPin == SrcPinTable.end())
				{
					SrcPinTable.emplace(make_tuple(_x, _y, _z), true);
					this->two_pin_net_connection.Source_pin_path_list.push_back(p);
					if (debug_mode)
						printf("Subtree RipUpPath Pin::(%d,%d,%d)\n", _x, _y, _z);
				}

				// save the source, it is the target of the next rnd
				auto FindTempPin = this->tempTargetTable->find(make_tuple(_x, _y, _z));
				if (FindTempPin == this->tempTargetTable->end())
				{
					this->tempTargetTable->emplace(make_tuple(_x, _y, _z), true);
					this->temp_target_path_list->push_back(p);
					if (debug_mode)
					{
						printf("TEMP::Subtree RipUpPath Pin::(%d,%d,%d)\n", _x, _y, _z);
						stop = true;
					}
				}

				auto FindCachePin = this->cachePinTable->find(make_tuple(_x, _y, _z));
				if (FindCachePin == this->cachePinTable->end())
				{
					if (this->cache_pin_path_list->size() == 0)
					{
						CacheFirstTime = true;
						CacheUsing = true;
					}

					this->cachePinTable->emplace(make_tuple(_x, _y, _z), true);
					this->cache_pin_path_list->push_back(p);
					if (debug_mode)
					{
						printf("Cache::Subtree RipUpPath Pin::(%d,%d,%d)\n", _x, _y, _z);
						stop = true;
					}
				}
			}
		}
	}
	else if ((*this->TwoPinNetData).subtree2subtree == true && (*this->TwoPinNetData).subtree_is_pin == true)
	{
		if (debug_mode)
			printf("CASE::SUB2PIN\n");
		// sub2pin
		//this->two_pin_net_connection.target_pin_list.clear();
		// already done at MazeRouteKernel.cpp -> NetConstruction
	}
	else if ((*this->TwoPinNetData).subtree2subtree == false && (*this->TwoPinNetData).subtree_is_pin == false)
	{
		if (debug_mode)
			printf("PIN2PIN\n");
		// pin2pin
	}
/*
	if (!CacheFirstTime && CacheUsing)
	{
		
		//this->two_pin_net_connection.Target_pin_path_list = *this->cache_pin_path_list;
	}
	else
	{
		// clear the vector of TempPin
		if(CacheFirstTime)
			this->PinToBitwise.clear();

		// target == PreRoutedNet
		for(int index = 0 ; index < (int)(*this->TwoPinNetData).PreRoutedNet.size() ; index++)
		{
			Parser::ISPD_2PinNet &temp2PinNet = (*this->TwoPinNetList).at((*this->TwoPinNetData).PreRoutedNet.at(index));
			for (auto &tarnode : temp2PinNet.SrcNodes)
			{
				// only use pin point
				Pin p;
				p.SetPinPoint(kPinPoint, tarnode.x, tarnode.y, tarnode.z);
				auto FindPin = TarPinTable.find(make_tuple(tarnode.x, tarnode.y, tarnode.z));
				if (FindPin == TarPinTable.end())
				{
					TarPinTable.emplace(make_tuple(tarnode.x, tarnode.y, tarnode.z), true);
					this->two_pin_net_connection.Target_pin_path_list.push_back(p);
					if (debug_mode)
					{
						printf("Target::PreRoutedNet Source Pin::(%d,%d,%d)\n", tarnode.x, tarnode.y, tarnode.z);
						stop = true;
					}
				}

				// save the source, it is the target of the next rnd
				PinToBitwise.push_back(p);

				auto FindCachePin = this->cachePinTable->find(make_tuple(tarnode.x, tarnode.y, tarnode.z));
				if (FindCachePin == this->cachePinTable->end())
				{

					this->cachePinTable->emplace(make_tuple(tarnode.x, tarnode.y, tarnode.z), true);
					this->cache_pin_path_list->push_back(p);
					if (debug_mode)
					{
						printf("Cache::PreRoutedNet Source Pin::(%d,%d,%d)\n", tarnode.x, tarnode.y, tarnode.z);
						stop = true;
					}
				}

			} // add its src tar node, if need
	
			for (auto &tarnode : temp2PinNet.TarNodes)
			{
				// only use pin point
				Pin p;
				p.SetPinPoint(kPinPoint, tarnode.x, tarnode.y, tarnode.z);
				auto FindPin = TarPinTable.find(make_tuple(tarnode.x, tarnode.y, tarnode.z));
				if (FindPin == TarPinTable.end())
				{
					TarPinTable.emplace(make_tuple(tarnode.x, tarnode.y, tarnode.z), true);
					this->two_pin_net_connection.Target_pin_path_list.push_back(p);
					if (debug_mode)
					{
						printf("Target::PreRoutedNet Target Pin::(%d,%d,%d)\n", tarnode.x, tarnode.y, tarnode.z);
						stop = true;
					}
				}

				// save the source, it is the target of the next rnd
				PinToBitwise.push_back(p);

				auto FindCachePin = this->cachePinTable->find(make_tuple(tarnode.x, tarnode.y, tarnode.z));
				if (FindCachePin == this->cachePinTable->end())
				{

					this->cachePinTable->emplace(make_tuple(tarnode.x, tarnode.y, tarnode.z), true);
					this->cache_pin_path_list->push_back(p);
					if (debug_mode)
					{
						printf("Cache::PreRoutedNet Target Pin::(%d,%d,%d)\n", tarnode.x, tarnode.y, tarnode.z);
						stop = true;
					}
				}
			} // add its src tar node, if need
	
			for(int index2 = 0 ; index2 < (int)temp2PinNet.RipUp_path.size() ; index2++)
			{
				int _x = temp2PinNet.RipUp_path.at(index2).x;
				int _y = temp2PinNet.RipUp_path.at(index2).y;
				int _z = temp2PinNet.RipUp_path.at(index2).z;
				Pin p;
				p.SetPinPoint(kPinPoint, _x, _y, _z);
				auto FindPin = TarPinTable.find(make_tuple(_x, _y, _z));
				if (FindPin == TarPinTable.end())
				{
					TarPinTable.emplace(make_tuple(_x, _y, _z), true);
					this->two_pin_net_connection.Target_pin_path_list.push_back(p);
					if (debug_mode)
					{
						printf("Target::PreRoutedNet RipUp Pin::(%d,%d,%d)\n", _x, _y, _z);
						stop = true;
					}
				}

				// save the source, it is the target of the next rnd
				PinToBitwise.push_back(p);

				auto FindCachePin = this->cachePinTable->find(make_tuple(_x, _y, _z));
				if (FindCachePin == this->cachePinTable->end())
				{

					this->cachePinTable->emplace(make_tuple(_x, _y, _z), true);
					this->cache_pin_path_list->push_back(p);
					if (debug_mode)
					{
						printf("Cache::PreRoutedNet RipUp Pin::(%d,%d,%d)\n", _x, _y, _z);
						stop = true;
					}
				}
			}
		}
	} // else
*/
	if (debug_mode) // debug_mode
	{
		printf("pause...\n");
		int readnum;
		//cin>>readnum;
	}
	/*
	void ISPD_set_Target_array_();
	void ISPD_reset_Target_array_();
	void ISPD_Insert_Target_array_(vector <Pin> &element);
	*/
	

	//printf("#Src(%d) #Tar(%d)\n", this->two_pin_net_connection.Source_pin_path_list.size(), this->two_pin_net_connection.Target_pin_path_list.size());
	bool exits = false;
	if (PinToBitwise.empty())
	{
		//cout << "ERROR TAR NO PINS (BITWISE)\n";
		//exits = true;
	}

	if (this->two_pin_net_connection.Source_pin_path_list.size() == 0)
	{
		//cout << "ERROR SRC NO PINS\n";
		exits = true;
	}
	/*
	if (this->two_pin_net_connection.Target_pin_path_list.size() == 0)
	{
		cout << "ERROR TAR NO PINS\n";
		exits = true;
	}
	
	if(exits)
		exit(-1);
		*/
}