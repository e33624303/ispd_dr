#ifndef GRAGH_STRUCT_H
#define GRAGH_STRUCT_H
#include <cfloat>
#include <iostream>
#include <algorithm>
#include <vector>
#include <list>
#include <map>
#include <tuple>
#include <cmath>
#include <unordered_set>
#include "Definition.h"
#include "lef_def_parser/Structure.h"
using namespace std;


typedef pair< coor_t, coor_t > index_pair;

// This Structure use for maze routing node with cost
struct Node {
	int x;
	int y;
	int8_t z;
	float cost;
	Node(
		const int x_val = -1,
		const int y_val = -1,
		const int8_t z_val = -1,
		const float cost = FLT_MAX);
	Node(const Node &node,const float cost_);
	Node(const Node &node);
	void SetNode(
		const int x_val,
		const int y_val,
		const int8_t z_val);
	void SetNode(const Node &node_val);

	friend bool operator>(const Node &lhs, const Node &rhs);
	friend bool operator < (const Node &lhs , const Node&rhs);
	friend bool operator == (const Node &lhs, const Node &rhs);
	friend ostream &operator<<(ostream &os, const Node &node);
};

struct Point{
public:
	coor_t X, Y;
	coor_t x(){ return X; }
	coor_t y(){ return Y; }
};

class ViaBlock{
public:
	int layer;
	int via_type_id;
	int net_id;
	ViaBlock(){
		layer = -1;
		via_type_id = -1;
		net_id = -1;
	}
};

// This structure use for note EVERY Grid Node ,so take care the memory usage
struct MazeNode {
	public:
	bool RaiseAble; // the node can go raising
	bool FallAble; // the node can go falling
	bool InPlane; // the node in this layer
	MazeNode()
	{
		this->RaiseAble = false;
		this->FallAble = false;
		this->InPlane = false;
	}
	MazeNode(
		bool InPlane,
		bool raise_able,
		bool fall_able
	);
	MazeNode(const MazeNode &maze_node);
};




enum LineEndType {
	kSegNone,
	kSegLeft,
	kSegRight,
	kSegTop,
	kSegBottom
};


// The position of the segment
struct TurnNode {
	int x;
	int y;
	int z;
	LineEndType line_end_type;

	TurnNode(
		const int x_val = -1,
		const int y_val = -1,
		const int z_val = -1,
		const LineEndType line_end_type_val = kSegNone
		);
	TurnNode(
		const Node &node_val,
		const LineEndType line_end_type_val = kSegNone
		);
	friend ostream &operator<<(ostream &os, const TurnNode &turn_node);
};


// The routing region of a net
struct BoundingBox {
	int left;
	int right;
	int top;
	int bottom;
	int top_layer;
	int bottom_layer;
};


// 2016-02-16
// The information of a pin. 
enum PinType {
	kPinPoint,
	kPinLine,
	kPinRectangle,
	kPinNone
};

// a greater functor for MazeNode
struct GreaterMazeNode
{
	bool operator () (const Node &lhs, const Node &rhs) const
	{
		return  lhs.cost > rhs.cost;
	}
};

struct Pin {

	struct PinPoint
	{
		// one point
		int x;
		int y;
		int z;

		void SetPinPoint(
			const int x_val,
			const int y_val,
			const int z_val
			);
	};

	struct PinLine
	{
		// two ends of a line
		int x1, y1, z1;
		int x2, y2, z2;
		void SetPinLine(
			const int x_val1, const int y_val1, const int z_val1,
			const int x_val2, const int y_val2, const int z_val2);
	};

	struct PinRectangle
	{
		// two corners of a rectangle
		int left_x, bottom_y, z1;
		int right_x, top_y, z2;

		void SetPinRectangle(
			const int left_x_val, const int bottom_y_val, const int z_val1,
			const int right_x_val, const int top_y_val, const int z_val2
			);
	};

	union PinPosition
	{
		PinPoint pin_point;
		PinLine pin_line;
		PinRectangle pin_rect;
	};

	PinType pin_type;
	PinPosition pin_position;

	Pin();
	Pin(const Pin &pin);
	void operator=(const Pin &pin);

	void SetPinPoint(
		const PinType pin_type_val,
		const int x, const int y, const int z
		);
	void SetPinLine(
		const PinType pin_type_val,
		const int x1, const int y1, const int z1,
		const int x2, const int y2, const int z2
		);
	void SetPinRectangle(
		const PinType pin_type_val,
		const int left_x, const int bottom_y, const int z_val1,
		const int right_x, const int top_y, const int z_val2
		);

	void GetPinPointPosition(int &x, int &y, int &z) const;
	void GetPinLinePosition(
		int &x1, int &y1, int &z1,
		int &x2, int &y2, int &z2
		) const;
	void GetPinRectanglePosition(
		int &left, int &bottom, int &z1,
		int &right, int &top, int &z2
		) const;

	void GetPinNodes(vector<Node> &nodes) const;
	void GetWholePinNodes(vector<Node> &nodes) const;
	void GetPinLineEndPoints(vector<Node> &line_end_nodes) const;
	void GetCenterNode(Node &center_node) const;

	bool IsNodeWithinPin(const TurnNode &node) const;
	bool IsNodeWithinPin(const Node &node) const;

	int GetLayer() const;
	friend ostream &operator<<(ostream &os, const Pin &pin);
};


// replace oaBox
struct Box{
public:
	// pair(x,y)
	index_pair LB;
	index_pair RT;

	Box() : LB(make_pair(-1, -1)), RT(make_pair(-1, -1))
	{
		// default constructor
	}

	void set(index_pair lb, index_pair rt){
		// error check
		if (rt.first < lb.first)
		{
			cout << "ERROR Box::set(index_pair lb, index_pair rt) : right < left\n";
			return;
		} // if
		else if (rt.second < lb.second)
		{
			cout << "ERROR Box::set(index_pair lb, index_pair rt) : top < bottom\n";
			return;
		} // else if

		this->LB = lb;
		this->RT = rt;
	}
	void set(coor_t left, coor_t bottom, coor_t right, coor_t top){
		// error check
		if (right < left)
		{
			cout << "ERROR Box::set(coor_t left, coor_t bottom, coor_t right, coor_t top) : right < left\n";
			return;
		} // if
		else if (top < bottom)
		{
			cout << "ERROR Box::set(coor_t left, coor_t bottom, coor_t right, coor_t top) : top < bottom\n";
			return;
		} // else if

		this->LB = make_pair(left, bottom);
		this->RT = make_pair(right, top);
	}
	index_pair lowerLeft(){
		return this->LB;
	}
	index_pair upperRight(){
		return this->RT;
	}

	bool overlaps(Box b){
		// 
		if (b.LB.first <= this->RT.first && b.LB.second <= this->RT.second &&
			b.RT.first >= this->RT.first && b.RT.second >= this->RT.second)
		{
			// b is on upper right of (this)
			return true;
		} // if
		else if (b.LB.first <= this->RT.first && b.RT.second >= this->LB.second &&
			b.RT.first >= this->RT.first && b.LB.second <= this->LB.second)
		{
			// b is on lower right of (this)
			return true;
		} // else if
		else if (b.RT.first >= this->LB.first && b.LB.second <= this->RT.second &&
			b.LB.first <= this->LB.first && b.RT.second >= this->RT.second)
		{
			// b is on upper left of (this)
			return true;
		} // else if
		else if (b.RT.first >= this->LB.first && b.RT.second >= this->LB.second &&
			b.LB.first <= this->LB.first && b.LB.second <= this->LB.second)
		{
			// b is on lower left of (this)
			return true;
		} // else if
		else
			return false;

	}

	void getCenter(Point &p){
		p.X = (RT.first - LB.first);
		p.Y = (RT.second - LB.second);
	}

	friend bool operator== (Box a, Box b){
		return (a.LB == b.LB) & (a.RT == b.RT);
	}

};


// The two terminals of a 2-pin net
struct TwoPinNetConnection {

	// for OA identification
	//oaBox source_box;
	//oaBox target_box;

	int src_type;	// 0 -> pin , 1 -> multi-pin , 2 -> merge-path
	int tar_type;   // 0 -> pin , 1 -> multi-pin , 2 -> merge-path
	// replace oaBox
	Box source_box;
	Box target_box;
	// ISPD we can use multiple pin instead of box
	vector <Pin> source_pin_list;
	vector <Pin> target_pin_list;

	// 
	vector<Pin> Source_pin_path_list;
	vector<Pin> Target_pin_path_list;

	vector<Pin> pseudo_Source_pin_path_list;
	vector<Pin> pseudo_Target_pin_path_list;

	vector<Pin> real_Source_pin_path_list;
	vector<Pin> real_Target_pin_path_list;

	vector <Parser::I_Rect> Src_Pin_Shape; // for blockage unmark
	vector <Parser::I_Rect> Tar_Pin_Shape; // for blockage unmark

	// mapping pin position
	Pin source;
	Pin target;

	// a table that recognize the target nodes
	unordered_set<ULL> target_table;

	TwoPinNetConnection();
	TwoPinNetConnection(
		TwoPinNetConnection &connection
		);

	void clear()
	{
		source_pin_list.clear();
		target_pin_list.clear();

		Source_pin_path_list.clear();
		Target_pin_path_list.clear();

		pseudo_Source_pin_path_list.clear();
		pseudo_Target_pin_path_list.clear();

		real_Source_pin_path_list.clear();
		real_Target_pin_path_list.clear();

		Src_Pin_Shape.clear();
		Tar_Pin_Shape.clear();
	}

	void SetTargetTable(
		const int layout_width,
		const int layout_height,
		const int layout_layer
		);
	void EraseTargetKey(const ULL target_key);
	int GetEstimatedWireLength() const;

	bool DoesSourceOverlapTarget() const;
	bool IsSourceEqualToTarget() const;
	bool IsStraight() const;
	float GetStraightRatio() const;

	void PrintTargetTable();
	inline bool IsTargetReached(ULL index_key) const
	{
		const bool is_target_reached =
			(this->target_table.find(index_key) !=
			this->target_table.end());
		return is_target_reached;
	}

};


// The detail path info of a 2-pin net
struct TwoPinNetDetailedPath {
	list<Node> detail_grid_path;
	vector<TurnNode> path_turn_nodes;
	vector <Parser::wire_path> WireList;

	void clear()
	{
		detail_grid_path.clear();
		path_turn_nodes.clear();
		WireList.clear();
	}
};


// The list of nets which occupied a grid node
struct NetInfo {
	list<int> net_list;
};

// For replacing oaNet
struct Net{
	public :

};


// The nets needed to be rerouted and its basic info imported by file 
struct TwoPinRipUpNet {
	// for OA identification

	// for A* bound
	Parser::I_Rect TargetBound; 

	// pin data cache
	map<tuple<int, int, int>, bool> *cachePinTable;
	vector<Pin> *cache_pin_path_list; 
	// pin data cache
	map<tuple<int, int, int>, bool> *tempTargetTable;
	vector<Pin> *temp_target_path_list;

	// for DEBUG
	string netname;
	// for ISPD
	Parser::ISPD_2PinNet *TwoPinNetData;
	vector<Parser::ISPD_2PinNet> *TwoPinNetList;
	vector<Parser::IPSD_Routing_PIN> *DesignPinList;
	vector<Pin> PinToBitwise;
	// for rip-up & reroute
	bool routed;
	vector<Pin> PreNetPath;
	vector<int> ReferenceRipUpNet;

	//oaNet *p_net;
	Net *p_net;
	// for router identification
	int net_id;
	int two_pin_net_id;

	// cost evaluation
	int rip_up_times;
	int violation_num;
	int wire_length;

	// ISPD global guide for each net
	vector <Parser::I_Rect> Global_guide;

	TwoPinNetConnection two_pin_net_connection;
	TwoPinNetDetailedPath two_pin_net_detailed_path;
	bool path_is_found;
	bool both_pins_are_overlapped;
	bool has_only_one_pin;


	bool src_need_refine;
	bool tar_need_refine;

	TwoPinRipUpNet();
	TwoPinRipUpNet(const TwoPinRipUpNet &net);

	void clear()
	{
		cachePinTable->clear();
		cachePinTable = NULL;

		cache_pin_path_list->clear();
		cache_pin_path_list = NULL;

		tempTargetTable->clear();
		tempTargetTable = NULL;

		temp_target_path_list->clear();
		temp_target_path_list = NULL;

		TwoPinNetData->clear();
		TwoPinNetData = NULL;

		for(int i = 0; i < TwoPinNetList->size();i++){
			TwoPinNetList[i].clear();
		}
		TwoPinNetList->clear();
		TwoPinNetList = NULL;

		DesignPinList->clear();
		DesignPinList = NULL;
		
		p_net = NULL;
		Global_guide.clear();
		two_pin_net_connection.clear();
		two_pin_net_detailed_path.clear();
	}

	void SetPinsToConnection();

	inline TwoPinNetConnection& GetConnection()
	{
		SetPinsToConnection();
		return this->two_pin_net_connection;
	}
	inline Pin& GetSource()
	{
		return this->GetConnection().source;
	}
	inline Pin& GetTarget()
	{
		return this->GetConnection().target;
	}
	inline list<Node>& GetDetailedGridPath()
	{
		return this->two_pin_net_detailed_path.detail_grid_path;
	}
	inline vector<TurnNode>& GetPathTurnNodes()
	{
		return this->two_pin_net_detailed_path.path_turn_nodes;
	}

	bool operator < (const TwoPinRipUpNet &a) const;
	void PrintTargetTable()
	{
		cout << "net id = " << net_id << " ";
		cout << "2-pin net id = " << two_pin_net_id << endl;
		two_pin_net_connection.PrintTargetTable();
	}
};


// Used to indentify the net id of a 2-pin net and ite 2-pin net id 
struct NetIdPair {
	int net_id;
	int two_pin_net_id;
	int two_pin_net_index;

	NetIdPair(
		const int net_id_val = -1,
		const int two_pin_net_id_val = -1,
		const int two_pin_net_index_val = -1
		)
	{
		net_id = net_id_val;
		two_pin_net_id = two_pin_net_id_val;
		two_pin_net_index = two_pin_net_index_val;
	}
};
typedef map<int, NetIdPair> NetIdTable;
typedef NetIdTable::iterator NetIdTableIterator;


// track_capacity_ usage
const int kNoBlockage = 0;
const int kBlockage = -1;


// moving direction
const int kRight = 1;
const int kLeft = 2;
const int kTop = 3;
const int kBottom = 4;
const int kUp = 5;
const int kDown = 6;

const int kNone = 0;
const int kPinPseudoLocation = 20;
const int kPinLocation = 21;
const int kPinLocationRectangle = 22;
const int kPinBlockage = 31;
const int kPotentialBlockage = 32;
const int kNormalBlockage = 36;


// const int kSegRightViolation = 51;
// const int kSegLeftViolation = 52;
// const int kSegTopViolation = 53;
// const int kSegBottomViolation = 54;





#endif
