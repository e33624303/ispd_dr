#ifndef ROUTING_INFO_H
#define ROUTING_INFO_H

#include <cstdio>
using std::sscanf;

#include <cstring>
using std::strcmp;

#include <iostream>
using std::cout;
using std::endl;

#include<fstream>
using std::ifstream;

#include <ostream>
using std::ostream;

#include <sstream>
using std::stringstream;

#include <string>
using std::string;

#include <vector>
using std::vector;

#include "Definition.h"
#include "graph_struct.h"

/*
#include "oaDesignDB.h"
using namespace oa;
*/

struct ViaDef{
public:
};

struct RoutingLayerInfo
{
	//oaLayerNum layer = 0;
	int LayerNum = 0;
	//oaString layer_name = "";
	string layer_name = "";

	bool is_vertical = false;
	// bool is_valid_preferred_dir = false;

	// layer metal constraint info
	//oaUInt4 spacing = 0;
	UInt spacing = 0;
	int spc;
	//oaUInt4 width = 0;
	UInt width = 0;

	// track info
	//oaUInt4 h_pitch = 0, v_pitch = 0;
	//oaUInt4 h_offset = 0, v_offset = 0;
	UInt h_pitch = 0, v_pitch = 0;
	UInt h_offset = 0, v_offset = 0;

	//oaUInt4 h_track_num = 0, v_track_num = 0;
	//oaUInt4 h_track_spacing = 0, v_track_spacing = 0;
	UInt h_track_num = 0, v_track_num = 0;
	UInt h_track_spacing = 0, v_track_spacing = 0;

	//oaInt4 h_track_start = 0, v_track_start = 0;
	int h_track_start = 0, v_track_start = 0;

	// mask info
	//oaUInt4 mask_num = 1;
	UInt mask_num = 1;
	bool has_v_track = false;
	bool has_h_track = false;

	// end-to-end spacing constraint info
	bool has_line_end_rule = false;
	//oaUInt4 spacing_e2e = 0;
	//oaUInt4 parity_track_spacing_e2e = 0;
	UInt spacing_e2e = 0;
	UInt parity_track_spacing_e2e = 0;

	// routing via info
	//oaViaDef *upper_via_def = NULL;
	//oaViaDef *lower_via_def = NULL;
	ViaDef *upper_via_def = NULL;
	ViaDef *lower_via_def = NULL;

	//oaUInt4 upper_enc1_width = 0;
	//oaUInt4 upper_enc1_height = 0;
	//oaUInt4 upper_enc2_width = 0;
	//oaUInt4 upper_enc2_height = 0;
	UInt upper_enc1_width = 0;
	UInt upper_enc1_height = 0;
	UInt upper_enc2_width = 0;
	UInt upper_enc2_height = 0;

	//oaUInt4 lower_enc1_width = 0;
	//oaUInt4 lower_enc1_height = 0;
	//oaUInt4 lower_enc2_width = 0;
	//oaUInt4 lower_enc2_height = 0;
	UInt lower_enc1_width = 0;
	UInt lower_enc1_height = 0;
	UInt lower_enc2_width = 0;
	UInt lower_enc2_height = 0;

	/*inline oaLayerNum GetLayer()
		{ return layer; }*/
	inline int GetLayer()
	{
		return LayerNum;
	}

	friend ostream &operator<<(ostream &os, const RoutingLayerInfo &info)
	{
		// os << "---------------------" << endl;
		os << "Layer: " << info.LayerNum << endl;
		os << "Layer Name: " << info.layer_name << endl;

		os << " @ Vertical: " << std::boolalpha << info.is_vertical << endl;; 
		os << " - Spacing:  " << info.spacing << endl;
		os << " - Width:    " << info.width << endl;
		os << " - Pitch H:  " << info.h_pitch << "  V: " << info.v_pitch << endl;
		os << " - offset H: " << info.h_offset << "  V: " << info.v_offset << endl << endl;
		
		os << " @ Has H Track = " << info.has_h_track << endl;
		os << " @ Has V Track = " << info.has_v_track << endl;
		os << " - Trach Num   H: " << info.h_track_num << "  V: " << info.v_track_num << endl;
		os << " - Track Space H: " << info.h_track_spacing << "  V: " << info.v_track_spacing << endl;
		os << " - Track Start H: " << info.h_track_start << "  V: " << info.v_track_start << endl << endl;

		os << " @ Has Line Rule =  " << std::boolalpha << info.has_line_end_rule << endl; 
		os << " - Mask Num:     " << info.mask_num << endl;
		os << " - End-End Spacing: " << info.spacing_e2e << endl;
		os << " - Parity Spacing:  " << info.parity_track_spacing_e2e << endl << endl;

		os << " @ Has Upper Via Def = " << std::boolalpha << (info.upper_via_def != NULL) << endl;
		os << " - Upper Enc1 W: " << info.upper_enc1_width << "  ";
		os << "H: " << info.upper_enc1_height << endl;
		os << " - Upper Enc2 W: " << info.upper_enc2_width << "  ";
		os << "H: " << info.upper_enc2_height << endl;
		os << " @ Has Lower Via Def = " << std::boolalpha << (info.lower_via_def != NULL) << endl;
		os << " - Lower Enc1 W: " << info.lower_enc1_width << "  ";
		os << "H: " << info.lower_enc1_height << endl;
		os << " - Lower Enc2 W: " << info.lower_enc2_width << "  ";
		os << "H: " << info.lower_enc2_height << endl;
		os << "---------------------" << endl;
		return os;
	}

};


struct RoutingLayerInfos
{
	// Put routing layers in sequence from low layer to high layer.
	vector<RoutingLayerInfo> routing_layer_infos;
	//oaUInt4 DBU;
	UInt DBU;

	// Get rules from OA database.
	/*void GetRoutingLayerInfos(oaTech *tech, oaBlock *top_block);
	
	// Get the Constraints from OA Library.
	void InitLayerInfos(oaTech *tech, oaBlock *top_block);
	// Get the Constraints from the Rule File.
	void InitLayerInfos(const char* rule_file);
	// Get the line-end rules in the Tech DB.
	void GetLineEndRulesInTechDB(oaTech* tech);
	// Get the via defs for routing usaeg.
	void GetRoutingViaInfos(oaTech *tech);
	void SetViaEncInfos();
	// Get the box of the via in via def.
	//void GetViaBox(oaBox &box, oaViaDef *p_def);
	void GetViaBox(Box &box, oaViaDef *p_def);

	//void GetViaEnc1Box(oaBox &box, oaViaDef *p_def);
	void GetViaEnc1Box(Box &box, oaViaDef *p_def);

	//void GetViaEnc2Box(oaBox &box, oaViaDef *p_def);
	void GetViaEnc2Box(Box &box, oaViaDef *p_def);*/

	inline vector<RoutingLayerInfo>& GetLayerInfos()
	{
		return this->routing_layer_infos;
	}

	inline int GetSize()
	{
		return (int)this->routing_layer_infos.size();
	}
	
	inline RoutingLayerInfo& GetInfo(int layer)
	{
		return this->routing_layer_infos[layer];
	}
	
	RoutingLayerInfo& operator[] (int layer)
	{
		return routing_layer_infos[layer];
	}
};


#endif

