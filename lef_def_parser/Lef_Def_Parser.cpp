#include "MainParser.h"
#include <stdlib.h>
#include <map>
#include <math.h>
#include "../MazeRouteKernel.h"

using namespace std;
//const int _ISPD_OFFSET_ = 2000;

/*class UmapS2IHash{
	public:
	size_t operator () (const string &str) const{
		size_t id;
		//printf("%s\n", str.c_str());
		sscanf(str.c_str(), "inst%d", &id);
		//printf("inst ID : %d\n", id);
		return id;
	}
};*/

pair<int,int> ISPD_FindComponent(vector<MACRO> macro_list, int i ,string p_name){
	for (int j = 0; macro_list[i].Pin_list.size(); j++){
		if (macro_list[i].Pin_list[j].Name == p_name){
				return make_pair(i,j);
			}
	}
}

void ISPD_Pin_Extraction(ISPD_Routing_Net &ispd_net, Umap_S2I_inst &ComponentTable,
	Umap_S2I &RoutingPinTable, Umap_S2I &MacroTable, vector < Umap_S2I > &MacroPinTable, Design &design){

	ispd_net.PIN_list.resize(ispd_net.Original_Data.CPN_list.size());
	//unordered_map < string, int >::iterator iter;
	for (int i = 0; i < ispd_net.Original_Data.CPN_list.size(); i++){
		bool type = true;
		string component_id = ispd_net.Original_Data.CPN_list[i].Component_Name;
		int RoutingPin_id = -1;
		if (strcmp(component_id.c_str(),"PIN") == 0){
			type = false;
			ispd_net.PIN_list[i].type = type;
			auto iter = RoutingPinTable.find(ispd_net.Original_Data.CPN_list[i].Pin_Name);
			RoutingPin_id = iter->second;
			int layer;
			sscanf(design.Routing_Pin_list[RoutingPin_id].Layer.c_str(), "Metal%d", &layer);
			ispd_net.PIN_list[i].Layer = layer;
			float x = (float)design.Routing_Pin_list[RoutingPin_id].Place_Index.first / (float)design.ISPD_OFFSET;
			float y = (float)design.Routing_Pin_list[RoutingPin_id].Place_Index.second / (float)design.ISPD_OFFSET;
			ispd_net.PIN_list[i].Pin_index = make_pair(x, y);
			ispd_net.PIN_list[i].PinOrient = design.Routing_Pin_list[RoutingPin_id].Orient;
			ispd_net.PIN_list[i].Original_PIN = design.Routing_Pin_list[RoutingPin_id];
			continue;
		}
		auto iter = ComponentTable.find(component_id);
		int C_id = iter->second;
		//printf("ID : %d\n",C_id);
		ispd_net.PIN_list[i].type = type;
		Component this_component = design.Component_list[C_id];
		string component_name = this_component.Name;
		string pin_name = ispd_net.Original_Data.CPN_list[i].Pin_Name;
		iter = MacroTable.find(component_name);
		int macro_id = iter->second;
		iter = MacroPinTable[macro_id].find(pin_name);
		pair<int, int> id_pair = make_pair(macro_id, iter->second);
		
		// Original_Data
		ispd_net.PIN_list[i].Original_MARCO = design.Macro_list[id_pair.first];
		//ispd_net.PIN_list[i].Original_PIN = design.Macro_list[id_pair.first].Pin_list[id_pair.second];
		// Macro place + orient
		ispd_net.PIN_list[i].Macro_Place = make_pair((float)this_component.index.first / (float)design.ISPD_OFFSET, (float)this_component.index.second / (float)design.ISPD_OFFSET);
		ispd_net.PIN_list[i].Macro_orient = this_component.Dir;
		// Macro port
		if (type){
			for (int port = 0; port < design.Macro_list[id_pair.first].Pin_list[id_pair.second].port.rect_list.size(); port++){
				ispd_net.PIN_list[i].Rect_list.push_back(design.Macro_list[id_pair.first].Pin_list[id_pair.second].port.rect_list[port]);
			}
		}
	}
}


void ISPD_Construct_Netlist_Information(Design &design,bool net_log){

	Umap_S2I_inst ComponentTable;
	for (int i = 0; i < design.Component_list.size(); i++){
		ComponentTable.emplace(design.Component_list[i].ID, i);
	}
	Umap_S2I RoutingPinTable;
	for (int i = 0; i < design.Routing_Pin_list.size(); i++){
		RoutingPinTable.emplace(design.Routing_Pin_list[i].Name, i);
	}
	Umap_S2I MacroTable;
	vector < Umap_S2I > MacroPinTable;
	for (int i = 0; i < design.Macro_list.size(); i++){
		MacroTable.emplace(design.Macro_list[i].Name, i);
		Umap_S2I MPT;
		for (int j = 0; j < design.Macro_list[i].Pin_list.size();j++){
			MPT.emplace(design.Macro_list[i].Pin_list[j].Name, j);
		}
		MacroPinTable.push_back(MPT);
	}

	design.ComponentTable = ComponentTable;
	design.RoutingPinTable = RoutingPinTable;
	design.MacroTable = MacroTable;
	design.MacroPinTable = MacroPinTable;

	for (int i = 0; i < design.Net_Name_list.size(); i++){
		ISPD_Routing_Net ispd_net;
		ispd_net.Original_Data = design.Net_Name_list[i];
		ISPD_Pin_Extraction(ispd_net, ComponentTable, RoutingPinTable, MacroTable, MacroPinTable , design);
		design.ispd_routing_net.push_back(ispd_net);
		// log
		if (net_log){
			for (int j = 0; j < ispd_net.PIN_list.size(); j++){
				if (ispd_net.PIN_list[j].type){
					printf("Net%d::MARCO%d PLACE : (%.3f,%.3f)\n", i, j, ispd_net.PIN_list[j].Macro_Place.first, ispd_net.PIN_list[j].Macro_Place.second);
					printf("Net%d::MARCO%d ORIENT : %s \n", i, j, ispd_net.PIN_list[j].Macro_orient.c_str());
					for (int r = 0; r < ispd_net.PIN_list[j].Rect_list.size(); r++){
						printf("	Net%d::MARCO%d::RECT%d : LB(%.3f,%.3f) RT(%.3f,%.3f) %s \n", i, j, r,
							ispd_net.PIN_list[j].Rect_list[r].LB.first, ispd_net.PIN_list[j].Rect_list[r].LB.second,
							ispd_net.PIN_list[j].Rect_list[r].RT.first, ispd_net.PIN_list[j].Rect_list[r].RT.second,
							ispd_net.PIN_list[j].Rect_list[r].Layer.c_str());
					}
				}
				else{
					printf("Net%d::PIN%d PLACE : (%.3f,%.3f)\n", i, j, ispd_net.PIN_list[j].Pin_index.first, ispd_net.PIN_list[j].Pin_index.second);
					printf("Net%d::PIN%d Layer : %d \n", i, j, ispd_net.PIN_list[j].Layer);
					printf("Net%d::PIN%d Orient : %s \n", i, j, ispd_net.PIN_list[j].PinOrient.c_str());
				}
			}
			printf("-----------------------------------------\n");
		}
	}
}

void Distribute_Track_list(vector <Track> &Track_list, vector < pair<Parser::Track, Parser::Track> > &ispd_track_list){

	int total_layer = Track_list.size() / 2;
	if (Track_list.size() % 2 != 0) {
		printf("ERROR::Distribute_Track_list Track_list.size() not even\n");
		exit(1);
	}

	ispd_track_list.resize(total_layer);
	for (int i = 0; i < Track_list.size(); i++){
		int layer = -1;
		sscanf(Track_list[i].Layer.c_str(), "Metal%d", &layer);
		if (layer == -1) {
			printf("ERROR::Distribute_Track_list layer = -1\n"); exit(1);
		}
		if (strcmp(Track_list[i].Dir.c_str(), "X") == 0){
			ispd_track_list[layer - 1].first = Track_list[i];
		}
		else if (strcmp(Track_list[i].Dir.c_str(), "Y") == 0){
			ispd_track_list[layer - 1].second = Track_list[i];
		}
		else{
			printf("ERROR::Distribute_Track_list Dir wrong\n"); exit(1);
		}

	}
}

namespace ISPD_GEOMETRY
{
void CounterClockwiseRotate90(I_Rect &Bounding, bool fixed, vector<I_Rect> &input_rectangle, IntPair &ori)
{
	// ori is for fixed
	if (fixed)
	{
		// fixed
		for (auto &Rectlist : input_rectangle)
		{
			int OriLBY = Rectlist.LB.second;
			Rectlist.LB = make_pair(Rectlist.RT.second * -1, Rectlist.LB.first);
			Rectlist.RT = make_pair(OriLBY * -1, Rectlist.RT.first);
		} // for

		int OriBoundLBY = Bounding.LB.second;
		Bounding.LB = make_pair(Bounding.RT.second * -1, Bounding.LB.first);
		Bounding.RT = make_pair(OriBoundLBY * -1, Bounding.RT.first);
	}
	else
	{
		// not fixed
		int Final_move = Bounding.RT.second - Bounding.LB.second;
		// Bounding.RT is 0
		for (auto &Rectlist : input_rectangle)
		{
			int OriLBY = Rectlist.LB.second;
			Rectlist.LB = make_pair(Rectlist.RT.second * -1 + Final_move, Rectlist.LB.first);
			Rectlist.RT = make_pair(OriLBY * -1 + Final_move, Rectlist.RT.first);
			//cout << ">> " << Rectlist.LB.first << " " << Rectlist.LB.second << " " << Rectlist.RT.first << " " << Rectlist.RT.second << endl;
		} // for

		int OriBoundLBY = Bounding.LB.second;
		Bounding.LB = make_pair(Bounding.RT.second * -1 + Final_move, Bounding.LB.first);
		Bounding.RT = make_pair(OriBoundLBY * -1 + Final_move, Bounding.RT.first);
	}
}

void FlipCenter(I_Rect &Bounding, vector<I_Rect> &input_rectangle)
{
	float CenterY = (Bounding.LB.first + Bounding.RT.first) / 2 - Bounding.LB.first;

	for (auto &Rectlist : input_rectangle)
	{
		int LBx = ((CenterY > Rectlist.RT.first) ? abs((int)(Rectlist.RT.first - CenterY) * 2) : -1 * abs((int)(Rectlist.RT.first - CenterY) * 2)) + Rectlist.RT.first;
		int LBy = Rectlist.LB.second;
		int RTx = ((CenterY > Rectlist.LB.first) ? abs((int)(Rectlist.LB.first - CenterY) * 2) : -1 * abs((int)(Rectlist.LB.first - CenterY) * 2)) + Rectlist.LB.first;
		int RTy = Rectlist.RT.second;
		Rectlist.LB = make_pair(LBx, LBy);
		Rectlist.RT = make_pair(RTx, RTy);
	} // for
}
};


void ISPD_AllShape2Obstacle(Design &design)
{
	map < pair <string, string>, int > PinNetMap;
	for (int i = 0; i < design.ispd_routing_net.size(); i++){
		for (int pin = 0; pin < design.ispd_routing_net[i].Original_Data.CPN_list.size(); pin++){
			string C_name = design.ispd_routing_net[i].Original_Data.CPN_list[pin].Component_Name;
			string P_name = design.ispd_routing_net[i].Original_Data.CPN_list[pin].Pin_Name;
			PinNetMap.emplace(make_pair(C_name, P_name), i);
		}
	}


	for (auto &thisCP : design.Component_list)
	{

		int Macroindex = 0;		
		Macroindex = design.ConponentId2MacroIndex(thisCP.ID);
		MACRO &tempMACRO = design.Macro_list.at(Macroindex);

		// MACRO, its location can be moved
		int MacroPlacedX = thisCP.index.first;
		int MacroPlacedY = thisCP.index.second;

		int MacroWidth = lround(tempMACRO.SIZE_BY.first * design.ISPD_OFFSET);
		int MacroHeight = lround(tempMACRO.SIZE_BY.second * design.ISPD_OFFSET);
		I_Rect Bound = I_Rect(MacroPlacedX, MacroPlacedY, MacroPlacedX + MacroWidth, MacroPlacedY + MacroHeight, 0);
		// macro -> not fixed, pin -> fixed
		bool fixed = false;

		IntPair ori_pair(MacroPlacedX, MacroPlacedY);

		vector<I_Rect> tempIRect_list;
		string C_name = thisCP.ID;
		for(auto &tempPin : tempMACRO.Pin_list)
		{

			int net_id = UndefineValue;
			string P_name = tempPin.Name;
			auto cp_search = PinNetMap.find(make_pair(C_name, P_name));
			if (cp_search != PinNetMap.end()){
				net_id = cp_search->second;
				//printf("Net(%d) : Component(%s) , Pin(%s)\n", net_id, C_name.c_str(), P_name.c_str());
			}
			else{
				net_id = UndefineValue;
				//printf("Net(%d) : Component(%s) , Pin(%s)\n", net_id, C_name.c_str(), P_name.c_str());
			}

			for (auto &tempRect : tempPin.port.rect_list)
			{
				/*
				printf("LB(%.3f,%.3f) RT(%.3f,%.3f) %s \n",
					   tempRect.LB.first, tempRect.LB.second,
					   tempRect.RT.first, tempRect.RT.second,
					   tempRect.Layer.c_str());
				*/
				if (tempRect.Layer.size() >= 6)
					tempRect.Layer.erase(tempRect.Layer.begin(), tempRect.Layer.begin() + 5);

				I_Rect tempIRect = I_Rect(lround(tempRect.LB.first * design.ISPD_OFFSET), lround(tempRect.LB.second * design.ISPD_OFFSET),
										lround(tempRect.RT.first * design.ISPD_OFFSET), lround(tempRect.RT.second * design.ISPD_OFFSET), atoi(tempRect.Layer.c_str()));
				/*
				printf("LB(%d,%d) RT(%d,%d) %d \n",
					   tempIRect.LB.first, tempIRect.LB.second,
					   tempIRect.RT.first, tempIRect.RT.second,
					   tempIRect.Layer);
				*/
				tempIRect_list.push_back(tempIRect);
			}

			if (thisCP.Dir.compare("N") == 0)
			{
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
			}
			else if (thisCP.Dir.compare("W") == 0)
			{
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
			}
			else if (thisCP.Dir.compare("S") == 0)
			{
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
			}
			else if (thisCP.Dir.compare("E") == 0)
			{
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
			}
			else if (thisCP.Dir.compare("FN") == 0)
			{
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::FlipCenter(Bound, tempIRect_list);
			}
			else if (thisCP.Dir.compare("FW") == 0)
			{
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::FlipCenter(Bound, tempIRect_list);
			}
			else if (thisCP.Dir.compare("FS") == 0)
			{
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::FlipCenter(Bound, tempIRect_list);
			}
			else if (thisCP.Dir.compare("FE") == 0)
			{
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				ISPD_GEOMETRY::FlipCenter(Bound, tempIRect_list);
			}

			for (auto &tempRect : tempIRect_list)
			{
				tempRect.LB.first += MacroPlacedX;
				tempRect.LB.second += MacroPlacedY;
				tempRect.RT.first += MacroPlacedX;
				tempRect.RT.second += MacroPlacedY;
			}

			for (auto &tempRect : tempIRect_list)
			{
				if (P_name == "OBS" && tempRect.Layer == 2)
				{
					design.AllOBSs.push_back(tempRect);
					/*
					printf("OBS placed : (%s) (%s) LB(%d,%d) RT(%d,%d) %d \n", thisCP.Name.c_str(), tempPin.Name.c_str(),
						   tempRect.LB.first, tempRect.LB.second,
						   tempRect.RT.first, tempRect.RT.second,
						   tempRect.Layer);
					*/
				}
				else if (P_name == "VSS" && tempRect.Layer == 2)
				{
					design.AllOBSs.push_back(tempRect);
					/*
					printf("VSS placed : (%s) (%s) LB(%d,%d) RT(%d,%d) %d \n", thisCP.Name.c_str(), tempPin.Name.c_str(),
						   tempRect.LB.first, tempRect.LB.second,
						   tempRect.RT.first, tempRect.RT.second,
						   tempRect.Layer);
					*/
				}
				else if (P_name == "VDD" && tempRect.Layer == 2)
				{
					design.AllOBSs.push_back(tempRect);
					/*
					printf("VDD placed : (%s) (%s) LB(%d,%d) RT(%d,%d) %d \n", thisCP.Name.c_str(), tempPin.Name.c_str(),
						   tempRect.LB.first, tempRect.LB.second,
						   tempRect.RT.first, tempRect.RT.second,
						   tempRect.Layer);
					*/
				}
				else 
				{
					if (tempRect.LB.first > tempRect.RT.first)
					{
						int temp = tempRect.LB.first;
						tempRect.LB.first = tempRect.RT.first;
						tempRect.RT.first = temp;
					}

					if(tempRect.LB.second > tempRect.RT.second)
					{
						int temp = tempRect.LB.second ;
						tempRect.LB.second = tempRect.RT.second ;
						tempRect.RT.second = temp;
					}

					design.AllShapes.push_back(tempRect);
					design.AllShapes_NetId.push_back(net_id);
					/*
					printf("IRect placed : (%s) (%s) LB(%d,%d) RT(%d,%d) %d \n", thisCP.Name.c_str(), tempPin.Name.c_str(),
						   tempRect.LB.first, tempRect.LB.second,
						   tempRect.RT.first, tempRect.RT.second,
						   tempRect.Layer);
					*/
				}
			}				
		
			tempIRect_list.clear();
			/*
			for (int i = 0; i < thisNet.PIN_list.size(); i++)
			{
				if(thisNet.PIN_list.at(i).IRect_list.size() != thisNet.PIN_list.at(i).Rect_list.size()
					&& thisNet.PIN_list.at(i).type == true)
				{
					cout << "ERROR :: " << thisNet.Original_Data.Net_name<<endl;
					cout << thisNet.PIN_list.at(i).Original_MARCO.Name << endl;
					cout << thisNet.PIN_list.at(i).IRect_list.size() << "  " << thisNet.PIN_list.at(i).Rect_list.size()<<endl;
					int xx;
					cin>>xx;
				}
			}
			*/

		} // for, Macro's Pins
	} // for
}


void ISPD_Insert_PinShape2Layout(Design &design)
{
	int net_count = 0;
	for (auto &thisNet : design.ispd_routing_net)
	{		
		auto &thisOriData = thisNet.Original_Data;

		vector<pair<string, int> > duplicatePair;
		map<string, int> NetTable;
		for (int i = 0; i < thisNet.PIN_list.size(); i++)
		{
			
			//cout << "i = " << i << endl;
			// Component id -> macro index -> (macro name -> pin in list)
			if(thisNet.PIN_list.at(i).type)
			{
				auto candidate = NetTable.find(thisNet.PIN_list.at(i).Original_MARCO.Name);
				if (candidate != NetTable.end())
				{
					// already inside
					duplicatePair.push_back(make_pair(thisNet.PIN_list.at(i).Original_MARCO.Name, i));
				}
				else
				{
					NetTable.emplace(thisNet.PIN_list.at(i).Original_MARCO.Name, i);
				}
			}
			else
			{
				auto candidate = NetTable.find(thisNet.PIN_list.at(i).Original_PIN.Name);
				if (candidate != NetTable.end())
				{
					// already inside
					duplicatePair.push_back(make_pair(thisNet.PIN_list.at(i).Original_PIN.Name, i));
				}
				else
				{
					NetTable.emplace(thisNet.PIN_list.at(i).Original_PIN.Name, i);
				}
			}
		}
		
		//cout << "CP size = " << thisOriData.CPN_list.size() << endl;
		
		for (auto &thisCPlist : thisOriData.CPN_list)
		{
			string CPN_list_PinName = thisCPlist.Pin_Name;
			// use componentID to find macro index
			int Macroindex = 0;
			string MacroName;
			if (thisCPlist.Component_Name.at(1) != 'I') 
			{
				Macroindex = design.ConponentId2MacroIndex(thisCPlist.Component_Name); 
				// use macro index to find macro name
				MacroName = design.Macro_list.at(Macroindex).Name;
			}
			else 
			{
				// PIN here
				MacroName = thisCPlist.Pin_Name;
			}
			// use macro name and pin name to find correct pin
			auto PinIndex = NetTable.find(MacroName);
			// use PinIndex to find Current Pin in the list

			auto &thisPin = thisNet.PIN_list.at(PinIndex->second);

			int pin_index_tuple = PinIndex->second;

			thisPin.pseudo = false; // all of these are not pseudo pin

			for(int x = 0 ; x < duplicatePair.size() ; x++)
			{
				// found in duplicated array
				if(duplicatePair.at(x).first.compare(MacroName) == 0)
				{
					NetTable.erase(PinIndex);
					NetTable.emplace(duplicatePair.at(x).first, duplicatePair.at(x).second);
					duplicatePair.erase(duplicatePair.begin()+x);

					break;
				}
			}

			if (thisPin.type == true)
			{
				// MACRO, its location can be moved
				auto &thisMACRO = thisPin.Original_MARCO;
				int MacroPlacedX = thisPin.IMacro_Place.first = lround(thisPin.Macro_Place.first * design.ISPD_OFFSET);
				int MacroPlacedY = thisPin.IMacro_Place.second = lround(thisPin.Macro_Place.second * design.ISPD_OFFSET);

				int MacroWidth = lround(thisMACRO.SIZE_BY.first * design.ISPD_OFFSET);
				int MacroHeight = lround(thisMACRO.SIZE_BY.second * design.ISPD_OFFSET);
				I_Rect Bound = I_Rect(MacroPlacedX, MacroPlacedY, MacroPlacedX + MacroWidth, MacroPlacedY + MacroHeight, thisPin.Layer);
				// macro -> not fixed, pin -> fixed
				bool fixed = (thisPin.type)?false:true;

				IntPair ori_pair(MacroPlacedX, MacroPlacedY);
				
				vector <I_Rect> tempIRect_list;
				int tempPinShapeArea = 0;

				for(auto & tempRect : thisPin.Rect_list)
				{
					/*
					printf("LB(%.3f,%.3f) RT(%.3f,%.3f) %s \n",
						   tempRect.LB.first, tempRect.LB.second,
						   tempRect.RT.first, tempRect.RT.second,
						   tempRect.Layer.c_str());
					*/
					if( tempRect.Layer.size() >= 6)
						tempRect.Layer.erase(tempRect.Layer.begin(), tempRect.Layer.begin()+5);

					I_Rect tempIRect = I_Rect(lround(tempRect.LB.first * design.ISPD_OFFSET), lround(tempRect.LB.second * design.ISPD_OFFSET),
											  lround(tempRect.RT.first * design.ISPD_OFFSET), lround(tempRect.RT.second * design.ISPD_OFFSET), atoi(tempRect.Layer.c_str()));

					int width = tempIRect.RT.first - tempIRect.LB.first;
					int height = tempIRect.RT.second - tempIRect.LB.second;
					tempPinShapeArea = tempPinShapeArea + (width * height);					
					/*
					printf("LB(%d,%d) RT(%d,%d) %d \n",
						   tempIRect.LB.first, tempIRect.LB.second,
						   tempIRect.RT.first, tempIRect.RT.second,
						   tempIRect.Layer);
					*/
					tempIRect_list.push_back(tempIRect);
				}

				thisPin.PinShapeArea = tempPinShapeArea;

				design.SortedShapesList.push_back(make_tuple(tempPinShapeArea, net_count, pin_index_tuple));

				if (thisPin.Macro_orient.compare("N") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				}
				else if (thisPin.Macro_orient.compare("W") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				}
				else if (thisPin.Macro_orient.compare("S") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				}
				else if (thisPin.Macro_orient.compare("E") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				}
				else if (thisPin.Macro_orient.compare("FN") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::FlipCenter(Bound, tempIRect_list);
				}
				else if (thisPin.Macro_orient.compare("FW") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::FlipCenter(Bound, tempIRect_list);
				}
				else if (thisPin.Macro_orient.compare("FS") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::FlipCenter(Bound, tempIRect_list);
					
				}
				else if (thisPin.Macro_orient.compare("FE") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::FlipCenter(Bound, tempIRect_list);
				}

				for (auto &tempRect : tempIRect_list)
				{
					tempRect.LB.first += MacroPlacedX;
					tempRect.LB.second += MacroPlacedY;
					tempRect.RT.first += MacroPlacedX;
					tempRect.RT.second += MacroPlacedY;
				}

				for (auto &tempRect : tempIRect_list)
				{
					/*
					printf("placed : (%s) LB(%d,%d) RT(%d,%d) %d \n", MacroName.c_str(),
						   tempRect.LB.first, tempRect.LB.second,
						   tempRect.RT.first, tempRect.RT.second,
						   tempRect.Layer);
					*/

					thisPin.IRect_list.push_back(tempRect);
				}

			} // if
			else
			{

				thisPin.IPin_index.first = lround(thisPin.Pin_index.first * design.ISPD_OFFSET);
				thisPin.IPin_index.second = lround(thisPin.Pin_index.second * design.ISPD_OFFSET);
				
				// PIN, its location cannnot be moved
				auto &thisPIN = thisPin.Original_PIN;
				
				I_Rect Bound = I_Rect(thisPIN.LayerBound.LB.first, thisPIN.LayerBound.LB.second, thisPIN.LayerBound.RT.first, thisPIN.LayerBound.RT.second, thisPin.Layer);
				// macro -> not fixed, pin -> fixed
				bool fixed = true;
				IntPair ori_pair(thisPIN.LayerBound.LB.first, thisPIN.LayerBound.LB.second);

				vector<I_Rect> tempIRect_list;

				thisPin.PinShapeArea = (Bound.RT.first-Bound.LB.first)*(Bound.RT.second-Bound.LB.second);

				design.SortedShapesList.push_back(make_tuple(thisPin.PinShapeArea, net_count, pin_index_tuple));

				tempIRect_list.push_back(Bound);

				if (thisPin.PinOrient.compare("N") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				}
				else if (thisPin.PinOrient.compare("W") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				}
				else if (thisPin.PinOrient.compare("S") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				}
				else if (thisPin.PinOrient.compare("E") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
				}
				else if (thisPin.PinOrient.compare("FN") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::FlipCenter(Bound, tempIRect_list);
				}
				else if (thisPin.PinOrient.compare("FW") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::FlipCenter(Bound, tempIRect_list);
				}
				else if (thisPin.PinOrient.compare("FS") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::FlipCenter(Bound, tempIRect_list);
				}
				else if (thisPin.PinOrient.compare("FE") == 0)
				{
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::CounterClockwiseRotate90(Bound, fixed, tempIRect_list, ori_pair);
					ISPD_GEOMETRY::FlipCenter(Bound, tempIRect_list);
				}

				for (auto &tempRect : tempIRect_list)
				{
					tempRect.LB.first += thisPIN.Place_Index.first;
					tempRect.LB.second += thisPIN.Place_Index.second;
					tempRect.RT.first += thisPIN.Place_Index.first;
					tempRect.RT.second += thisPIN.Place_Index.second;
				}

				for (auto &tempRect : tempIRect_list)
				{
					/*
					printf("placed : (%s) LB(%d,%d) RT(%d,%d) %d \n", MacroName.c_str(),
						   tempRect.LB.first, tempRect.LB.second,
						   tempRect.RT.first, tempRect.RT.second,
						   tempRect.Layer);
					*/

					thisPin.IRect_list.push_back(tempRect);
				}
				
				// PIN, its location is fixed
			} // else
		}
/*
		for (int i = 0; i < thisNet.PIN_list.size(); i++)
		{
			if(thisNet.PIN_list.at(i).IRect_list.size() != thisNet.PIN_list.at(i).Rect_list.size()
				&& thisNet.PIN_list.at(i).type == true)
			{
				cout << "ERROR :: " << thisNet.Original_Data.Net_name<<endl;
				cout << thisNet.PIN_list.at(i).Original_MARCO.Name << endl;
				cout << thisNet.PIN_list.at(i).IRect_list.size() << "  " << thisNet.PIN_list.at(i).Rect_list.size()<<endl;
				int xx;
				cin>>xx;
			}
		}
*/
		net_count++;
	} // for

/*
	for (int index = 0; index < (int)design.SortedShapesList.size(); index++)
	{
		cout << ">> " << get<1>(design.SortedShapesList.at(index)) << " " << get<2>(design.SortedShapesList.at(index)) << endl;
	}
	*/
}

void MainParser(int argc, char** argv, Design &design){
	// -lef [lef file] -def [def file] -guide [guide_file] -threads [num_threads] -output <output_file_name>

	string Lef_File, Def_File, Guide_File, Output_File;
	int Thread_num;

	if (argc != 11)
	{
		cout << "usage -lef [lef file] -def [def file] -guide [guide_file] -threads [num_threads] -output <output_file_name>\n";
		exit(0);
	}

	// parse input
	for(int argc_index = 1; argc_index < argc ; argc_index++)
	{
		if(strcmp(argv[argc_index], "-lef") == 0)
		{
			Lef_File = argv[argc_index+1] ;
		}
		else if (strcmp(argv[argc_index], "-def") == 0)
		{
			Def_File = argv[argc_index + 1];
		}
		else if (strcmp(argv[argc_index], "-guide") == 0)
		{
			Guide_File = argv[argc_index + 1];
		}
		else if (strcmp(argv[argc_index], "-threads") == 0)
		{
			Thread_num = atoi(argv[argc_index + 1]);
		}
		else if (strcmp(argv[argc_index], "-output") == 0)
		{
			Output_File = argv[argc_index + 1];
		}

		argc_index++;
	}

	// parse end

	//printf("%s\n", argv[1]);
	//printf("%s\n", argv[2]);
	char leffile[100];
	char deffile[100];
	
	strcpy(leffile, Lef_File.c_str());
	strcpy(deffile, Def_File.c_str());
	Lef_main(leffile, argc, argv, design);
	Def_main(deffile, argc, argv, design);
	//design.Design_Log();
	//design.Design_Log();
	clock_t start = clock();
	ISPD_Construct_Netlist_Information(design,false);
	Distribute_Track_list(design.Track_list, design.ispd_track_list);
	clock_t end = clock();

	double duration = (double)(end - start) / CLOCKS_PER_SEC;
	printf("Lef/Def Parser Time Cost : %.2lf sec\n", duration);

	printf("ISPD_AllShape2Obstacle\n");
	ISPD_AllShape2Obstacle(design);
	// put pin onto layout
	printf("ISPD_Insert_PinShape2Layout\n");
	ISPD_Insert_PinShape2Layout(design);

	stable_sort(design.SortedShapesList.begin(), design.SortedShapesList.end(), [](const tuple<int, int, int> a, const tuple<int, int, int> b) -> bool { return std::get<0>(a) <= std::get<0>(b); });

	// here, read guide file
	ifstream readfile;
	string fileName = Guide_File.c_str();

	readfile.open(fileName.c_str(), ios_base::in);

	GuideParser GP;
	start = clock();
	printf("Start Guide Parser\n");
	GP.ISPD2018_guide_fileInput(readfile, design);
	end = clock();

	duration = (double)(end - start) / CLOCKS_PER_SEC;
	printf("Guide Parser Time Cost : %.2lf sec\n", duration);

	design.Lef_File = Lef_File;
	design.Def_File = Def_File;
	design.Guide_File = Guide_File;
	design.Thread_num = Thread_num;
	design.Output_File = Output_File;
	
}


void MainWriter(int argc, char** argv,Design &design){
	Def_write(argc, argv, design, design.Output_File);
}