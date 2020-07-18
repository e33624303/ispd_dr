#ifndef IROUTE_GENERATION
#define IROUTE_GENERATION
#define Use_all_Pin true
#include <vector>
#include <map>
#include <iostream>
#include <sstream>
#include <cstdio>
#include "lef_def_parser/Structure.h"
using namespace std;
using namespace Parser;
//rectangle overlap
namespace TRACKASSIGNMENT
{
    struct big_pin_index {
        int Layer;
        int id;
    };
    struct panel_index {
        int Layer;
        int id;
    };
    class IRoute
    {
    public:
        IntPair LB;
        IntPair RT;
        int Layer;
        int Net_index;
        int List_index;
        vector<big_pin_index> pins;
        bool obs;
    };
    
    class Panel
    {
    public:
        bool Direction; // 0 :ver, 1 :hor
        IntPair LB;
        IntPair RT;
        int Layer;
        vector<IRoute*> iroute_list; // pointer to (IRoute in Original_IRoute_list)
        vector<int> track;
        map<int, vector<IRoute*> > net_check;
    };
    
    class Track_Index
    {
    public:
        string dir; // X:VERTICAL Y:HORIZONTAL
        int coordinate;
        string Layer;
    };
    
    class Big_Pin
    {
    public:
        int Layer;
        int pin_index;
        FloatPair LB;
        FloatPair RT;
        vector<Track_Index> tracks;
    };
    
    class Guide_Component
    {
    public:
        int Layer;
        int guide_index;
        FloatPair LB;
        FloatPair RT;
        vector<Track_Index> tracks;
        
        vector<IRoute> Original_IRoute_list;
        // for 2-pin net construct
        vector<int> UpperLayerList;
        vector<int> SameLayerList;
        vector<int> LowerLayerList;
        bool visited;
        vector<I_Rect> up_region;
        vector<I_Rect> same_region;
        vector<I_Rect> low_region;
        
        int SubnetPointer;
    };
    
    class Routing_Net_Information
    {
    public:
        vector<vector<Big_Pin> > Layer_Array_for_Pin;
        vector<vector<Guide_Component> > Layer_Array_for_Guide;
    };
    
    class Design_Information
    {
    public:
        vector<Routing_Net_Information> routing_net_information;
    };

    int line_distance(I_Rect &rect1, I_Rect &rect2)
    {
        bool R1_vertical = (rect1.LB.first == rect1.RT.first) ? true : false;
        bool R2_vertical = (rect2.LB.first == rect2.RT.first) ? true : false;
        if (R1_vertical && R2_vertical)
        {
            bool r1_top_inside = (rect1.RT.second >= rect2.LB.second && rect1.RT.second <= rect2.RT.second);
            bool r1_bottom_inside = (rect1.LB.second >= rect2.LB.second && rect1.LB.second <= rect2.RT.second);
            if(r1_top_inside || r1_bottom_inside) // (1,1)(1,0)(0,1)
                return abs(rect1.LB.first - rect2.LB.first);
            else
            {
                // (0,0)
                int x_dis = abs(rect1.LB.first - rect2.LB.first);
                int y_dis = min(min(abs(rect1.LB.second - rect2.LB.second), abs(rect1.LB.second - rect2.RT.second)), 
                                min(abs(rect1.RT.second - rect2.LB.second), abs(rect1.RT.second - rect2.RT.second)) );
                return x_dis + y_dis;
            }
        }
        else if (!R1_vertical && R2_vertical)
        {
            bool r1_inside = (rect1.RT.second >= rect2.LB.second && rect1.RT.second <= rect2.RT.second);
            if(r1_inside)
            {
                return min(abs(rect1.LB.first - rect2.LB.first), abs(rect1.RT.first - rect2.LB.first));
            }
            else
            {
                int x_dis = min(abs(rect1.LB.first - rect2.LB.first), abs(rect1.RT.first - rect2.LB.first));
                int y_dis = min(min(abs(rect1.LB.second - rect2.LB.second), abs(rect1.LB.second - rect2.RT.second)),
                                min(abs(rect1.RT.second - rect2.LB.second), abs(rect1.RT.second - rect2.RT.second)));
                return x_dis + y_dis;
            }
        }
        else if (R1_vertical && !R2_vertical)
        {
            bool r2_inside = (rect2.RT.second >= rect1.LB.second && rect2.RT.second <= rect1.RT.second);
            if (r2_inside)
            {
                return min(abs(rect2.LB.first - rect1.LB.first), abs(rect2.RT.first - rect1.LB.first));
            }
            else
            {
                int x_dis = min(abs(rect2.LB.first - rect1.LB.first), abs(rect2.RT.first - rect1.LB.first));
                int y_dis = min(min(abs(rect2.LB.second - rect1.LB.second), abs(rect2.LB.second - rect1.RT.second)),
                                min(abs(rect2.RT.second - rect1.LB.second), abs(rect2.RT.second - rect1.RT.second)));
                return x_dis + y_dis;
            }
        }
        else if (!R1_vertical && !R2_vertical)
        {
            bool r1_right_inside = (rect1.RT.first >= rect2.LB.first && rect1.RT.first <= rect2.RT.first);
            bool r1_left_inside = (rect1.LB.first >= rect2.LB.first && rect1.LB.first <= rect2.RT.first);
            if (r1_right_inside || r1_left_inside) // (1,1)(1,0)(0,1)
                return abs(rect1.LB.second - rect2.LB.second);
            else
            {
                // (0,0)
                int y_dis = abs(rect1.LB.second - rect2.LB.second);
                int x_dis = min(min(abs(rect1.LB.first - rect2.LB.first), abs(rect1.LB.first - rect2.RT.first)),
                                min(abs(rect1.RT.first - rect2.LB.first), abs(rect1.RT.first - rect2.RT.first)));
                return x_dis + y_dis;
            }
        }

        return -1;
    }
    
    bool rec_overlap_no_abut(FloatPair lb1, FloatPair rt1, FloatPair lb2, FloatPair rt2)
    {
        if (lb1.first < lb2.first)
        {
            if (rt1.first > lb2.first)
            {
                if (lb1.second < lb2.second)
                {
                    if (rt1.second > lb2.second)
                        return 1;
                    else
                        return 0;
                }
                else
                {
                    if (rt2.second > lb1.second)
                        return 1;
                    else
                        return 0;
                }
            }
            else
                return 0;
        }
        else
        {
            if (rt2.first > lb1.first)
            {
                if (lb1.second < lb2.second)
                {
                    if (rt1.second > lb2.second)
                        return 1;
                    else
                        return 0;
                }
                else
                {
                    if (rt2.second > lb1.second)
                        return 1;
                    else
                        return 0;
                }
            }
            else
                return 0;
        }
    }
    
    bool rec_overlap(FloatPair lb1, FloatPair rt1, FloatPair lb2, FloatPair rt2)
    {
        if (lb1.first < lb2.first)
        {
            if (rt1.first >= lb2.first)
            {
                if (lb1.second < lb2.second)
                {
                    if (rt1.second >= lb2.second)
                        return 1;
                    else
                        return 0;
                }
                else
                {
                    if (rt2.second >= lb1.second)
                        return 1;
                    else
                        return 0;
                }
            }
            else
                return 0;
        }
        else
        {
            if (rt2.first >= lb1.first)
            {
                if (lb1.second < lb2.second)
                {
                    if (rt1.second >= lb2.second)
                        return 1;
                    else
                        return 0;
                }
                else
                {
                    if (rt2.second >= lb1.second)
                        return 1;
                    else
                        return 0;
                }
            }
            else
                return 0;
        }
    }
    bool rec_overlap(IntPair lb1, IntPair rt1, IntPair lb2, IntPair rt2)
    {
        if (lb1.first < lb2.first)
        {
            if (rt1.first >= lb2.first)
            {
                if (lb1.second < lb2.second)
                {
                    if (rt1.second >= lb2.second)
                        return 1;
                    else
                        return 0;
                }
                else
                {
                    if (rt2.second >= lb1.second)
                        return 1;
                    else
                        return 0;
                }
            }
            else
                return 0;
        }
        else
        {
            if (rt2.first >= lb1.first)
            {
                if (lb1.second < lb2.second)
                {
                    if (rt1.second >= lb2.second)
                        return 1;
                    else
                        return 0;
                }
                else
                {
                    if (rt2.second >= lb1.second)
                        return 1;
                    else
                        return 0;
                }
            }
            else
                return 0;
        }
    }
    bool rec_overlap(IntPair lb1, IntPair rt1, FloatPair lb2, FloatPair rt2)
    {
        if (lb1.first < lb2.first)
        {
            if (rt1.first >= lb2.first)
            {
                if (lb1.second < lb2.second)
                {
                    if (rt1.second >= lb2.second)
                        return 1;
                    else
                        return 0;
                }
                else
                {
                    if (rt2.second >= lb1.second)
                        return 1;
                    else
                        return 0;
                }
            }
            else
                return 0;
        }
        else
        {
            if (rt2.first >= lb1.first)
            {
                if (lb1.second < lb2.second)
                {
                    if (rt1.second >= lb2.second)
                        return 1;
                    else
                        return 0;
                }
                else
                {
                    if (rt2.second >= lb1.second)
                        return 1;
                    else
                        return 0;
                }
            }
            else
                return 0;
        }
    }
    bool rec_overlap(FloatPair lb1, FloatPair rt1, IntPair lb2, IntPair rt2)
    {
        if (lb1.first < lb2.first)
        {
            if (rt1.first >= lb2.first)
            {
                if (lb1.second < lb2.second)
                {
                    if (rt1.second >= lb2.second)
                        return 1;
                    else
                        return 0;
                }
                else
                {
                    if (rt2.second >= lb1.second)
                        return 1;
                    else
                        return 0;
                }
            }
            else
                return 0;
        }
        else
        {
            if (rt2.first >= lb1.first)
            {
                if (lb1.second < lb2.second)
                {
                    if (rt1.second >= lb2.second)
                        return 1;
                    else
                        return 0;
                }
                else
                {
                    if (rt2.second >= lb1.second)
                        return 1;
                    else
                        return 0;
                }
            }
            else
                return 0;
        }
    }
    
    void find_overlap_region(IntPair lb1, IntPair rt1, IntPair lb2, IntPair rt2, IntPair &newlb, IntPair &newrt)
    {
        // have overlaped
        vector<int> Xarray;
        vector<int> Yarray;
        Xarray.resize(4);
        Yarray.resize(4);
        Xarray.at(0) = lb1.first;
        Xarray.at(1) = rt1.first;
        Xarray.at(2) = lb2.first;
        Xarray.at(3) = rt2.first;
        Yarray.at(0) = lb1.second;
        Yarray.at(1) = rt1.second;
        Yarray.at(2) = lb2.second;
        Yarray.at(3) = rt2.second;
        stable_sort(Xarray.begin(), Xarray.end());
        stable_sort(Yarray.begin(), Yarray.end());
        newlb.first = Xarray.at(1);
        newlb.second = Yarray.at(1);
        newrt.first = Xarray.at(2);
        newrt.second = Yarray.at(2);
        return;
    }
    
    void build_layer_array(ISPD_Routing_Net &net, Routing_Net_Information &net_tmp, vector<Track> &track_list)
    {
        for (int i = 0; i < net.PIN_list.size(); i++)
        {
            Big_Pin pin_tmp;
            pin_tmp.LB.first = 1e9, pin_tmp.LB.second = 1e9;
            pin_tmp.RT.first = -1e9, pin_tmp.RT.second = -1e9;
           
            pin_tmp.Layer = net.PIN_list[i].IRect_list[0].Layer;

            vector<int> layerlist;
            
            for (int j = 0; j < net.PIN_list[i].IRect_list.size(); j++)
            {
                int RECTLAYER = net.PIN_list[i].IRect_list.at(j).Layer;
                bool duplicated = false;
                for(auto &layerindex : layerlist)
                {
                    if(RECTLAYER == layerindex)
                    {
                        duplicated = true;
                        break;
                    }
                }
                if(!duplicated)
                {
                    layerlist.push_back(RECTLAYER);
                    stable_sort(layerlist.begin(), layerlist.end());
                }
                
                if (net.PIN_list[i].IRect_list[j].LB.first < pin_tmp.LB.first)
                    pin_tmp.LB.first = net.PIN_list[i].IRect_list[j].LB.first;
                if (net.PIN_list[i].IRect_list[j].LB.second < pin_tmp.LB.second)
                    pin_tmp.LB.second = net.PIN_list[i].IRect_list[j].LB.second;
                if (net.PIN_list[i].IRect_list[j].RT.first > pin_tmp.RT.first)
                    pin_tmp.RT.first = net.PIN_list[i].IRect_list[j].RT.first;
                if (net.PIN_list[i].IRect_list[j].RT.second > pin_tmp.RT.second)
                    pin_tmp.RT.second = net.PIN_list[i].IRect_list[j].RT.second;
            }
            pin_tmp.pin_index = i;
            string pin_tmp_dir = (pin_tmp.Layer % 2 == 1) ? "Y" : "X";
            string pin_tmp_layer = "Metal";
            stringstream ss;
            ss << pin_tmp.Layer;
            string layer_id = ss.str();
            pin_tmp_layer += layer_id;
            for (int j = 0; j < track_list.size(); j++)
            {
                if (track_list[j].Dir == pin_tmp_dir && track_list[j].Layer == pin_tmp_layer)
                {
                    Track_Index tindx;
                    tindx.dir = pin_tmp_dir;
                    tindx.Layer = pin_tmp_layer;
                    float top, bot;
                    if (tindx.dir == "X")
                    {
                        top = pin_tmp.RT.first;
                        bot = pin_tmp.LB.first;
                    }
                    else
                    {
                        top = pin_tmp.RT.second;
                        bot = pin_tmp.LB.second;
                    }
                    int cnt = 0;
                    for (int k = track_list[j].start_index; k < top && cnt < track_list[j].Do; k += track_list[j].Step)
                    {
                        if (k >= bot)
                        {
                            tindx.coordinate = k;
                            pin_tmp.tracks.push_back(tindx);
                        }
                        cnt++;
                    }
                    break;
                }
            }
            
            
            for (auto &layerindex : layerlist)
            {
                pin_tmp.Layer = layerindex;
                net_tmp.Layer_Array_for_Pin[pin_tmp.Layer].push_back(pin_tmp);
            }
        }
        for (int i = 0; i < net.GUIDE_list.size(); i++)
        {
            Guide_Component guide_tmp;
            guide_tmp.Layer = net.GUIDE_list[i].Layer;


            guide_tmp.LB.first = net.GUIDE_list[i].LB.first;
            guide_tmp.LB.second = net.GUIDE_list[i].LB.second;
            guide_tmp.RT.first = net.GUIDE_list[i].RT.first;
            guide_tmp.RT.second = net.GUIDE_list[i].RT.second;
            guide_tmp.guide_index = i;
            string guide_tmp_dir = (guide_tmp.Layer % 2 == 1) ? "Y" : "X";
            string guide_tmp_layer = "Metal";
            stringstream ss;
            ss << guide_tmp.Layer;
            string layer_id = ss.str();
            guide_tmp_layer += layer_id;
            for (int j = 0; j < track_list.size(); j++)
            {
                if (track_list[j].Dir == guide_tmp_dir && track_list[j].Layer == guide_tmp_layer)
                {
                    Track_Index tindx;
                    tindx.dir = guide_tmp_dir;
                    tindx.Layer = guide_tmp_layer;
                    float top, bot;
                    if (tindx.dir == "X")
                    {
                        top = guide_tmp.RT.first;
                        bot = guide_tmp.LB.first;
                    }
                    else
                    {
                        top = guide_tmp.RT.second;
                        bot = guide_tmp.LB.second;
                    }
                    int cnt = 0;
                    for (int k = track_list[j].start_index; k < top && cnt < track_list[j].Do; k += track_list[j].Step)
                    {
                        if (k >= bot)
                        {
                            tindx.coordinate = k;
                            guide_tmp.tracks.push_back(tindx);
                        }
                        cnt++;
                    }
                    break;
                }
            }
            net_tmp.Layer_Array_for_Guide[guide_tmp.Layer].push_back(guide_tmp);
        }
    }
    
    
    
    class IRouteGenerator // TODO
    {
    public:
        Design_Information design_information;
        // for ILP input
        vector<vector<Panel> > Hor_Panel_list; //first index: layer, second:
        
        vector<vector<Panel> > Ver_Panel_list;
        
        vector<IRoute> Obs_iroute;
        
        map<int, panel_index> panel_map;
        
        IRouteGenerator()
        {
            ;
        }
        
        void build_whole_design(Design &design)
        {
            
            this->design_information.routing_net_information.resize(design.ispd_routing_net.size());

            for (int i = 0; i < design.ispd_routing_net.size(); i++)
            {
                this->design_information.routing_net_information[i].Layer_Array_for_Pin.resize(design.Metal_Layer_Num + 1);
                this->design_information.routing_net_information[i].Layer_Array_for_Guide.resize(design.Metal_Layer_Num + 1);
                build_layer_array(design.ispd_routing_net[i], this->design_information.routing_net_information[i], design.Track_list);
            }
            
            // DEBUG
            /*
             for (int i = 0; i < this->design_information.routing_net_information.size(); i++)
             {
             cout << "---------net:  " << design.ispd_routing_net[i].Original_Data.Net_name << "--------\n\n";
             for (int j = 0; j < this->design_information.routing_net_information[i].Layer_Array_for_Guide.size(); j++)
             {
             printf("Layer %d : \n\n", j);
             printf("Guide:\n\n");
             for (int k = 0; k < this->design_information.routing_net_information[i].Layer_Array_for_Guide[j].size(); k++)
             {
             printf("Guide LB (%f, %f), RT (%f, %f)\n", this->design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].LB.first, this->design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].LB.second, this->design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].RT.first, this->design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].RT.second);
             printf("Guide Layer = %d, Guide id = %d\n", this->design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Layer, this->design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].guide_index);
             printf("Tracks:\n");
             for (int l = 0; l < this->design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].tracks.size(); l++)
             cout << "Track dir = " << this->design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].tracks[l].dir << " , Layer = " << this->design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].tracks[l].Layer << ", coordinate = " << this->design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].tracks[l].coordinate << endl;
             }
             printf("\n\n\nPin:\n\n");
             for (int k = 0; k < this->design_information.routing_net_information[i].Layer_Array_for_Pin[j].size(); k++)
             {
             printf("Pin LB (%f, %f), RT (%f, %f)\n", this->design_information.routing_net_information[i].Layer_Array_for_Pin[j][k].LB.first, this->design_information.routing_net_information[i].Layer_Array_for_Pin[j][k].LB.second, this->design_information.routing_net_information[i].Layer_Array_for_Pin[j][k].RT.first, this->design_information.routing_net_information[i].Layer_Array_for_Pin[j][k].RT.second);
             printf("Pin Layer = %d, Pin id = %d\n", this->design_information.routing_net_information[i].Layer_Array_for_Pin[j][k].Layer, this->design_information.routing_net_information[i].Layer_Array_for_Pin[j][k].pin_index);
             printf("Tracks:\n");
             for (int l = 0; l < this->design_information.routing_net_information[i].Layer_Array_for_Pin[j][k].tracks.size(); l++)
             cout << "Track dir = " << this->design_information.routing_net_information[i].Layer_Array_for_Pin[j][k].tracks[l].dir << " , Layer = " << this->design_information.routing_net_information[i].Layer_Array_for_Pin[j][k].tracks[l].Layer << ", coordinate = " << this->design_information.routing_net_information[i].Layer_Array_for_Pin[j][k].tracks[l].coordinate << endl;
             }
             
             }
             }
             */
        }
        
        void InitialPanelList(Design& design, bool &L3_disable)
        {
            int start_layer = 3;
            if(L3_disable)
            {
                start_layer = 4;
            }
            Hor_Panel_list.resize(design.Metal_Layer_Num+1);
            Ver_Panel_list.resize(design.Metal_Layer_Num+1);
            int zerocnt = 0, iroute_cnt = 0;
            int bothzerocnt = 0;
            int bigpoint = 0;
            int layerviolation = 0;
            int notmultiplegcell = 0;
            int unitpoint = 0;
            int total = 0;
            int small_h = 0, small_v = 0, small = 0;
            for (int i = 0; i < design_information.routing_net_information.size(); i++) {
                for (int j = start_layer; j < design_information.routing_net_information[i].Layer_Array_for_Guide.size(); j++) {
                    total += design_information.routing_net_information[i].Layer_Array_for_Guide[j].size();
                    for (int k = 0; k < design_information.routing_net_information[i].Layer_Array_for_Guide[j].size(); k++) {
                        Guide_Component guide_tmp = design_information.routing_net_information[i].Layer_Array_for_Guide[j][k];
                        vector<float> horizontal_breakpoint;
                        vector<float> vertical_breakpoint;
                        int hstart = (int)guide_tmp.LB.first;
                        for (int l = hstart; l <= guide_tmp.RT.first; l += design.GCell_Width)
                            horizontal_breakpoint.push_back(l);
                        
                        int vstart = (int)guide_tmp.LB.second;
                        for (int l = vstart; l <= guide_tmp.RT.second; l += design.GCell_Height)
                            vertical_breakpoint.push_back(l);
                        
                        if (horizontal_breakpoint.size() < 2 && (horizontal_breakpoint[horizontal_breakpoint.size()-1]+design.GCell_Width < design.Die_Area.RT.first)) small_h++;
                        if (vertical_breakpoint.size() < 2) small_v++;
                        if (horizontal_breakpoint.size() < 2 && vertical_breakpoint.size() < 2) small++;
                        if (horizontal_breakpoint.size() > vertical_breakpoint.size()) {
                            if (vertical_breakpoint.size() > 2) {
                                bigpoint++;
                            }
                            else if (horizontal_breakpoint.size() >= 4) {
                                if (((int)(guide_tmp.RT.first - guide_tmp.LB.first))%design.GCell_Width != 0) notmultiplegcell++;
                                if (design.Layer_list[(guide_tmp.Layer-1)*2].metalLayer.DIRECTION == false) {
                                    layerviolation++;
                                    continue;
                                }
                                iroute_cnt++;
                                IRoute iroute_tmp;
                                iroute_tmp.obs = 0;
                                iroute_tmp.List_index = -1;
                                iroute_tmp.Layer = guide_tmp.Layer;
                                iroute_tmp.Net_index = i;
                                iroute_tmp.LB.first = horizontal_breakpoint[1];
                                iroute_tmp.RT.first = horizontal_breakpoint[horizontal_breakpoint.size()-2];
                                for (int l = 0; l < design_information.routing_net_information[i].Layer_Array_for_Pin[1].size(); l++) {
                                    if (Use_all_Pin == false) {
                                        if (rec_overlap(guide_tmp.LB, guide_tmp.RT, design_information.routing_net_information[i].Layer_Array_for_Pin[1][l].LB, design_information.routing_net_information[i].Layer_Array_for_Pin[1][l].RT)) {
                                            big_pin_index ptmp;
                                            ptmp.Layer = 1;
                                            ptmp.id = l;
                                            iroute_tmp.pins.push_back(ptmp);
                                        }
                                    }
                                    else {
                                        big_pin_index ptmp;
                                        ptmp.Layer = 1;
                                        ptmp.id = l;
                                        iroute_tmp.pins.push_back(ptmp);
                                    }
                                }
                                int map_index = 1 | ((guide_tmp.Layer&0x1F) << 1) | ((vstart / design.GCell_Height) << 6);
                                if (panel_map.find(map_index) != panel_map.end()) {
                                    panel_index ptmp = panel_map.find(map_index)->second;
                                    iroute_tmp.LB.second = Hor_Panel_list[ptmp.Layer][ptmp.id].track[(Hor_Panel_list[ptmp.Layer][ptmp.id].track.size())/2];
                                    iroute_tmp.RT.second = iroute_tmp.LB.second;
                                    design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list.push_back(iroute_tmp);
                                    Hor_Panel_list[ptmp.Layer][ptmp.id].iroute_list.push_back(&(design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list[0]));
                                    if (Hor_Panel_list[ptmp.Layer][ptmp.id].net_check.find(iroute_tmp.Net_index) != Hor_Panel_list[ptmp.Layer][ptmp.id].net_check.end()) {
                                        Hor_Panel_list[ptmp.Layer][ptmp.id].net_check[iroute_tmp.Net_index].push_back(&(design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list[0]));
                                    }
                                    else {
                                        vector<IRoute*> vtmp;
                                        vtmp.push_back(&(design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list[0]));
                                        Hor_Panel_list[ptmp.Layer][ptmp.id].net_check[iroute_tmp.Net_index] = vtmp;
                                    }
                                    /*if ((Hor_Panel_list[ptmp.Layer][ptmp.id].LB.second == 313500 || Hor_Panel_list[ptmp.Layer][ptmp.id].RT.second == 313500) && (ptmp.Layer == 3)) {
                                     cout << "Panel: LB.first = " << Hor_Panel_list[ptmp.Layer][ptmp.id].LB.first << ", LB.second = " << Hor_Panel_list[ptmp.Layer][ptmp.id].LB.second << endl;
                                     cout << "       RT.first = " << Hor_Panel_list[ptmp.Layer][ptmp.id].RT.first << ", RT.second = " << Hor_Panel_list[ptmp.Layer][ptmp.id].RT.second << endl;
                                     for (int ii = 0; ii < Hor_Panel_list[ptmp.Layer][ptmp.id].track.size(); ii++)
                                     cout << "Track " << ii << " = " << Hor_Panel_list[ptmp.Layer][ptmp.id].track[ii] << endl;
                                     cout << "Guide: LB.first = " << guide_tmp.LB.first << ", LB.second = " << guide_tmp.LB.second << endl;
                                     cout << "       RT.first = " << guide_tmp.RT.first << ", RT.second = " << guide_tmp.RT.second << endl;
                                     int isize = Hor_Panel_list[ptmp.Layer][ptmp.id].iroute_list.size()-1;
                                     cout << "Iroute: LB.first = " << Hor_Panel_list[ptmp.Layer][ptmp.id].iroute_list[isize]->LB.first << ", LB.second = " << Hor_Panel_list[ptmp.Layer][ptmp.id].iroute_list[isize]->LB.second << endl;
                                     cout << "        RT.first = " << Hor_Panel_list[ptmp.Layer][ptmp.id].iroute_list[isize]->RT.first << ", RT.second = " << Hor_Panel_list[ptmp.Layer][ptmp.id].iroute_list[isize]->RT.second << endl;
                                     string pause_;
                                     cin >> pause_;
                                     }*/
                                }
                                else {
                                    Panel panel_tmp;
                                    panel_tmp.Direction = 1;
                                    panel_tmp.LB.first = 0;
                                    panel_tmp.RT.first = design.Die_Area.RT.first;
                                    panel_tmp.LB.second = vstart;
                                    panel_tmp.RT.second = vstart + design.GCell_Height;
                                    if (panel_tmp.RT.second > design.Die_Area.RT.second)
                                        panel_tmp.RT.second = design.Die_Area.RT.second;
                                    panel_tmp.Layer = guide_tmp.Layer;
                                    Hor_Panel_list[panel_tmp.Layer].push_back(panel_tmp);
                                    string tmp_dir = "Y";
                                    string tmp_layer = "Metal";
                                    stringstream ss;
                                    ss << guide_tmp.Layer;
                                    string layer_id = ss.str();
                                    tmp_layer += layer_id;
                                    for (int l = 0; l < design.Track_list.size(); l++) {
                                        int cnt = 0;
                                        if (design.Track_list[l].Dir == tmp_dir && design.Track_list[l].Layer == tmp_layer) {
                                            for (int ll = design.Track_list[l].start_index; ll < panel_tmp.RT.second && cnt < design.Track_list[l].Do; ll+= design.Track_list[l].Step)
                                            {
                                                if (ll >= panel_tmp.LB.second)
                                                    Hor_Panel_list[panel_tmp.Layer][Hor_Panel_list[panel_tmp.Layer].size()-1].track.push_back(ll);
                                                cnt++;
                                            }
                                            break;
                                        }
                                    }
                                    panel_index ptmp2;
                                    ptmp2.Layer = panel_tmp.Layer;
                                    ptmp2.id = Hor_Panel_list[panel_tmp.Layer].size()-1;
                                    panel_map[map_index] = ptmp2;
                                    
                                    panel_index ptmp = panel_map.find(map_index)->second;
                                    iroute_tmp.LB.second = Hor_Panel_list[ptmp.Layer][ptmp.id].track[(Hor_Panel_list[ptmp.Layer][ptmp.id].track.size())/2];
                                    iroute_tmp.RT.second = iroute_tmp.LB.second;
                                    design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list.push_back(iroute_tmp);
                                    Hor_Panel_list[ptmp.Layer][ptmp.id].iroute_list.push_back(&(design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list[0]));
                                    if (Hor_Panel_list[ptmp.Layer][ptmp.id].net_check.find(iroute_tmp.Net_index) != Hor_Panel_list[ptmp.Layer][ptmp.id].net_check.end()) {
                                        Hor_Panel_list[ptmp.Layer][ptmp.id].net_check[iroute_tmp.Net_index].push_back(&(design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list[0]));
                                    }
                                    else {
                                        vector<IRoute*> vtmp;
                                        vtmp.push_back(&(design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list[0]));
                                        Hor_Panel_list[ptmp.Layer][ptmp.id].net_check[iroute_tmp.Net_index] = vtmp;
                                    }
                                    /*if ((Hor_Panel_list[ptmp.Layer][ptmp.id].LB.second == 313500 || Hor_Panel_list[ptmp.Layer][ptmp.id].RT.second == 313500) && (ptmp.Layer == 3)) {
                                     cout << "Panel: LB.first = " << Hor_Panel_list[ptmp.Layer][ptmp.id].LB.first << ", LB.second = " << Hor_Panel_list[ptmp.Layer][ptmp.id].LB.second << endl;
                                     cout << "       RT.first = " << Hor_Panel_list[ptmp.Layer][ptmp.id].RT.first << ", RT.second = " << Hor_Panel_list[ptmp.Layer][ptmp.id].RT.second << endl;
                                     for (int ii = 0; ii < Hor_Panel_list[ptmp.Layer][ptmp.id].track.size(); ii++)
                                     cout << "Track " << ii << " = " << Hor_Panel_list[ptmp.Layer][ptmp.id].track[ii] << endl;
                                     cout << "Guide: LB.first = " << guide_tmp.LB.first << ", LB.second = " << guide_tmp.LB.second << endl;
                                     cout << "       RT.first = " << guide_tmp.RT.first << ", RT.second = " << guide_tmp.RT.second << endl;
                                     int isize = Hor_Panel_list[ptmp.Layer][ptmp.id].iroute_list.size()-1;
                                     cout << "Iroute: LB.first = " << Hor_Panel_list[ptmp.Layer][ptmp.id].iroute_list[isize]->LB.first << ", LB.second = " << Hor_Panel_list[ptmp.Layer][ptmp.id].iroute_list[isize]->LB.second << endl;
                                     cout << "        RT.first = " << Hor_Panel_list[ptmp.Layer][ptmp.id].iroute_list[isize]->RT.first << ", RT.second = " << Hor_Panel_list[ptmp.Layer][ptmp.id].iroute_list[isize]->RT.second << endl;
                                     string pause_;
                                     cin >> pause_;
                                     }*/
                                }
                            }
                        }
                        else if (horizontal_breakpoint.size() == vertical_breakpoint.size()) {
                            if (horizontal_breakpoint.size() > 2) bigpoint++;
                            else unitpoint++;
                        }
                        else {
                            if (horizontal_breakpoint.size() > 2) bigpoint++;
                            else if (vertical_breakpoint.size() >= 4) {
                                iroute_cnt++;
                                if (((int)(guide_tmp.RT.second - guide_tmp.LB.second))%design.GCell_Height != 0) notmultiplegcell++;
                                if (design.Layer_list[(guide_tmp.Layer-1)*2].metalLayer.DIRECTION == true) {
                                    layerviolation++;
                                    continue;
                                }
                                IRoute iroute_tmp;
                                iroute_tmp.obs = 0;
                                iroute_tmp.List_index = -1;
                                iroute_tmp.Layer = guide_tmp.Layer;
                                iroute_tmp.Net_index = i;
                                iroute_tmp.LB.second = vertical_breakpoint[1];
                                iroute_tmp.RT.second = vertical_breakpoint[vertical_breakpoint.size()-2];
                                for (int l = 0; l < design_information.routing_net_information[i].Layer_Array_for_Pin[1].size(); l++) {
                                    if (Use_all_Pin == false) {
                                        if (rec_overlap(guide_tmp.LB, guide_tmp.RT, design_information.routing_net_information[i].Layer_Array_for_Pin[1][l].LB, design_information.routing_net_information[i].Layer_Array_for_Pin[1][l].RT)) {
                                            big_pin_index ptmp;
                                            ptmp.Layer = 1;
                                            ptmp.id = l;
                                            iroute_tmp.pins.push_back(ptmp);
                                        }
                                    }
                                    else {
                                        big_pin_index ptmp;
                                        ptmp.Layer = 1;
                                        ptmp.id = l;
                                        iroute_tmp.pins.push_back(ptmp);
                                    }
                                }
                                int map_index = 0 | ((guide_tmp.Layer&0x1F) << 1) | ((hstart / design.GCell_Width) << 6);
                                if (panel_map.find(map_index) != panel_map.end()) {
                                    panel_index ptmp = panel_map.find(map_index)->second;
                                    iroute_tmp.LB.first = Ver_Panel_list[ptmp.Layer][ptmp.id].track[(Ver_Panel_list[ptmp.Layer][ptmp.id].track.size())/2];
                                    iroute_tmp.RT.first = iroute_tmp.LB.first;
                                    design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list.push_back(iroute_tmp);
                                    Ver_Panel_list[ptmp.Layer][ptmp.id].iroute_list.push_back(&(design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list[design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list.size()-1]));
                                    if (Ver_Panel_list[ptmp.Layer][ptmp.id].net_check.find(iroute_tmp.Net_index) != Ver_Panel_list[ptmp.Layer][ptmp.id].net_check.end()) {
                                        Ver_Panel_list[ptmp.Layer][ptmp.id].net_check[iroute_tmp.Net_index].push_back(&(design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list[0]));
                                    }
                                    else {
                                        vector<IRoute*> vtmp;
                                        vtmp.push_back(&(design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list[0]));
                                        Ver_Panel_list[ptmp.Layer][ptmp.id].net_check[iroute_tmp.Net_index] = vtmp;
                                    }
                                }
                                else {
                                    Panel panel_tmp;
                                    panel_tmp.Direction = 0;
                                    panel_tmp.LB.second = 0;
                                    panel_tmp.RT.second = design.Die_Area.RT.second;
                                    panel_tmp.LB.first = hstart;
                                    panel_tmp.RT.first = hstart + design.GCell_Width;
                                    if (panel_tmp.RT.first > design.Die_Area.RT.first)
                                        panel_tmp.RT.first = design.Die_Area.RT.first;
                                    panel_tmp.Layer = guide_tmp.Layer;
                                    Ver_Panel_list[panel_tmp.Layer].push_back(panel_tmp);
                                    string tmp_dir = "X";
                                    string tmp_layer = "Metal";
                                    stringstream ss;
                                    ss << guide_tmp.Layer;
                                    string layer_id = ss.str();
                                    tmp_layer += layer_id;
                                    for (int l = 0; l < design.Track_list.size(); l++) {
                                        int cnt = 0;
                                        if (design.Track_list[l].Dir == tmp_dir && design.Track_list[l].Layer == tmp_layer) {
                                            for (int ll = design.Track_list[l].start_index; ll < panel_tmp.RT.first && cnt < design.Track_list[l].Do; ll += design.Track_list[l].Step)
                                            {
                                                if (ll >= panel_tmp.LB.first)
                                                    Ver_Panel_list[panel_tmp.Layer][Ver_Panel_list[panel_tmp.Layer].size()-1].track.push_back(ll);
                                                cnt++;
                                            }
                                            break;
                                        }
                                    }
                                    panel_index ptmp2;
                                    ptmp2.Layer = panel_tmp.Layer;
                                    ptmp2.id = Ver_Panel_list[panel_tmp.Layer].size()-1;
                                    panel_map[map_index] = ptmp2;
                                    
                                    panel_index ptmp = panel_map.find(map_index)->second;
                                    iroute_tmp.LB.first = Ver_Panel_list[ptmp.Layer][ptmp.id].track[(Ver_Panel_list[ptmp.Layer][ptmp.id].track.size())/2];
                                    iroute_tmp.RT.first = iroute_tmp.LB.first;
                                    design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list.push_back(iroute_tmp);
                                    Ver_Panel_list[ptmp.Layer][ptmp.id].iroute_list.push_back(&(design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list[design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list.size()-1]));
                                    if (Ver_Panel_list[ptmp.Layer][ptmp.id].net_check.find(iroute_tmp.Net_index) != Ver_Panel_list[ptmp.Layer][ptmp.id].net_check.end()) {
                                        Ver_Panel_list[ptmp.Layer][ptmp.id].net_check[iroute_tmp.Net_index].push_back(&(design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list[0]));
                                    }
                                    else {
                                        vector<IRoute*> vtmp;
                                        vtmp.push_back(&(design_information.routing_net_information[i].Layer_Array_for_Guide[j][k].Original_IRoute_list[0]));
                                        Ver_Panel_list[ptmp.Layer][ptmp.id].net_check[iroute_tmp.Net_index] = vtmp;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            // build obstacle iroutes
            for (int i = 0; i < design.AllOBSs.size(); i++) {
                int obs_layer = design.AllOBSs[i].Layer;
                /*printf("\nObs[%d]: Layer = %d, LB.first = %d, LB.second = %d\n", i, obs_layer, design.AllOBSs[i].LB.first, design.AllOBSs[i].LB.second);
                 printf("                     RT.first = %d, RT.second = %d\n", design.AllOBSs[i].RT.first, design.AllOBSs[i].RT.second);
                 printf("Obs Iroute: \n\n");*/
                if (obs_layer % 2 == 1) {
                    stringstream ss;
                    string tmp_dir = "Y";
                    string tmp_layer = "Metal";
                    ss << obs_layer;
                    string layer_id = ss.str();
                    tmp_layer += layer_id;
                    for (int l = 0; l < design.Track_list.size(); l++) {
                        int cnt = 0;
                        if (design.Track_list[l].Dir == tmp_dir && design.Track_list[l].Layer == tmp_layer) {
                            for (int ll = design.Track_list[l].start_index; ll < design.AllOBSs[i].RT.second && cnt < design.Track_list[l].Do; ll+= design.Track_list[l].Step)  {
                                if (ll >= design.AllOBSs[i].LB.second) {
                                    IRoute iroute_tmp;
                                    iroute_tmp.Layer = obs_layer;
                                    iroute_tmp.LB.first = design.AllOBSs[i].LB.first;
                                    iroute_tmp.LB.second = ll;
                                    iroute_tmp.RT.first = design.AllOBSs[i].RT.first;
                                    iroute_tmp.RT.second = ll;
                                    iroute_tmp.obs = 1;
                                    Obs_iroute.push_back(iroute_tmp);
                                }
                                cnt++;
                            }
                            break;
                        }
                    }
                }
                else {
                    stringstream ss;
                    string tmp_dir = "X";
                    string tmp_layer = "Metal";
                    ss << obs_layer;
                    string layer_id = ss.str();
                    tmp_layer += layer_id;
                    for (int l = 0; l < design.Track_list.size(); l++) {
                        int cnt = 0;
                        if (design.Track_list[l].Dir == tmp_dir && design.Track_list[l].Layer == tmp_layer) {
                            for (int ll = design.Track_list[l].start_index; ll < design.AllOBSs[i].RT.first && cnt < design.Track_list[l].Do; ll+= design.Track_list[l].Step)  {
                                if (ll >= design.AllOBSs[i].LB.first) {
                                    IRoute iroute_tmp;
                                    iroute_tmp.Layer = obs_layer;
                                    iroute_tmp.LB.first = ll;
                                    iroute_tmp.LB.second = design.AllOBSs[i].LB.second;
                                    iroute_tmp.RT.first = ll;
                                    iroute_tmp.RT.second = design.AllOBSs[i].RT.second;
                                    iroute_tmp.obs = 1;
                                    Obs_iroute.push_back(iroute_tmp);
                                }
                                cnt++;
                            }
                            break;
                        }
                    }
                }
            }
            int panel_not_found = 0;
            for (int i = 0; i < Obs_iroute.size(); i++) {
                int obs_layer = Obs_iroute[i].Layer;
                bool find_panel = 0;
                if (obs_layer % 2 == 0) {
                    for (int j = 0; j < Ver_Panel_list[obs_layer].size(); j++)
                        if (rec_overlap(Ver_Panel_list[obs_layer][j].LB, Ver_Panel_list[obs_layer][j].RT, Obs_iroute[i].LB, Obs_iroute[i].RT)) {
                            Ver_Panel_list[obs_layer][j].iroute_list.push_back(&(Obs_iroute[i]));
                            find_panel = 1;
                            break;
                        }
                }
                else {
                    for (int j = 0; j < Hor_Panel_list[obs_layer].size(); j++)
                        if (rec_overlap(Hor_Panel_list[obs_layer][j].LB, Hor_Panel_list[obs_layer][j].RT, Obs_iroute[i].LB, Obs_iroute[i].RT)) {
                            Hor_Panel_list[obs_layer][j].iroute_list.push_back(&(Obs_iroute[i]));
                            find_panel = 1;
                            break;
                        }
                }
                if (find_panel == 0) panel_not_found++;
            }
            //debug
            cout << "-----iroute = " << iroute_cnt << "-----" << endl;
            cout << "-----bigpoint = " << bigpoint << "-----" << endl;
            cout << "-----unitpoint = " << unitpoint << "-----" << endl;
            cout << "-----bothzerocnt = " << bothzerocnt << "-----" << endl;;
            cout << "-----zerocnt = " << zerocnt << "-----" << endl;
            cout << "-----layerviolation = " << layerviolation << "-----" << endl;
            cout << "-----notmultiplegcell = " << notmultiplegcell << "-----" << endl;
            cout << "-----total = " << total << "-----" << endl;
            cout << "-----small_h = " << small_h << "-----" << endl;
            cout << "-----small_v = " << small_v << "-----" << endl;
            cout << "-----small = " << small << "-----" << endl;
            bool stop = 0;
            /*cout << "Horizontal Panels:\n\n";
             for (int i = 0; i < Hor_Panel_list.size(); i++) {
             cout << "Layer " << i << ": \n\n";
             for (int j = 0; j < Hor_Panel_list[i].size(); j++) {
             cout << "-----Panel" << j << endl;
             cout << "Panel Dir : " << Hor_Panel_list[i][j].Direction;
             cout << "Panel LB : (" << Hor_Panel_list[i][j].LB.first << ", " << Hor_Panel_list[i][j].LB.second << ")\n";
             cout << "Panel RT : (" << Hor_Panel_list[i][j].RT.first << ", " << Hor_Panel_list[i][j].RT.second << ")\n";
             cout << "Panel track num : " << Hor_Panel_list[i][j].track.size() << endl;
             cout << "Panel iroute cnt : " << Hor_Panel_list[i][j].iroute_list.size() << endl;
             for (int k = 0; k < Hor_Panel_list[i][j].iroute_list.size(); k++) {
             cout << "iroute Net_index : " << Hor_Panel_list[i][j].iroute_list[k]->Net_index << endl;
             cout << "iroute LB : " << Hor_Panel_list[i][j].iroute_list[k]->LB.first << ", RT : " << Hor_Panel_list[i][j].iroute_list[k]->RT.first;
             if (Hor_Panel_list[i][j].iroute_list[k]->Layer != i) {
             cout << "  Layer error!!\n";
             stop = 1;
             break;
             }
             }
             if (stop) break;
             }
             if (stop) break;
             }*//*
                 cout << "\n\nVertical Panels:\n\n";
                 for (int i = 0; i < Ver_Panel_list.size(); i++) {
                 cout << "Layer " << i << ": \n\n";
                 for (int j = 0; j < Ver_Panel_list[i].size(); j++) {
                 cout << "-----Panel" << j << endl;
                 cout << "Panel Dir : " << Ver_Panel_list[i][j].Direction;
                 cout << "Panel LB : (" << Ver_Panel_list[i][j].LB.first << ", " << Ver_Panel_list[i][j].LB.second << ")\n";
                 cout << "Panel RT : (" << Ver_Panel_list[i][j].RT.first << ", " << Ver_Panel_list[i][j].RT.second << ")\n";
                 cout << "Panel track num : " << Ver_Panel_list[i][j].track.size() << endl;
                 cout << "Panel iroute cnt : " << Ver_Panel_list[i][j].iroute_list.size() << endl;
                 for (int k = 0; k < Ver_Panel_list[i][j].iroute_list.size(); k++) {
                 cout << "iroute pin cnt : " << Ver_Panel_list[i][j].iroute_list[k]->pins.size() << endl;
                 if (Ver_Panel_list[i][j].iroute_list[k]->Layer != i) {
                 cout << "  Layer error!!\n";
                 stop = 1;
                 break;
                 }
                 }
                 if (stop) break;
                 }
                 if (stop) break;
                 }
                 */
            // Add panel to panel list
            // these panels lb rt must abut on GCell
        }
        
        
        void BuildGuideConnection()
        {
            for (int i = this->design_information.routing_net_information.size()-1; i >= 0 ; i--)
            {
                int current_net_index = i;
                //printf("BuildGuideConnection::Net Index(%d)==================\n", current_net_index);
                for (int j = 0; j < this->design_information.routing_net_information[i].Layer_Array_for_Guide.size(); j++)
                {
                    for (int k = 0; k < this->design_information.routing_net_information[i].Layer_Array_for_Guide[j].size(); k++)
                    {
                        Guide_Component &guide = this->design_information.routing_net_information[i].Layer_Array_for_Guide[j].at(k);
                        /*
                         printf("BuildGuideConnection::Guide LB (%f, %f), RT (%f, %f) Layer = %d, Guide id = %d\n",
                         guide.LB.first, guide.LB.second, guide.RT.first, guide.RT.second, guide.Layer, guide.guide_index);
                         */
                        guide.visited = false;
                        
                        int current_guide_layer = guide.Layer;
                        int upper_guide_layer = guide.Layer + 1;
                        int lower_guide_layer = guide.Layer - 1;
                        int UpCounter = 0, LowCounter = 0, SameCounter = 0;
                        for (int same_index = 0; same_index < this->design_information.routing_net_information[i].Layer_Array_for_Guide[current_guide_layer].size(); same_index++)
                        {
                            TRACKASSIGNMENT::Guide_Component &temp_guide = this->design_information.routing_net_information[i].Layer_Array_for_Guide[current_guide_layer].at(same_index);
                            if (TRACKASSIGNMENT::rec_overlap(temp_guide.LB, temp_guide.RT, guide.LB, guide.RT))
                            {
                                SameCounter++;
                                /*
                                 printf("BuildGuideConnection::SameOverlap Guide LB (%f, %f), RT (%f, %f) Layer = %d, Guide id = %d\n",
                                 temp_guide.LB.first, temp_guide.LB.second, temp_guide.RT.first, temp_guide.RT.second, temp_guide.Layer, temp_guide.guide_index);
                                 */
                                // find the overlap region
                                I_Rect overRect;
                                TRACKASSIGNMENT::find_overlap_region(guide.LB, guide.RT, temp_guide.LB, temp_guide.RT, overRect.LB, overRect.RT);
                                //printf("BuildGuideConnection::Overlap Region LB (%d, %d), RT (%d, %d)\n", overRect.LB.first, overRect.LB.second, overRect.RT.first, overRect.RT.second);
                                guide.same_region.push_back(overRect);
                                guide.SameLayerList.push_back(same_index);
                            }
                        }
                        
                        if (upper_guide_layer < this->design_information.routing_net_information[i].Layer_Array_for_Guide.size())
                        {
                            for (int upper_index = 0; upper_index < this->design_information.routing_net_information[i].Layer_Array_for_Guide[upper_guide_layer].size(); upper_index++)
                            {
                                TRACKASSIGNMENT::Guide_Component &temp_guide = this->design_information.routing_net_information[i].Layer_Array_for_Guide[upper_guide_layer].at(upper_index);
                                if (TRACKASSIGNMENT::rec_overlap(temp_guide.LB, temp_guide.RT, guide.LB, guide.RT))
                                {
                                    UpCounter++;
                                    /*
                                     printf("BuildGuideConnection::UpperOverlap Guide LB (%f, %f), RT (%f, %f) Layer = %d, Guide id = %d\n",
                                     temp_guide.LB.first, temp_guide.LB.second, temp_guide.RT.first, temp_guide.RT.second, temp_guide.Layer, temp_guide.guide_index);
                                     */
                                    // find the overlap region
                                    I_Rect overRect;
                                    TRACKASSIGNMENT::find_overlap_region(guide.LB, guide.RT, temp_guide.LB, temp_guide.RT, overRect.LB, overRect.RT);
                                    //printf("BuildGuideConnection::Overlap Region LB (%d, %d), RT (%d, %d)\n", overRect.LB.first, overRect.LB.second, overRect.RT.first, overRect.RT.second);
                                    guide.up_region.push_back(overRect);
                                    guide.UpperLayerList.push_back(upper_index);
                                }
                            }
                        }
                        if (lower_guide_layer >= 0)
                        {
                            for (int lower_index = 0; lower_index < this->design_information.routing_net_information[i].Layer_Array_for_Guide[lower_guide_layer].size(); lower_index++)
                            {
                                TRACKASSIGNMENT::Guide_Component &temp_guide = this->design_information.routing_net_information[i].Layer_Array_for_Guide[lower_guide_layer].at(lower_index);
                                if (TRACKASSIGNMENT::rec_overlap(temp_guide.LB, temp_guide.RT, guide.LB, guide.RT))
                                {
                                    LowCounter++;
                                    /*
                                     printf("BuildGuideConnection::LowerOverlap Guide LB (%f, %f), RT (%f, %f) Layer = %d, Guide id = %d\n",
                                     temp_guide.LB.first, temp_guide.LB.second, temp_guide.RT.first, temp_guide.RT.second, temp_guide.Layer, temp_guide.guide_index);
                                     */
                                    // find the overlap region
                                    I_Rect overRect;
                                    TRACKASSIGNMENT::find_overlap_region(guide.LB, guide.RT, temp_guide.LB, temp_guide.RT, overRect.LB, overRect.RT);
                                    //printf("BuildGuideConnection::Overlap Region LB (%d, %d), RT (%d, %d)\n", overRect.LB.first, overRect.LB.second, overRect.RT.first, overRect.RT.second);
                                    guide.low_region.push_back(overRect);
                                    guide.LowerLayerList.push_back(lower_index);
                                }
                            }
                        }
                        
                        //printf("BuildGuideConnection::UP(%d) LOW(%d)\n\n", UpCounter, LowCounter);
                    }//for, go through all guide in a layer
                }//for, go through all layer
                
                //int wwww;
                //cin>>wwww;
            } // for, go through all net
            
        } // BUILDGUIDECONNECTION
    }; // class IROUTEGENERATOR
} // namespace TRACKASSIGNMENT
#endif

