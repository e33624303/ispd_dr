#ifndef GUIDEPARSER_H
#define GUIDEPARSER_H

#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <time.h>
#include <fstream>
#include <deque>
#include <utility>
#include "Structure.h"

using namespace std;
class GuideParser
{
  public:
    GuideParser()
    {
        this->GCell_height = -1;
        this->GCell_width = -1;
    }

    struct rectangle
    {
        unsigned int lbx, lby, rtx, rty, layer;

        rectangle(unsigned int _lbx, unsigned int _lby, unsigned int _rtx, unsigned int _rty, unsigned int _layer)
        {
            lbx = _lbx;
            lby = _lby;
            rtx = _rtx;
            rty = _rty;
            layer = _layer;
        }
    };
    
    vector<vector<rectangle> > net_guide;
    int GCell_height;
    int GCell_width;

    void ISPD2018_guide_fileInput(istream &file, Design &design) // file for the input.guide
    {
        int counter = 0;

        vector<rectangle> temp_rect_list;
        vector<I_Rect> *net_guide2;
        map <string,int> NetTable;
        for(int i = 0 ; i < design.ispd_routing_net.size() ; i++)
        {
            NetTable.emplace(design.ispd_routing_net.at(i).Original_Data.Net_name, i);
        }
        string line_guide;
        while (!file.eof())
        {
            counter++;

            getline(file, line_guide); // net name
            auto design_net = NetTable.find(line_guide);
            if (design_net == NetTable.end())
                break;

            net_guide2 = &design.ispd_routing_net.at(design_net->second).GUIDE_list;

            vector<rectangle> temp_guide;

            if (line_guide.size() > 0 && line_guide.at(0) != '(')
            {
                
                getline(file, line_guide); // (
                while (line_guide.size() > 0 && line_guide.at(0) != ')')
                {
                    if (line_guide.find(" ") != string::npos)
                    {
                        int M_pos = line_guide.find(' ');

                        int index_start_coor;
                        unsigned int _x[2], _y[2], layer_number;

                        string layer_num_s;
                        layer_num_s.clear();
                        for (int ind = 0; ind < 4; ind++)
                        {
                            for (index_start_coor = 0; line_guide.at(index_start_coor) != ' ';)
                            {
                                layer_num_s.push_back(line_guide.at(index_start_coor));
                                line_guide.erase(line_guide.begin());
                            }

                            line_guide.erase(line_guide.begin());

                            if (ind == 0)
                                _x[0] = atoi(layer_num_s.c_str());
                            else if (ind == 1)
                                _x[1] = atoi(layer_num_s.c_str());
                            else if (ind == 2)
                                _y[0] = atoi(layer_num_s.c_str());
                            else if (ind == 3)
                                _y[1] = atoi(layer_num_s.c_str());

                            layer_num_s.clear();
                        } // for

                        layer_num_s.push_back(line_guide.at(line_guide.size() - 1));
                        layer_number = atoi(layer_num_s.c_str());

                        int c_height, c_width;
                        c_width = _y[0] - _x[0];
                        c_height = _y[1] - _x[1];
/*
                        if (c_width < 6000)
                            printf("Not width : (%d,%d) (%d,%d)\n", _x[0], _x[1], _y[0], _y[1]);
                        if (c_height < 5700)
                            printf("Not height : (%d,%d) (%d,%d)\n", _x[0], _x[1], _y[0], _y[1]);
*/
/*
                        if (_x[0] % 6000 != 0)
                            printf("Not width : (%d,%d) (%d,%d)\n", _x[0], _x[1], _y[0], _y[1]);
                        if (_x[1] % 5700 != 0)
                            printf("Not height : (%d,%d) (%d,%d)\n", _x[0], _x[1], _y[0], _y[1]);
*/
                        if (!count_height.empty())
                        {
                            bool found_same = false;
                            for (int index3 = 0; index3 < count_height.size(); index3++)
                            {
                                if (c_height % count_height.at(index3).first == 0)
                                {
                                    count_height.at(index3).second++;
                                    found_same = true;
                                    break;
                                }
                            }
                            if (!found_same)
                            {
                                count_height.push_back(make_pair(c_height, 1));
                            }
                        }
                        else
                        {
                            count_height.push_back(make_pair(c_height, 1));
                        } // if

                        if (!count_width.empty())
                        {
                            bool found_same = false;
                            for (int index3 = 0; index3 < count_width.size(); index3++)
                            {
                                if (c_width % count_width.at(index3).first == 0)
                                {
                                    count_width.at(index3).second++;
                                    found_same = true;
                                    break;
                                }
                            }
                            if (!found_same)
                            {
                                count_width.push_back(make_pair(c_width, 1));
                            }
                        }
                        else
                        {
                            count_width.push_back(make_pair(c_width, 1));
                        } // if

                        //cout << ">> " << _x[0] << " " << _x[1] << " " << _y[0] << " " << _y[1] << " " << layer_number << endl;
                        rectangle temp_rect = rectangle(_x[0], _x[1], _y[0], _y[1], layer_number);
                        temp_guide.push_back(temp_rect);
                        I_Rect temp_R = I_Rect(_x[0], _x[1], _y[0], _y[1], layer_number);
                        net_guide2->push_back(temp_R);
                    }

                    getline(file, line_guide);

                } // while

                net_guide.push_back(temp_guide);

            } // if

            temp_guide.clear();
        }

        if(counter-1 != design.ispd_routing_net.size())
        {
            cout << "ERROR::GuideParser : Number of guides incorrect " << counter << " of " << design.ispd_routing_net.size() << "\n";
            exit(0);
        }
        // Calculate the common GCell height & width
        GuideCount(design);
        return;
    } // read input()

  private:

    vector<pair<int, int>> count_height_base;
    vector<pair<int, int>> count_width_base;
    vector<pair<int, int>> count_height;
    vector<pair<int, int>> count_width;

    void GuideCount(Design &design)
    {
        vector<int> height_nonuniform, width_nonuniform;
        int max_candidate = 0;
        int max_count = 0;
        // count the common height
        for (int index3 = 0; index3 < count_height.size(); index3++)
        {
            if (count_height.at(index3).second > max_count)
            {
                max_count = count_height.at(index3).second;
                max_candidate = count_height.at(index3).first;
            }
        }
        design.GCell_Height = max_candidate;
        this->GCell_height = max_candidate;

        count_height_base.push_back(make_pair(max_candidate, 0));

        for (int index3 = 0; index3 < count_height.size(); index3++)
        {
            int now_number = count_height.at(index3).first % max_candidate;

            if (!count_height_base.empty())
            {
                bool found_same = false;
                for (int index4 = 0; index4 < count_height_base.size(); index4++)
                {
                    if (now_number % count_height_base.at(index4).first == 0)
                    {
                        if (count_height_base.at(index4).first != max_candidate)
                        {
                            // catch these cases
                            height_nonuniform.push_back(count_height.at(index3).first);
                        }

                        count_height_base.at(index4).second++;
                        found_same = true;
                        break;
                    }
                }
                if (!found_same)
                {
                    count_height_base.push_back(make_pair(now_number, 1));
                }
            }
            else
            {

            } // if
        }
        // width
        max_count = 0;
        for (int index3 = 0; index3 < count_width.size(); index3++)
        {

            if (count_width.at(index3).second > max_count)
            {
                max_count = count_width.at(index3).second;
                max_candidate = count_width.at(index3).first;
            }
        }

        design.GCell_Width = max_candidate;
        this->GCell_width = max_candidate;

        count_width_base.push_back(make_pair(max_candidate, 0));

        for (int index3 = 0; index3 < count_width.size(); index3++)
        {
            int now_number = count_width.at(index3).first % max_candidate;

            if (!count_width_base.empty())
            {
                bool found_same = false;
                for (int index4 = 0; index4 < count_width_base.size(); index4++)
                {
                    if (now_number % count_width_base.at(index4).first == 0)
                    {
                        if (count_width_base.at(index4).first != max_candidate)
                        {
                            // catch these cases
                            width_nonuniform.push_back(count_width.at(index3).first);
                        }

                        count_width_base.at(index4).second++;
                        found_same = true;
                        break;
                    }
                }
                if (!found_same)
                {
                    count_width_base.push_back(make_pair(now_number, 1));
                }
            }
            else
            {

            } // if
        }
        //
/*
        cout << "Kind of height count : " << count_height_base.size() << "\n";
        for (int index3 = 0; index3 < count_height_base.size(); index3++)
        {
            cout << "(" << count_height_base.at(index3).first << ") : " << count_height_base.at(index3).second << endl;
        }
        cout << "Kind of width count : " << count_width_base.size() << "\n";
        for (int index3 = 0; index3 < count_width_base.size(); index3++)
        {
            cout << "(" << count_width_base.at(index3).first << ") : " << count_width_base.at(index3).second << endl;
        }
*/

        if (count_width_base.size() > 2)
            cout << "ERROR : GCell width base has more than " << count_width_base.size() << endl;
        if (count_height_base.size() > 2)
            cout << "ERROR : GCell height base has more than " << count_height_base.size() << endl;

        //design.GCell_Original_x = (int)design.Die_Area.RT.first - count_width_base.at(1).first % design.GCell_Width;
        //design.GCell_Original_y = (int)design.Die_Area.RT.second - count_height_base.at(1).first % design.GCell_Height;

        cout << "GuideParser::GCell_width = " << design.GCell_Width << ", GCell_height = " << design.GCell_Height << endl;
        cout << "GuideParser::GCell_Original_x = " << design.GCell_Original_x << ", GCell_Original_y = " << design.GCell_Original_y << endl;
        /*
        vector<rectangle> height_final;
        vector<rectangle> width_final;

        int H_RT_point = 0, W_LB_point = 0;

        // verify that abut on layout boundary
        
        for (auto &index : net_guide)
        {
            for(auto &guider : index)
            {
                for (auto &wu : width_nonuniform)
                {
                    if (guider.rtx - guider.lbx == wu)
                    {
                        //rectangle temp_rect = rectangle(guider.lbx, guider.lby, guider.rtx, guider.rty, guider.layer);
                        width_final.push_back(guider);
                    }
                }

                for (auto &hu : height_nonuniform)
                {
                    if (guider.rty - guider.lby == hu)
                    {
                        height_final.push_back(guider);
                    }
                }
            }
        }
        */

        
        /*
        cout << "#width nonuniform list :\n";
        for (auto &index : width_final)
        {
            cout << ">> " << index.lbx << " " << index.lby << " " << index.rtx << " " << index.rty << " " << index.layer << endl;
        }

        cout << "#height nonuniform list :\n";
        for (auto &index : height_final)
        {
            cout << ">> " << index.lbx << " " << index.lby << " " << index.rtx << " " << index.rty << " " << index.layer << endl;
        }
        */
        
    } // GuideCount()
    
};

//


#endif