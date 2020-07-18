#include<iostream>
#include<vector>
#include<cstdio>
#include<string>
#include<algorithm>
#include<queue>
#include<map>
#include<fstream>
#include "iroute_generation.hpp"
//#include<priority_queue>

#define top_slope 1.5
#define down_slope 0.6
#define top_len 1.5
#define down_len 0.6

using namespace std;
using namespace TRACKASSIGNMENT ;
/*
 vector<vector<Big_Pin> > Layer_Array_for_Pin;
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
 vector<big_pin_index> pins;
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
 */
namespace Trackassignment_one
{
    /*
     struct iroute{
     int name ;
     float left_point ;
     float right_point ;
     vector<float> pin ;
     vector<float> pin_x ;
     };
     struct final_iroute{
     int iroute_number;
     int track_number;
     };*/
    
    struct num_iroute{
        int a ;
        int b ;
    };
    
    struct num_iroute_a{
        int a ;
        float b ;
    };
    
    struct verti{
        int num ;
        float a ;
    };
	
	//int global_count_h = 0 , global_count_v = 0 ;
    int cout_i , cout_j ;
    //class TRACKASSIGNMENT::IRouteGenerator ;
    bool cmp( const TRACKASSIGNMENT::IRoute *a , const TRACKASSIGNMENT::IRoute *b ){
        return a->LB.first < b->LB.first ;
    }
    
    bool cmp_v( const TRACKASSIGNMENT::IRoute *a , const TRACKASSIGNMENT::IRoute *b ){
        return a->LB.second < b->LB.second ;
    }
    
    bool cmp_pri( const num_iroute &a , const num_iroute &b ){
        return a.b < b.b ;
    }
    
    bool cmp_len( const num_iroute_a &a , const num_iroute_a &b ){
        return a.b < b.b ;
    }
    
    class TA{
    public:
        
        TA(  TRACKASSIGNMENT::Panel &Panel_test , IRouteGenerator &IRouteGenerator )
        {
            //fcerr.open("ans.txt",ios::out|ios::app);
            //fcerr << endl << cout_i << " " << cout_j << endl ;
			
            //fcerr << "\n\nstart\n\n" <<endl ;
            Panel_in =  Panel_test ;
			IR = IRouteGenerator ;
			//fcerr << Panel_in.track.size() << " " << Panel_in.iroute_list.size() << endl ;
			//cerr << Panel_in.iroute_list.size() << endl ;
			/*
            for ( int i = 1 ; i < IR.Hor_Panel_list.size() ; i++ ){
                for ( int j = 0 ; j < IR.Hor_Panel_list[i].size() ; j++ ){
                    panela = i ;
                    panelb = j ;
                    //fcerr << i << " " << j << endl ;
                    each_panel();
                }
            }
            
            hv = 1 ;
            fcerr << "\n\nverrrr\n\n" <<endl ;
            
            for ( int i = 1 ; i < IR.Ver_Panel_list.size() ; i++ ){
                for ( int j = 0 ; j < IR.Ver_Panel_list[i].size() ; j++ ){
                    panela = i ;
                    panelb = j ;
                    //fcerr << i << " " << j << endl ;
                    each_panel_v();
                }
            }*/
			//fcerr << " 3small " << endl ;
            hv = Panel_in.Direction ;
			//cerr << " canit" << endl ;
			if ( hv == true ) each_panel();
			else each_panel_v();
			//fcerr << endl ;
        }
        
        float sum_slope = 0 ;
        float sum_len = 0 ;
        int tmp_iroute_count = 0 ;
        int tracksize = 0 ;
        //int panela , panelb ;
        bool hv = 0 ;
        fstream fcerr ;
		
        IRouteGenerator IR ;
        TRACKASSIGNMENT::Panel Panel_in ;
        
		vector<TRACKASSIGNMENT::IRoute*> tmp_iroute ;
        //.Layer_Array_for_Pin[.layer][tmp_iroute[0].pins[0].id]
        //vector<iroute> all_iroute ;
        vector<int> tmp_priority ;
        //vector<int> track ;
        vector<int> track_right ;
        vector<vector<float>> tmp_len_cost_all ;
        vector<vector<float>> tmp_slope_all ;
        vector<vector<int>> tmp_graph_a ;// better track for each iroute
        vector<vector<int>> tmp_graph_b ;// which trach can put in
        vector<vector<int>> tmp_graph_c ;
        //vector<final_iroute> final_iroute_list ;
        vector<int> right_for_each_track ;
        map<int,vector<verti>> map_pinx_num;
        map<int,vector<verti>>::iterator it;
        
        
        
        void init(){
			
            tracksize = Panel_in.track.size() ;
            sum_slope = 0 ;
            sum_len = 0 ;
            tmp_graph_a.clear();
            tmp_graph_b.clear();
            tmp_graph_c.clear();
            tmp_len_cost_all.clear();
            tmp_slope_all.clear();
            tmp_priority.clear();
            tmp_priority.resize(tmp_iroute.size(),0);
            map_pinx_num.clear();
            
            tmp_graph_a.resize(tmp_iroute.size(),vector<int>(tracksize,0));
            tmp_graph_c.resize(tmp_iroute.size(),vector<int>(tracksize,0));
            //tmp_graph_b.resize(tmp_iroute.size(),vector<int>(track.size()));
            //fcerr << "clear " ;
            
            for ( int i = 0 ; i < tmp_iroute.size() ; i++ ){
                vector<int> tmp(tracksize,0);
                for ( int j = 0 ; j < tracksize ; j++ ){
                    if ( hv == true ){
                        if ( tmp_iroute[i]->LB.first > right_for_each_track[j] ) tmp[j] = 1 ;  //123
                    }else {
                        if ( tmp_iroute[i]->LB.second > right_for_each_track[j] ) tmp[j] = 1 ;  //123
                    }
                    
                }
                tmp_graph_b.push_back(tmp);
            }
        }
        
        void best_track_func( ){ // the best track for iroute
            vector<float> tmp_len ;
            vector<float> tmp_slope ;
                        
            for ( int num = 0 ; num < tmp_iroute.size() ; num++ ){
                for ( int i = 0 ; i < tracksize ; i++ ){
                    int tmp_len_cost = 0 ;
					if ( tmp_iroute[num]->obs == false )
                    for ( int j = 0 ; j < tmp_iroute[num]->pins.size() ; j++ ){
                        if ( hv == true )tmp_len_cost += abs( Panel_in.track[i] -  ( IR.design_information.routing_net_information[tmp_iroute[num]->Net_index].Layer_Array_for_Pin[tmp_iroute[num]->pins[j].Layer][tmp_iroute[num]->pins[j].id].LB.second + IR.design_information.routing_net_information[tmp_iroute[num]->Net_index].Layer_Array_for_Pin[tmp_iroute[num]->pins[j].Layer][tmp_iroute[num]->pins[j].id].RT.second )/2 );
                        else if ( hv == false )tmp_len_cost += abs( Panel_in.track[i] -  ( IR.design_information.routing_net_information[tmp_iroute[num]->Net_index].Layer_Array_for_Pin[tmp_iroute[num]->pins[j].Layer][tmp_iroute[num]->pins[j].id].LB.first + IR.design_information.routing_net_information[tmp_iroute[num]->Net_index].Layer_Array_for_Pin[tmp_iroute[num]->pins[j].Layer][tmp_iroute[num]->pins[j].id].RT.first )/2 );
                    }
                    tmp_len.push_back( tmp_len_cost );
                }
                // min of track
                float tmp_min = tmp_len[0];
                for ( int i = 1 ; i < tracksize ; i++ ){
                    if ( tmp_len[i] < tmp_min ) tmp_min = tmp_len[i] ;
                }
                //get same
                for ( int i = 0 ; i < tracksize ; i++ ){
                    tmp_len[i] -= tmp_min ;
                    sum_len += tmp_len[i] ;
                }
                
                for ( int i = 1 ; i < tracksize ; i++ ){
                    float tmp_a = Panel_in.track[i] - Panel_in.track[i-1];
                    float tmp_b = tmp_len[i] - tmp_len[i-1] ;
                    float tmp_c = abs( tmp_b/tmp_a ) ;
                    tmp_slope.push_back( tmp_c );
                    sum_slope += tmp_c ;
                }
                
                tmp_len_cost_all.push_back(tmp_len);
                tmp_slope_all.push_back(tmp_slope);
                tmp_len.clear() ;
                tmp_slope.clear() ;
                
            }
        }
        /*
        void vertical(){
            
            for ( int i = 0 ; i < tmp_iroute.size() ; i++ ){
                for ( int j = 0 ; j < tmp_iroute[i]->pins.size() ; j++ ){
                    verti tmp ;
                    tmp.num = i ;
                    tmp.a = ( IR.design_information.routing_net_information[tmp_iroute[i]->Net_index].Layer_Array_for_Pin[tmp_iroute[i]->pins[j].Layer][tmp_iroute[i]->pins[j].id].LB.second + IR.design_information.routing_net_information[tmp_iroute[i]->Net_index].Layer_Array_for_Pin[tmp_iroute[i]->pins[j].Layer][tmp_iroute[i]->pins[j].id].RT.second )/2 ;
                    map_pinx_num[ ( IR.design_information.routing_net_information[tmp_iroute[i]->Net_index].Layer_Array_for_Pin[tmp_iroute[i]->pins[j].Layer][tmp_iroute[i]->pins[j].id].LB.first + IR.design_information.routing_net_information[tmp_iroute[i]->Net_index].Layer_Array_for_Pin[tmp_iroute[i]->pins[j].Layer][tmp_iroute[i]->pins[j].id].RT.first )/2 ] ;
                    map_pinx_num[ ( IR.design_information.routing_net_information[tmp_iroute[i]->Net_index].Layer_Array_for_Pin[tmp_iroute[i]->pins[j].Layer][tmp_iroute[i]->pins[j].id].LB.first + IR.design_information.routing_net_information[tmp_iroute[i]->Net_index].Layer_Array_for_Pin[tmp_iroute[i]->pins[j].Layer][tmp_iroute[i]->pins[j].id].RT.first )/2 ].push_back( tmp );
                }
            }
            for ( it = map_pinx_num.begin() ; it != map_pinx_num.end() ; it++ ){
                int size = it->second.size() ;
                if ( size == 1 ) continue ;
                else {
                    vector<int> tmp_v(tracksize+1,-1);
                    //int tmp_avg = track / size ;
                    for ( int i = 0 ; i < size ; i++ ){
                        int tmp_f = it->second[i].a;
                        for ( int j = 0 ; j < tracksize ; j++ ){
                            if ( tmp_f < IR.Hor_Panel_list[panela][panelb].track[j] ){
                                tmp_v[j] = it->second[i].num ;
                                break;
                            }else if ( j == tracksize-1 ){
                                tmp_v[j+1] = it->second[i].num ;
                            }
                        }
                    }
                    int tmp_re = 0 ;
                    int tmp_num = -1 ;
                    for ( int i = 0 ; i < tmp_v.size() ; i++ ){
                        if ( tmp_v[i] != -1 ){
                            for ( int j = tmp_re ; j < i ; j++ ){
                                if ( tmp_num != -1 ) tmp_graph_c[tmp_num][j] = 1 ;
                                tmp_graph_c[tmp_v[i]][j] = 1 ;
                                tmp_num = tmp_v[i] ;
                            }
                        }
                    }
                }
            }
            
            
        }*/
        
        void lencost_pri(){
			//cerr << tracksize <<" "<< sum_slope << " " << top_slope ;
			//cerr << sum_len << " " << down_len ;
			
            sum_slope = sum_slope * top_slope / ( tmp_iroute.size() * tracksize ) ;
            sum_len = sum_len * down_len / ( tmp_iroute.size() * tracksize );
			
            //cerr << " 0 " ;
            for ( int i = 0 ; i < tmp_iroute.size() ; i++ ){
                int tmp_dir = 0 ;
                if ( tmp_slope_all[i][0] > tmp_slope_all[i][1] ) tmp_dir = -1 ;
                else tmp_dir = 1 ;
                for ( int j = 0 ; j < tracksize ; j++ ){
                    //f ( tmp_graph_b[i][j] == 0 || tmp_graph_c[i][j] == 0 ) continue ; //vertical
                    if ( tmp_graph_b[i][j] == 0 ) continue ;
                    if ( j != 0 && j != (tracksize-1) && tmp_slope_all[i][j-1] < tmp_slope_all[i][j] ) tmp_dir = 1 ;
                    if ( tmp_slope_all[i][j] <= sum_slope && j != (tracksize-1) ){
                        tmp_priority[i]++;
                        if ( tmp_dir == 1 ){
                            tmp_graph_a[i][j] = 1 ;
                        }else{
                            tmp_graph_a[i][j+1] = 1 ;
                        }
                    }else if ( tmp_len_cost_all[i][j] <= sum_len ){
                        tmp_graph_a[i][j] = 1 ;
                        tmp_priority[i]++;
                    }
                }
            }
        }
        
        void put_it(){
            vector<int> track_use(tracksize,1) ;
            vector<int> iroute_use(tmp_iroute.size(),1) ;
            /*
            for ( int k = 0 ; k < tmp_iroute.size() ; k++ ){
                fcerr << tmp_priority[k] << " " ;
            }
            fcerr << endl ;*/
            int obs_count = 0 ;
			for ( int i = 0 ; i < tmp_iroute.size() ; i++ ){
				if ( tmp_iroute[i]->obs == true ){
					obs_count++;
					iroute_use[i] = 0 ;
					for ( int j = 0 ; j < tracksize ; j++ ){
						if ( hv == true && Panel_in.track[j] == tmp_iroute[i]->LB.second ){
							track_use[j] = 0 ;
							right_for_each_track[j] = tmp_iroute[i]->RT.first+200 ;
						} 
						if ( hv == false && Panel_in.track[j] == tmp_iroute[i]->LB.first ){
							track_use[j] = 0 ;
							right_for_each_track[j] = tmp_iroute[i]->RT.second+200 ;
						}
					}
				}
			}
			
            for ( int i = 0 ; i < tmp_iroute.size()-obs_count ; i++ ){
                
                vector<num_iroute> tmp_num_iroute ;
                for ( int k = 0 ; k < tmp_iroute.size() ; k++ ){
                    if ( iroute_use[k] == 1 ){
                        num_iroute a ;
                        a.a = k ;
                        a.b = tmp_priority[k] ;
                        tmp_num_iroute.push_back(a);
                    }
                }
                
                sort( tmp_num_iroute.begin() , tmp_num_iroute.end() , cmp_pri );
                iroute_use[tmp_num_iroute[0].a] = 0 ;
                
                vector<num_iroute_a> tmp_num_track ;
                for ( int j = 0 ; j < tracksize ; j++ ){
                    num_iroute_a a ;
                    a.a = j ;
                    a.b = tmp_len_cost_all[tmp_num_iroute[0].a][j];
                    tmp_num_track.push_back(a);
                }
                sort(tmp_num_track.begin(),tmp_num_track.end(),cmp_len);
                
                
                bool if_putin = false ;
                for ( int j = 0 ; j < tracksize ; j++ ){
                    if ( hv == true && track_use[tmp_num_track[j].a] && tmp_iroute[tmp_num_iroute[0].a]->LB.first > right_for_each_track[tmp_num_track[j].a] ){
                        //fcerr << tmp_iroute_count+tmp_num_iroute[0].a <<" "<< tmp_num_track[j].a <<" "<< right_for_each_track[tmp_num_track[j].a] <<" "<<tmp_iroute[tmp_num_iroute[0].a]->LB.first<<" ";
                        //fcerr << tmp_num_iroute[0].a << " " ;
                        right_for_each_track[tmp_num_track[j].a] = tmp_iroute[tmp_num_iroute[0].a]->RT.first+200 ;
                        Panel_in.iroute_list[tmp_iroute_count+tmp_num_iroute[0].a]->LB.second = Panel_in.track[tmp_num_track[j].a] ;
                        Panel_in.iroute_list[tmp_iroute_count+tmp_num_iroute[0].a]->RT.second = Panel_in.track[tmp_num_track[j].a] ;
                        //fcerr << right_for_each_track[tmp_num_track[j].a] << endl ;
                        track_use[tmp_num_track[j].a] = 0 ;
                        for ( int z = 0 ; z < tmp_iroute.size() ; z++ ){
                            if ( tmp_graph_a[z][tmp_num_track[j].a] ) tmp_priority[z]--;
                        }
                        //tmp_iroute_count
                        /*fcerr << IR.Hor_Panel_list[panela][panelb].iroute_list[tmp_iroute_count+tmp_num_iroute[0].a]->LB.second << " "
                        << IR.Hor_Panel_list[panela][panelb].iroute_list[tmp_iroute_count+tmp_num_iroute[0].a]->LB.first  << " "
                        << IR.Hor_Panel_list[panela][panelb].iroute_list[tmp_iroute_count+tmp_num_iroute[0].a]->RT.first  << "|||" ;
                        */
						//cerr << tmp_iroute_count+tmp_num_iroute[0].a << Panel_in.iroute_list[tmp_iroute_count+tmp_num_iroute[0].a]->LB.first
						
						if_putin = true ;
                        break;
                    }else if ( hv == false && track_use[tmp_num_track[j].a] && tmp_iroute[tmp_num_iroute[0].a]->LB.second > right_for_each_track[tmp_num_track[j].a] ){
                        //fcerr << tmp_iroute_count+tmp_num_iroute[0].a <<" "<< tmp_num_track[j].a <<" "<< right_for_each_track[tmp_num_track[j].a] <<" "<<tmp_iroute[tmp_num_iroute[0].a]->LB.second<<" ";
                        //fcerr << tmp_num_iroute[0].a << " " ;
                        right_for_each_track[tmp_num_track[j].a] = tmp_iroute[tmp_num_iroute[0].a]->RT.second+200 ;
                        Panel_in.iroute_list[tmp_iroute_count+tmp_num_iroute[0].a]->LB.first = Panel_in.track[tmp_num_track[j].a] ;
                        Panel_in.iroute_list[tmp_iroute_count+tmp_num_iroute[0].a]->RT.first = Panel_in.track[tmp_num_track[j].a] ;
                        //fcerr << right_for_each_track[tmp_num_track[j].a] << endl ;
                        track_use[tmp_num_track[j].a] = 0 ;
                        for ( int z = 0 ; z < tmp_iroute.size() ; z++ ){
                            if ( tmp_graph_a[z][tmp_num_track[j].a] ) tmp_priority[z]--;
                        }
                        //tmp_iroute_count
                        /*fcerr << IR.Ver_Panel_list[panela][panelb].iroute_list[tmp_iroute_count+tmp_num_iroute[0].a]->LB.first << " "
                        << IR.Ver_Panel_list[panela][panelb].iroute_list[tmp_iroute_count+tmp_num_iroute[0].a]->RT.second  << " "
                        << IR.Ver_Panel_list[panela][panelb].iroute_list[tmp_iroute_count+tmp_num_iroute[0].a]->LB.second  << "|||" ;
                        */
						if_putin = true ;
                        break;
                    }/*else if ( hv == true ){
						global_count_h++ ;
					}else if ( hv == false ){
						global_count_v++ ;
					}*/
                }
				//if ( if_putin == false && hv == true ) global_count_h++ ;
				//if ( if_putin == false && hv == false ) global_count_v++ ;
                
                //tmp_num_iroute.clear();
            }
            tmp_iroute_count += tmp_iroute.size() ;
        }
        
        void put_iroute_in_track( ){
			
            //cerr << "put_iroute  "  ;
            init();
            //cerr << "init  " ;
            best_track_func();
            //cerr << "bset  " ;
            //vertical();
            //fcerr << "ver  " ;
            lencost_pri();
            //cerr << "lencost  " ;
            put_it();
            //cerr << "put_it\n" ;
            tmp_iroute.clear();
        }
        
        void test(){
            
        }
        
        void each_panel( ){
            tmp_iroute_count = 0 ;
            //fcerr << "panel " << panela << " " << panelb << " panel" << endl ;
            
            right_for_each_track.clear() ;
            right_for_each_track.resize(Panel_in.track.size(),0);
            /*
            for ( int i = 0 ; i < Panel_in.track.size() ; i++ ){
                fcerr << IR.Hor_Panel_list[panela][panelb].track[i] << " " ;//<< right_for_each_track[i] <<  " || ";
            }
            fcerr << endl ;
            */
			tracksize = Panel_in.track.size();
			if ( tracksize == 1 ){
				for ( int i = 0 ; i < Panel_in.iroute_list.size() ; i++ ){
					Panel_in.iroute_list[i]->LB.second = Panel_in.track[0] ;
                    Panel_in.iroute_list[i]->RT.second = Panel_in.track[0] ;
				}
			}else {
				//fcerr << "ir: " ;
				sort( Panel_in.iroute_list.begin() , Panel_in.iroute_list.end() , cmp );
				float min_right_point = Panel_in.iroute_list[0]->RT.first ;
				//fcerr << "sort " << Panel_in.iroute_list.size() << endl << "iroute:" ;
				for ( int i = 0 ; i < Panel_in.iroute_list.size() ; i++ ){
					
					if ( Panel_in.iroute_list[i]->LB.first > min_right_point ){
						//fcerr << endl ;
						put_iroute_in_track() ;
						//fcerr << "ir: " ;
						min_right_point = Panel_in.iroute_list[i]->RT.first ;
					}else if (Panel_in.iroute_list[i]->RT.first < min_right_point ){
						min_right_point = Panel_in.iroute_list[i]->RT.first ;
					}
					
					tmp_iroute.push_back( Panel_in.iroute_list[i] ) ;
					//fcerr << i << " " ;
					if ( i == Panel_in.iroute_list.size()-1 ){
						//fcerr << i << " " << endl ;
						//fcerr << endl ;
						put_iroute_in_track() ;
						//fcerr << " final " << endl ;
						//fcerr << "iroute: " ;
					}//else fcerr << i << " " ;
				}
			}
        }
        
        void each_panel_v( ){
            tmp_iroute_count = 0 ;
            //fcerr << "panel " << panela << " " << panelb << " panel" << endl ;
            //fcerr << "2small" << endl ;
            right_for_each_track.clear() ;
			//fcerr << "123" << endl ;
			tracksize = Panel_in.track.size();
			if ( tracksize == 1 ){
				for ( int i = 0 ; i < Panel_in.iroute_list.size() ; i++ ){
					Panel_in.iroute_list[i]->LB.first = Panel_in.track[0] ;
					Panel_in.iroute_list[i]->RT.first = Panel_in.track[0] ;
				}				
			}else {
				//cerr << "??\n" ;
				//right_for_each_track.resize(tracksize,0);
				for ( int i = 0 ; i < tracksize ; i++ ){
					right_for_each_track.push_back(0) ;
				}
				
				//fcerr << "ir: " ;
				sort( Panel_in.iroute_list.begin() , Panel_in.iroute_list.end() , cmp_v );
				float min_right_point = Panel_in.iroute_list[0]->LB.second ;
				//fcerr << "sort " << Panel_in.iroute_list.size() << endl << "iroute:" ;
				for ( int i = 0 ; i < Panel_in.iroute_list.size() ; i++ ){
					
					if ( Panel_in.iroute_list[i]->RT.second > min_right_point ){
						//fcerr << endl ;
						put_iroute_in_track() ;
						//fcerr << "ir: " ;
						min_right_point = Panel_in.iroute_list[i]->LB.second ;
					}else if ( Panel_in.iroute_list[i]->LB.second < min_right_point ){
						min_right_point = Panel_in.iroute_list[i]->LB.second ;
					}
					
					tmp_iroute.push_back( Panel_in.iroute_list[i] ) ;
					//fcerr << i << " " ;
					if ( i == Panel_in.iroute_list.size()-1 ){
						//fcerr << endl ;
						put_iroute_in_track() ;
						//fcerr << " final " << endl ;
						//fcerr << "iroute: " ;
					}//else fcerr << i << " " ;
				}
			}
        }
        /*
         void main_func( IRouteGenerator &IRouteGenerator ){
         IR = IRouteGenerator ;
         for ( int i = 1 ; i < IR.Hor_Panel_list.size() ; i++ ){
         for ( int j = 0 ; j < IR.Hor_Panel_list[i].size() ; j++ ){
         panela = i ;
         panelb = j ;
         each_panel();
         }
         }
         }*/
    };
    
}

