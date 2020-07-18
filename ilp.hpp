#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <glpk.h>
#include <cmath>
#include <ctime>
#include <sstream>
#include <map>
#include <vector>
#include <utility>
#include <algorithm>
#include "iroute_generation.hpp"
#define max_size 1000000000

using namespace std;
using namespace TRACKASSIGNMENT;


/*
 INPUT:
 panel :
 iroute : lb(x,y),rt(x,y),layer,net_index
 
 
 
 OUTPUT:
 horizontal : iroute->track(layer)
 vertical : iroute->track(layer)
 
 return net issue (which iroute)
 
 */
/* pin problem not solved*/
/*
 solve overlap problem by zone
 */

int penalty = 0;
int max_violation = 0;

double compute_x(TRACKASSIGNMENT::IRoute &iroute,int track,int bot,int top,int layer,vector<vector<Big_Pin> > Layer_Array_for_Pin){
    double result=0;
    //cout<<"compute!\n";
    for(int i=0;i<iroute.pins.size();i++){
        //cout<<"compute "<<i<<"        "<<iroute.pins.size()<<endl;
        if(Layer_Array_for_Pin[1][iroute.pins[i].id].LB.first>top){
            //cout<<"case1\n";
            result+=llabs(Layer_Array_for_Pin[1][iroute.pins[i].id].LB.first-track);
        }
        else if(Layer_Array_for_Pin[1][iroute.pins[i].id].RT.first<bot){
            //cout<<"case2\n";
            result+=llabs(Layer_Array_for_Pin[1][iroute.pins[i].id].RT.first-track);
        }
        else{
            //cout<<"case3\n";
            result+=llabs( (Layer_Array_for_Pin[1][iroute.pins[i].id].LB.first+Layer_Array_for_Pin[1][iroute.pins[i].id].RT.first)/2-track );
        }
    }
    return result;
}

double compute_y(TRACKASSIGNMENT::IRoute &iroute,int track,int bot ,int top,int layer,vector<vector<Big_Pin> > Layer_Array_for_Pin){
    double result=0;
    //cout<<"pins size is "<<iroute.pins.size()<<endl;
    for(int i=0;i<iroute.pins.size();i++){
        //if pin is outside the panel
        if(Layer_Array_for_Pin[1][iroute.pins[i].id].LB.second>top){
            result+=llabs(Layer_Array_for_Pin[1][iroute.pins[i].id].LB.second-track);
        }
        else if(Layer_Array_for_Pin[1][iroute.pins[i].id].RT.second<bot){
            result+=llabs(Layer_Array_for_Pin[1][iroute.pins[i].id].RT.second-track);
        }
        else{
            result+=llabs( (Layer_Array_for_Pin[1][iroute.pins[i].id].LB.second+Layer_Array_for_Pin[1][iroute.pins[i].id].RT.second)/2-track );
        }
    }
    //cout<<"result="<<result<<endl;
    return result;
}

void solve_problem_horizontal(Panel &Hor_Panel_list,Design_Information &design,int track_width,int layer,vector<vector<Panel> > &Hor_Panel_list2){
    glp_term_out(GLP_OFF);
    int number;
    int track_size,iroute_size;
    track_size=Hor_Panel_list.track.size();                    //track size not finished
    iroute_size=Hor_Panel_list.iroute_list.size();    //check iroute size correct or not
    //cout<<"iroute size : "<<iroute_size<<"    track size : "<<track_size<<endl;
    
    glp_prob *lp;
    int *ia,*ja;
    double *ar;
    double cost;
    
    /*for(int i=0 ; i<iroute_size ; i++){
            if(Hor_Panel_list.iroute_list[i]->LB.first==360000 && Hor_Panel_list.iroute_list[i]->LB.second==310650)
                cout<<"top is "<<i<<" !!!"<<endl;
            if(Hor_Panel_list.iroute_list[i]->LB.first==336000 && Hor_Panel_list.iroute_list[i]->LB.second==310650)
                cout<<"down is "<<i<<" !!!"<<endl;
    }*/


    
    int size=0;         //size to check whether the iroute is overlap
    int obstacle_size=0;

    //cout<<"before overlap\n";
    vector<int> vec_old,vec_new;        //vec_old,vec_new : old/new version of overlap condition
    vector< vector<int> > vec_ilp;         //final version of overlap condition
    vector<int> vec_tmp;
    vector< vector<int> > vec_obs;
    if(iroute_size>1){
        vector<int> iroute_start;
        for(int i=0 ; i<iroute_size ; i++){
            if(Hor_Panel_list.iroute_list[i]->obs == 1){
                obstacle_size++;
                vec_tmp.push_back(i+1);
                for(int j=0;j<track_size;j++){
                    if(Hor_Panel_list.iroute_list[i]->LB.second<=Hor_Panel_list.track[j] && Hor_Panel_list.iroute_list[i]->RT.second>=Hor_Panel_list.track[j]){
                        vec_tmp.push_back(j+1);
                    }
                }
                vec_obs.push_back(vec_tmp);
            }
            vec_tmp.clear();
            iroute_start.push_back(Hor_Panel_list.iroute_list[i]->LB.first);
            iroute_start.push_back(Hor_Panel_list.iroute_list[i]->RT.first);
        }
        sort(iroute_start.begin() , iroute_start.end());
        /*for(int i=0;i<iroute_start.size();i++)
         cout<<iroute_start[i]<<" ";
         cout<<endl;*/
        for(int i=0 ; i<iroute_start.size() ; i++){
            for(int j=0 ; j<iroute_size ; j++){
                if( Hor_Panel_list.iroute_list[j]->LB.first<=(iroute_start[i]+track_width) && Hor_Panel_list.iroute_list[j]->RT.first>=(iroute_start[i]-track_width) )
                    vec_new.push_back(j+1);
            }
            vector<int> intersection;
            set_intersection(vec_new.begin() , vec_new.end() , vec_old.begin() , vec_old.end() , inserter(intersection , intersection.begin() ) );
            if(intersection == vec_old && intersection != vec_new){
                //cout<<"now is i : "<<i<<"  cor is : "<<iroute_start[i]<<"   case 1 "<<endl;
                vec_old=vec_new;
                if(i == iroute_start.size()-1 )
                    vec_ilp.push_back(vec_new);
            }
            else if(intersection == vec_new ){
                if(intersection != vec_old){
                    //cout<<"now is i : "<<i<<"  cor is : "<<iroute_start[i]<<"   case 2 "<<endl;
                    vec_ilp.push_back(vec_old);
                    vec_old=vec_new;
                }
            }
            else{
                //cout<<"now is i : "<<i<<"  cor is : "<<iroute_start[i]<<"   case 3 "<<endl;
                vec_ilp.push_back(vec_old);
                vec_old=vec_new;
                if( i == iroute_start.size()-1 )
                    vec_ilp.push_back(vec_new);
            }
            vec_new.clear();
        }
        size=vec_ilp.size();
        //cout<<"begin size is : "<<size<<endl;
        
        //remove unnecessary subset of overlap condition
        if(size>0){
            for(int i=0 ; i<vec_ilp.size()-1 ; i++){
                vector<int> intersection;
                if(vec_ilp[i].size()>1 && vec_ilp[i+1].size()>1){
                    set_intersection(vec_ilp[i].begin() , vec_ilp[i].end() , vec_ilp[i+1].begin() , vec_ilp[i+1].end() , inserter(intersection , intersection.begin() ));
                    if(vec_ilp[i] == intersection && vec_ilp[i+1] != intersection){
                        for(int j=i ; j<vec_ilp.size()-1 ; j++){
                            vec_ilp[j]=vec_ilp[j+1];
                        }
                        i--;
                        vec_ilp.resize(vec_ilp.size()-1);
                    }
                    else if(vec_ilp[i] != intersection && vec_ilp[i+1] == intersection){
                        for(int j=i+1 ; j<vec_ilp.size()-1 ; j++){
                            vec_ilp[j]=vec_ilp[j+1];
                        }
                        i--;
                        vec_ilp.resize(vec_ilp.size()-1);
                    }
                }
            }
            //cout<<"after size is : "<<vec_ilp.size()<<endl;
            /*for(int i=0 ; i<vec_ilp.size() ; i++){
             for(int j=0 ; j<vec_ilp[i].size() ; j++){
             cout<<vec_ilp[i][j]<<" ";
             }
             cout<<endl;
             }
             cout<<Hor_Panel_list.LB.first<<"  @@  "<<Hor_Panel_list.RT.first<<endl;*/
            
            /*for(int i=0 ; i<iroute_size-1 ; i++){
             for(int j=0 ; j<iroute_size ; j++){
             if( rec_overlap(Hor_Panel_list.iroute_list[i]->LB,Hor_Panel_list.iroute_list[i]->RT,Hor_Panel_list.iroute_list[j]->LB,Hor_Panel_list.iroute_list[j]->RT) == 1 ){
             size++;
             map.insert( pair<int,int>(i,j) );
             cout<<i<<" "<<j<<endl;
             }
             }
             }*/
            
            size=vec_ilp.size();
        }
        //cout<<"finish vector part\n";
    }
    //cout<<"before violation\n";
    multimap<int,int> violation;
    vector<vector<int> > violation_map;
    vector<int> violation_tmp;
    if(layer>3){
        int vio=0;
        for(int k=0 ; k<Hor_Panel_list2[layer-1].size() ; k++){
            int time=0;
            for(int i=1;i<=iroute_size;i++){
                map< int, vector<TRACKASSIGNMENT::IRoute*> >::iterator iter;
                //cout<<"true!\n\n\n";
                iter=Hor_Panel_list2[layer-1][k].net_check.find(Hor_Panel_list.iroute_list[i-1]->Net_index);
                if(iter!=Hor_Panel_list2[layer-1][k].net_check.end()){
                    time++;
                    //cout<<"now i = "<<i<<"      iter->second = "<<iter->second[0]->LB.first<<endl;
                    violation_tmp.push_back(i);     //be careful to check this value
                    violation_tmp.push_back(iter->second[0]->LB.first);
                    violation_tmp.push_back(iter->second[0]->LB.second);
                    violation_map.push_back(violation_tmp);
                    violation_tmp.clear();   
                    //cout<<"find one!\n\n\n\n\n";
                }
            }
            //consider only improve the condition that contains only two violations
            if(time>=2){
                for(int i=0 ; i<violation_map.size()-1 ; i++){
                    for(int j=i+1 ; j<violation_map.size() ; j++){
                        if( violation_map[i][1] == violation_map[j][1] ){
                            pair< multimap<int,int>::iterator , multimap<int,int>::iterator> ret;
                            bool exist=false;
                            if(violation_map[i][2] > violation_map[j][2]){
                                ret=violation.equal_range(violation_map[i][0]);
                                for(multimap<int,int>::iterator it=ret.first ; it!=ret.second ; ++it){
                                    if(violation_map[j][0] == it->second){
                                        exist=true;
                                        break;
                                    }
                                }
                                if(!exist){
                                    violation.insert( pair<int,int>(violation_map[i][0] , violation_map[j][0]) );
                                    //cout<<"Violation OF "<<violation_map[i][0]<<" "<<violation_map[j][0]<<" !!!\n\n";
                                }
                            }
                            else{
                                ret=violation.equal_range(violation_map[j][0]);
                                for(multimap<int,int>::iterator it=ret.first ; it!=ret.second ; ++it){
                                    if(violation_map[i][0] == it->second){
                                        exist=true;
                                        break;
                                    }
                                }
                                if(!exist){
                                    violation.insert( pair<int,int>(violation_map[j][0] , violation_map[i][0]) );
                                    //cout<<"Violation OF "<<violation_map[j][0]<<" "<<violation_map[i][0]<<" !!!\n\n";
                                }
                            }
                        }
                        //cout<<violation_map[i][0]<<"   "<<violation_map[j][0]<<endl;
                    }
                }
                vio++;
                penalty++;
            }
            violation_map.clear();
        }

        if(max_violation<vio)
            max_violation=vio;
        /*cout<<"max is "<<max_violation<<endl;
        cout<<"violation is "<<vio<<endl;
        cout<<"penalty is "<<penalty<<endl;
        cout<<"violation size is "<<violation.size()<<endl;*/
    }

    int violation_size=violation.size();
    //int violation_size=0;
    /*test the efficiency of the violation condition , all size is testing*/
    bool case_violation_before=false;
    /*if( violation.size()<10  && track_size > 10){
        case_violation_before=true;
        violation_size*=3;
        cout<<"case violation compute before!!!\n\n\n";
    }*/
    //cout<<"before declaration\n";
    ia = new int[1+(iroute_size+size*track_size+violation_size+obstacle_size)*iroute_size*track_size];
    ja = new int[1+(iroute_size+size*track_size+violation_size+obstacle_size)*iroute_size*track_size];
    ar = new double[1+(iroute_size+size*track_size+violation_size+obstacle_size)*iroute_size*track_size];
    //cout<<"here 2\n";
    lp=glp_create_prob();
    glp_set_prob_name(lp , "ILP");
    glp_set_obj_dir(lp , GLP_MIN);
    glp_add_rows(lp , iroute_size+size*track_size+violation_size+obstacle_size);
    //cout<<"now add "<<iroute_size+size*track_size<<" rows\n";
    //cout<<"before add row\n";
    for(int i=1;i<=iroute_size+size*track_size+violation_size;i++){
        string row="row";
        stringstream ss;
        ss<<i;
        string str;
        ss>>str;
        row+=str;
        //cout<<"now row is : "<<i<<endl;
        if(i<=iroute_size){
            /*cout<<"now set row name : "<<row.c_str()<<endl;
             cout<<"now set row bonds : 1.0 1.0 \n";*/
            glp_set_row_name(lp , i , row.c_str());
            glp_set_row_bnds(lp , i , GLP_FX , 1.0 , 1.0);
        }
        else if(i<=iroute_size+size*track_size && i>iroute_size){
            /*cout<<"now set row name : "<<row.c_str()<<endl;
             cout<<"now set row bonds : 0.0 1.0 \n";*/
            glp_set_row_name(lp , i , row.c_str());
            glp_set_row_bnds(lp , i , GLP_DB , 0.0 , 1.0);
        }
        else if(i>iroute_size+size*track_size && i<=iroute_size+size*track_size+violation_size && case_violation_before == false){
            glp_set_row_name(lp , i , row.c_str());
            glp_set_row_bnds(lp , i , GLP_DB , 0.0 , 1.0);
        }
        else if(i>iroute_size+size*track_size && i<=iroute_size+size*track_size+violation_size && case_violation_before == true){
            glp_set_row_name(lp , i , row.c_str());
            glp_set_row_bnds(lp , i , GLP_FX , 0.0 , 0.0);
        }
        else{
            glp_set_row_name(lp , i , row.c_str());
            glp_set_row_bnds(lp , i , GLP_FX , 1.0 , 1.0);
        }
    }
    //cout<<"\n\n\n";
    //cout<<"before add col!\n\n";
    glp_add_cols(lp , iroute_size*track_size);
    //cout<<"now add "<<iroute_size*track_size<<" cols\n";
    for(int i=1 ; i<=iroute_size ; i++){
        for(int j=1 ; j<=track_size ; j++){
            stringstream ss;
            string col="y";
            string x;
            string y;
            ss<<i;
            ss>>x;
            ss.str("");
            ss.clear();
            ss<<j;
            ss>>y;
            col=col+x+y;
            /*cout<<"now col is : "<<track_size*(i-1)+j<<endl;
             cout<<"now set col name : "<<col.c_str()<<endl;
             cout<<"now set col bonds : 0.0 1.0\n";
             cout<<"now set obj coef -> compute : "<<i<<" track : "<<j<<endl;*/
            glp_set_col_name(lp , track_size*(i-1)+j , col.c_str());
            glp_set_col_bnds(lp , track_size*(i-1)+j , GLP_DB , 0 , 1);
            glp_set_col_kind(lp , track_size*(i-1)+j , GLP_BV);          // change made
            glp_set_col_stat(lp , track_size*(i-1)+j , GLP_NU);
            if(Hor_Panel_list.iroute_list[i-1]->obs == 0)
                glp_set_obj_coef(lp , track_size*(i-1)+j , compute_y(*(Hor_Panel_list.iroute_list[i-1]),Hor_Panel_list.track[j-1],Hor_Panel_list.LB.second,Hor_Panel_list.RT.second,Hor_Panel_list.Layer,design.routing_net_information[Hor_Panel_list.iroute_list[i-1]->Net_index].Layer_Array_for_Pin ));
            else
                glp_set_obj_coef(lp , track_size*(i-1)+j , 0);
        }
    }
    //cout<<"\n\n\n";
   // cout<<"after add col\n";
    if( (iroute_size+size*track_size+violation_size+obstacle_size)*iroute_size*track_size > max_size ){
        //cout<<"[Error] max_size too small@@\n";
    }
    int turn=0;
    int round=0;
    int opop;
    //cout<<"total i is "<<iroute_size+size*track_size+violation_size+obstacle_size<<endl;
    //cout<<"iroute_size : "<<iroute_size<<endl;
    //cout<<"size is : "<<size<<endl;
    //cout<<"violation_size is : "<<violation_size<<endl;
    //cout<<"obstacle_size is : "<<obstacle_size<<endl;
    //cin>>opop;
    for(int i=1 ; i<=iroute_size+size*track_size+violation_size+obstacle_size ; i++){
        if(i>iroute_size && i<=iroute_size+size*track_size)
            turn++;
        int a,b,obs,which_track;
        if(i>iroute_size+size*track_size && i<=iroute_size+size*track_size+violation_size && case_violation_before == false){
            /*if(violation.size() == 0)
                cout<<"error! the size of violation is empty!\n";*/
            a=violation.begin()->first;
            b=violation.begin()->second;
            //cout<<"begin size is "<<violation.size();
            violation.erase(violation.begin());
            //cout<<"  after size is "<<violation.size()<<endl;
        } 
        else if(i>iroute_size+size*track_size && i<=iroute_size+size*track_size+violation_size && case_violation_before == true){
            turn++;
            /*if(violation.size() == 0)
                cout<<"error! the size of violation is empty!\n";*/
            a=violation.begin()->first;
            b=violation.begin()->second;
            //cout<<"begin size is "<<violation.size();
            if(turn==3){
                violation.erase(violation.begin());
            }
            //cout<<"  after size is "<<violation.size()<<endl;
        }
        else if(i>iroute_size+size*track_size+violation_size){
            obs=vec_obs[turn][0];
            which_track=vec_obs[turn][1];
            //cout<<"turn"<<turn<<"! now obstacle iroute is "<<obs<<" and it need to be put on track "<<which_track<<endl;
            turn++;
        }
        for(int j=1 ; j<=iroute_size*track_size ; j++){
            ia[(i-1)*(iroute_size*track_size)+j]=i;
            ja[(i-1)*(iroute_size*track_size)+j]=j;
            
            //SOLVE : ONE IROUTE PUT IN ONE TRACK
            if(i<=iroute_size){
                if(j>=(i-1)*track_size+1 && j<=(i)*track_size){
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                    /*cout<<"ia["<<(i-1)*(iroute_size*track_size)+j<<"]="<<i<<"     ";
                     cout<<"ja["<<(i-1)*(iroute_size*track_size)+j<<"]="<<j<<"     ";
                     cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]=1"<<endl;*/
                }
                else{
                    ar[(i-1)*(iroute_size*track_size)+j]=0;
                    /*cout<<"ia["<<(i-1)*(iroute_size*track_size)+j<<"]="<<i<<"     ";
                     cout<<"ja["<<(i-1)*(iroute_size*track_size)+j<<"]="<<j<<"     ";
                     cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]=0"<<endl;*/
                }
            }
            //overlap condition
            else if(i<=iroute_size+size*track_size && i>iroute_size){
                bool exist=false;
                for(int k=0 ; k<vec_ilp[round].size() ; k++){
                    if( j==(vec_ilp[round][k]-1)*track_size+turn ){
                        exist=true;
                        break;
                    }
                }
                if(exist && vec_ilp[round].size()>1){
                    /*cout<<"ia["<<(i-1)*(iroute_size*track_size)+j<<"]="<<i<<"     ";
                     cout<<"ja["<<(i-1)*(iroute_size*track_size)+j<<"]="<<j<<"     ";
                     cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<1<<endl;*/
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                }
                else{
                    ar[(i-1)*(iroute_size*track_size)+j]=0;
                    //cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]=0"<<endl;
                }
            }
            else if(i>iroute_size+size*track_size && i<=iroute_size+size*track_size+violation_size && case_violation_before == false){
                //first put down ane second put top better
                if(j>(a-1)*track_size && ( j-((a-1)*track_size) )<=3 ){
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                    /*cout<<"now down is "<<a<<"  top is "<<b<<endl;
                    cout<<"ia["<<(i-1)*(iroute_size*track_size)+j<<"]="<<i<<"     ";
                    cout<<"ja["<<(i-1)*(iroute_size*track_size)+j<<"]="<<j<<"     ";
                    cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<1<<endl;*/
                }
                else if(j<=b*track_size && (b*track_size-j)<3 ){
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                    /*cout<<"now down is "<<a<<"  top is "<<b<<endl;
                    cout<<"ia["<<(i-1)*(iroute_size*track_size)+j<<"]="<<i<<"     ";
                    cout<<"ja["<<(i-1)*(iroute_size*track_size)+j<<"]="<<j<<"     ";
                    cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<1<<endl;*/
                }
                else{ 
                    ar[(i-1)*(iroute_size*track_size)+j]=0;
                    //cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<0<<endl;
                }
            }
            else if(i>iroute_size+size*track_size && i<=iroute_size+size*track_size+violation_size && case_violation_before == true){
                //y(a,track)
                if( j == ((a-1)*track_size+turn) ){
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                    //cout<<"now down is "<<a<<"  top is "<<b<<endl;
                    //cout<<"ia["<<(i-1)*(iroute_size*track_size)+j<<"]="<<i<<"     ";
                    //cout<<"ja["<<(i-1)*(iroute_size*track_size)+j<<"]="<<j<<"     ";
                    //cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<1<<endl;
                }
                //y(b,track+1.....15)
                else if( j>(b-1)*track_size+turn && j<=b*track_size ){
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                    //cout<<"now down is "<<a<<"  top is "<<b<<endl;
                    //cout<<"ia["<<(i-1)*(iroute_size*track_size)+j<<"]="<<i<<"     ";
                    //cout<<"ja["<<(i-1)*(iroute_size*track_size)+j<<"]="<<j<<"     ";
                    //cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<1<<endl;
                }
                else
                    ar[(i-1)*(iroute_size*track_size)+j]=0;
            }
            else{
                if(j == (obs-1)*track_size+which_track ){
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                    //cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<1<<endl;
                }
                else{
                    ar[(i-1)*(iroute_size*track_size)+j]=0;   
                }
            }
        }
        if(turn==track_size && i<=iroute_size+size*track_size){
            //cout<<"[Warning] : now is end of the vector!!!!!!\n";
            round++;
            turn=0;
        }
        else if(i>iroute_size+size*track_size && i<=iroute_size+size*track_size+violation_size && case_violation_before == true && turn == 3){
            turn=0;
        }
        //cout<<"##############################\n";
    }
    
    
    glp_load_matrix(lp , (iroute_size+size*track_size+violation_size+obstacle_size)*iroute_size*track_size , ia , ja , ar);
    glp_simplex(lp , NULL);
    glp_intopt(lp , NULL);     //change made
    //cost=glp_get_obj_val(lp);
    cost=glp_mip_obj_val(lp);
    double data[1+iroute_size][1+track_size];
    for(int i=1;i<=iroute_size;i++){
        /*if (Hor_Panel_list.Layer == 1 && (Hor_Panel_list.LB.second == 279300 || Hor_Panel_list.RT.second == 279300)) {
         cout << "Panel: LB.first = " << Hor_Panel_list.LB.first << ", LB.second = " << Hor_Panel_list.LB.second << endl;
         cout << "       RT.first = " << Hor_Panel_list.RT.first << ", RT.second = " << Hor_Panel_list.RT.second << endl;
         for (int j = 0; j < track_size; j++)
         cout << "Track " << j << " = " <<Hor_Panel_list.track[j] << endl;
         cout << "Before: \n";
         cout << "Iroute: LB.first = " << Hor_Panel_list.iroute_list[i-1]->LB.first << ", LB.second = " << Hor_Panel_list.iroute_list[i-1]->LB.second << endl;
         cout << "        RT.first = " << Hor_Panel_list.iroute_list[i-1]->RT.first << ", RT.second = " << Hor_Panel_list.iroute_list[i-1]->RT.second << endl;
         }*/
        for(int j=1;j<=track_size;j++){
            data[i][j]=glp_mip_col_val(lp, (i-1)*track_size+j);
            if( glp_mip_col_val(lp,(i-1)*track_size+j)==1 ){
                /*if(i==31){
                    cout<<"now down in track "<<j<<" is true"<<endl;
                }
                if(i==44){
                    cout<<"now top in track "<<j<<" is true"<<endl;
                }*/
                if(Hor_Panel_list.iroute_list[i-1]->obs == 0){
                    Hor_Panel_list.iroute_list[i-1]->LB.second=Hor_Panel_list.track[j-1];
                    Hor_Panel_list.iroute_list[i-1]->RT.second=Hor_Panel_list.track[j-1];
                }
                /*if(Hor_Panel_list.iroute_list[i-1]->LB.first==360000 ){
                    cout<<"now top is "<<i-1<<" put on track "<<j-1<<endl;
                }
                if(Hor_Panel_list.iroute_list[i-1]->LB.first==336000){
                    cout<<"now down is "<<i-1<<" put on track "<<j-1<<endl;
                }*/
                /*if (Hor_Panel_list.Layer == 1 && (Hor_Panel_list.LB.second == 279300 || Hor_Panel_list.RT.second == 279300)) {
                 cout << "After: \n";
                 cout << "Iroute: LB.first = " << Hor_Panel_list.iroute_list[i-1]->LB.first << ", LB.second = " << Hor_Panel_list.iroute_list[i-1]->LB.second << endl;
                 cout << "        RT.first = " << Hor_Panel_list.iroute_list[i-1]->RT.first << ", RT.second = " << Hor_Panel_list.iroute_list[i-1]->RT.second << endl;
                 string pause_;
                 cin >> pause_;
                 }*/
            }
        }
    }
    /*printf("\nVertical length cost = %g\n",cost);
    for(int i=1;i<=iroute_size;i++){
     for(int j=1;j<=track_size;j++){
     printf("y%d%d = %g ",i,j,data[i][j]);
     }
     printf("\n");
     }*/
    /*int vio=0;
    if(layer>2){
        for(int j=1;j<=track_size;j++){
            cout<<"size : "<<Hor_Panel_list2[layer-1].size()<<endl;
            for(int k=0 ; k<Hor_Panel_list2[layer-1].size() ; k++){
                int time=0;
                for(int i=1;i<=iroute_size;i++){
                    map<int,bool>::iterator iter;
                    if(data[i][j]==1){
                        //cout<<"true!\n\n\n";
                        iter=Hor_Panel_list2[layer-1][k].net_check.find(Hor_Panel_list.iroute_list[i-1]->Net_index);
                        if(iter!=Hor_Panel_list2[layer-1][k].net_check.end()){
                            time++;
                            //cout<<"find one!\n\n\n\n\n";
                        }
                    }
                }
                if(time>=3){
                    cout<<"Violation!!!\n\n\n\n\n\n\n";
                    vio++;
                    penalty++;
                }
            }
        }
        if(max_violation<vio)
            max_violation=vio;
        cout<<"this panel violation is "<<vio<<endl;
        cout<<"now penalty is "<<penalty<<endl;
        cout<<"max is "<<max_violation<<endl;
    }*/
    

    glp_delete_prob(lp);
    delete [] ia;
    delete [] ja;
    delete [] ar;
    //cout<<"finish horizontal\n";
}
void solve_problem_vertical(Panel &Ver_Panel_list,Design_Information &design,int track_width,int layer,vector<vector<Panel> > &Ver_Panel_list2){
    glp_term_out(GLP_OFF);
    int number;
    int track_size,iroute_size;
    track_size=Ver_Panel_list.track.size();
    iroute_size=Ver_Panel_list.iroute_list.size();
    //cout<<"iroute size : "<<iroute_size<<"    track size : "<<track_size<<endl;
    glp_prob *lp2;
    int *ia,*ja;
    double *ar;
    double cost;
    
    multimap<int,int> map;
    multimap<int,int>::iterator it;
    int size=0;
    int obstacle_size=0;
    vector<int> vec_tmp;
    vector<vector<int> > vec_obs;
    
    for(int i=0 ; i<iroute_size;i++){
        //cout<<"now is iroute "<<i<<"  LB.first = "<<Ver_Panel_list.iroute_list[i]->LB.first<<"  RT.first = "<<Ver_Panel_list.iroute_list[i]->RT.first<<endl;
        if(Ver_Panel_list.iroute_list[i]->obs == 1){
            obstacle_size++;
            vec_tmp.push_back(i+1);
          //  cout<<"vec_tmp push_back "<<i+1;
           // cout<<"  now LB.first = "<<Ver_Panel_list.iroute_list[i]->LB.first<<"  RT.first = "<<Ver_Panel_list.iroute_list[i]->RT.first<<endl;
            for(int j=0 ; j<track_size ; j++){
                if(Ver_Panel_list.iroute_list[i]->LB.first<=Ver_Panel_list.track[j] && Ver_Panel_list.iroute_list[i]->RT.first>=Ver_Panel_list.track[j]){
                    vec_tmp.push_back(j+1);
             //       cout<<"vec_tmp push_back track of "<<j+1<<endl;
                }
            }
            vec_obs.push_back(vec_tmp);
        }
        vec_tmp.clear();
    }
    
    //cout<<"begin overlap\n";
    vector<int> vec_old,vec_new;
    vector< vector<int> > vec_ilp;
    if(iroute_size>1){
        vector<int> iroute_start;
        for(int i=0 ; i<iroute_size ; i++){
            iroute_start.push_back(Ver_Panel_list.iroute_list[i]->LB.second);
            iroute_start.push_back(Ver_Panel_list.iroute_list[i]->RT.second);
        }
        sort(iroute_start.begin() , iroute_start.end());
        
        for(int i=0 ; i<iroute_start.size() ; i++){
            for(int j=0 ; j<iroute_size ; j++){
                if( Ver_Panel_list.iroute_list[j]->LB.second<=(iroute_start[i]+track_width) && Ver_Panel_list.iroute_list[j]->RT.second>=(iroute_start[i]-track_width) )
                    vec_new.push_back(j+1);
            }
            vector<int> intersection;
            set_intersection(vec_new.begin() , vec_new.end() , vec_old.begin() , vec_old.end() , inserter(intersection , intersection.begin() ) );
            if(intersection == vec_old && intersection != vec_new){
                //cout<<"now is i : "<<i<<"  cor is : "<<iroute_start[i]<<"   case 1 "<<endl;
                vec_old=vec_new;
                if(i == iroute_start.size()-1 )
                    vec_ilp.push_back(vec_new);
            }
            else if(intersection == vec_new ){
                if(intersection != vec_old){
                    //cout<<"now is i : "<<i<<"  cor is : "<<iroute_start[i]<<"   case 2 "<<endl;
                    vec_ilp.push_back(vec_old);
                    vec_old=vec_new;
                }
            }
            else{
                //cout<<"now is i : "<<i<<"  cor is : "<<iroute_start[i]<<"   case 3 "<<endl;
                vec_ilp.push_back(vec_old);
                vec_old=vec_new;
                if(i == iroute_start.size()-1 )
                    vec_ilp.push_back(vec_new);
            }
            vec_new.clear();
        }
        size=vec_ilp.size();
        //cout<<"begin size is : "<<size<<endl;
        if(size>0){
            for(int i=0 ; i<vec_ilp.size()-1 ; i++){
                vector<int> intersection;
                if(vec_ilp[i].size()>1 && vec_ilp[i+1].size()>1){
                    set_intersection(vec_ilp[i].begin() , vec_ilp[i].end() , vec_ilp[i+1].begin() , vec_ilp[i+1].end() , inserter(intersection , intersection.begin() ));
                    if(vec_ilp[i] == intersection && vec_ilp[i+1] != intersection){
                        for(int j=i ; j<vec_ilp.size()-1 ; j++){
                            vec_ilp[j]=vec_ilp[j+1];
                        }
                        i--;
                        vec_ilp.resize(vec_ilp.size()-1);
                    }
                    else if(vec_ilp[i] != intersection && vec_ilp[i+1] == intersection){
                        for(int j=i+1 ; j<vec_ilp.size()-1 ; j++){
                            vec_ilp[j]=vec_ilp[j+1];
                        }
                        i--;
                        vec_ilp.resize(vec_ilp.size()-1);
                    }
                }
            }
            /*for(int i=0 ; i<iroute_size-1 ; i++){
             for(int j=i+1 ; j<iroute_size ; j++){
             if(rec_overlap(Ver_Panel_list.iroute_list[i]->LB , Ver_Panel_list.iroute_list[i]->RT , Ver_Panel_list.iroute_list[j]->LB , Ver_Panel_list.iroute_list[j]->RT) ==1 ){
             size++;
             map.insert( pair<int,int>(i,j) );
             }
             }
             }*/
            
            size=vec_ilp.size();
        }
        //cout<<"finish vector part\n";
    }
    //cout<<"after overlap\n";

    multimap<int,int> violation;
    vector<vector<int> > violation_map;
    vector<int> violation_tmp;
    if(layer>3){
        int vio=0;
        for(int k=0 ; k<Ver_Panel_list2[layer-1].size() ; k++){
            int time=0;
            for(int i=1;i<=iroute_size;i++){
                //map< int , vector<TRACKASSIGNMENT::IRoute*> >::iterator iter;
                std::map< int, vector<TRACKASSIGNMENT::IRoute*> >::iterator iter;
                iter=Ver_Panel_list2[layer-1][k].net_check.find(Ver_Panel_list.iroute_list[i-1]->Net_index);
                if(iter!=Ver_Panel_list2[layer-1][k].net_check.end()){
                    time++;
                    //cout<<"now i = "<<i<<"      iter->second = "<<iter->second[0]->LB.second<<endl;
                    violation_tmp.push_back(i);
                    violation_tmp.push_back(iter->second[0]->LB.second);
                    violation_tmp.push_back(iter->second[0]->LB.first);
                    violation_map.push_back(violation_tmp);
                    violation_tmp.clear();
                }
            }
            if(time>=2){
                for(int i=0 ; i<violation_map.size()-1 ; i++){
                    for(int j=i+1 ; j<violation_map.size() ; j++){
                        if(violation_map[i][1] == violation_map[j][1]){
                            pair< multimap<int,int>::iterator , multimap<int,int>::iterator >ret;
                            bool exist=false;
                            if(violation_map[i][2]>violation_map[j][2]){
                                ret=violation.equal_range(violation_map[i][0]);
                                for(multimap<int,int>::iterator it=ret.first ; it!=ret.second ; ++it){
                                    if(violation_map[j][0] == it->second){
                                        exist=true;
                                        break;
                                    }
                                }
                                if(!exist){
                                    violation.insert( pair<int,int>(violation_map[i][0] , violation_map[j][0]) );
                                    //cout<<"Violation OF "<<violation_map[i][0]<<" "<<violation_map[j][0]<<" !!!\n\n";
                                }
                            }
                            else{
                                ret=violation.equal_range(violation_map[j][0]);
                                for(multimap<int,int>::iterator it=ret.first ; it!=ret.second ; ++it){
                                    if(violation_map[i][0] == it->second){
                                        exist=true;
                                        break;
                                    }
                                }
                                if(!exist){
                                    violation.insert( pair<int,int>(violation_map[j][0] , violation_map[i][0]) );
                                    //cout<<"Violation OF "<<violation_map[j][0]<<" "<<violation_map[i][0]<<" !!!\n\n";    
                                }
                            }
                        }
                    }
                }
                vio++;
                penalty++;
            }
            violation_map.clear();
        }

        if(max_violation<vio)
            max_violation=vio;
        /*cout<<"max is "<<max_violation<<endl;
        cout<<"violation is "<<vio<<endl;
        cout<<"penalty is "<<penalty<<endl;
        cout<<"violation size is "<<violation.size()<<endl;*/
    }
    int violation_size=violation.size();
    //int violation_size=0;
    bool case_violation_before=false;


   // cout<<"after violation\n";
    //cout<<(iroute_size+size*track_size+violation_size+obstacle_size)*iroute_size*track_size<<"  too big!"<<endl;
    if( (iroute_size+size*track_size+violation_size+obstacle_size)*iroute_size*track_size > max_size ){
      //  cout<<"[Error] max_size too small@@\n";
       // cout<<(iroute_size+size*track_size+violation_size+obstacle_size)*iroute_size*track_size<<"  too big!"<<endl;
    }

    ia = new int[1+(iroute_size+size*track_size+violation_size+obstacle_size)*iroute_size*track_size];
    ja = new int[1+(iroute_size+size*track_size+violation_size+obstacle_size)*iroute_size*track_size];
    ar = new double[1+(iroute_size+size*track_size+violation_size+obstacle_size)*iroute_size*track_size];

   // cout<<"after declaration\n";

    lp2=glp_create_prob();
    glp_set_prob_name(lp2 , "ILP2");
    glp_set_obj_dir(lp2 , GLP_MIN);
    glp_add_rows(lp2 , iroute_size+size*track_size+violation_size+obstacle_size);
    //cout<<"now add "<<iroute_size+size*track_size<<" rows\n";
    for(int i=1;i<=iroute_size+size*track_size+violation_size+obstacle_size;i++){
        string row="row";
        stringstream ss;
        ss<<i;
        string str;
        ss>>str;
        row+=str;
        //cout<<"now row is : "<<i<<endl;
        if(i<=iroute_size){
            //cout<<"now set row name : "<<row.c_str()<<endl;
            //cout<<"now set row bonds : 1.0 1.0 \n";
            glp_set_row_name(lp2 , i , row.c_str());
            glp_set_row_bnds(lp2 , i , GLP_FX , 1.0 , 1.0);
        }
        else if(i<=iroute_size+size*track_size && i>iroute_size){
            //cout<<"now set row name : "<<row.c_str()<<endl;
            //cout<<"now set row bonds : 0.0 1.0 \n";
            glp_set_row_name(lp2 , i , row.c_str());
            glp_set_row_bnds(lp2 , i , GLP_DB , 0.0 , 1.0);
        }
        else if(i>iroute_size+size*track_size && i<=iroute_size+size*track_size+violation_size && case_violation_before == false){
            glp_set_row_name(lp2 , i , row.c_str());
            glp_set_row_bnds(lp2 , i , GLP_DB , 0.0 , 1.0);
        }
        else if(i>iroute_size+size*track_size && i<=iroute_size+size*track_size+violation_size && case_violation_before == true){
            glp_set_row_name(lp2 , i , row.c_str());
            glp_set_row_bnds(lp2 , i , GLP_FX , 0.0 , 0.0);
        }
        else{
            glp_set_row_name(lp2 , i , row.c_str());
            glp_set_row_bnds(lp2 , i , GLP_FX , 1.0 , 1.0);
        }
    }
    //cout<<"after add rows\n";
    //cout<<"\n\n\n";
    glp_add_cols(lp2 , iroute_size*track_size);
    //cout<<"now add "<<iroute_size*track_size<<" cols\n";
    for(int i=1 ; i<=iroute_size ; i++){
        for(int j=1 ; j<=track_size ; j++){
            stringstream ss;
            string col="y";
            string x;
            string y;
            ss<<i;
            ss>>x;
            ss.str("");
            ss.clear();
            ss<<j;
            ss>>y;
            col=col+x+y;
            /*cout<<"now col is : "<<track_size*(i-1)+j<<endl;
            cout<<"now set col name : "<<col.c_str()<<endl;
            cout<<"now set col bonds : 0.0 1.0\n";
            cout<<"now set obj coef -> compute : "<<i<<" track : "<<j<<endl;*/
            glp_set_col_name(lp2 , track_size*(i-1)+j , col.c_str());
            glp_set_col_bnds(lp2 , track_size*(i-1)+j , GLP_DB , 0 , 1);    
            glp_set_col_kind(lp2 , track_size*(i-1)+j , GLP_BV);          // change made
            glp_set_col_stat(lp2 , track_size*(i-1)+j , GLP_NU);
            if(Ver_Panel_list.iroute_list[i-1]->obs == 0)
                glp_set_obj_coef(lp2 , track_size*(i-1)+j , compute_x( *(Ver_Panel_list.iroute_list[i-1]) , Ver_Panel_list.track[j-1] , Ver_Panel_list.LB.first , Ver_Panel_list.RT.first ,  Ver_Panel_list.Layer , design.routing_net_information[Ver_Panel_list.iroute_list[i-1]->Net_index].Layer_Array_for_Pin ) );
            else
                glp_set_obj_coef(lp2 , track_size*(i-1)+j , 0);
        }
    }
    //cout<<"\n\n\n";
    //cout<<"after add cols\n";
    int turn=0;
    int round=0;
    for(int i=1 ; i<=iroute_size+size*track_size+violation_size+obstacle_size ; i++){
        if(i>iroute_size && i<=iroute_size+size*track_size)
            turn++;
        int a,b,obs,which_track;
        if(i>iroute_size+size*track_size && i<=iroute_size+size*track_size+violation_size && case_violation_before == false){
            a=violation.begin()->first;
            b=violation.begin()->second;
            violation.erase(violation.begin());
        }
        else if(i>iroute_size+size*track_size && i<=iroute_size+size*track_size+violation_size && case_violation_before == true){
            turn++;
            a=violation.begin()->first;
            b=violation.begin()->second;
            if(turn == 3)
                violation.erase(violation.begin());
        }
        else if(i>iroute_size+size*track_size+violation_size){
            obs=vec_obs[turn][0];
            which_track=vec_obs[turn][1];
            //cout<<"turn"<<turn<<"! now obstacle iroute is "<<obs<<" and it need to be put on track "<<which_track<<endl;
            turn++;
        }
        for(int j=1 ; j<=iroute_size*track_size ; j++){
            
            ia[(i-1)*(iroute_size*track_size)+j]=i;
            ja[(i-1)*(iroute_size*track_size)+j]=j;
            /*cout<<"ia["<<(i-1)*(iroute_size*track_size)+j<<"]="<<i<<"     ";
            cout<<"ja["<<(i-1)*(iroute_size*track_size)+j<<"]="<<j<<"     "<<endl;*/
            //SOLVE : ONE IROUTE PUT IN ONE TRACK
            if(i<=iroute_size){
                if(j>=(i-1)*track_size+1 && j<=(i)*track_size){
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                    //cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]=1"<<endl;
                }
                else{
                    ar[(i-1)*(iroute_size*track_size)+j]=0;
                    //cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]=0"<<endl;
                }
            }
            //SOLVE : X LENGTH CANT'T EXCEED TRACK X'S SIZE
            //NOT SOLVE : MUST HAVE DISTANCE BETWEEN IROUTE
            else if(i<=iroute_size+size*track_size && i>iroute_size){
                bool exist=false;
                
                for(int k=0 ; k<vec_ilp[round].size() ; k++){
                    if( j==(vec_ilp[round][k]-1)*track_size+turn ){
                        exist=true;
                        break;
                    }
                }
                if(exist && vec_ilp[round].size()>1){
                    /*cout<<"ia["<<(i-1)*(iroute_size*track_size)+j<<"]="<<i<<"     ";
                     cout<<"ja["<<(i-1)*(iroute_size*track_size)+j<<"]="<<j<<"     ";
                     cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<1<<endl;*/
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                }
                else{
                    ar[(i-1)*(iroute_size*track_size)+j]=0;
                    //cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]=0"<<endl;
                }
            }
            else if(i>iroute_size+size*track_size && i<=iroute_size+size*track_size+violation_size && case_violation_before == false){
                //first put down ane second put top better
                if(j>(a-1)*track_size && ( j-((a-1)*track_size) )<=3 ){
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                    /*cout<<"now down is "<<a<<"  top is "<<b<<endl;
                    cout<<"ia["<<(i-1)*(iroute_size*track_size)+j<<"]="<<i<<"     ";
                    cout<<"ja["<<(i-1)*(iroute_size*track_size)+j<<"]="<<j<<"     ";
                    cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<1<<endl;*/
                }
                else if(j<=b*track_size && (b*track_size-j)<3 ){
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                    /*cout<<"now down is "<<a<<"  top is "<<b<<endl;
                    cout<<"ia["<<(i-1)*(iroute_size*track_size)+j<<"]="<<i<<"     ";
                    cout<<"ja["<<(i-1)*(iroute_size*track_size)+j<<"]="<<j<<"     ";
                    cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<1<<endl;*/
                }
                else{ 
                    ar[(i-1)*(iroute_size*track_size)+j]=0;
                    //cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<0<<endl;
                }
            }
            else if(i>iroute_size+size*track_size && i<=iroute_size+size*track_size+violation_size && case_violation_before == true){
                //y(a,track)
                if( j == ((a-1)*track_size+turn) ){
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                    /*cout<<"now down is "<<a<<"  top is "<<b<<endl;
                    cout<<"ia["<<(i-1)*(iroute_size*track_size)+j<<"]="<<i<<"     ";
                    cout<<"ja["<<(i-1)*(iroute_size*track_size)+j<<"]="<<j<<"     ";
                    cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<1<<endl;*/
                }
                //y(b,track+1.....15)
                else if( j>(b-1)*track_size+turn && j<=b*track_size ){
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                    /*cout<<"now down is "<<a<<"  top is "<<b<<endl;
                    cout<<"ia["<<(i-1)*(iroute_size*track_size)+j<<"]="<<i<<"     ";
                    cout<<"ja["<<(i-1)*(iroute_size*track_size)+j<<"]="<<j<<"     ";
                    cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<1<<endl;*/
                }
                else
                    ar[(i-1)*(iroute_size*track_size)+j]=0;
            }
            else{
                if(j == (obs-1)*track_size+which_track ){
                    ar[(i-1)*(iroute_size*track_size)+j]=1;
                    cout<<"ar["<<(i-1)*(iroute_size*track_size)+j<<"]="<<1<<endl;
                }
                else{
                    ar[(i-1)*(iroute_size*track_size)+j]=0;   
                }
            }
        }
        if(turn==track_size && i<=iroute_size+size*track_size){
            round++;
            turn=0;
        }
        else if(i>iroute_size+size*track_size &&  i<=iroute_size+size*track_size+violation_size && case_violation_before == true && turn == 3){
            turn=0;
        }
        //cout<<"##############################\n";
    }
   // cout<<"after set line\n";
    glp_load_matrix(lp2 , (iroute_size+size*track_size+violation_size+obstacle_size)*iroute_size*track_size , ia , ja , ar);
    glp_simplex(lp2, NULL);
    glp_intopt(lp2 , NULL);     //change made
    //cost=glp_get_obj_val(lp);
    cost=glp_mip_obj_val(lp2);
    double data[1+iroute_size][1+track_size];
    for(int i=1;i<=iroute_size;i++){
        for(int j=1;j<=track_size;j++){
            data[i][j]=glp_mip_col_val(lp2, (i-1)*track_size+j);
            if( glp_mip_col_val(lp2, (i-1)*track_size+j) == 1 ){
                if(Ver_Panel_list.iroute_list[i-1]->obs == 0){
                    Ver_Panel_list.iroute_list[i-1]->LB.first=Ver_Panel_list.track[j-1];
                    Ver_Panel_list.iroute_list[i-1]->RT.first=Ver_Panel_list.track[j-1];
                }
            }
        }
    }
    /*printf("\nVertical length cost = %g\n",cost);
    //cout<<"vertical length cost "<<cost<<endl;
    for(int i=1;i<=iroute_size;i++){
     for(int j=1;j<=track_size;j++){
     printf("y%d%d = %g ",i,j,data[i][j]);
     }
     printf("\n");
     }*/
    
    
    
    glp_delete_prob(lp2);                //why
    delete [] ia;
    delete [] ja;
    delete [] ar;
    //cout<<"finish vertical\n";
}

