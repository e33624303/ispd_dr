#ifndef DEFINITION_H
#define DEFINITION_H
#include <iostream>
#include <unordered_map>
#include <vector>
using namespace std;

typedef int coor_t;
typedef unsigned int UInt;
typedef unsigned long long ULL;
typedef int CostInt;

#define CHECK_GRIDMAP_CONSTRUCTION
#define HARD_BLOCKAGE -4
#define SPACING_BLOCKAGE -2
#define NO_BLOCKAGE -1
#define SAMENET -3
#define ENCLOSURE_OFFSET 0

#define WRONG_DIR_PENALTY 2
#define OUY_OF_TRACK_PENALTY 3
#define OUT_OF_GUIDE_PENALTY 2
#define ViaCost 3
#define STACK_VIA_PENALTY 500
#define SHORT_PENALTY 500
#define CONGESTION_PENALTY 100
#define PINSHAPE_COVER_PENALTY 500
#define DUMMY_VIA_PENALTY 50

#define ALLOW_SHORT
#define REJECT_M1_ROUTING
#define PseudoPatch
//#define A_star_TargetBox
#define PinCoverCost
#define A_Star_Distance_Penalty 50

#define VIA_TYPE_BIT 5

//#define FIB_HEAP
//#define NCTU_GR_HEAP
#define LUCKYNUMBER 1
#define Short_Routing_Region // using bounding box to restrict routing region when short route 
//#define AllUseGuideRange


// Via Blockage Map
// Open one of them

//#define _SpaceEvaluationGraph_
#define _VIA_BIT_MAP_
#define _EOL_CHECKING_
//#define _CrossLineGraph_

#endif