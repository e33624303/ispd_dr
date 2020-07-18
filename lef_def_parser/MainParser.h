#ifndef PARSER_H
#define PARSER_H
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <map>
#include <string>
#include <time.h>
#include <unordered_map>

#include "Structure.h"
#include "Lef_Parser.h"
#include "Def_Parser.h"
#include "GuideParser.hpp"
#include "DEF_Writer.h"
using namespace Parser;

void MainParser(int argc, char** argv, Design &design);
//Design This_Design;
void MainWriter(int argc, char** argv, Design &design);
#endif