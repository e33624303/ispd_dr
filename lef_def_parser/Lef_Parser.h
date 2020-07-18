#ifndef LEF_PARSER_H
#define LEF_PARSER_H
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>

#include "lef/lef/lefrReader.hpp"
#include "lef/lef/lefwWriter.hpp"
#include "lef/lef/lefiDebug.hpp"
#include "lef/lef/lefiEncryptInt.hpp"
#include "lef/lef/lefiUtil.hpp"
#include "Structure.h"
using namespace Parser;
int Lef_main(char * LefFilName, int argc, char **argv, Design &design);
//Design This_Design;

#endif