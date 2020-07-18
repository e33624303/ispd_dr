#ifndef DEF_PARSER_H
#define DEF_PARSER_H
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#ifndef WIN32
#   include <unistd.h>
#endif /* not WIN32 */
#include "def/defrReader.hpp"
#include "def/defiAlias.hpp"
#include "Structure.h"
using namespace Parser;
int Def_main(char * DefFilName, int argc, char **argv , Design &design);
//Design This_Design;

#endif