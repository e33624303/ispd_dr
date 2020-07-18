#ifndef DEF_WRITER_H
#define DEF_WRITER_H
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "def/defwWriter.hpp"
#include "def/defwWriterCalls.hpp"
#include "Structure.h"

int Def_write(int argc, char** argv, Parser::Design &design, string FileName);
#endif