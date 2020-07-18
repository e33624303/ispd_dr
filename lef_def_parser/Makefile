CXX		=	g++ -std=c++1y -I../lef_5.8-p027/lef/lef -I../def_5.8-p027/def/def -O3
EXE		=	LEF_PARSER
SRC_DIR	=	src
OBJ_DIR	=	objs
LEF_DIR =	../lef_5.8-p027/lef/lef
DEF_DIR =   ../def_5.8-p027/def/def

all: $(EXE)


$(EXE): $(OBJ_DIR)/main.o $(OBJ_DIR)/LEF_Parser.o $(OBJ_DIR)/DEF_Parser.o ../lef_5.8-p027/lef/lib/liblef.a ../def_5.8-p027/def/lib/libdef.a
	$(CXX) $^ -o $@ -lrt

$(OBJ_DIR)/LEF_Parser.o: $(SRC_DIR)/LEF_Parser.cpp $(SRC_DIR)/Lef_Parser.h $(SRC_DIR)/Structure.h $(LEF_DIR)/lefrReader.hpp $(LEF_DIR)/lefwWriter.hpp $(LEF_DIR)/lefiEncryptInt.hpp $(LEF_DIR)/lefiDebug.hpp $(LEF_DIR)/lefiUtil.hpp | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/DEF_Parser.o: $(SRC_DIR)/DEF_Parser.cpp $(SRC_DIR)/Def_Parser.h $(SRC_DIR)/Structure.h $(DEF_DIR)/defrReader.hpp $(DEF_DIR)/defiAlias.hpp | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/main.o: $(SRC_DIR)/Lef_Def_Parser.cpp $(SRC_DIR)/Def_Parser.h $(SRC_DIR)/Lef_Parser.h | obj_dir
	$(CXX) -c $< -o $@

	
obj_dir:
	mkdir -p $(OBJ_DIR)

clean:
	rm -rf $(OBJ_DIR) $(EXE)
	cd filter && make clean
	cd ..
