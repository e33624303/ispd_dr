CXX		=	g++ -std=c++1y -static -I../lef_5.8-p027/lef/lef -I../def_5.8-p027/def/def -O3 -lglpk
EXE		=	ispd18dr
SRC_DIR	=	.
OBJ_DIR	=	objs
LEF_DIR =	./lef_def_parser/lef/lef
DEF_DIR =   ./lef_def_parser/def
PARSER_DIR =	./lef_def_parser
FLUTE_DIR = ./flute-3.1

FLUTE_SRC     = dist.c dl.c err.c heap.c mst2.c neighbors.c bookshelf_IO.c memAlloc.c flute.c flute_mst.c
FLUTE_OBJ     = $(FLUTE_DIR/FLUTE_SRC:.c=.o)

all: $(EXE)


$(EXE): $(OBJ_DIR)/SpaceEvaluationGraph.o $(OBJ_DIR)/MazeRouteKernel.o $(OBJ_DIR)/history_congestion_graph.o $(OBJ_DIR)/line_end_graph.o $(OBJ_DIR)/small_router.o $(OBJ_DIR)/routing_info.o $(OBJ_DIR)/graph_struct.o $(OBJ_DIR)/mst_template.o $(OBJ_DIR)/Lef_Def_Parser.o $(OBJ_DIR)/LEF_Parser.o $(OBJ_DIR)/DEF_Parser.o $(OBJ_DIR)/DEF_Writer.o $(OBJ_DIR)/dl.o $(OBJ_DIR)/flute.o $(OBJ_DIR)/flute_mst.o $(OBJ_DIR)/dist.o $(OBJ_DIR)/dist.o $(OBJ_DIR)/err.o $(OBJ_DIR)/heap.o $(OBJ_DIR)/mst2.o $(OBJ_DIR)/neighbors.o $(OBJ_DIR)/bookshelf_IO.o $(OBJ_DIR)/memAlloc.o /usr/local/lib/libglpk.a $(LEF_DIR)/liblef.a $(DEF_DIR)/libdef.a
	$(CXX) $^ -o $@ -lrt

$(OBJ_DIR)/MazeRouteKernel.o: $(SRC_DIR)/MazeRouteKernel.cpp $(SRC_DIR)/graph_struct.h $(SRC_DIR)/small_router.h  $(FLUTE_DIR)/flute.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/SpaceEvaluationGraph.o: $(SRC_DIR)/SpaceEvaluationGraph.cpp $(SRC_DIR)/SpaceEvaluationGraph.hpp $(SRC_DIR)/graph_struct.h $(SRC_DIR)/RTree.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/history_congestion_graph.o: $(SRC_DIR)/history_congestion_graph.cpp $(SRC_DIR)/history_congestion_graph.h $(SRC_DIR)/graph_struct.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/line_end_graph.o: $(SRC_DIR)/line_end_graph.cpp $(SRC_DIR)/line_end_graph.h $(SRC_DIR)/graph_struct.h $(SRC_DIR)/routing_info.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/small_router.o: $(SRC_DIR)/small_router.cpp $(SRC_DIR)/small_router.h $(SRC_DIR)/graph_struct.h $(SRC_DIR)/history_congestion_graph.h $(SRC_DIR)/line_end_graph.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/routing_info.o: $(SRC_DIR)/routing_info.cpp $(SRC_DIR)/routing_info.h $(SRC_DIR)/graph_struct.h $(SRC_DIR)/Definition.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/graph_struct.o: $(SRC_DIR)/graph_struct.cpp $(SRC_DIR)/graph_struct.h $(SRC_DIR)/Definition.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/mst_template.o: $(SRC_DIR)/mst_template.cpp $(SRC_DIR)/mst_template.h $(SRC_DIR)/Definition.h | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/LEF_Parser.o: $(PARSER_DIR)/LEF_Parser.cpp $(PARSER_DIR)/Lef_Parser.h $(PARSER_DIR)/Structure.h $(LEF_DIR)/lefrReader.hpp $(LEF_DIR)/lefwWriter.hpp $(LEF_DIR)/lefiEncryptInt.hpp $(LEF_DIR)/lefiDebug.hpp $(LEF_DIR)/lefiUtil.hpp | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/DEF_Parser.o: $(PARSER_DIR)/DEF_Parser.cpp $(PARSER_DIR)/Def_Parser.h $(PARSER_DIR)/Structure.h $(DEF_DIR)/defrReader.hpp $(DEF_DIR)/defiAlias.hpp | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/Lef_Def_Parser.o: $(PARSER_DIR)/Lef_Def_Parser.cpp $(PARSER_DIR)/MainParser.h $(PARSER_DIR)/Def_Parser.h $(PARSER_DIR)/Lef_Parser.h | obj_dir
	$(CXX) -c $< -o $@
	
$(OBJ_DIR)/DEF_Writer.o: $(PARSER_DIR)/DEF_Writer.cpp $(PARSER_DIR)/DEF_Writer.h $(PARSER_DIR)/Structure.h $(DEF_DIR)/defwWriterCalls.hpp $(DEF_DIR)/defwWriter.hpp | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/flute.o: $(FLUTE_DIR)/flute.c $(FLUTE_DIR)/flute.h | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/flute_mst.o: $(FLUTE_DIR)/flute_mst.c $(FLUTE_DIR)/flute.h $(FLUTE_DIR)/dl.h | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/dl.o: $(FLUTE_DIR)/dl.c $(FLUTE_DIR)/dl.h | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/dist.o: $(FLUTE_DIR)/dist.c $(FLUTE_DIR)/global.h | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/err.o: $(FLUTE_DIR)/err.c | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/heap.o: $(FLUTE_DIR)/heap.c $(FLUTE_DIR)/heap.h $(FLUTE_DIR)/err.h | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/mst2.o: $(FLUTE_DIR)/mst2.c $(FLUTE_DIR)/global.h $(FLUTE_DIR)/neighbors.h $(FLUTE_DIR)/dist.h $(FLUTE_DIR)/heap.h $(FLUTE_DIR)/err.h | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/neighbors.o: $(FLUTE_DIR)/neighbors.c $(FLUTE_DIR)/global.h $(FLUTE_DIR)/err.h $(FLUTE_DIR)/dist.h | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/bookshelf_IO.o: $(FLUTE_DIR)/bookshelf_IO.c $(FLUTE_DIR)/memAlloc.h $(FLUTE_DIR)/bookshelf_IO.h | obj_dir
	$(CXX) -c $< -o $@

$(OBJ_DIR)/memAlloc.o: $(FLUTE_DIR)/memAlloc.c $(FLUTE_DIR)/memAlloc.h | obj_dir
	$(CXX) -c $< -o $@
 
obj_dir:
	mkdir -p $(OBJ_DIR)


clean:
	rm -rf $(OBJ_DIR) $(EXE)
	cd ..
