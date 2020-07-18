#include "routing_info.h"

/*void RoutingLayerInfos::GetRoutingLayerInfos(oaTech *tech, oaBlock *top_block)
{
	this->InitLayerInfos(tech, top_block);
	this->GetLineEndRulesInTechDB(tech);
	this->GetRoutingViaInfos(tech);
	this->SetViaEncInfos();
	// debug
	cout << "========== Routing Layer Infos ==========" << endl;
	for(LayerInfo &info : this->routing_layer_infos)
		cout << info;
}*/

/*void RoutingLayerInfos::InitLayerInfos(oaTech *tech, oaBlock *top_block)
{
	if(tech == NULL || top_block == NULL)
	{
		cout << "The 'tech' or the 'top block' is failed!" << endl;
		exit(1);
	}

	cout << "========== Get Constraints from Tech DB ==========" << endl;
	this->DBU = top_block->getDBUPerUU();

	cout << "\tGet the Valid Routing Layer..." << endl;
	oaLayerArray validRoutingLayers;
	oaConstraintGroup* pLefDefaultCG = oaConstraintGroup::find(tech, oaString("LEFDefaultRouteSpec"));
	if(pLefDefaultCG)
	{
		oaSimpleConstraint* pLayerConst = oaSimpleConstraint::find(pLefDefaultCG, oaSimpleConstraintDef::get(oacValidRoutingLayers));
		if(pLayerConst)
		{
			oaLayerArrayValue* pLayerAryVal = (oaLayerArrayValue*)pLayerConst->getValue();
			if(pLayerAryVal)
			{
				// oaLayerArray routingLayerAry;
				pLayerAryVal->get(validRoutingLayers);
			}
		}
		cout << "\tDone!!! Get the Valid Routing Layers." << endl << endl;
	}
	else
	{
		cout << "\tUnable to Get the Valie Routing Layers." << endl << endl;
		exit(1);
	}

	
	this->routing_layer_infos.resize(validRoutingLayers.getSize());
	if(pLefDefaultCG)
	{
		cout << "\tGet the Layer Constraints..." << endl;
		oaLayerConstraintDef* pMinWidthDef = oaLayerConstraintDef::get(oacMinWidth);
		oaLayerConstraintDef* pMinSpacingDef = oaLayerConstraintDef::get(oacMinSpacing);
		oaLayerConstraintDef* pHGridPitchDef = oaLayerConstraintDef::get(oacHorizontalRouteGridPitch);
		oaLayerConstraintDef* pVGridPitchDef = oaLayerConstraintDef::get(oacVerticalRouteGridPitch);
		oaLayerConstraintDef* pHGridOffsetDef = oaLayerConstraintDef::get(oacHorizontalRouteGridOffset);
		oaLayerConstraintDef* pVGridOffsetDef = oaLayerConstraintDef::get(oacVerticalRouteGridOffset);

		for(oaUInt4 index = 0; index < validRoutingLayers.getSize(); index++)
		{
			LayerInfo &layer_info = this->routing_layer_infos[index]; 

			oaLayerNum layer = validRoutingLayers[index];
			layer_info.layer = layer;
			cout << "\tLayer " << layer << endl;
			

			oaPhysicalLayer* pLayer = oaPhysicalLayer::find(tech, layer);
			
			oaString layerName;
			pLayer->getName(layerName);
			layer_info.layer_name = layerName;

			oaPrefRoutingDir routingDir = pLayer->getPrefRoutingDir();
			oaBoolean isVertical = false;
			// oaBoolean validPrefDir = false;
			if(routingDir == oacVertPrefRoutingDir)
			{
				isVertical = true;
				// validPrefDir = true;
			}
			else if(routingDir == oacHorzPrefRoutingDir)
			{
				isVertical = false;
				// validPrefDir = true;
			}
			else
			{
				isVertical = false;
				// validPrefDir = false;
			}
			layer_info.is_vertical = isVertical;

			oaLayerConstraint* pSpacingConstraint = oaLayerConstraint::find(pLefDefaultCG, layer, pMinSpacingDef);
			cout << "\t\tGet Min Spacing..." << endl;
			if(pSpacingConstraint)
			{
				oaValue* pValue = pSpacingConstraint->getValue();
				switch(pValue->getType())
				{
					case oacIntValueType:
					{
						oaInt4 nSpacing = ((oaIntValue*)pValue)->get();
						cout << "\t\tGet spacing value!" << endl;
						cout << "\t\tspacing\t" << nSpacing << endl;
						layer_info.spacing = nSpacing;
					}
					break;
					case oacInt1DTblValueType:
					{
						oa1DLookupTbl<oaInt4, oaInt4> lookTbl1D;
						((oaInt1DTblValue*)pValue)->get(lookTbl1D);
						cout << "\t\tGet spacing 1DLookTbl!" << endl;
						cout << "\t\tUnable handle 1DLookTbl!" << endl;
						exit(1);
					}
					break;
					case oacInt2DTblValueType:
					{
						oa2DLookupTbl<oaInt4, oaInt4, oaInt4> lookTbl2D;
						((oaInt2DTblValue*)pValue)->get(lookTbl2D);
						cout << "\t\tGet spacing 2DLookTbl!" << endl;
						cout << "\t\tUnable handle 2DLookTbl!" << endl;
						exit(1);
					}
					break;
					default:
						;
				}
			}
			else
			{
				cout << "\t\tUnable to Get the Min Spacing." << endl;
			}

			oaLayerConstraint* pWidthConstraint = oaLayerConstraint::find(pLefDefaultCG, layer, pMinWidthDef); 
			cout << "\t\tGet Min Width..." << endl;
			if(pWidthConstraint)
			{
				oaIntValue* pMinWidthVal = (oaIntValue*) pWidthConstraint->getValue();
				if(pMinWidthVal)
				{
					oaInt4 minWidthVal;
					minWidthVal = pMinWidthVal->get();
					layer_info.width = minWidthVal; 
					// cout << (oaFloat)minWidthVal/DBU << endl;
				}
			}
			else
			{
				cout << "\t\tUnable to Get the Min Width." << endl;
			}

			cout << "\t\tGet Horizontal Pitch..." << endl; 
			oaLayerConstraint* pHGridPitchConstraint = oaLayerConstraint::find(pLefDefaultCG, layer, pHGridPitchDef);
			if(pHGridPitchConstraint)
			{
				oaIntValue* pHGridPitchVal = (oaIntValue*) pHGridPitchConstraint->getValue();
				if(pHGridPitchVal)
				{
					oaInt4 hGridPitch;
					hGridPitch = pHGridPitchVal->get();
					layer_info.h_pitch = hGridPitch;
					// cout << (oaFloat)hGridPitch/DBU << endl;
				}
			}
			else
			{
				cout << "\t\tUnable to Get the Horizontal Pitch." << endl;
			}

			cout << "\t\tGet Vetical Pitch..." << endl;
			oaLayerConstraint* pVGridPitchConstraint = oaLayerConstraint::find(pLefDefaultCG, layer, pVGridPitchDef);
			if(pVGridPitchConstraint)
			{
				oaIntValue* pVGridPitchVal = (oaIntValue*) pVGridPitchConstraint->getValue();
				if(pVGridPitchVal)
				{
					oaInt4 vGridPitch;
					vGridPitch = pVGridPitchVal->get();
					layer_info.v_pitch = vGridPitch;
					// cout << (oaFloat)vGridPitch/DBU << endl;
				}
			}
			else
			{
				cout << "\t\tUnable to Get the Vertical Pitch" << endl;
			}

			oaLayerConstraint *pPitchConstraint = NULL;
			const bool is_vertical = layer_info.is_vertical;
			if(is_vertical)
				pPitchConstraint = pHGridPitchConstraint;
			else
				pPitchConstraint = pVGridPitchConstraint;

			if(!pSpacingConstraint && pWidthConstraint && pPitchConstraint)
			{
				if(is_vertical)
					layer_info.spacing = layer_info.h_pitch - layer_info.width;
				else
					layer_info.spacing = layer_info.v_pitch - layer_info.width;
			}
			else if(pSpacingConstraint && !pWidthConstraint && pPitchConstraint)
			{
				if(is_vertical)
					layer_info.width = layer_info.h_pitch - layer_info.spacing;
				else
					layer_info.width = layer_info.v_pitch - layer_info.spacing;
			}
			else if(pSpacingConstraint && pWidthConstraint && !pPitchConstraint)
			{
				const oaUInt4 pitch = layer_info.spacing + layer_info.width;
				if(is_vertical)
					layer_info.h_pitch = pitch;
				else
					layer_info.v_pitch = pitch;
			}
			else
			{}

			cout << "\t\tGet Horizontal Offset..." << endl;
			oaLayerConstraint* pHGridOffsetConstraint = oaLayerConstraint::find(pLefDefaultCG, layer, pHGridOffsetDef);
			if(pHGridOffsetConstraint)
			{
				oaIntValue* pHGridOffsetVal = (oaIntValue*) pHGridOffsetConstraint->getValue();
				if(pHGridOffsetVal)
				{
					oaInt4 hGridOffset;
					hGridOffset = pHGridOffsetVal->get();
					layer_info.h_offset = hGridOffset;
					// cout << (oaFloat)hGridOffset/DBU << endl;
				}
			}
			else
			{
				cout << "\t\tUnable to Get the Horizontal Offset." << endl;
			}
			
			cout << "\t\tGet Vertical Offset..." << endl;
			oaLayerConstraint* pVGridOffsetConstraint = oaLayerConstraint::find(pLefDefaultCG, layer, pVGridOffsetDef);
			if(pVGridOffsetConstraint)
			{
				oaIntValue* pVGridOffsetVal = (oaIntValue*) pVGridOffsetConstraint->getValue();
				if(pVGridOffsetVal)
				{
					oaInt4 vGridOffset;
					vGridOffset = pVGridOffsetVal->get();
					layer_info.v_offset = vGridOffset;
					// cout << (oaFloat)vGridOffset/DBU << endl;
				}
			}
			else
			{
				cout << "\t\tUnable to Get the Vertical Offset." << endl;
			}
			
		}
		cout << "\tDone!!! Get the Layer Constraints." << endl << endl;
	}

	cout << "\tGet Track Pattern..." << endl;
	oaIter<oaTrackPattern> trackIter(top_block->getTrackPatterns());
	while(oaTrackPattern *p_track = trackIter.getNext())
	{
		const oaBoolean is_track_pitch_horizontal = p_track->isHorizontal();
		const oaInt4 track_start = p_track->getStartCoord();
		const oaUInt4 track_spacing = p_track->getTrackSpacing();
		const oaUInt4 track_num = p_track->getNumTracks();
		const oaLayerNum layer_num = p_track->getRoutingLayer();

		// Search the layer where the track-pattern is.
		LayerInfo *p_info = NULL;
		for(LayerInfo &info : routing_layer_infos)
		{
			if(info.layer == layer_num)
				p_info = &info;
		}

		if(p_info == NULL) continue;

		if(is_track_pitch_horizontal)
		{
			if(p_info->has_v_track)
			{
				p_info->mask_num += 1;
				if(p_info->v_track_start > track_start)
					p_info->v_track_start = track_start;
				p_info->v_track_num += track_num;
			}
			else
			{
				p_info->has_v_track = true;
				p_info->v_track_start = track_start;
				p_info->v_track_spacing = track_spacing;
				p_info->v_track_num = track_num;
			}
		}
		else
		{
			if(p_info->has_h_track)
			{
				p_info->mask_num += 1;
				if(p_info->h_track_start > track_start)
					p_info->h_track_start = track_start;
				p_info->h_track_num += track_num;
			}
			else
			{
				p_info->has_h_track = true;
				p_info->h_track_start = track_start;
				p_info->h_track_spacing = track_spacing;
				p_info->h_track_num = track_num;
			}
		}
	}
	// Precess the difference between Tech pitch and Track-Pattern spacing.
	for(LayerInfo &info : routing_layer_infos)
	{
		if(info.h_pitch != 0 && info.h_pitch < info.v_track_spacing)
			info.v_track_spacing = info.h_pitch;
		if(info.v_pitch != 0 && info.v_pitch < info.h_track_spacing)
			info.h_track_spacing = info.v_pitch;
	}
	cout << "\tDone!!! Get Track Pattern." << endl << endl;

	// debug
	// for(LayerInfo &info : this->routing_layer_infos)
	// {
		// cout << info << endl;
	// }

}
*/
/*
void RoutingLayerInfos::InitLayerInfos(const char *rule_file)
{
	ifstream input(rule_file);
	if(!input.is_open())
	{
		cout << "RoutingLayerInfos: Unable to open the file: " << rule_file << endl;
		input.close();
		exit(1);
	}

	cout << "========== Get Constraints from Rule File ==========" << endl;
	const unsigned int max_layer = 30;
	const int buffer_size = 128;
	char buffer[buffer_size];
	
	input.getline(buffer, buffer_size);
	unsigned int layer_num = 0;
	sscanf(buffer, "ROUTING LAYER NUM %d", &layer_num);

	if(layer_num <= 0 && layer_num > max_layer)
	{
		cout << "RoutingLayerInfos: Invalid layer number: " << layer_num << endl;
		input.close();
		exit(1);
	}

	// an empty line
	input.getline(buffer, buffer_size);

	for(unsigned int index = 0; index < layer_num; index++)
	{
		input.getline(buffer, buffer_size);
		stringstream iss(buffer);
		string key, layer_name;
		iss >> key >> layer_name;
		if(key != string("LAYER"))
		{
			cout << "RoutingLayerInfos: Wrong key word: " << key << endl;
			break;
		}

		cout << key << " " << layer_name << endl;
		LayerInfo *p_info = NULL;
		for(LayerInfo &layer_info : this->routing_layer_infos)
		{
			if(oaString(layer_name.c_str()) == layer_info.layer_name)
			{
				p_info = &layer_info;
				break;
			}
		}

		while(input.getline(buffer, buffer_size))
		{
			stringstream iss(buffer);
			string key, val1, val2;
			iss >> key >> val1 >> val2;
			if(key == string("END"))
			{
				// an empty line
				input.getline(buffer, buffer_size);
				cout << endl;
				break;
			}
			else if(key == string("DIRECTION"))
			{
				cout << key << " " << val1 << endl;
			}
			else if(key == string("SPACING"))
			{
				cout << key << " " << val1 << endl;
			}
			else if(key == string("WIDTH"))
			{
				cout << key << " " << val1 << endl;
			}
			else if(key == string("PITCH"))
			{
				cout << key << " " << val1 << " " << val2 << endl;
			}
			else if(key == string("OFFSET"))
			{
				cout << key << " " << val1 << " " << val2 << endl;
			}
			else if(key == string("ENDEND"))
			{
				cout << key << " " << val1 << endl;
				const oaFloat end_val = stof(val1, nullptr);
				const oaUInt4 e2e_spacing = DBU * end_val;
				p_info->spacing_e2e = e2e_spacing;
				p_info->has_line_end_rule = true;
			}
			else if(key == string("PARITYSPACING"))
			{
				cout << key << " " << val1 << endl;
				const oaFloat end_val = stof(val1, nullptr);
				const oaUInt4 parity_spacing_val = DBU * end_val;
				p_info->parity_track_spacing_e2e = parity_spacing_val;
				p_info->has_line_end_rule = true;
			}
			else
			{
				cout << "RoutingLayerInfos: Wrong key word: " << key << "in " << rule_file << endl;
				input.close();
				exit(1);
			}
		}
	}

	input.close();
	return;
}
*/

/*
void RoutingLayerInfos::GetLineEndRulesInTechDB(oaTech *tech)
{
	
	oaString group_name("EndEndSpacingRuleGroup");
	oaConstraintGroup *p_group = oaConstraintGroup::find(tech, group_name);
	if(p_group == NULL)
	{
		cout << "RoutingInfo: No end-end spacing rule in Tech DB" << endl;
		return;
	}

	// End-End Spacing
	{
		oaConstraintDef *p_def = oaConstraintDef::find(oaString("EndEndSpacing"));
		if(p_def == NULL)
		{
			cout << "RoutingInfo: No \"EndEndSpacing\" definition in EndEndSpacingRuleGroup." << endl;
		}
		else
		{
			for(LayerInfo &info : this->routing_layer_infos)
			{
				const oaLayerNum layer_num = info.layer;
				
				oaLayerConstraintDef *p_constraint_def = (oaLayerConstraintDef*)p_def;
				oaLayerConstraint *p_constraint = 
					oaLayerConstraint::find(p_group, layer_num, p_constraint_def);
				
				info.has_line_end_rule = false;
				if(p_constraint)
				{
					oaIntValue *p_val = (oaIntValue*)p_constraint->getValue();
					if(p_val)
					{
						oaUInt4 spacing_e2e = p_val->get();
						info.has_line_end_rule = true;
						info.spacing_e2e = spacing_e2e;
						cout << "Got the EndEndSpacing Rule in Layer " << info.layer_name << endl;
					}
				}
			}
		}
	}

	// Parity Track End-End Spacing
	{
		oaConstraintDef *p_def = oaConstraintDef::find(oaString("ParityEndEndSpacing"));
		if(p_def == NULL)
		{
			cout << "RoutingInfo: No \"ParityEndEndSpacing\" definition in EndEndSpacingRuleGroup." << endl;
		}
		else
		{
			for(LayerInfo &info : this->routing_layer_infos)
			{
				const oaLayerNum layer_num = info.layer;
				
				oaLayerConstraintDef *p_constraint_def = (oaLayerConstraintDef*)p_def;
				oaLayerConstraint *p_constraint = 
					oaLayerConstraint::find(p_group, layer_num, p_constraint_def);
				
				if(p_constraint)
				{
					oaIntValue *p_val = (oaIntValue*)p_constraint->getValue();
					if(p_val)
					{
						oaUInt4 parity_spacing = p_val->get();
						info.has_line_end_rule = true;
						info.parity_track_spacing_e2e = parity_spacing;
						cout << "Got the ParityEndEndSpacing Rule in Layer " << info.layer_name << endl;
					}
				}
			}
		}
	}

	return;
}
*/
/*
void RoutingLayerInfos::GetRoutingViaInfos(oaTech *tech)
{
	oaCollection< oaViaDef, oaTech > defs = tech->getViaDefs();
	if(defs.isEmpty())
	{
		cout << "RoutingLayerInfos: No via defs!" << endl;
		exit(1);
	}

	// 1. Collect the via defs having only one cut.
	vector<oaViaDef*> one_cut_defs;
	oaIter<oaViaDef> def_iter(defs);
	while(oaViaDef *p_def = def_iter.getNext())
	{
		oaString name;
		p_def->getName(name);
		oaLayerNum layer_num1 = p_def->getLayer1Num();
		oaLayerNum layer_num2 = p_def->getLayer2Num();
		oaLayerNum via_layer_num = 
			(layer_num1 + layer_num2) / 2;
		switch(p_def->getType())
		{
			case oacStdViaDefType:
			{
				// oaViaParam params;
				// ((oaStdViaDef*)p_def)->getParams(params);
				// const int cut_count = 
					// params.getCutRows() * params.getCutColumns();
				// if(cut_count == 1)
					// one_cut_defs.push_back(p_def);
			}
			break;
			case oacCustomViaDefType:
			{
				oaScalarName lib_name;
				oaScalarName cell_name;
				oaScalarName view_name;
				((oaCustomViaDef*)p_def)->getLibName(lib_name);
				((oaCustomViaDef*)p_def)->getCellName(cell_name);
				((oaCustomViaDef*)p_def)->getViewName(view_name);
				oaDesign *p_design = 
					oaDesign::find(lib_name, cell_name, view_name);
				if(!p_design)
					p_design = oaDesign::open(lib_name, cell_name, view_name, 'r');
				if(p_design)
				{
					oaBlock *p_via_block = p_design->getTopBlock();
					if(p_via_block)
					{
						oaCollection<oaShape, oaBlock> shapes = 
							p_via_block->getShapes();
						oaIter<oaShape> shape_iter(shapes);
						int cut_count = 0;
						while(oaShape* p_shape = shape_iter.getNext())
						{
							const oaLayerNum layer = p_shape->getLayerNum();
							if(layer == via_layer_num)
								cut_count++;
						}
						if(cut_count == 1)
							one_cut_defs.push_back(p_def);
					}
				}
			}
			break;
			default:
				;
		}
	}

	cout << "========== One Cut Via Def ==========" << endl;
	for(oaViaDef *p_def : one_cut_defs)
	{
		oaString name;
		p_def->getName(name);
		cout << "Via Def: " << name << endl;
	}
	
	// 2. Choose the smallest via of each layer.
	vector<oaViaDef*> small_defs;
	small_defs.resize(routing_layer_infos.size() - 1);
	for(oaViaDef *p_def : small_defs)
		p_def = NULL;
	for(oaViaDef *p_def : one_cut_defs)
	{
		switch(p_def->getType())
		{
			case oacStdViaDefType:
			case oacCustomViaDefType:
			{
				const oaUInt4 index = p_def->getLayer1Num()/2 - 1;
				if(small_defs[index] == NULL)
				{
					small_defs[index] = p_def;
				}
				else
				{
					// Via
					oaBox box1, box2;
					GetViaBox(box1, small_defs[index]);
					GetViaBox(box2, p_def);
					oaUInt4 area1 = box1.getWidth() * box1.getHeight();
					oaUInt4 area2 = box2.getWidth() * box2.getHeight();
					
					// Enclosure1
					oaBox enc1_box1, enc1_box2;
					GetViaEnc1Box(enc1_box1, small_defs[index]);
					GetViaEnc1Box(enc1_box2, p_def);
					LayerInfo &layer1_info = 
						routing_layer_infos[(p_def->getLayer1Num()/2-1)];
					oaUInt4 layer1_width = layer1_info.width; 
					oaUInt4 enc1_width1 = (layer1_info.is_vertical) ?
						(enc1_box1.getWidth()) : (enc1_box1.getHeight());
					oaUInt4 enc1_width2 = (layer1_info.is_vertical) ?
						(enc1_box2.getWidth()) : (enc1_box2.getHeight());
					
					// Enclosure2
					oaBox enc2_box1, enc2_box2;
					GetViaEnc2Box(enc2_box1, small_defs[index]);
					GetViaEnc2Box(enc2_box2, p_def);
					LayerInfo &layer2_info = 
						routing_layer_infos[(p_def->getLayer2Num()/2-1)];
					oaUInt4 layer2_width = layer2_info.width; 
					oaUInt4 enc2_width1 = (layer2_info.is_vertical) ?
						(enc2_box1.getWidth()) : (enc2_box1.getHeight());
					oaUInt4 enc2_width2 = (layer2_info.is_vertical) ?
						(enc2_box2.getWidth()) : (enc2_box2.getHeight());
					if((area1 >= area2) && 
						(enc1_width2 == layer1_width || enc1_width2 < enc1_width1) &&
						(enc2_width2 == layer2_width || enc2_width2 < enc2_width1))
					{
						small_defs[index] = p_def;
					}
				}
			}
			break;
			default:
				;
		}
	}
	
	cout << "========== Routing Via Def ==========" << endl;
	int index = 0;
	for(oaViaDef *p_def : small_defs)
	{
		cout << "Layer: " << (index * 2 + 3) << endl;
		if(p_def != NULL)
		{
			oaString name;
			p_def->getName(name);
			cout << "Via Def: " << name << endl;
		}
		else
		{
			cout << "No Def! " << endl;
			exit(1);
		}
		index++;
	}

	for(oaViaDef *p_def : small_defs)
	{
		const oaLayerNum layer1_num = p_def->getLayer1Num();
		const oaLayerNum layer2_num = p_def->getLayer2Num();
		if(layer2_num > layer1_num)
		{
			routing_layer_infos[layer2_num/2-1].lower_via_def = p_def;
			routing_layer_infos[layer1_num/2-1].upper_via_def = p_def;
		}
		else
		{
			routing_layer_infos[layer2_num/2-1].upper_via_def = p_def;
			routing_layer_infos[layer1_num/2-1].lower_via_def = p_def;
		}
	}
	return;
}
*/
/*
void RoutingLayerInfos::SetViaEncInfos()
{
	oaBox enc1_box, enc2_box;
	for(LayerInfo &info : routing_layer_infos)
	{
		if(info.upper_via_def != NULL)
		{
			GetViaEnc1Box(enc1_box, info.upper_via_def);
			info.upper_enc1_width = enc1_box.getWidth();
			info.upper_enc1_height = enc1_box.getHeight();
			GetViaEnc2Box(enc2_box, info.upper_via_def);
			info.upper_enc2_width = enc2_box.getWidth();
			info.upper_enc2_height = enc2_box.getHeight();
		}

		if(info.lower_via_def != NULL)
		{
			GetViaEnc1Box(enc1_box, info.lower_via_def);
			info.lower_enc1_width = enc1_box.getWidth();
			info.lower_enc1_height = enc1_box.getHeight();
			GetViaEnc2Box(enc2_box, info.lower_via_def);
			info.lower_enc2_width = enc2_box.getWidth();
			info.lower_enc2_height = enc2_box.getHeight();
		}
	}
	return;
}
*/
/*
void RoutingLayerInfos::GetViaBox(oaBox &box, oaViaDef *p_def)
{
	switch(p_def->getType())
	{
		case oacStdViaDefType:
		{
			oaViaParam params;
			((oaStdViaDef*)p_def)->getParams(params);
			oaInt4 half_width = (oaInt4)params.getCutWidth() / 2;
			oaInt4 half_height = (oaInt4)params.getCutHeight() / 2;
			box.set(-half_width, -half_height, half_width, half_height);
		}
		break;
		case oacCustomViaDefType:
		{
			oaScalarName lib_name;
			oaScalarName cell_name;
			oaScalarName view_name;
			((oaCustomViaDef*)p_def)->getLibName(lib_name);
			((oaCustomViaDef*)p_def)->getCellName(cell_name);
			((oaCustomViaDef*)p_def)->getViewName(view_name);
			oaDesign *p_design = 
				oaDesign::find(lib_name, cell_name, view_name);
			if(!p_design)
				p_design = oaDesign::open(lib_name, cell_name, view_name, 'r');
			if(p_design)
			{
				oaBlock *p_via_block = p_design->getTopBlock();
				if(p_via_block)
				{
					oaCollection<oaShape, oaBlock> shapes = 
						p_via_block->getShapes();
					oaIter<oaShape> shape_iter(shapes);
					while(oaShape* p_shape = shape_iter.getNext())
					{
						const oaLayerNum layer = p_shape->getLayerNum();
						const oaLayerNum via_layer = 
							(p_def->getLayer1Num()+p_def->getLayer2Num()) / 2;
						if(layer == via_layer)
							p_shape->getBBox(box);
					}
				}
			}
		}
		break;
		default:
			;
	}
}
*/
/*
void RoutingLayerInfos::GetViaEnc1Box(oaBox &box, oaViaDef *p_def)
{
	switch(p_def->getType())
	{
		case oacStdViaDefType:
		{
			oaViaParam params;
			((oaStdViaDef*)p_def)->getParams(params);
			oaInt4 half_cut_width = (oaInt4)params.getCutWidth() / 2;
			oaInt4 half_cut_height = (oaInt4)params.getCutHeight() / 2;
			oaInt4 half_width = params.getLayer1Enc().x() + half_cut_width;
			oaInt4 half_height = params.getLayer1Enc().y() + half_cut_height;
			box.set(-half_width, -half_height, half_width, half_height);
		}
		break;
		case oacCustomViaDefType:
		{
			oaScalarName lib_name;
			oaScalarName cell_name;
			oaScalarName view_name;
			((oaCustomViaDef*)p_def)->getLibName(lib_name);
			((oaCustomViaDef*)p_def)->getCellName(cell_name);
			((oaCustomViaDef*)p_def)->getViewName(view_name);
			oaDesign *p_design = 
				oaDesign::find(lib_name, cell_name, view_name);
			if(!p_design)
				p_design = oaDesign::open(lib_name, cell_name, view_name, 'r');
			if(p_design)
			{
				oaBlock *p_via_block = p_design->getTopBlock();
				if(p_via_block)
				{
					oaCollection<oaShape, oaBlock> shapes = 
						p_via_block->getShapes();
					oaIter<oaShape> shape_iter(shapes);
					while(oaShape* p_shape = shape_iter.getNext())
					{
						const oaLayerNum layer = p_shape->getLayerNum();
						const oaLayerNum layer1_num = p_def->getLayer1Num(); 
						if(layer == layer1_num)
							p_shape->getBBox(box);
					}
				}
			}
		}
		break;
		default:
			;
	}

}

void RoutingLayerInfos::GetViaEnc2Box(oaBox &box, oaViaDef *p_def)
{
	switch(p_def->getType())
	{
		case oacStdViaDefType:
		{
			oaViaParam params;
			((oaStdViaDef*)p_def)->getParams(params);
			oaInt4 half_cut_width = (oaInt4)params.getCutWidth() / 2;
			oaInt4 half_cut_height = (oaInt4)params.getCutHeight() / 2;
			oaInt4 half_width = params.getLayer2Enc().x() + half_cut_width;
			oaInt4 half_height = params.getLayer2Enc().y() + half_cut_height;
			box.set(-half_width, -half_height, half_width, half_height);
		}
		break;
		case oacCustomViaDefType:
		{
			oaScalarName lib_name;
			oaScalarName cell_name;
			oaScalarName view_name;
			((oaCustomViaDef*)p_def)->getLibName(lib_name);
			((oaCustomViaDef*)p_def)->getCellName(cell_name);
			((oaCustomViaDef*)p_def)->getViewName(view_name);
			oaDesign *p_design = 
				oaDesign::find(lib_name, cell_name, view_name);
			if(!p_design)
				p_design = oaDesign::open(lib_name, cell_name, view_name, 'r');
			if(p_design)
			{
				oaBlock *p_via_block = p_design->getTopBlock();
				if(p_via_block)
				{
					oaCollection<oaShape, oaBlock> shapes = 
						p_via_block->getShapes();
					oaIter<oaShape> shape_iter(shapes);
					while(oaShape* p_shape = shape_iter.getNext())
					{
						const oaLayerNum layer = p_shape->getLayerNum();
						const oaLayerNum layer2_num = p_def->getLayer2Num(); 
						if(layer == layer2_num)
							p_shape->getBBox(box);
					}
				}
			}
		}
		break;
		default:
			;
	}
}
*/

