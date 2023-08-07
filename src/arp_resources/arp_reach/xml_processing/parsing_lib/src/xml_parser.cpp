#include "../include/interfaces/xml_parser.hpp"

/**
 * @author Natalie Chmura 
 * 
 * @brief This contains a class that reads information from the XML file genrated by the reach study
 * and parses it into an array of data nodes! (^u^)
 */


namespace ReachXML {

// PUBLIC:

std::vector<XML_PROCESSING_POSTRUCTS_H::Postructs::ReachData> XMLParser::parseXML(std::string fname) {

    rapidxml::xml_document<> doc;
    rapidxml::xml_node<> * root_node;
    //Read the file into a vector
    std::ifstream theFile (fname);
    std::vector<char> buffer((std::istreambuf_iterator<char>(theFile)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	// Parse the buffer using the rapid-xml file parsing library into doc 
	doc.parse<0>(&buffer[0]);
    //Find the root of the data, in our case boost_serialization
    root_node = doc.first_node(0);

    int count = 0;

    try {
        int count = XMLParser::getItemCount(root_node);
    } catch (const std::exception &ex) {
        std::cerr << "File must contain reach points! \n";
    }

    rapidxml::xml_node<> * item_node = XMLParser::descendToItem(root_node);
    std::vector<XML_PROCESSING_POSTRUCTS_H::Postructs::ReachData> poses = XMLParser::populatePoses(item_node, count);

    return poses;
}

std::map<XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData, XML_PROCESSING_POSTRUCTS_H::Postructs::ResultData> XMLParser::parseMap(std::string fname) {

    rapidxml::xml_document<> doc;
    rapidxml::xml_node<> * root_node;
    //Read the file into a vector
    std::ifstream theFile (fname);
    std::vector<char> buffer((std::istreambuf_iterator<char>(theFile)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	// Parse the buffer using the rapid-xml file parsing library into doc 
	doc.parse<0>(&buffer[0]);
    //Find the root of the data, in our case boost_serialization
    root_node = doc.first_node(0);

    int count = 0;

    try {
        count = XMLParser::getItemCount(root_node);
    } catch (const std::exception &ex) {
        std::cerr << "File must contain reach points! \n";
    }

    rapidxml::xml_node<> * item_node = XMLParser::descendToItem(root_node);
    std::map<XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData, XML_PROCESSING_POSTRUCTS_H::Postructs::ResultData> poses = XMLParser::populatePoseMap(item_node, count);

    return poses;
}

// PRIVATE:

int XMLParser::getItemCount(rapidxml::xml_node<> * root_node) {

    try {
        // return (int)(db->results->count->item_version->item->count.value())
        return std::stoi(root_node->first_node()->first_node()->first_node()
        ->next_sibling()->next_sibling()->first_node()->value());
    } catch (const std::exception &ex) {
        std::cerr << "File not formatted correctly - no nodes found\n";
        return 0;
    }
}

rapidxml::xml_node<> * XMLParser::descendToItem(rapidxml::xml_node<> * root_node){

    try {
        // return (db->results->count->item_version->item->count->item_version->item)
        return root_node->first_node()->first_node()->first_node()->next_sibling()
            ->next_sibling()->first_node()->next_sibling()->next_sibling();
    } catch (const std::exception &ex) {
        std::cerr << "File not formatted correctly, default configurations not used\n";
        return root_node;
    }
}

std::vector<XML_PROCESSING_POSTRUCTS_H::Postructs::ReachData> XMLParser::populatePoses(rapidxml::xml_node<> * item_node, int count) {
    std::vector<XML_PROCESSING_POSTRUCTS_H::Postructs::ReachData> poseVector;
    poseVector.reserve(count);

    for (int i = 0; i < count && item_node; i++) {
        XMLParser::populateStruct(item_node, &poseVector[i]);
        item_node = item_node->next_sibling();
    }

    return poseVector;
}

std::map<XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData, XML_PROCESSING_POSTRUCTS_H::Postructs::ResultData> XMLParser::populatePoseMap(rapidxml::xml_node<> * item_node, int count) {
    std::map<XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData, XML_PROCESSING_POSTRUCTS_H::Postructs::ResultData> poseMap;

    for (int i = 0; i < count && item_node; i++) {
        XML_PROCESSING_POSTRUCTS_H::Postructs::ReachData pose;
        XMLParser::populateStruct(item_node, &pose);
        poseMap.emplace(pose.pose, pose.result);
        item_node = item_node->next_sibling();
    }

    return poseMap;
}

void XMLParser::populateStruct(rapidxml::xml_node<> * item_node, struct XML_PROCESSING_POSTRUCTS_H::Postructs::ReachData *data) {

    try
    {
        data->pose.quater = XML_PROCESSING_ARRAY_TRANSFORM_H::ReachArray::ArrayTF::getQuaternion(XMLParser::getPoseMatrix(item_node));
        data->pose.translation = XML_PROCESSING_ARRAY_TRANSFORM_H::ReachArray::ArrayTF::getTranslation(XMLParser::getPoseMatrix(item_node));
        // reachable = item->reached
        data->result.reachable = std::stoi(item_node->first_node()->value());
        // score = item->:reached->goal->seed_state->goal_state->score
        data->result.score = std::atof(item_node->first_node()->next_sibling()->next_sibling()->next_sibling()->next_sibling()->value());
    } catch(const std::exception &ex) {
        std::cerr << "Reach score data not in original format\n";
    }
}

double * XMLParser::getPoseMatrix(rapidxml::xml_node<> * item_node) {
    
    static double numMatrix[XMLParser::MATRIX_SIZE];

    try {
        // item_node = item->reached->goal->matrix->count->item_version->[first]item 
        item_node = item_node->first_node()->next_sibling()->first_node()
            ->first_node()->next_sibling()->next_sibling();
    } catch (const std::exception &ex) {
        std::cerr << "File not formatted correctly, default configurations not used\n";
        return numMatrix;
    }

    try {
        for (int i = 0; i < XMLParser::MATRIX_SIZE && item_node; i++) {
            numMatrix[i] = atof(item_node->value());
            item_node = item_node->next_sibling();
        }
    } catch (const std::exception &ex) {
        std::cerr << "Size of matrix inconsistent with Isometry3D - should have 16 entries\n";
        return numMatrix;
    }
    
    return numMatrix;  
}

} //namespace ReachXML