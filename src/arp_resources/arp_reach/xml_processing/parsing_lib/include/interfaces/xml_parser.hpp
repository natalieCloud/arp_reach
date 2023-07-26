#ifndef XML_PARSING_XML_PARSER_H
#define XML_PARSING_XML_PARSER_H

#include "../plugins/rapidxml.hpp"
#include "array_transform.hpp"
#include "../pose_structs.hpp"

#include <stdio.h>
#include <stdlib.h>

#include <fstream>

#include <vector>
#include <map>
#include <string>

/**
 * @author Natalie Chmura 
 * 
 * @brief This contains a class that reads information from the XML file genrated by the reach study
 * and parses it into an array of data nodes! (^u^)
 */

/**
 * @namespace ReachXML
*/
namespace ReachXML {

/**
 * @class XMLParser
 * 
 * @brief This class takes the results generated by the reach study done by the ros-industrial reach
 * (Generated into an xml format) and parses the information into a vector of "ReachData" nodes. 
 * This parsing is done by leveraging the use of the rapidxml package!
*/
class XMLParser {

    public:

        /**
         * @brief This function takes the name of a file and parses it into a vector of data structs that
         * contian a pose in space, and an associated reachability score
         * 
         * @param fname: The name of the file that the user pases in for the xml to read from.
         */
        static std::vector<XML_PROCESSING_POSTRUCTS_H::Postructs::ReachData> parseXML(const std::string fname);

        /**
         * @brief This function takes the name of a file and parses it into a vector of data structs that
         * contian a pose in space, and an associated reachability score
         * 
         * @param fname: The name of the file that the user pases in for the xml to read from.
         */
        static std::map<XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData, XML_PROCESSING_POSTRUCTS_H::Postructs::ResultData> parseMap(const std::string fname);

        /**
         * @brief The size of the pose matrix (Will be uneeded? if we change to a quaternion!)
         */
        static const int MATRIX_SIZE = 16; 

    private:

        /**
         * @brief This function takes the root of a tree and traces through its
         *  children to obtain the number of child "item" nodes.
         * 
         * @param root_node: The node at the root of the file
         * 
         * @returns The count of all of the item nodes
         */
        static int getItemCount(rapidxml::xml_node<> * root_node);

        /**
         * @brief This function takes the root node of an xml tree and traces through its
         * children to obtain the first instance of the child "item" node that contains the
         * pose matrix and reachability score for that pose. 
         * 
         * @param root_node: The node at the root of the file
         * 
         * @returns A pointer to the the first item child node
         */
        static rapidxml::xml_node<> * descendToItem(rapidxml::xml_node<> * root_node);

        /**
         * @brief This function creates a vector with populated ReachData structs
         * 
         * @param item_node: The first child node in the xml tree
         * 
         * @param count: The number of item child nodes in the xml tree
         * 
         * @returns A vector populated with filled ReachData nodes 
         */
        static std::vector<XML_PROCESSING_POSTRUCTS_H::Postructs::ReachData> populatePoses(rapidxml::xml_node<> * item_node, int count);

        /**
         * @brief This function creates a map with populated ReachData structs
         * 
         * @param item_node: The first child node in the xml tree
         * 
         * @param count: The number of item child nodes in the xml tree
         * 
         * @returns A map populated with filled ReachData key-value pairs 
         */
        static std::map<XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData, XML_PROCESSING_POSTRUCTS_H::Postructs::ResultData> populatePoseMap(rapidxml::xml_node<> * item_node, int count); 

        /**
         * @brief This function populates a struct with all the pose and reach data
         * 
         * @param item_node: The first child node in the xml tree
         * 
         * @param data: A pointer to the struct that will contain the data
         */
        static void populateStruct(rapidxml::xml_node<> * item_node, struct XML_PROCESSING_POSTRUCTS_H::Postructs::ReachData *data);

        /**
         * @brief Obtains the data from the xml tree ( represented by an Isometry#D pose matrix)
         * and represents the matrix in array format.
         * 
         * @param item_node: The first child node in the xml tree
         * 
         * @returns An array of Isometery3D data
         */
        static double * getPoseMatrix(rapidxml::xml_node<> * item_node); 
};

} //namespace ReachXML 

#endif // XML_PARSING_XML_PARSER_H