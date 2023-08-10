#ifndef XML_PROCESSING_RESULT_THREADING_H
#define XML_PROCESSING_RESULT_THREADING_H

#include "../pose_structs.hpp"
#include "float_standard.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <stdio.h>
#include <iostream>

#include <thread>
#include <mutex>
#include <map>
#include <vector>

/**
 * @author Natalie Chmura
 * 
 * @brief This file uses a multi-threaded approach to sort through the data structs 
 * generated by xml_parsing and constructs a array containing the reesult data from 
 * those scores, in sorted order depending on the points requested by the client node!
 */

/**
 * @namespace Scorter
 * 
 * @brief Score-Sorter [descended into lame pun namespaces]
*/
namespace Scorter {

/**
 * @class Retriever
 * 
 * @brief This class contains functions that aid in the sorting and populating the
 * results of the reach_study array!
 */
class Retriever {

    public:

    /**
     * @brief Runs the hardware allowing number of threads through the returned map of score data,
     * who each assign their poses score result to its associated index in the result array!
     * 
     * @param poseKeys: A poseArray that will act as the keys for the data in the reachStudyMap
     * 
     * @param reachStudyMap: A map that contains all of the data pulled from the reach_study in xml_parsing
     * with the keys being of type pose and the results containing the return sucess and score!
     * 
     * @returns The vector containing all of the score data!
     */
    static std::vector<double> getScoreData(geometry_msgs::msg::PoseArray poseKeys, 
        std::map<XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData, XML_PROCESSING_POSTRUCTS_H::Postructs::ResultData> reachStudyMap,
        int size);

    private:

    /**
     * @brief Runs through the beginning and end indicies of a list adding the associated key values 
     * to a new "results" list! 
     * 
     * @param poseKeys: A poseArray that will act as the keys for the data in the reachStudyMap
     * 
     * @param reachStudyMap: A map that contains all of the data pulled from the reach_study in xml_parsing
     * with the keys being of type pose and the results containing the return sucess and score!
     * 
     * @param results: An vector that contains all of the return scores indexed in the same manner
     * as the poseKey results! (Non returnable since the same map needs to be populated without
     * being overwritten! ^^)
     * 
     * @returns A vector that contains all of the return scores indexed in the same manner as the poseKey
     * results!
     */
    static void populateResults(int start, int end, int max, geometry_msgs::msg::PoseArray poseKeys,
        std::map<XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData, XML_PROCESSING_POSTRUCTS_H::Postructs::ResultData> reachStudyMap,
        double * results);

    /**
     * @brief Puts the pose data into the poseData (struct) format to be used as a key! For the map!
     * 
     * @param pose: A pose that will act as the key in a data set in reachStudyMap once converted into 
     * a poseData struct!
     * 
     * @returns: The pose data struct that contains the pose data and is able to be used as a key in the map!
     */ 
    static XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData getKey(geometry_msgs::msg::Pose * pose);

};

} // namespace Scorter


#endif // XML_PROCESSING_RESULT_THREADING_HPS_RESULT_THREADING_H