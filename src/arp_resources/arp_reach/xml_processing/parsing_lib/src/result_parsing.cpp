#include "../include/interfaces/result_parsing.hpp"

/**
 * @author Natalie Chmura
 * 
 * @brief This contains a function that will use threading to parse the
 * information from the xml_parser file and returns an array of scores per
 * pose array!
 */

namespace Scorter {

std::mutex sharedMutex;

std::vector<double> Retriever::getScoreData(geometry_msgs::msg::PoseArray poseKeys,
        std::map<XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData, XML_PROCESSING_POSTRUCTS_H::Postructs::ResultData> reachStudyMap,
        int size) {
    
    std::vector<double> results;
    double resultArr[size];
    double * arrPtr = resultArr;

    int num_threads = std::thread::hardware_concurrency();
    if(!num_threads) {num_threads++;}

    int inc_size = size/num_threads + 1;

    std::vector<std::thread> threads;
    threads.reserve(num_threads);

    for (int i = 0; i < num_threads; i++) {
        threads.emplace_back(
                std::thread {Retriever::populateResults,
                (i * inc_size), ((i + 1) * inc_size), size, poseKeys, reachStudyMap, arrPtr}
        );
    }
    for (auto& t : threads) {
        t.join();
    }

    results.insert(results.end(), &resultArr[0], &resultArr[size]);
    return results;
}

void Retriever::populateResults(int start, int end, int max, geometry_msgs::msg::PoseArray poseKeys,
        std::map<XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData, XML_PROCESSING_POSTRUCTS_H::Postructs::ResultData> reachStudyMap,
        double * results) {

    std::lock_guard<std::mutex> lock(sharedMutex);
    for (int i = start; i < end && i < max; i++) {
        geometry_msgs::msg::Pose * temp = &poseKeys.poses[i];
        results[i] = reachStudyMap[Retriever::getKey(temp)].score;
    }

}

XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData Retriever::getKey(geometry_msgs::msg::Pose * pose) {
    XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData poseD;
    poseD.translation.x() = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(pose->position.x);
    poseD.translation.y() = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(pose->position.y);
    poseD.translation.z() = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(pose->position.z);
    poseD.quater.x() = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(pose->orientation.x);
    poseD.quater.y() = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(pose->orientation.y);
    poseD.quater.z() = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(pose->orientation.z);
    poseD.quater.w() = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(pose->orientation.w);
    return poseD;
}

} //namespace Scorter