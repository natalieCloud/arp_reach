#ifndef XML_PROCESSING_POSTRUCTS_H
#define XML_PROCESSING_POSTRUCTS_H

#include <eigen3/Eigen/Geometry>

#include <stdlib.h>

/**
 * @namespace Postructs
 * 
 * @brief Defines our *Pose Structs* (ba-dum-tssh) 
 */
namespace Postructs {

/**
 * @struct PoseData
 * @brief Struct that contains the pose information
 * 
 * @param translation: A 3x1 vector that represents the pose's xyz coordinates
 * @param quater: A quaternion that represents the pose's rotation
 */
struct PoseData {
    Eigen::Vector3d translation;
    Eigen::Quaternion<_Float64> quater;

    //Operator overloads for use as a key
    //Sorting is more or less arbitrary however for time complexity sake I decided to go with 
    //the ordered map- Ordered: W(n) ∈ O(log(n)) vs.  Undordered: W(n) ∈ O(n)
    bool operator==(const PoseData &p) const {
        return translation == p.translation && quater == p.quater;
    }
    bool operator<(const PoseData &p) const {
        return translation.x() < p.translation.x() || (translation.x() == p.translation.x() && quater.w() == p.quater.w());
    }
};

/**
 * @struct ScoreData
 * @brief Struct that contains the score information for a pose
 * 
 * @param result: A boolean that signals if the pose is reachable (true) or not (false).
 * @param score: The reach score calculated by the reach study
 */
struct ResultData {
    bool reachable;
    _Float64 score;
};
    
/**
 * @struct ReachData
 * @brief Struct that contains the pose and score data.
 * 
 * @param translation: A 3x1 vector that represents the pose's xyz coordinates
 * @param quater: A quaternion that represents the pose's rotation
 * @param reachResult: Is the pose considered "reachable", 0 no 1 yes
 * @param reachScore: Score for the reachability of the pose
 */ 
struct ReachData {
    struct PoseData pose;
    struct ResultData result; 
};

} // namespace Restructs



#endif // XML_PROCESSING_POSTRUCTS_H