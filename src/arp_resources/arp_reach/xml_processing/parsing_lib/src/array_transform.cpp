#include "../include/interfaces/array_transform.hpp"

/**
 * @author Natalie Chmura 
 * 
 * @brief A class that transforms a 16-point pose array into its corresponding quaternion pose 
 * and transformation matrix!
 */

namespace ReachArray {

    Eigen::Quaternion<_Float64> ArrayTF::getQuaternion(_Float64 * poseArray) {
        Eigen::Quaternion<_Float64> quat = Eigen::Quaternion<_Float64>(setIsometry(poseArray).rotation());
        quat.x() = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(quat.x());
        quat.y() = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(quat.y());
        quat.z() = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(quat.z());
        quat.w() = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(quat.w());
        return quat;
    }

    Eigen::Vector3d ArrayTF::getTranslation(_Float64 * poseArray) {
        auto loc = Eigen::Matrix4d(poseArray).block<3,1>(0, 3);
        loc(0, 0) = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(loc(0, 0)); //x
        loc(1, 0) = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(loc(1, 0)); //y
        loc(2, 0) = XML_PARSING_FLOAT_STANDARD_H::FloatSt::RoundSt::roundNano(loc(2, 0)); //z
        return loc;
    }

    Eigen::Isometry3d ArrayTF::setIsometry(_Float64 * poseArray) {
        Eigen::Matrix4d matrix(poseArray);
        Eigen::Isometry3d poseMatrix = Eigen::Isometry3d::Identity();

        //Applying the rotation before the transformation altered the transformation
        //which is why the transform is applied first! (^-^)
        poseMatrix.translate(matrix.block<3, 1>(0, 3));
        poseMatrix.rotate(matrix.block<3, 3>(0, 0));

        return poseMatrix;
    }

} //namespace ReachArray