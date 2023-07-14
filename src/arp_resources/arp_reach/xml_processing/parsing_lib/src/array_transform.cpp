#include "../include/interfaces/array_transform.hpp"

/**
 * @author Natalie Chmura 
 * 
 * @brief A class that transforms a 16-point pose array into its corresponding quaternion pose 
 * and transformation matrix!
 */

namespace ReachArray {

    Eigen::Quaternion<_Float64> ArrayTF::getQuaternion(_Float64 * poseArray) {
        return Eigen::Quaternion<_Float64>(setIsometry(poseArray).rotation());
    }

    Eigen::Vector3d ArrayTF::getTranslation(_Float64 * poseArray) {
        return Eigen::Matrix4d(poseArray).block<3,1>(0, 3);
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