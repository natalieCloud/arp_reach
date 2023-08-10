#include "../include/interfaces/PoseArrayToPCD.hpp"

/**
 * @author Natalie Chmura
 * 
 * @brief A C++ file that processes an input of type poseArray and turns it into a .pcd 
 * file that can be used in the reach study
*/
namespace ProcessPCD {

int PCDFileProcessor::writeFile(geometry_msgs::msg::PoseArray waypoints, std::string fname) {

    try {
        
        std::ofstream pcdFile(fname);

        pcdFile << "# .PCD v0.7 - Point Cloud Data file format\n";
        pcdFile << "VERSION 0.7\nFIELDS x y z normal_x normal_y normal_z curvature\n";
        pcdFile << "SIZE 4 4 4 4 4 4 4\nTYPE F F F F F F F\nCOUNT 1 1 1 1 1 1 1\n";
        pcdFile << "WIDTH " << waypoints.poses.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n";
        pcdFile << "POINTS " << waypoints.poses.size() << "\nDATA ascii\n";

        for(geometry_msgs::msg::Pose pose : waypoints.poses) {
            pcdFile << pose.position.x << " " << pose.position.y << " " << pose.position.z;
            pcdFile << " " << pose.orientation.x << " " << pose.orientation.y << " ";
            pcdFile  << pose.orientation.z << " " << pose.orientation.w << "\n";
        }

        pcdFile.close();

    } catch (const std::exception &ex) {
        std::cerr << ex.what() << std::endl;
        return 0;
    }

    return 1;
}

geometry_msgs::msg::PoseArray PCDFileProcessor::readPoses(std::string fname) {
    
    try {

        std::ifstream pcdFile(fname);

        std::string line;

        getline(pcdFile, line); // PCD v0.7 etc
        getline(pcdFile, line); // Version 0.7
        getline(pcdFile, line); // Fields
        getline(pcdFile, line); // Size
        getline(pcdFile, line); // Type
        getline(pcdFile, line); // Count
        getline(pcdFile, line); // Width
        getline(pcdFile, line); // Height
        getline(pcdFile, line); // Viewpoint
        getline(pcdFile, line); // Points
        getline(pcdFile, line); // Data

        auto poseArr = geometry_msgs::msg::PoseArray();
        poseArr.header.frame_id = "world";

        while (getline(pcdFile, line)) {
            auto pose = geometry_msgs::msg::Pose();

            std::stringstream str(line);
            str >> pose.position.x;
            str >> pose.position.y;
            str >> pose.position.z;
            str >> pose.orientation.x;
            str >> pose.orientation.y;
            str >> pose.orientation.z;
            str >> pose.orientation.w;

            poseArr.poses.push_back(pose);
        }

        pcdFile.close();

        return poseArr;

    } catch (const std::exception &ex) {
        std::cerr << ex.what() << std::endl;
        return geometry_msgs::msg::PoseArray();
    }
}

} // namespace ProcessPCD