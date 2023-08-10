#include "../include/interfaces/YAMLFileWriter.hpp"

/**
 * @author Natalie Chmura
 * 
 * @brief A C++ file that writes a YAML file with a designated input path for the 
 *  generated pcd file also made in the pcd_processing package!
*/

namespace ProcessYAML {

void YAMLFileWriter::writeYAML(std::string pathname, std::string fname) {

    YAML::Emitter out;
    out << YAML::BeginMap;

    out << YAML::Key << "optimization";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "radius";
    out << YAML::Value << "0.2";
    out << YAML::Key << "max_steps";
    out << YAML::Value << "10";
    out << YAML::Key << "step_improvment_threshold";
    out << YAML::Value << "0.01";
    out << YAML::EndMap;

    out << YAML::Key << "ik_solver";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Value << "MoveItIKSolver";
    out << YAML::Key << "distance_threshold";
    out << YAML::Value << "0.0";
    out << YAML::Key << "planning_group";
    out << YAML::Value << "manipulator";
    out << YAML::Key << "touch_links";
    out << YAML::Value << YAML::BeginSeq << YAML::EndSeq;
    out << YAML::EndMap;

    out << YAML::Key << "evaluator";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Key << "MultiplicativeEvaluator";
    out << YAML::Key << "plugins";
    out << YAML::Value << YAML::BeginSeq;
    out << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Value << "ManipulabilityMoveIt";
    out << YAML::Key << "planning_group";
    out << YAML::Value << "manipulator";
    out << YAML::EndMap;
    out << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Value << "DistancePenaltyMoveIt";
    out << YAML::Key << "planning_group";
    out << YAML::Value << "manipulator";
    out << YAML::Key << "distance_threshold";
    out << YAML::Value << "0.025";
    out << YAML::Key << "exponent";
    out << YAML::Value << "2";
    out << YAML::Key << "touch_links";
    out << YAML::Value << YAML::BeginSeq << YAML::EndSeq;
    out << YAML::EndMap;
    out << YAML::EndSeq;
    out << YAML::EndMap;

    out << YAML::Key << "display";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Value << "ROSDisplay";
    out << YAML::Key << "kinematic_base_frame";
    out << YAML::Value << "base_link";
    out << YAML::Key << "marker_scale";
    out << YAML::Value << "0.05";
    out << YAML::Key << "use_full_color_range";
    out << YAML::Value << "True";
    out << YAML::EndMap;

    out << YAML::Key << "target_pose_generator";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Value << "PointCloudTargetPoseGenerator";
    out << YAML::Key << "pcd_file";
    out << YAML::Value << pathname;
    out << YAML::EndMap;

    out << YAML::Key << "logger";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Value << "BoostProgressConsoleLogger";
    out << YAML::EndMap;

    out << YAML::EndMap;

    std::ofstream fout(fname);
    fout << out.c_str();
    fout.close();
}

} // namepsace ProcessYAML