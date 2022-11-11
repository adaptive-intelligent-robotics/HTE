#ifndef ROBOT_DART_DESCRIPTOR_HEXA_LEG_HPP
#define ROBOT_DART_DESCRIPTOR_HEXA_LEG_HPP

// for size_t
#include <cstddef>

namespace robot_dart {
    class RobotDARTSimu;
    class Robot;

    namespace descriptor {

      struct HexaLegDescriptor:public BaseDescriptor{
        public:
	HexaLegDescriptor(RobotDARTSimu* simu, size_t desc_dump = 1):BaseDescriptor(simu,desc_dump)
	{}
	HexaLegDescriptor(RobotDARTSimu* simu, int leg, size_t desc_dump = 1): BaseDescriptor(simu, desc_dump), _leg(leg) 
	{}

            // This function is called every desc_dump steps of the simulation
            void operator()()
            {
                    // Get current position of the leg
                    Eigen::Vector3d position = _simu->robots().back()->body_pose_vec("leg_" + std::to_string(_leg) + "_3").tail(3);
                    // Store this position in vector of positions
                    //body_pose_vec
                    _positions.push_back(position);

                    // Get current position of the leg
                    Eigen::Vector3d body_position = _simu->robots().back()->body_pose_vec("base_link").tail(3); 
                    // Store this position in vector of positions
                    _body_joint_positions.push_back(body_position);
            }

            // Get list of positions
            void get(std::vector<Eigen::Vector3d> &positions)
            {
                    positions = _positions;
            }

            void get_body_joint(std::vector<Eigen::Vector3d> &positions)
            {
                    positions = _body_joint_positions;
            }

    private:
            std::vector<Eigen::Vector3d> _positions;
            std::vector<Eigen::Vector3d> _body_joint_positions;
            int _leg;
      };
    } // namespace descriptor
} // namespace robot_dart

#endif
