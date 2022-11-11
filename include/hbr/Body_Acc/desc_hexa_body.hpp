#ifndef ROBOT_DART_DESCRIPTOR_HEXA_BODYACC_HPP
#define ROBOT_DART_DESCRIPTOR_HEXA_BODYACC_HPP

// for size_t
#include <cstddef>

namespace robot_dart {
    class RobotDARTSimu;
    class Robot;

    namespace descriptor {

      struct HexaBodyAccDescriptor:public BaseDescriptor{
        public:
	HexaBodyAccDescriptor(RobotDARTSimu* simu, size_t desc_dump = 1):BaseDescriptor(simu,desc_dump)
	{}
	std::vector<Eigen::VectorXf> pos_traj;
	
	virtual void operator()()
	{
	  auto pos=_simu->robots().back()->skeleton()->getPositions().head(6).cast <float> ();
	  pos_traj.push_back(pos);
	  
	}
	
	void get(std::vector<Eigen::VectorXf>& pos_results)
	{
	  pos_results = pos_traj;
	}

      };
    } // namespace descriptor
} // namespace robot_dart

#endif
