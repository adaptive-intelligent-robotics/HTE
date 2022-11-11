#ifndef ROBOT_DART_DESCRIPTOR_HEXA_BODY_HPP
#define ROBOT_DART_DESCRIPTOR_HEXA_BODY_HPP

// for size_t
#include <cstddef>

namespace robot_dart {
    class RobotDARTSimu;
    class Robot;

    namespace descriptor {

      struct HexaBodyDescriptor:public BaseDescriptor{
        public:
	HexaBodyDescriptor(RobotDARTSimu* simu, size_t desc_dump = 1):BaseDescriptor(simu,desc_dump)
	{}
	std::vector<Eigen::VectorXf> pos_traj;
	
	virtual void operator()()
	{
	  auto pos=_simu->robots().back()->skeleton()->getPositions().head(6).tail(3).cast <float> ();
	  pos_traj.push_back(pos.head(3));
	  //std::cout<<pos.head(2).transpose()<<std::endl;
	  //if( traj.back()[0] > 1 || traj.back()[1] > 1 || traj.back()[0] < -1 || traj.back()[1] < -1 )
	  //std::cout<<"ERROR "<<traj.back().transpose()<<std::endl;
	}
	
	void get(std::vector<Eigen::VectorXf>& pos_results)
	{
	  pos_results = pos_traj;
	}

      };
    } // namespace descriptor
} // namespace robot_dart

#endif
