#ifndef ROBOT_DART_DESCRIPTOR_HEXA_ACCCELERATION_HPP
#define ROBOT_DART_DESCRIPTOR_HEXA_ACCELERATION_HPP

// for size_t
#include <cstddef>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Frame.hpp>

namespace robot_dart {
    class RobotDARTSimu;
    class Robot;

    namespace descriptor {

      struct HexaBodyAcc:public BaseDescriptor{
        public:
	HexaBodyAcc(RobotDARTSimu* simu, size_t desc_dump = 1):BaseDescriptor(simu,desc_dump)
	{dart::dynamics::BodyNodePtr body_to_check = _simu->robots().back()->skeleton()->getBodyNode("base_link"); 
	this->centerTf = (body_to_check)->getWorldTransform();
	//enforce 2 D transformation in 3d block (we don't account for z)
	this->centerTf(2,2)=1;
	this->centerTf(2,0)=0;
	this->centerTf(2,1)=0;
	this->centerTf(0,2)=0;
	this->centerTf(1,2)=0;
	this->centerTf(2,3)=0;


	this->center_body = dart::dynamics::SimpleFrame(dart::dynamics::Frame::World(), "center",this->centerTf);}

	Eigen::Isometry3d centerTf=Eigen::Isometry3d::Identity();
	std::vector<Eigen::VectorXf> acc_traj;
	std::vector<Eigen::VectorXf> vel_traj;
	dart::dynamics::SimpleFrame center_body;
	
	virtual void operator()()
	{
	  dart::dynamics::BodyNodePtr body_to_check = _simu->robots().back()->skeleton()->getBodyNode("base_link"); 
	  

	  auto body_acceleration= body_to_check->getCOMSpatialAcceleration(dart::dynamics::Frame::World(),dart::dynamics::Frame::World()).head(6).cast <float> (); 
	  auto body_velocity= body_to_check->getCOMSpatialVelocity(dart::dynamics::Frame::World(),dart::dynamics::Frame::World()).head(6).cast <float> (); 
	  acc_traj.push_back(body_acceleration);
	  vel_traj.push_back(body_velocity);
	}
	
	void get_acc(std::vector<Eigen::VectorXf>& pos_results)
	{
	  pos_results = acc_traj;
	}



	void get_vel(std::vector<Eigen::VectorXf>& pos_results)
	{
	  pos_results = vel_traj;
	}


	};
    } // namespace descriptor
} // namespace robot_dart

#endif
