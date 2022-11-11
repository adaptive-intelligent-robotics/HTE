#ifndef ROBOT_DART_DESCRIPTOR_HEXA_ACCCELERATION_OMNI_HPP
#define ROBOT_DART_DESCRIPTOR_HEXA_ACCELERATION_OMNI_HPP

// for size_t
#include <cstddef>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Frame.hpp>

namespace robot_dart {
    class RobotDARTSimu;
    class Robot;

    namespace descriptor {

      struct HexaBodyAccOmni:public BaseDescriptor{
        public:
	HexaBodyAccOmni(RobotDARTSimu* simu, size_t desc_dump = 1):BaseDescriptor(simu,desc_dump)
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

	std::vector<Eigen::VectorXf> acc_traj;
	std::vector<Eigen::VectorXf> vel_traj;
	std::vector<Eigen::VectorXf> test_vel_traj;
	Eigen::Isometry3d centerTf;
	dart::dynamics::SimpleFrame center_body;
	// 
	
	virtual void operator()()
	{
	  dart::dynamics::BodyNodePtr body_to_check = _simu->robots().back()->skeleton()->getBodyNode("base_link"); 

	  



	//   Eigen::Vector3d body_rotation = body_to_check->getCOMSpatialAcceleration().head(3); 
	//   auto body_acceleration= body_to_check->getCOMSpatialAcceleration(&(this->center_body),&(*body_to_check)).head(6).cast <float> (); 
	//   auto body_velocity= body_to_check->getCOMSpatialVelocity(&(this->center_body),&(*body_to_check)).head(6).cast <float> (); 

	  auto body_acceleration= body_to_check->getCOMSpatialAcceleration(dart::dynamics::Frame::World(),&(this->center_body)).head(6).cast <float> (); 
	  auto body_velocity= body_to_check->getCOMSpatialVelocity(dart::dynamics::Frame::World(),&(this->center_body)).head(6).cast <float> (); 



	  auto test_acceleration= body_to_check->getCOMSpatialAcceleration().head(6).cast <float> (); 
	  auto test_velocity= body_to_check->getCOMLinearVelocity(&(this->center_body),&(this->center_body)).head(3).cast <float> (); 


	//   auto body_acceleration= body_to_check->getCOMSpatialAcceleration().head(6).cast <float> (); 
	//   auto body_velocity= body_to_check->getCOMSpatialVelocity().head(6).cast <float> (); 


	//   auto body_acceleration= body_to_check->getCOMLinearAcceleration().head(6).cast <float> (); 
	//   auto body_velocity= body_to_check->getCOMLinearVelocity().head(6).cast <float> (); 
	//  std::cout<<"ACC"<<std::endl;
	//   auto pos=_simu->robots().back()->skeleton()->getPositions().head(6).tail(3).cast <float> ();
	  acc_traj.push_back(body_acceleration);
	  vel_traj.push_back(body_velocity);
	  test_vel_traj.push_back(body_velocity);
	//   std::cout<<body_acceleration.tail(3).transpose()<<std::endl;
	  //if( traj.back()[0] > 1 || traj.back()[1] > 1 || traj.back()[0] < -1 || traj.back()[1] < -1 )
	  //std::cout<<"ERROR "<<traj.back().transpose()<<std::endl;
	}
	
	void get_acc(std::vector<Eigen::VectorXf>& pos_results)
	{
	  pos_results = acc_traj;
	}



	void get_vel(std::vector<Eigen::VectorXf>& pos_results)
	{
	  pos_results = vel_traj;
	}


	void reset()
	{

		// double x=0,y=0,yaw=0;
		// for(int i=0;i<test_vel_traj.size();i++){
        //     x += test_vel_traj[i](0)/test_vel_traj.size(); // We need the x acceleration   
        //     y += test_vel_traj[i](1)/test_vel_traj.size() ; // We need the x acceleration  
        //     // yaw += test_vel_traj[i](2)/test_vel_traj.size() ; //We 
        //   }

        //   x = (x+0.3)/0.6; //use this for velocity
        //   y = (y+0.3)/0.6;
        // //   yaw = (yaw+1.0)/2.0;

		//   Eigen::VectorXf real_obs(2);
        //   real_obs<< x,y;
		//   std::cout<<real_obs.transpose()<<std::endl;
	
		dart::dynamics::BodyNodePtr body_to_check = _simu->robots().back()->skeleton()->getBodyNode("base_link"); 
		this->centerTf = (body_to_check)->getWorldTransform();
		//enforce 2 D transformation in 3d block (we don't account for z)
		this->centerTf(2,2)=1;
		this->centerTf(2,0)=0;
		this->centerTf(2,1)=0;
		this->centerTf(0,2)=0;
		this->centerTf(1,2)=0;
		this->centerTf(2,3)=0;


		// this->center_body = dart::dynamics::SimpleFrame(dart::dynamics::Frame::World(), "center",this->centerTf);

		this->center_body.setTransform(centerTf);
		// this->center_body.setRotation(centerTf.rotation());

		// std::cout<<((body_to_check)->getCOM()).transpose()<<std::endl;
		// std::cout<<((body_to_check)->getCOM(&(this->center_body))).transpose()<<std::endl;

		
		auto pos=_simu->robots().back()->skeleton()->getPositions().head(6);

		// centerTf.matrix().block<3,3>(0,0) = rot;
		// centerTf.matrix().block<3,1>(0,3) = pos.tail(3);
		// this->centerTf.translation() = pos.tail(3);
		// this->centerTf.rotation() = rot;
		// std::cout<<rot<<std::endl;
		// std::cout<<pos.transpose()<<std::endl;
		// std::cout<<this->center_body.getTransform().matrix()<<std::endl;

		// this->center_body.setRelativeTransform(centerTf);
		this->vel_traj.clear();
		this->acc_traj.clear();
		this->test_vel_traj.clear();
	}



    };
	struct PositionTraj : public BaseDescriptor {
	public:
		PositionTraj(RobotDARTSimu* simu, size_t desc_dump = 1):BaseDescriptor(simu,desc_dump){}

		void operator()(const Eigen::Vector6d& init_trans)
		{
			auto pose = _simu->robots().back()->skeleton()->getPositions();
			// Eigen::Vector6d pose = rob->pose();
			Eigen::Matrix3d rot = dart::math::expMapRot({pose[0], pose[1], pose[2]});
			Eigen::Matrix3d init_rot = dart::math::expMapRot({init_trans[0], init_trans[1], init_trans[2]});
			Eigen::MatrixXd init_homogeneous(4, 4);
			init_homogeneous << init_rot(0, 0), init_rot(0, 1), init_rot(0, 2), init_trans[3], init_rot(1, 0), init_rot(1, 1), init_rot(1, 2), init_trans[4], init_rot(2, 0), init_rot(2, 1), init_rot(2, 2), init_trans[5], 0, 0, 0, 1;
			Eigen::MatrixXd final_homogeneous(4, 4);
			final_homogeneous << rot(0, 0), rot(0, 1), rot(0, 2), pose[3], rot(1, 0), rot(1, 1), rot(1, 2), pose[4], rot(2, 0), rot(2, 1), rot(2, 2), pose[5], 0, 0, 0, 1;
			Eigen::Vector4d pos = {init_trans[3], init_trans[4], init_trans[5], 1.0};
			pos = init_homogeneous.inverse() * final_homogeneous * pos;

			_pos_traj.push_back({pos[0], pos[1], pos[2]});
		}

		void get(std::vector<Eigen::Vector3d>& results)
		{
			results = _pos_traj;
		}

	protected:
		std::vector<Eigen::Vector3d> _pos_traj;
	};

	struct RotationTraj : public BaseDescriptor {
	public:
		RotationTraj(RobotDARTSimu* simu, size_t desc_dump = 1):BaseDescriptor(simu,desc_dump){}

		void operator()(const Eigen::Vector6d& init_trans)
		{
			// roll-pitch-yaw
			auto pose = _simu->robots().back()->skeleton()->getPositions();
			auto rob = _simu->robots().back();//->skeleton()->getPositions();
			// Eigen::Matrix3d rot = dart::math::expMapRot(rob->rot());
			Eigen::Matrix3d rot = dart::math::expMapRot({pose[0], pose[1], pose[2]});
			Eigen::Matrix3d init_rot = dart::math::expMapRot({init_trans[0], init_trans[1], init_trans[2]});
			auto rpy = dart::math::matrixToEulerXYZ(init_rot.inverse() * rot);

			_rotation_traj.push_back(rpy);
		}

		void get(std::vector<Eigen::Vector3d>& results)
		{
			results = _rotation_traj;
		}

	protected:
		std::vector<Eigen::Vector3d> _rotation_traj;
	};
    } // namespace descriptor
} // namespace robot_dart

#endif
