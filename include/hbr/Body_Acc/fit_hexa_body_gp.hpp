#ifndef ___FIT_HEXA_BODY_ACC_HPP__
#define ___FIT_HEXA_BODY_ACC_HPP__

#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robot_pool.hpp>

#include <robot_dart/gui/magnum/graphics.hpp>
#include <robot_dart/gui/magnum/windowless_graphics.hpp>

#include "../stats/delete_gen_files.hpp"
#include "../stats/qd_container_angle.hpp"
#include "../stats/stat_qdprintmap_acc.hpp"
#include "../stats/qd_progress_fit.hpp"
#include "../stats/stat_map_legconfig.hpp"
#include "../stats/stat_map_traj.hpp"
#include "../stats/stat_map_vel.hpp"
#include "../stats/stat_leg_grid.hpp"


#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Frame.hpp>

#include "desc_hexa_acceleration.hpp"
#include "desc_hexa_forces.hpp"
#include "desc_hexa_body.hpp"
#include "hexapod_controller_simple.hpp"

#include "../HBR/qd_layer.hpp"
#include "../HBR/parameters_hbr.hpp"

#include "../Leg/fit_hexa_leg.hpp"
#include "hexa_control.hpp"

//Limbo libraries for GP
#include "../limbo/matern_five_halves_hbr.hpp"
#include "limbo/kernel/exp.hpp"
#include "limbo/kernel/squared_exp_ard.hpp"
#include "limbo/mean/data.hpp"
#include "limbo/model/gp.hpp"
#include "limbo/model/gp/kernel_lf_opt.hpp"
#include "limbo/tools.hpp"
#include "limbo/tools/macros.hpp"

#include "limbo/serialize/text_archive.hpp"

#include <chrono>

#include <sferes/misc/rand.hpp>

#include <tbb/mutex.h>


namespace layer1 {
  // b-a (function for angle calculation)
  double angle_dist(double a, double b)
  {
      double theta = b - a;
      while (theta < -M_PI)
          theta += 2 * M_PI;
      while (theta > M_PI)
          theta -= 2 * M_PI;
      return theta;
  }

namespace pool {
   std::shared_ptr<robot_dart::Robot> robot_creator()
   {
     auto robot = std::make_shared<robot_dart::Robot>("exp/hte/resources/hexapod_v2.urdf");
     robot->set_position_enforced(true);
     robot->set_actuator_types("servo");
     return robot;     
   }
   robot_dart::RobotPool robot_pool(robot_creator, NUM_THREADS);

} // namespace pool
bool check_rotation(robot_dart::RobotDARTSimu* simu){
      dart::dynamics::BodyNode* base_link_body_node = simu->robots().back()->body_node("base_link");
      Eigen::Matrix3d rot = base_link_body_node->getTransform().linear();
       // Then Rotation coordinates
      Eigen::Vector3d forward_direction = rot.col(0);
      Eigen::Vector3d left_direction = rot.col(1);
      Eigen::Vector3d up_direction = rot.col(2);

      Eigen::Vector3d u_z{0., 0., 1.};

      // u_r and u_theta -> Cylindrical coordinate system
      Eigen::Vector3d u_r{forward_direction(0), forward_direction(1), 0.};
      u_r = u_r.normalized();
      Eigen::Vector3d u_theta = u_z.cross(u_r);
      auto abs_pitch_angle = static_cast<float>(
        acos(forward_direction.dot(u_z)));
      auto abs_roll_angle = static_cast<float>(
        acos(u_theta.dot(left_direction)));
      auto abs_yaw_angle = static_cast<float>(
        acos(u_r.dot(Eigen::Vector3d(1., 0., 0.)))
      );

      // Get values of angles depending on the direction of the vector

      float pitch_angle;
      float roll_angle;
      float yaw_angle;


      if (u_z.dot(up_direction) > 0.) {
        pitch_angle = abs_pitch_angle;
      } else {
        pitch_angle = -1.f * abs_pitch_angle;
      }

      if (u_theta.dot(up_direction) < 0.) {
        roll_angle = abs_roll_angle;
      } else {
        roll_angle = -1.f * abs_roll_angle;
      }

      if (u_r.dot(Eigen::Vector3d(0., 1., 0.)) > 0) {
        yaw_angle = abs_yaw_angle;
      } else {
        yaw_angle = -1.f * abs_yaw_angle;
      }
      if(pitch_angle>M_PI_4 && pitch_angle< 3*M_PI_4){
        if(roll_angle>-M_PI_4 && roll_angle<M_PI_4){
          return true;
        }
        else
          return false;
      }
      else
        return false;
    }

  FIT_QD(Fit_hexa_body)
  {
  public:
    Fit_hexa_body(){}
    
    typedef std::vector<boost::shared_ptr<layer0::phen_t> > pop_t;
    typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajor > Mat;
    
    using Kernel_t = limbo::kernel::MaternFiveHalvesHBR<Params>;
    using Mean_t = limbo::mean::Data<Params>;
    typedef limbo::model::GP<Params, Kernel_t, Mean_t> GP_t;
    
    GP_t gp;
    Eigen::VectorXd mu;
    double sigma;
    bool samples_updated=false;
    std::vector<Eigen::VectorXd> samples,observations;
    std::vector<std::pair<Eigen::VectorXd,Eigen::VectorXd>> gp_buffer;
    
    

    template<typename Indiv>
      void eval(Indiv& ind)
    {

      Eigen::Vector6d vel_results,ini;
      std::vector<Eigen::VectorXf> pos_results,acc_results;//,vel_results;
      std::vector<double> forces;
      std::vector<double> desc;

      std::vector<std::vector<double>> all_forces;
      std::vector<Eigen::Vector6d> all_vel;
      std::vector<std::vector<Eigen::VectorXf>> all_pos,all_acc;


      Eigen::VectorXd init_vel(24),init_acc(24),state(18);//initializing the new velocity state
      init_vel.setZero(24);
      init_acc.setZero(24);
      state.setZero(18);
      state[5] = 0.15;//height of the robot

      double num_samples = 10;
      for(size_t i=0;i<num_samples;i++){

        std::tie(vel_results,acc_results,pos_results, forces) = simulate(ind,init_vel,init_acc);
        
        all_forces.push_back(forces);
        all_vel.push_back(vel_results);
        all_pos.push_back(pos_results);
        all_acc.push_back(acc_results);
        
        double x=0, y=0, yaw=0 ; 
        x = (vel_results(3)) ; // We need the x acceleration   and we use the first initial point 
        y = (vel_results(4)) ; // We need the x acceleration  
        yaw = std::round(vel_results(2) * 100) / 100.0; //We want the yaw acceleration
        
        double desc_x, desc_y,desc_yaw ; 

        desc_x = (double) (x+0.5)/1.0; //use this for velocity
        desc_y = (double)  (y+0.5)/1.0;
        desc_yaw = (double) (yaw+1.0)/2.0;

        desc.clear();
        desc.push_back(desc_x);
        desc.push_back(desc_y);
        desc.push_back(desc_yaw);

        Eigen::VectorXd desc_eigen(9);
        desc_eigen << desc_x, desc_y, desc_yaw,forces[0],forces[1],forces[2],forces[3],forces[4],forces[5];

        this->add_sample(state,desc_eigen,ind);

        init_vel = this->current_vel;//using speed at end of our last sample
        init_acc = this->current_acc;//using speed at end of our last sample
        state = this->current_state;


        this->all_bd.push_back(desc);

      }
     
      this->gp.compute(this->samples,this->observations,true);//training GP with samples from the simulation runs.
      
      std::vector<double> med_desc;
      double med_var;
      size_t index_bd;

      std::tie(med_desc,med_var,index_bd) = geo_med(all_bd);

      for (size_t i = 0; i < 6; ++i) {
       med_desc.push_back(all_forces[index_bd][i]);
      }

      // Performance - Angle Difference (desrird angle and obtained angle fomr simulation)
      // Change of orientation of axis in counting the desried angle to account for frontal axis of the newer robot (x-axis:frontal axis)
      double ang_y = all_pos[index_bd].back()(4); 
      double ang_x = all_pos[index_bd].back()(3); 
      
      // Computation of desired angle (yaxis-north x-axis(postive))
      double B = std::sqrt((ang_x / 2.0) * (ang_x / 2.0) + (ang_y / 2.0) * (ang_y / 2.0));
      double alpha = std::atan2(ang_y, ang_x);
      double A = B / std::cos(alpha);
      double beta = std::atan2(ang_y, ang_x - A);

      if (ang_x < 0)
          beta = beta - M_PI;
      while (beta < -M_PI)
          beta += 2 * M_PI;
      while (beta > M_PI)
          beta -= 2 * M_PI;
              
      double angle_diff = std::abs(angle_dist(beta, this->_angle));

      this->_value = -angle_diff;


      this->set_desc(med_desc);


      if(med_desc[0]<0 || med_desc[0]>1 ||med_desc[1]<0 || med_desc[1]>1 || med_desc[2]<0 || med_desc[2]>1|| !(this->_valid_orientation)){ 
        this->_dead=true; //if something is wrong, we kill this solution.
      }
         
      
    }

    
    template<typename Indiv>
    std::tuple<Eigen::Vector6d,std::vector<Eigen::VectorXf>,std::vector<Eigen::VectorXf>, std::vector<double> > simulate(Indiv& ind, Eigen::VectorXd init_vel, Eigen::VectorXd init_acc, bool render=false,const std::string& video_name="hexapod.mp4") 
    {

      Eigen::VectorXd ctrl;
      Eigen::Vector6d init_trans(6);
      _children.clear();

      ctrl.resize(18);
      for(size_t i=0;i<ctrl.size();i++)
        {
            ctrl[i] = round( ind.data(i) * 1000.0 ) / 1000.0;// limite numerical issues
        }

      auto g_robot = pool::robot_pool.get_robot();  
      g_robot->skeleton()->setPosition(5, 0.15);
      
      g_robot->skeleton()->setVelocities(init_vel);
      g_robot->skeleton()->setAccelerations(init_acc);

      robot_dart::RobotDARTSimu simu(0.01);

      simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
      simu.add_checkerboard_floor(10.);
      simu.add_robot(g_robot);
      std::shared_ptr<robot_dart::control::HexaBodyControl> controller;

      dart::dynamics::BodyNodePtr body_to_check;
      dart::dynamics::SimpleFrame center_body;


      if(render){
        // std::cout<<"RENDERING"<<std::endl;
        auto graphics = std::make_shared<robot_dart::gui::magnum::WindowlessGraphics>(&simu);
        graphics->set_enable(true);
        graphics->record_video(video_name, 30);
        simu.set_graphics(graphics);
      }
      
      
      auto start = std::chrono::high_resolution_clock::now();
      for(int step=0;step<1;step++)
      {

          std::vector<double> correct_ctrl;
          std::vector<double> leg_ctrl;
          double height,duty_cycle;
          double end_pos;  
          double init_amplitude;

          for (int leg = 0; leg<6; leg++)
          {         
            height = ctrl[leg*3+1];
            end_pos = ctrl[leg*3+0];
            duty_cycle = ctrl[leg*3+2];
            
            Eigen::Vector3d descriptor (end_pos, height,duty_cycle);

            auto P1 = layer0::layer_ptr->container().archive().nearest(descriptor);
            auto indiv = P1.second;
            _children.push_back(indiv);
            std::vector<double> desc_l1 = indiv->fit().desc();
            // Return controller
            std::vector<float> gen = indiv->data();

            Eigen::Vector3d origin_pos = g_robot->body_pose_vec("leg_" + std::to_string(leg) + "_3").tail(3);

            double orig_x_pos = 0.0;
            

            std::vector<double> ctrl_l1 (gen.begin(), gen.end());
            correct_ctrl.insert(correct_ctrl.end(),ctrl_l1.begin(), ctrl_l1.end());
          }


          double ctrl_dt = 0.01;
          double* ptr = &correct_ctrl[0];
          Eigen::Map<Eigen::VectorXd> new_ctrl(ptr, correct_ctrl.size()); 

          controller = std::make_shared<robot_dart::control::HexaBodyControl>(ctrl_dt, new_ctrl);
          g_robot->add_controller(controller);
          std::static_pointer_cast<robot_dart::control::HexaBodyControl>(g_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);
          if(step==0){
          std::static_pointer_cast<robot_dart::control::HexaBodyControl>(g_robot->controllers()[0])->activate(false);
          simu.run(0.2);
          //run the controller to go to the t=0 pos and stabilise for 0.4 second
          simu.world()->setTime(0);
          simu.run(0.1);
          simu.world()->setTime(0);
          simu.run(0.1);
          std::static_pointer_cast<robot_dart::control::HexaBodyControl>(g_robot->controllers()[0])->activate(true);
          simu.add_descriptor(std::make_shared<robot_dart::descriptor::HexaBodyAcc>(robot_dart::descriptor::HexaBodyAcc(&simu)));
          simu.add_descriptor(std::make_shared<robot_dart::descriptor::Forces>(robot_dart::descriptor::Forces(&simu)));
          simu.add_descriptor(std::make_shared<robot_dart::descriptor::HexaBodyAccDescriptor>(robot_dart::descriptor::HexaBodyAccDescriptor(&simu)));
          init_trans = simu.robots().back()->skeleton()->getPositions().head(6);
          
          body_to_check = simu.robots().back()->skeleton()->getBodyNode("base_link"); 
          Eigen::Isometry3d centerTf = (body_to_check)->getWorldTransform();
          center_body = dart::dynamics::SimpleFrame(dart::dynamics::Frame::World(), "center",centerTf);
          
          }
          simu.world()->setTime(0.0);
          simu.run(1);
          std::static_pointer_cast<robot_dart::control::HexaBodyControl>(g_robot->controllers()[0])->activate(false);
          this->current_vel = simu.robots().back()->skeleton()->getVelocities();
          this->current_acc = simu.robots().back()->skeleton()->getAccelerations();

          Eigen::VectorXd real_state(18);
          real_state << init_trans,(this->current_vel).head(6),(this->current_acc).head(6);
          this->current_state = real_state;
          g_robot->remove_controller(controller);
      }
      // Record end time
      auto finish = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = finish - start;

      // std::cout << "Elapsed time: " << elapsed.count() << " s\n";
      //obtain required data, fitness, descriptors for final position of the robot. 
      std::vector<Eigen::VectorXf> vel_results; // results is the final velocity of the robot (used to get the descriptor)
      std::static_pointer_cast<robot_dart::descriptor::HexaBodyAcc>(simu.descriptor(0))->get_vel(vel_results); //getting descriptor
      this -> _traj = vel_results;

      std::vector<Eigen::VectorXf> acc_results; // results is the final acceleration of the robot (used to get the descriptor)
      std::static_pointer_cast<robot_dart::descriptor::HexaBodyAcc>(simu.descriptor(0))->get_acc(acc_results); //getting descriptor

      std::vector<double> forces; // results is the final contacts per leg for the robot (used to get the descriptor)
      std::static_pointer_cast<robot_dart::descriptor::Forces>(simu.descriptor(1))->get(forces); //getting descriptor

      std::vector<Eigen::VectorXf> pos_results; // results is the final position of the robot (used to get the descriptor)
      std::static_pointer_cast<robot_dart::descriptor::HexaBodyAccDescriptor>(simu.descriptor(2))->get(pos_results); //getting descriptor
      this->_forces = forces;


      Eigen::Vector3d _final_pos;
      Eigen::Vector3d _final_rot;
      double _arrival_angle;
      double _covered_distance;
      
      auto pose = simu.robots().back()->skeleton()->getPositions(); //use current position to find speed
      Eigen::Matrix3d rot = dart::math::expMapRot({pose[0], pose[1], pose[2]});

      Eigen::Matrix3d init_rot = dart::math::expMapRot({0, 0, init_trans[2]});
      Eigen::MatrixXd init_homogeneous(4, 4);
      init_homogeneous << init_rot(0, 0), init_rot(0, 1), init_rot(0, 2), init_trans[3], init_rot(1, 0), init_rot(1, 1), init_rot(1, 2), init_trans[4], init_rot(2, 0), init_rot(2, 1), init_rot(2, 2), init_trans[5], 0, 0, 0, 1;
      Eigen::MatrixXd final_homogeneous(4, 4);
      final_homogeneous << rot(0, 0), rot(0, 1), rot(0, 2), pose[3], rot(1, 0), rot(1, 1), rot(1, 2), pose[4], rot(2, 0), rot(2, 1), rot(2, 2), pose[5], 0, 0, 0, 1;
    
      
      Eigen::Vector4d pos = {pose[3], pose[4], pose[5], 1.0};
      pos = init_homogeneous.inverse()*pos; // We want our final position to be expressed with respect to the initial frame

      _final_pos = pos.head(3);
      _covered_distance = std::round(_final_pos(0) * 100) / 100.0;

      // Angle computation
      _final_rot = dart::math::matrixToEulerXYZ(init_rot.inverse() * rot);

      // roll-pitch-yaw
      _arrival_angle = std::round(_final_rot(2) * 100) / 100.0; //yaw

      this->_angle = _arrival_angle;

      Eigen::Vector6d velocity(6),positions(6);
      positions << _final_rot, _final_pos;
      velocity << positions/1.0; //we run the experiment for 1 s, needs to be adjusted if this chanhes so that we can use the average velocity

      this->_valid_orientation = check_rotation(&simu);
      // CRITICAL : free your robot !
      pool::robot_pool.free_robot(g_robot);

      return std::make_tuple(velocity, acc_results,pos_results, forces);
    }

  std::tuple<std::vector<double>,double,size_t> geo_med(std::vector<std::vector<double>> global_desc){
    std::vector<double> median_desc(2,0);
    size_t size = global_desc.size();
    // GEOMETRIC MEDIAN
    std::vector<double> dist_desc(size,0);
    for(size_t i=0;i<size;i++)
	    for(size_t j=0;j<size;j++)
	      dist_desc[i]+=(global_desc[i][0]-global_desc[j][0])*(global_desc[i][0]-global_desc[j][0]) + (global_desc[i][1]-global_desc[j][1])*(global_desc[i][1]-global_desc[j][1]) + (global_desc[i][2]-global_desc[j][2])*(global_desc[i][1]-global_desc[j][2]);

      size_t index_min=0;
      double val=dist_desc[0];
      for(size_t i=1;i<size;i++)
	    {
	      if(dist_desc[i]<val)
	      {
	        val=dist_desc[i];
	        index_min=i;
	      }
	    }

      
      median_desc=global_desc[index_min];

      median_desc[0]=median_desc[0];
      median_desc[1]=median_desc[1];
      double variance = dist_desc[index_min];

      return {median_desc,variance,index_min};
  }
    
  double angle(){
    return this->_angle;
  }

  std::vector<double> forces(){
    return this->_forces;
  }

  std::vector<Eigen::VectorXd> get_samples(){
    return this->samples;
  }

  std::vector<Eigen::VectorXf> traj(){
    return this->_traj;
  }

  pop_t children(){
    return this->_children;
  }


  template<typename Indiv>
  void add_sample(Eigen::VectorXd sample,Eigen::VectorXd observation,Indiv& ind){
    ind.mylock.lock();
    if (this->samples.size()<500)
      {
      this->samples.push_back(sample);
      this->observations.push_back(observation);
      this->sample_it +=1;
      }
    else{
        this->samples[sample_it%500]=sample;
        this->observations[sample_it%500]=observation;
        this->sample_it +=1;
    }
    this->samples_updated=true;

    ind.mylock.unlock();
  }

  void fit_gp(bool compute=true,bool force_fit=false){
    // reducing size of samples
    if ((this->samples_updated==true)||force_fit){
    
      if(this->samples.size()!=this->observations.size()){
        std::cout<<"ISSUE "<<this->samples.size()<<" "<<this->observations.size()<<" "<<this->sample_it<<std::endl;
      }
      assert(("Different Sizes for the samples and observations",this->samples.size()==this->observations.size()));
      this->gp.compute(this->samples,this->observations,compute);
      this->samples_updated=false;
    }
  }
  
  void alive(){
    this->_dead = false;
  }

  
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      ar& BOOST_SERIALIZATION_NVP(this->_objs);
      ar& BOOST_SERIALIZATION_NVP(this->_value);
      ar& BOOST_SERIALIZATION_NVP(this->_dead);
      ar& BOOST_SERIALIZATION_NVP(this->_desc);
      ar& BOOST_SERIALIZATION_NVP(this->_novelty);
      ar& BOOST_SERIALIZATION_NVP(this->_curiosity);
      ar& BOOST_SERIALIZATION_NVP(this->_angle);
      ar& BOOST_SERIALIZATION_NVP(this->_forces);
      ar& BOOST_SERIALIZATION_NVP(this->_children);
      ar& BOOST_SERIALIZATION_NVP(this->samples);
      ar& BOOST_SERIALIZATION_NVP(this->observations);
    }

  
    
  private:
    Eigen::VectorXd _ctrl;
    std::vector<Eigen::VectorXf> _traj;
    double _angle;
    std::vector<double> _forces;
    pop_t _children;
    std::vector<Eigen::Vector6d> initial_states;
    std::vector<std::vector<double>> all_bd;
    Eigen::VectorXd current_vel,current_acc,current_state;
    int sample_it=0;
    bool _valid_orientation=false;

    
  };


using namespace sferes::gen::evo_float;
using namespace limbo;
  struct Params {
    //limbo parameters
      struct kernel_exp {
          BO_PARAM(double, noise, 0.01);
          BO_PARAM(double, sigma_sq, 1.0);
          BO_PARAM(double, l, 0.8);
      };
      struct kernel : public defaults::kernel {
      };
      struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {
      };
      struct opt_rprop : public defaults::opt_rprop {
      };
      struct kernel_maternfivehalves : public defaults::kernel_maternfivehalves {
          BO_PARAM(double, noise, 0.01);
          BO_PARAM(double, sigma_sq, 0.1);
          BO_PARAM(double, l, 5.0);
      };
      struct kernel_maternfivehalves_hbr : public defaults::kernel_maternfivehalves_hbr {
          BO_PARAM(double, noise, 0.01);
          BO_PARAM(double, sigma_sq, 0.1);
          BO_PARAM(double, l, 5.0);
      };
      struct nov {
          static size_t deep;
          static double l; 
          SFERES_CONST double k = 15;
          SFERES_CONST double eps = 0.1;
      };
    
      // TODO: move to a qd::
      struct pop {
          // number of initial random points
          SFERES_CONST size_t init_size = 500;
          // size of a batch
          SFERES_CONST size_t size = 200;
          // SFERES_CONST size_t nb_gen = 30001;
          SFERES_CONST size_t nb_gen = 101;
          SFERES_CONST size_t dump_period = 100;
          // SFERES_CONST size_t dump_period = 5000;
      };
      struct parameters {
        SFERES_CONST float min = 0.0;
        SFERES_CONST float max = 1.0;
      };
      struct evo_float {
          SFERES_CONST float cross_rate = 0.0f;
          SFERES_CONST float mutation_rate = 0.11f;
          SFERES_CONST float eta_m = 10.0f;
          SFERES_CONST float eta_c = 10.0f;
          SFERES_CONST mutation_t mutation_type = polynomial;
          SFERES_CONST cross_over_t cross_over_type = sbx;
      };
      struct qd {
            SFERES_CONST size_t behav_dim = 9;
            SFERES_ARRAY(size_t, grid_shape,15,15,15,2,2,2,2,2,2);
      };

      struct hbr{
        SFERES_CONST size_t prev_behav_dim = 2;
        SFERES_CONST size_t num_indiv = 6;
      };
  };
  
  double Params::nov::l = 0.05;
  size_t Params::nov::deep = 3;

  // using namespace sferes;
  typedef Fit_hexa_body<Params> fit_t;
  typedef sferes::gen::EvoFloat<18, Params> gen_t;
  typedef sferes::phen::Parameters_HBR<gen_t, fit_t, Params> phen_t;

  typedef sferes::qd::selector::Uniform<phen_t, Params> select_t;
  typedef sferes::qd::container::KdtreeStorage<boost::shared_ptr<phen_t>, Params::qd::behav_dim> storage_t;
  typedef sferes::qd::container::Archive<phen_t, storage_t, Params> container_t;

  // #if defined (GRAPHIC)
  //   typedef sferes::eval::Eval<Params> eval_t;
  // #else
  typedef sferes::eval::Parallel<Params> eval_t;
  // #endif


  typedef boost::fusion::vector<
      sferes::stat::BestFit<phen_t, Params>, 
      sferes::stat::QdContainer<phen_t, Params>, 
      sferes::stat::QdProgressFit<phen_t, Params>, 
      sferes::stat::QdPrintMapAcc<phen_t, Params>,
      sferes::stat::QdMapLegConfig<phen_t, Params>,
      sferes::stat::QdMapTraj<phen_t, Params>,
      sferes::stat::QdMapVel<phen_t, Params>,
      sferes::stat::QdMapLegGrid<phen_t, Params>,
      sferes::stat::State<phen_t, Params>,
      sferes::stat::DeleteGenFiles<phen_t, Params>
      >
      stat_t; 

  typedef sferes::modif::Dummy<> modifier_t;
  typedef sferes::qd::QDLayer<phen_t, eval_t, stat_t, modifier_t, select_t, container_t, Params> qd_t;

  qd_t* layer_ptr;

}//layer1



#endif
