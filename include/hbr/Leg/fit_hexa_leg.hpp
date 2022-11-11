#ifndef ___FIT_HEXA_LEG_HPP__
#define ___FIT_HEXA_LEG_HPP__

#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robot_pool.hpp>

// #ifdef GRAPHIC
// #include <robot_dart/gui/magnum/graphics.hpp>
// #endif

#include <robot_dart/gui/magnum/graphics.hpp>
#include <robot_dart/gui/magnum/windowless_graphics.hpp>

#include "../stats/delete_gen_files.hpp"
#include "../stats/stat_qdprintmap_leg.hpp"

// #include <robot_dart/control/hexa_control.hpp>
#include <sferes/misc/rand.hpp>

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include <sferes/qd/container/archive.hpp>
#include <sferes/qd/container/kdtree_storage.hpp>
#include <sferes/qd/container/sort_based_storage.hpp>

#include "../HBR/qd_layer.hpp"
#include "desc_hexa_leg.hpp"
#include "hexapod_controller_leg_simple.hpp"
#include "hexa_control.hpp"

namespace layer0 {
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

  FIT_QD(Fit_hexa_leg)
  {
  public:
    Fit_hexa_leg(){  }
    
    typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajor > Mat;

    template<typename Indiv>
      void eval(Indiv& ind)
    {
      std::default_random_engine generator;
      std::uniform_int_distribution<int> distribution(0,5);

      //Random Leg Choice since we don't want to build multiple repertoires and also only observe relative displacement.
      int leg = sferes::misc::rand(0,6);//distribution(generator);

      std::vector<Eigen::Vector3d> pos_results,body_pos_results;
      Eigen::Vector3d origin_pos;
      std::vector<double> initial_command;      
      
      std::tie(pos_results,origin_pos,body_pos_results,initial_command) = simulate(ind,leg);

      double initial_amplitude = initial_command[leg*3];//we need the first joint angle for the leg we used. This is the amplitude of the move.ss
      initial_amplitude = (initial_amplitude+(M_PI_4/2))/(M_PI_4);//Normalization


      float activation=0;
      for(size_t i=0;i<_ctrl.size()-1;i++)
        {
          activation-=abs(_ctrl[i]);
        }
      // the fitness is the average amplitude of each motor (corresponds to the average activation of the motors)
      this->_value = activation/3;

      double x, y, h ; 
      x = (pos_results.back()(0)) ; // only want the final x and y position only so  .back. (0) is the x coordinate  
      y = (pos_results.back()(1)) ; //(1) is the y coordinate 
      h = 0.0;

    double start_delta = body_pos_results.front()(2)-pos_results.front()(2);//taking the body reference insted of world reference
    
    for(int i = 0;i<pos_results.size();i++)
        {
                // Get z position
                Eigen::Vector3d position = pos_results[i];
                Eigen::Vector3d body_position = body_pos_results[i];
                double z = position(2)+start_delta-body_position(2);
                // Calculate difference from start position (z = 0)
                if(abs(z) > abs(h)) h = z;
                // if(h<0) std::cout<<i<<" HERE "<<h<<std::endl;
        }


    //caclulate displacement of leg with respect to the body (not the world)

      Eigen::Vector2d initial_position = pos_results.front().head(2);
      Eigen::Vector2d initial_body_position = body_pos_results.front().head(2);

      Eigen::Vector2d final_position = pos_results.back().head(2);
      Eigen::Vector2d final_body_position = body_pos_results.back().head(2);


      Eigen::Vector2d body_leg_final = (final_body_position-final_position);
      Eigen::Vector2d body_leg_initial = (initial_body_position-initial_position);

      Eigen::Vector2d displacement = ((final_body_position-final_position)-(initial_body_position-initial_position));


      double final_distance = displacement.norm();

      double angle = atan2(body_leg_final[1], body_leg_final[0]) - atan2(body_leg_initial[1], body_leg_initial[0]);
      if (angle > M_PI)        { angle -= 2 * M_PI; }
      else if (angle <= -M_PI) { angle += 2 * M_PI; }


        if (angle>0) {
              final_distance *= -1;
      }
  
      std::vector<double> desc;
      desc.push_back(((final_distance)+ 0.1) / 0.2);
      desc.push_back((h+0.06)/ 0.12);

      double duty_cycle = 0;
      if(_ctrl[5]<0.25){
         duty_cycle = 0.0;
      }
      else if(_ctrl[5]<0.5){
        duty_cycle = 0.25;
      }
      else if(_ctrl[5]<0.75){
        duty_cycle = 0.5;
      }
      else if(_ctrl[5]<1.0){
        duty_cycle = 0.75;
      }

      desc.push_back(duty_cycle);//using duty cycle as third dimension of bd

      this->set_desc(desc);

      if(desc[0]<0 || desc[0]>1 ||desc[1]<0 || desc[1]>1 ||desc[2]<0 || desc[2]>1)
        this->_dead=true; //if something is wrong, we kill this solution. 
      
    }
    

    template<typename Indiv>
    std::tuple<std::vector<Eigen::Vector3d>, Eigen::Vector3d,std::vector<Eigen::Vector3d>, std::vector<double> > simulate(Indiv& ind,int leg,bool render=false,const std::string& video_name="hexapod.mp4") 
    {


      int ctrl_size = 6;
      Eigen::VectorXd ctrl;
      ctrl.resize(ctrl_size+1);
    
      for(size_t i=0;i<ctrl.size()-1;i++)
        {
          ctrl[i] = round(ind.data(i) * 1000.0 ) / 1000.0;// limite numerical issues
        }

       //ADDING LEG TO CTRL and removing it later in the controller
      ctrl[ctrl_size]=(double) leg;
      this->_ctrl = ctrl;

      auto g_robot = pool::robot_pool.get_robot();  
      g_robot->skeleton()->setPosition(5, 0.15); 


      double ctrl_dt = 0.01;
      robot_dart::control::HexaLegPolicy<false> reference_policy;
      reference_policy.set_params(ctrl);

      //Initialization

      auto init_controller = std::make_shared<robot_dart::control::HexaLegControlInit>(ctrl_dt, ctrl);

      g_robot->add_controller(init_controller);
      std::static_pointer_cast<robot_dart::control::HexaLegControl>(g_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Constant(1, ctrl_dt));

      robot_dart::RobotDARTSimu simu(0.01);

    if(render){
        // std::cout<<"RENDERING"<<std::endl;
        auto graphics = std::make_shared<robot_dart::gui::magnum::WindowlessGraphics>(&simu);
        graphics->set_enable(true);
        graphics->record_video(video_name, 30);
        simu.set_graphics(graphics);
      }
      simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
      simu.add_checkerboard_floor(10.);
      simu.add_robot(g_robot);

      simu.run(0.3);//initialization
      g_robot->remove_controller(init_controller);

      auto controller = std::make_shared<robot_dart::control::HexaLegControl>(ctrl_dt, ctrl);
      controller->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);
      
      g_robot->add_controller(controller);      
      simu.add_descriptor(std::make_shared<robot_dart::descriptor::HexaLegDescriptor>(robot_dart::descriptor::HexaLegDescriptor(&simu,leg)));
  
      Eigen::Vector3d origin_pos = g_robot->body_pose_vec("leg_" + std::to_string(leg) + "_3").tail(3);

      simu.world()->setTime(0.0);
      simu.run(1.0);

      std::vector<double> initial_command = reference_policy.pos(0.0);// we want to capture the initial position of the leg for continuity purposes
      
      //obtain required data, fitness, descriptors
      // for final position of the robot. not need for this as descriptor is already the x and y position already b
      std::vector<Eigen::Vector3d> pos_results; // results is the final position of the robot leg (used to get the descriptor)
      std::dynamic_pointer_cast<robot_dart::descriptor::HexaLegDescriptor>(simu.descriptor(0))->get(pos_results); //getting descriptor

      std::vector<Eigen::Vector3d> body_pos_results; // results is the final position of the robot base (used to get the descriptor)
      std::dynamic_pointer_cast<robot_dart::descriptor::HexaLegDescriptor>(simu.descriptor(0))->get_body_joint(body_pos_results); //getting descriptor
      
      // CRITICAL : free your robot !
      pool::robot_pool.free_robot(g_robot);

      return std::make_tuple(pos_results,origin_pos,body_pos_results,initial_command);
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
    }
    
  private:
     Eigen::VectorXd _ctrl;

    
  };

using namespace sferes::gen::evo_float;
  struct Params {
      struct nov {
          SFERES_CONST size_t deep = 3;
          static double l;
          SFERES_CONST double k = 15; 
          SFERES_CONST double eps = 0.1;
      };
    
      struct pop {
          // number of initial random points
          SFERES_CONST size_t init_size = 200;
          // size of a batch
          SFERES_CONST size_t size = 200;
          // SFERES_CONST size_t nb_gen = 5001;
          SFERES_CONST size_t nb_gen = 101;
          SFERES_CONST size_t dump_period = 100;
      };
      struct parameters {
        SFERES_CONST float min = 0;
        SFERES_CONST float max = 1.0;
      };
      struct evo_float {
          SFERES_CONST float cross_rate = 0.0f;
          SFERES_CONST float mutation_rate = 0.17f;
          SFERES_CONST float eta_m = 10.0f;
          SFERES_CONST float eta_c = 10.0f;
          SFERES_CONST mutation_t mutation_type = polynomial;
          SFERES_CONST cross_over_t cross_over_type = sbx;
      };
      struct qd {
          SFERES_CONST size_t behav_dim = 3;
          SFERES_ARRAY(size_t, grid_shape, 100, 100);
      };

      struct hbr{
        SFERES_CONST size_t prev_behav_dim = 1;
        SFERES_CONST size_t num_indiv = 1;
      };
  };
  double Params::nov::l = 0.01;

  // using namespace sferes;
  typedef Fit_hexa_leg<Params> fit_t;
  typedef sferes::gen::EvoFloat<6, Params> gen_t;
  typedef sferes::phen::Parameters<gen_t, fit_t, Params> phen_t;

  typedef sferes::qd::selector::Uniform<phen_t, Params> select_t;
  typedef sferes::qd::container::KdtreeStorage<boost::shared_ptr<phen_t>, Params::qd::behav_dim> storage_t;
  typedef sferes::qd::container::Archive<phen_t, storage_t, Params> container_t;

  // #ifdef GRAPHIC
  // typedef sferes::eval::Eval<Params> eval_t;
  // #else
  typedef sferes::eval::Parallel<Params> eval_t;
  // #endif


  typedef boost::fusion::vector<
      sferes::stat::BestFit<phen_t, Params>, 
      sferes::stat::QdContainer<phen_t, Params>, 
      sferes::stat::QdProgress<phen_t, Params>, 
      sferes::stat::QdPrintMapLeg<phen_t, Params>,
      sferes::stat::DeleteGenFiles<phen_t, Params>
      >
      stat_t; 
  typedef sferes::modif::Dummy<> modifier_t;

  typedef sferes::qd::QDLayer<phen_t, eval_t, stat_t, modifier_t, select_t, container_t, Params> qd_t;

  qd_t* layer_ptr;

}//layer0





#endif
