#ifndef ___FIT_HEXA_OMNI_HPP__
#define ___FIT_HEXA_OMNI_HPP__

#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robot_pool.hpp>
#include <math.h> 

#include <robot_dart/gui/magnum/graphics.hpp>
#include <robot_dart/gui/magnum/windowless_graphics.hpp>

#include <sferes/misc.hpp>

#include "../stats/delete_gen_files.hpp"
#include "../stats/qd_container_angle.hpp"
#include "../stats/stat_qdprintmap.hpp"
#include "../stats/qd_progress_fit.hpp"
#include "../stats/qd_progress_err.hpp"
#include "../stats/stat_qdprintmap_unc.hpp"
#include "../stats/qd_multitask_eval.hpp"
#include "../stats/stat_multitask_grid.hpp"

#include "../modifier/gp_update.hpp"
#include "../modifier/gp_search.hpp"
#include "../modifier/age_increase.hpp"

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include "desc_hexa_body.hpp"
#include "desc_hexa_uni.hpp"
#include "desc_hexa_acceleration.hpp"
#include "../Body_Acc/desc_hexa_forces.hpp"

#include "./hexapod_controller_simple.hpp"

#include "../HBR/qd_layer.hpp"

#include "../HBR/parameters_hbr.hpp"
#include "./hexa_control.hpp"



namespace layer2 {
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
     auto robot = std::make_shared<robot_dart::Robot>("exp/HTE/resources/hexapod_v2.urdf");
     robot->set_position_enforced(true);
     robot->set_actuator_types("servo");
    //  robot->skeleton()->enableSelfCollisionCheck();
     return robot;     
   }
   robot_dart::RobotPool robot_pool(robot_creator, NUM_THREADS);

} // namespace pool


  FIT_QD(Fit_hexa_omni)
  {
  public:
    Fit_hexa_omni(){  }

    //typdefs for the Grid 
    typedef boost::shared_ptr<layer1::phen_t> indiv_t;
    typedef std::vector<boost::shared_ptr<layer1::phen_t> > pop_t;
    
    typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajor > Mat;

    template<typename Indiv>
      void eval(Indiv& ind)
    {
      this->_obs_descs.resize(64);
      for (size_t i = 0; i<64; i++){
	      this->_obs_descs[i] = std::vector<double>();
	    }
 
      this->_goal_x_pos = 0.0;
      this->_goal_y_pos = 0.0;
      this->_goal_yaw_pos = 0.0;

      auto data = simulate(ind);
      std::vector<double> desc;

      double x, y ; 
      x = (data.first.back()(0)) ; // only want the final x and y position only so  .back. (0) is the x coordinate  
      y = (data.first.back()(1)) ; //(1) is the y coordinate 

      double desc_x, desc_y ; 
      desc_x = (x+1.5)/3 ; 
      desc_y = (y+1.5)/3 ; // scaling (scaling based on estimate of travel distance etc) to maximise and ensure repertoire between 0 and 1. can also try to increase archive size to 100x100


      desc.push_back(desc_x);
      desc.push_back(desc_y);
      
      auto arr_angle = data.second ;
      this->_angle = arr_angle;
     
      if(data.first.size() == 0){
        this->_dead=true;
        this->_value = -1;
        std::vector<double> desc={0.0,0.0};
        this->set_desc(desc);
        return;
      }

      //CAlculate Fitness
      this->_value = virtual_fit(data);
      this->set_desc(desc);

      if(desc[0]<0 || desc[0]>1 ||desc[1]<0 || desc[1]>1)
        this->_dead=true; //if something is wrong, we kill this solution. 
    }
    
    template<typename Indiv>
    std::pair<std::vector<Eigen::VectorXf>, double > simulate(Indiv& ind, bool render=false,const std::string& video_name="hexapod.mp4",int leg_configuration=-1,bool use_children=false,int max_steps=5) 
    {

      Eigen::VectorXd ctrl;
      Eigen::VectorXd global_desired_pos = Eigen::VectorXd::Zero(3);
      this->_all_forces.clear();//empty to get newest forces for each run
      this->_waypoints.clear();
      this-> _orientation_error=0;
      this-> _obs_error=0;
      this->_gp_uncertainty=0;

      ctrl.resize(9);
      for(size_t i=0;i<ctrl.size();i++)
        {
          ctrl[i] = round( ind.data(i) * 1000.0 ) / 1000.0;// limite numerical issues
          if(i==1 || i==2){
            ctrl[i] *= 2*M_PI;
          }
          if (i==0){
            ctrl[i] *= std::sqrt(2);
          }
        }
      auto g_robot = pool::robot_pool.get_robot();  
      g_robot->skeleton()->setPosition(5, 0.15);


      //We want to save the children this individual will select in lower layers
      if(!(this->children_set)){
         _children.clear();
      }

      // Init Sim
      robot_dart::RobotDARTSimu simu(0.01);
      simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
      simu.add_checkerboard_floor(10.);
      simu.add_robot(g_robot);

      auto init_trans = simu.robots().back()->skeleton()->getPositions();
      this->current_vel = simu.robots().back()->skeleton()->getVelocities();
      this->current_acc = simu.robots().back()->skeleton()->getAccelerations();
      Eigen::VectorXd real_state(18);

      real_state << init_trans.head(6),(this->current_vel).head(6),(this->current_acc).head(6);
      this->current_state = real_state;

      std::shared_ptr<robot_dart::control::HexaOmniControl> controller;

      dart::dynamics::BodyNodePtr body_to_check;
      dart::dynamics::SimpleFrame center_body;

      #if defined (GRAPHIC)
        auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(&simu);
        graphics->set_enable(true);
        simu.set_graphics(graphics);
      #endif

      if(render){
        auto graphics = std::make_shared<robot_dart::gui::magnum::WindowlessGraphics>(&simu);
        graphics->set_enable(true);
        graphics->record_video(video_name, 30);
        simu.set_graphics(graphics);
      }
      
      std::vector<double> starting_points;
      starting_points.resize(6);
      double err_x = 0, err_y = 0, err_yaw = 0;
      Eigen::Vector4d err_pos;//used to calculate the error correction we need to do for the closed loop controller

      //Number of SubSteps we want to execute
      int steps = 3;
      this->_effective_steps = 0;
      double global_x = 0,global_y = 0,global_yaw = 0;
      int inter_steps = 0;
      bool do_update = true;


      for(int step=0;step<steps;step++)
      {

          std::vector<double> correct_ctrl;
          double x_velocity,prev_x;
          double y_velocity,prev_y;  
          double yaw_velocity;  

          int index = step%3;//number of different steps we have
          /// global trajectory
          auto init_position = simu.robots().back()->skeleton()->getPositions();
          Eigen::Matrix3d init_rot = dart::math::expMapRot({0, 0, init_position[2]});
          Eigen::MatrixXd init_homogeneous(4, 4);
          init_homogeneous << init_rot(0, 0), init_rot(0, 1), init_rot(0, 2), init_position[3], init_rot(1, 0), init_rot(1, 1), init_rot(1, 2), init_position[4], init_rot(2, 0), init_rot(2, 1), init_rot(2, 2), init_position[5], 0, 0, 0, 1;
        
          // Angle computation
          double _current_angle = std::round(dart::math::matrixToEulerXYZ(init_rot)(2)*100)/100;

          //we only have a 6 step trajectory
          this->_effective_steps++;
          if (step<5){

            global_x += (ctrl[index*3+0]*1)-0.5;
            global_y += (ctrl[index*3+1]*1)-0.5;
            global_yaw += (ctrl[index*3+2]*2)-1;

            if(render || (leg_configuration >= 0) ){
              global_x = (this->desc()[0]*3)-1.5;
              global_y = (this->desc()[1]*3)-1.5;
            }
            
            while ( global_yaw < -M_PI)
               global_yaw += 2 * M_PI;
            while ( global_yaw > M_PI)
               global_yaw -= 2 * M_PI;

            double bd_error  = std::sqrt( std::pow((init_position[3]+1.5)/3-(global_x+1.5)/3,2) + std::pow((init_position[4]+1.5)/3-(global_y+1.5)/3,2));
            inter_steps++;
            }
            
          else{
            if(render || (leg_configuration >= 0) ){
              global_x = (this->desc()[0]*3)-1.5;
              global_y = (this->desc()[1]*3)-1.5;

            }
            else{
              global_x = this->_goal_x_pos;
              global_y = this->_goal_y_pos;
            }
            
            global_yaw = angle_dist(_current_angle,this->_goal_yaw_pos);
            double bd_error  = std::sqrt( std::pow((init_position[3]+1.5)/3-(global_x+1.5)/3,2) + std::pow((init_position[4]+1.5)/3-(global_y+1.5)/3,2));
            this->_mt_error = bd_error;
          }


        //Saving positions we start from
        this->_waypoints.push_back(std::vector<double> {init_position[3],init_position[4]});
        
        Eigen::Vector4d global_pos = {global_x,global_y,0.0,1.0};
        global_pos = init_homogeneous.inverse()*global_pos;

        // Eigen::Map<Eigen::VectorXd> new_ctrl(ptr, correct_ctrl.size()); 
        Eigen::VectorXd new_ctrl;
        boost::shared_ptr<layer1::phen_t> temp_indiv;

        //Getting Control for the high level controller. FF grabs all the necessary controls from the lower levels.
        std::tie(new_ctrl, temp_indiv) = ff(global_pos,global_yaw,leg_configuration,render,use_children,step);
        double ctrl_dt = 0.01;

        controller = std::make_shared<robot_dart::control::HexaOmniControl>(ctrl_dt, new_ctrl);
        g_robot->add_controller(controller);
        std::static_pointer_cast<robot_dart::control::HexaOmniControl>(g_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);

        if(step==0){
          std::static_pointer_cast<robot_dart::control::HexaOmniControl>(g_robot->controllers()[0])->activate(false);
          simu.run(0.2);
          //run the controller to go to the t=0 pos and stabilise for 0.5 second
          simu.world()->setTime(0);
          simu.run(0.1);
          simu.world()->setTime(0);
          simu.run(0.1);
          std::static_pointer_cast<robot_dart::control::HexaOmniControl>(g_robot->controllers()[0])->activate(false);
          simu.run(0.5);
          std::static_pointer_cast<robot_dart::control::HexaOmniControl>(g_robot->controllers()[0])->activate(true);
          simu.add_descriptor(std::make_shared<robot_dart::descriptor::HexaBodyDescriptor>(robot_dart::descriptor::HexaBodyDescriptor(&simu)));
          // simu.add_descriptor(std::make_shared<robot_dart::descriptor::DutyCycle>(robot_dart::descriptor::DutyCycle(&simu)));
          
          simu.add_descriptor(std::make_shared<robot_dart::descriptor::HexaBodyAccOmni>(robot_dart::descriptor::HexaBodyAccOmni(&simu)));
          simu.add_descriptor(std::make_shared<robot_dart::descriptor::Forces>(robot_dart::descriptor::Forces(&simu)));
          // std::static_pointer_cast<robot_dart::descriptor::HexaBodyAccOmni>(simu.descriptor(1))->reset();//needed to instatiate position (maybe not very clean)
        }

        auto init_trans_check = simu.robots().back()->skeleton()->getPositions();
        body_to_check = simu.robots().back()->skeleton()->getBodyNode("base_link"); 
        Eigen::Isometry3d centerTf = (body_to_check)->getWorldTransform();
        center_body = dart::dynamics::SimpleFrame(dart::dynamics::Frame::World(), "center",centerTf);

        simu.world()->setTime(0.0);//Important to reset timer.
        simu.run(1);
        g_robot->remove_controller(controller);
      
        //Measure how much we deviate from wanted BD
        Eigen::Vector3d _final_pos;
        Eigen::Vector3d _final_rot;
        double _arrival_angle;
        double _covered_distance;
        
        auto pose = simu.robots().back()->skeleton()->getPositions();
        Eigen::Matrix3d rot = dart::math::expMapRot({pose[0], pose[1], pose[2]});
        init_rot = dart::math::expMapRot({0, 0, init_trans_check[2]});

        init_homogeneous << init_rot(0, 0), init_rot(0, 1), init_rot(0, 2), init_trans_check[3], init_rot(1, 0), init_rot(1, 1), init_rot(1, 2), init_trans_check[4], init_rot(2, 0), init_rot(2, 1), init_rot(2, 2), init_trans_check[5], 0, 0, 0, 1;
        Eigen::MatrixXd final_homogeneous(4, 4);
        final_homogeneous << rot(0, 0), rot(0, 1), rot(0, 2), pose[3], rot(1, 0), rot(1, 1), rot(1, 2), pose[4], rot(2, 0), rot(2, 1), rot(2, 2), pose[5], 0, 0, 0, 1;
        Eigen::Vector4d tpos = {init_trans_check[3], init_trans_check[4], init_trans_check[5], 1.0};
        Eigen::Vector4d pos = {pose[3], pose[4], pose[5], 1.0};
        

        pos = init_homogeneous.inverse()*pos; // We want our final position to be expressed with respect to the initial frame
        _final_pos = pos.head(3);
        _covered_distance = std::round(_final_pos(0) * 100) / 100.0;
        // Angle computation
        _final_rot = dart::math::matrixToEulerXYZ(init_rot.inverse() * rot);
        // roll-pitch-yaw
        _arrival_angle = std::round(_final_rot(2) * 100) / 100.0;

        Eigen::Vector6d velocity(6);
        velocity = body_to_check->getCOMSpatialVelocity(dart::dynamics::Frame::World(),&(center_body)).head(6).cast <double> (); 
        // std::cout<<((body_to_check)->getCOM(&(center_body))).transpose()<<std::endl;


        double x = _final_pos(0),y= _final_pos(1),yaw = _arrival_angle;

        double obs_x = (x+0.5)/1; //use this for velocity
        double obs_y = (y+0.5)/1;
        double obs_yaw = (yaw+1.0)/2.0;


        std::vector<double> forces; // results is the final position of the robot (used to get the descriptor)
        std::static_pointer_cast<robot_dart::descriptor::Forces>(simu.descriptor(2))->get(forces); //getting descriptor
        this->_all_forces.push_back(forces);

        #ifdef LEG_CONFIG
          Eigen::VectorXd real_obs(9);
          real_obs << obs_x,obs_y,obs_yaw,forces[0],forces[1],forces[2],forces[3],forces[4],forces[5];
        #else
          Eigen::VectorXd real_obs(3);
          real_obs << obs_x,obs_y,obs_yaw; //Since we execute for 1s positions/1s = avg velocity
        #endif
       
        //error for the x and y velocity compared to what we expected in the bd
        this-> _obs_error += std::abs((this->descriptor[0]-obs_x))+std::abs((this->descriptor[1]-obs_y))/5;

        //error in orientation from what we expected in the bd (yaw)
        this-> _orientation_error += std::abs((this->descriptor[2]-obs_yaw))/5;

        this -> _observations.push_back(real_obs);
        std::static_pointer_cast<robot_dart::descriptor::HexaBodyAccOmni>(simu.descriptor(1))->reset();

        this->current_vel = simu.robots().back()->skeleton()->getVelocities();
        this->current_acc = simu.robots().back()->skeleton()->getAccelerations();

        Eigen::VectorXd real_state(18);
        real_state <<pose.head(6), (this->current_vel).head(6),(this->current_acc).head(6);
        this->current_state = real_state;
      }

      //obtain required data, fitness, descriptors
      // for final position of the robot. not need for this as descriptor is already the x and y position already b
      std::vector<Eigen::VectorXf> pos_results; // results is the final position of the robot (used to get the descriptor)
      std::static_pointer_cast<robot_dart::descriptor::HexaBodyDescriptor>(simu.descriptor(0))->get(pos_results); //getting descriptor

      Eigen::Vector3d _final_pos;
      Eigen::Vector3d _final_rot;
      double _arrival_angle;
      double _covered_distance;
      
      auto pose = simu.robots().back()->skeleton()->getPositions();
      // std::cout<<pose.head(6).transpose()<<std::endl;
      Eigen::Matrix3d rot = dart::math::expMapRot({pose[0], pose[1], pose[2]});
      Eigen::Matrix3d init_rot = dart::math::expMapRot({init_trans[0], init_trans[1], init_trans[2]});
      Eigen::MatrixXd init_homogeneous(4, 4);
      init_homogeneous << init_rot(0, 0), init_rot(0, 1), init_rot(0, 2), init_trans[3], init_rot(1, 0), init_rot(1, 1), init_rot(1, 2), init_trans[4], init_rot(2, 0), init_rot(2, 1), init_rot(2, 2), init_trans[5], 0, 0, 0, 1;
      Eigen::MatrixXd final_homogeneous(4, 4);
      final_homogeneous << rot(0, 0), rot(0, 1), rot(0, 2), pose[3], rot(1, 0), rot(1, 1), rot(1, 2), pose[4], rot(2, 0), rot(2, 1), rot(2, 2), pose[5], 0, 0, 0, 1;

      Eigen::Vector4d pos = {pose[3], pose[4], pose[5], 1.0}; //We Want to Transform our final position (in the world frame) into the initial reference frame
      // pos = init_homogeneous.inverse() * final_homogeneous * pos;
      pos = init_homogeneous.inverse()*pos; //Since pos is our final position in the world frame we can transform it with the init_transform_matrix into the initial frame
      

      _final_pos = pos.head(3);
      _covered_distance = std::round(_final_pos(0) * 100) / 100.0;

      // Angle computation
      _final_rot = dart::math::matrixToEulerXYZ(init_rot.inverse() * rot);

      // roll-pitch-yaw
      _arrival_angle = std::round(_final_rot(2) * 100) / 100.0;


     // CRITICAL : free your robot !
      pool::robot_pool.free_robot(g_robot);

      this->children_set = true;
      this->_waypoints.push_back(std::vector<double> {pose[3],pose[4]});

      this->_mt_error = std::sqrt( std::pow((pose[3]+1.5)/3-(global_x+1.5)/3,2) + std::pow((pose[4]+1.5)/3-(global_y+1.5)/3,2));
      this->_goal_x_pos = global_x;
      this->_goal_y_pos = global_y;


      return {pos_results, _arrival_angle};
    }

std::tuple<Eigen::VectorXd,boost::shared_ptr<layer1::phen_t>> ff(Eigen::Vector4d global_pos,double _global_yaw,int leg_configuration,bool render = false,bool use_children = false,int step = 0){
    /*This function takes a position we want to reach ,an angle and leg configurations (which constitutes the BD from the layer below) 
    and select the controller that is getting us closest to the desired position. If use_children is true, we use the controllers and their secondary behaviours that we have found during training*/
    
    double global_x = 0,global_y = 0,global_yaw = _global_yaw;
    int inter_steps = 0;
    bool do_update = true;
    std::vector<double> correct_ctrl;
    double x_velocity,prev_x;
    double y_velocity,prev_y;  
    double yaw_velocity; 
  
  x_velocity = (global_pos[0]+0.5)/1;
  y_velocity = (global_pos[1]+0.5)/1;
  yaw_velocity = (global_yaw+1)/2.0;

  if(render || (leg_configuration >= 0) ){
    x_velocity = (global_pos[0]+0.5)/1;
    y_velocity = (global_pos[1]+0.5)/1;
    yaw_velocity = (global_yaw+1)/2.0;
  }


  // std::cout<<"Velocities "<<x_velocity<<"  "<<y_velocity<<"  "<<yaw_velocity<<std::endl;
  int k = 0;//Initial Leg Configuration
  this->descriptor.clear();

    #ifdef LEG_CONFIG
      this->descriptor ={x_velocity,y_velocity,yaw_velocity,0,0,0,0,0,0}; //Initialization of leg dimensions

      //We define the leg configuration that should be chosen as secondary behaviour
      if (leg_configuration>=0){
        k = leg_configuration;
      }
      auto leg_bits = std::bitset<6>(k);
      for(int i=0;i<leg_bits.size();i++){
        this->descriptor[i+3]= (double) leg_bits[leg_bits.size()-i-1];
      }
    #else
      this->descriptor ={x_velocity,y_velocity,yaw_velocity};
    #endif
    #ifdef GP_USE
      indiv_t i1;
    #else
      indiv_t i1 = indiv_t(new layer1::phen_t());
      i1->random();
      i1->fit().set_desc(this->descriptor);      //we need to use an individual to find the grid TODO: Write Grid class to directly access with BD    
    #endif


    Eigen::VectorXd mu, best_mu;
    double* copy_ptr = &(this->descriptor)[0];
    Eigen::Map<Eigen::VectorXd> descriptor_vector(copy_ptr, this->descriptor.size());

    // descriptor_vector << descriptor;// x_velocity,y_velocity,yaw_velocity; 
    double sigma, best_dist=10000, dist, best_sigma=100.0;
    indiv_t best_indiv, indiv,compare_indiv;

    std::vector<indiv_t> neigh_indiv;
        
    //unstructured archive
    typedef Eigen::Map<const Eigen::VectorXd> point_t;
  
    //trying to use all individuals
    neigh_indiv = layer1::layer_ptr->pop();

    //Check for the best controller given the wanted BD

    std::vector<std::pair<int,double>> all_distances;


    // std::cout<<"Wanted Vect "<<descriptor_vector.transpose()<<std::endl;
    int i = 0;

    if(render || (leg_configuration >= 0) ){
      neigh_indiv = layer1::layer_ptr->pop();
      Eigen::VectorXd desc_vel(9);
      
      for (auto& n : neigh_indiv){
       
        desc_vel[0] = n->fit().desc()[0];
        desc_vel[1] = n->fit().desc()[1];
        desc_vel[2] = n->fit().desc()[2];
        desc_vel[3] = n->fit().desc()[3];
        desc_vel[4] = n->fit().desc()[4];
        desc_vel[5] = n->fit().desc()[5];
        desc_vel[6] = n->fit().desc()[6];
        desc_vel[7] = n->fit().desc()[7];
        desc_vel[8] = n->fit().desc()[8];
        dist = (desc_vel.head(2)-descriptor_vector.head(2)).norm();
        
        double thresh = 0.01;
        if ((desc_vel.tail(6)-descriptor_vector.tail(6)).norm() > thresh){
          continue;
        }        
        //saving all individuals and sorting results
        std::pair<int,double> distance_pair;
        distance_pair = std::make_pair(i,dist);//we need indexing to choose the x nearest neighbour
        all_distances.push_back(distance_pair);
        i++;

        if((dist<best_dist)){
          best_dist = dist;
          best_indiv = n;
          best_sigma = sigma;
          best_mu = mu;

        }
      }
    }
    else{
      for(int config = 0;config<64;config++){
        auto leg_bits = std::bitset<6>(config);
        for(int i=0;i<leg_bits.size();i++){
          this->descriptor[i+3]= (double) leg_bits[i];
        }

        point_t p_i1(this->descriptor.data(), this->descriptor.size());
        auto neigh_nn = layer1::layer_ptr->container().archive().knn(p_i1,2500);
        neigh_indiv.clear();
        for(auto& nn: neigh_nn){
          neigh_indiv.push_back(nn.second);
        }

        for (auto& n : neigh_indiv){
          std::tie(mu, sigma) = n->fit().gp.query(this->current_state);
          dist = (mu.head(3)-descriptor_vector.head(3)).norm();

          if(dist<best_dist){
            best_dist = dist;
            best_indiv = n;
            best_sigma = sigma;
            best_mu = mu;
          }

          Eigen::VectorXd selected_descriptor(9);
          selected_descriptor << descriptor_vector;
        
        }
      }
    }
    this->_gp_uncertainty += best_sigma;///5;
    indiv = best_indiv;
    if(best_dist>0.2){
      this->_dead=true;//forcing small distances
    }
    
    // This fixes the children at the first passage!! Avoiding shift of BD!!!
    if(this->children_set){
      if(use_children){
        if(step < _children.size()){
          indiv = _children[step];
          indiv->fit().fit_gp(true,true);
          // std::cout<<step<<"   "<<indiv->fit().desc()[0]<<" "<<indiv->fit().desc()[1]<<"   "<<indiv->fit().gp.kernel_function().h_params().transpose()<<"\t\t"<<indiv->fit().gp.mean_observation().transpose()<<std::endl;
        }
        else{
          indiv = _children[_children.size()-1];
          // std::cout<<step<<"   "<<indiv->fit().desc()[0]<<" "<<indiv->fit().desc()[1]<<"   "<<indiv->fit().gp.kernel_function().h_params().transpose()<<"\t\t"<<indiv->fit().gp.mean_observation().transpose()<<std::endl;
        }
        indiv->develop();
      }
        
    }
    else{
      _children.push_back(indiv);
    }

    //grabbing low level controllers
    std::vector<float> body_gen = indiv->data();
    double height,end_pos,duty_cycle;
    for(int _leg=0;_leg<6;_leg++){
      duty_cycle = body_gen[_leg*3+2];
      height = body_gen[_leg*3+1];
      end_pos = body_gen[_leg*3+0];


      Eigen::Vector3d descriptor_leg (end_pos, height,duty_cycle);
      auto P1 = layer0::layer_ptr->container().archive().nearest(descriptor_leg);
      auto leg_child = P1.second;

      std::vector<float> gen = leg_child->data();
      std::vector<double> ctrl_l1 (gen.begin(), gen.end()); 
      correct_ctrl.insert(correct_ctrl.end(),ctrl_l1.begin(), ctrl_l1.end());
    }
    
    double* ptr = &correct_ctrl[0];
    Eigen::Map<Eigen::VectorXd> new_ctrl(ptr, correct_ctrl.size()); 
    return std::make_tuple(new_ctrl,indiv);
  }
  
  double angle(){
    return this->_angle;
  }

  int effective_steps(){
    return this->_effective_steps;
  }

  void set_state(Eigen::VectorXd state){
    this->current_state = state;
  }

  std::vector<std::vector<double>> all_forces(){
    return this->_all_forces;
  }

  std::vector<std::vector<double>> obs_descs(){
    return this->_obs_descs;
  }

  std::vector<std::vector<double>> waypoints(){
    return this->_waypoints;
  }

  void add_bd_config(int leg_config,std::vector<double> bd_desc){
    this->_obs_descs[leg_config] = bd_desc;
  }

  double obs_err(){
    return this->_obs_error;
  }
  double orientation_err(){
    return this->_orientation_error;
  }

  double full_err(){
    return (this->_orientation_error+this->_obs_error);
  }

  double duty(){
    return this->_duty;
  }

  double gp_uncertainty(){
    return this->_gp_uncertainty;
  }

  std::vector<Eigen::VectorXd> obs(){
    return this->_observations;
  }
  pop_t children(){
    return this->_children;
  }

  double age(){
    return this->_age;
  }

  double qd_score(){
   return this-> _local_qdscore;
  }
  
  double empowerment(){
    return this->_empowerment;
  }

  void reset_qd_score(){
    this-> _local_qdscore = 0.0;
  }

  void increase_qd_score(double increase){
   this-> _local_qdscore = (this-> _local_qdscore) + increase;
  }

  void increase_age(double inc){
    this->_age=(this->_age)+inc;
  }

  void alive(){
    this->_dead = false;
  }

  double virtual_fit(std::pair<std::vector<Eigen::VectorXf>, double > data){
      double x, y ; 
      x = (data.first.back()(0)) ; // only want the final x and y position only so  .back. (0) is the x coordinate  
      y = (data.first.back()(1)) ; //(1) is the y coordinate 
      
      // Performance - Angle Difference (desrird angle and obtained angle fomr simulation)
      // Change of orientation of axis in counting the desried angle to account for frontal axis of the newer robot (x-axis:frontal axis)
      double ang_y = y ; 
      double ang_x = x ; 
      
      // Computation of desired angle (yaxis-north x-axis(postive))
      double B = std::sqrt((ang_x / 2.0) * (ang_x / 2.0) + (ang_y / 2.0) * (ang_y / 2.0));
      double alpha = std::atan2(ang_y, ang_x);
      double A = B / std::cos(alpha);
      double beta = std::atan2(ang_y, ang_x - A);

      if (x < 0)
          beta = beta - M_PI;
      while (beta < -M_PI)
          beta += 2 * M_PI;
      while (beta > M_PI)
          beta -= 2 * M_PI;
              
      double angle_diff = std::abs(angle_dist(beta, this->_angle));
      return -angle_diff;

  }

  std::vector<double> virtual_desc(std::pair<std::vector<Eigen::VectorXf>, double > data){
    double x, y ; 
    x = (data.first.back()(0)) ; // only want the final x and y position only so  .back. (0) is the x coordinate  
    y = (data.first.back()(1)) ; //(1) is the y coordinate 
    
    std::vector<double> desc;
    double desc_x, desc_y ; 
    desc_x = (x+1.5)/3 ; //scaling the BD
    desc_y = (y+1.5)/3 ; //scaling the BD
    desc.push_back(desc_x);
    desc.push_back(desc_y);
    return desc;
  }

  void update_gp_children(bool h_opt=false){
    for(auto child:this->_children){
      if(h_opt){
        child->fit().fit_gp(false);
        std::cout<<"NB Samples "<<child->fit().gp.nb_samples()<<std::endl;
        child->fit().gp.optimize_hyperparams();
        continue;
      }
      child->fit().fit_gp(true);
      
    }

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
      ar& BOOST_SERIALIZATION_NVP(this->_traj);
      ar& BOOST_SERIALIZATION_NVP(this->_children);
      ar& BOOST_SERIALIZATION_NVP(this->children_set);
      ar& BOOST_SERIALIZATION_NVP(this->_observations);
      ar& BOOST_SERIALIZATION_NVP(this->_all_forces);
      ar& BOOST_SERIALIZATION_NVP(this->_obs_descs);
      ar& BOOST_SERIALIZATION_NVP(this->_age);
      ar& BOOST_SERIALIZATION_NVP(this->_local_qdscore);
      ar& BOOST_SERIALIZATION_NVP(this->_empowerment);
      ar& BOOST_SERIALIZATION_NVP(this->_mt_error);
    }
    
  private:
    Eigen::VectorXd _ctrl;
    std::vector<std::vector<double>> _all_forces, _obs_descs,_waypoints;
    std::vector<Eigen::VectorXf> _traj;
    std::vector<Eigen::VectorXd> _observations;
    double _angle=0;
    double _duty=0;
    double _obs_error=0;
    double _orientation_error=0;
    Eigen::VectorXd current_vel,current_acc,current_state;
    pop_t _children;
    double _gp_uncertainty = 0.0;
    bool children_set=false;
    double _age = 0.0;
    double _local_qdscore = 0.0;
    int _effective_steps = 0;
    double _vert_force = 0.0, _fric_coeff = 1.0;
    double _goal_x_pos = 0.0, _goal_y_pos = 0.0, _goal_yaw_pos = 0.0;
    double _empowerment = 0.0;
    double _mt_error = 0.0;
    std::vector<double> descriptor;

    
  };
using namespace sferes::gen::evo_float;
  struct Params {
      struct nov {
          SFERES_CONST size_t deep = 3;
          static double l; // TODO value ???
          SFERES_CONST double k = 15; // TODO right value?
          SFERES_CONST double eps = 0.1;// TODO right value??
      };
    
      // TODO: move to a qd::
      struct pop {
          // number of initial random points
          SFERES_CONST size_t init_size = 1000;
          // size of a batch
          SFERES_CONST size_t size = 200;
          // SFERES_CONST size_t nb_gen = 20001;
           SFERES_CONST size_t nb_gen = 101;
          // SFERES_CONST size_t dump_period = 5000;
          SFERES_CONST size_t dump_period = 100;
      };
      struct parameters {
        SFERES_CONST float min = 0.0;
        SFERES_CONST float max = 1.0;
      };
      struct evo_float {
          SFERES_CONST float cross_rate = 0.0f;
          SFERES_CONST float mutation_rate = 0.14f;
          SFERES_CONST float eta_m = 10.0f;
          SFERES_CONST float eta_c = 10.0f;
          SFERES_CONST mutation_t mutation_type = polynomial;
          SFERES_CONST cross_over_t cross_over_type = sbx;
      };
      struct qd {
          SFERES_CONST size_t behav_dim = 2;
          SFERES_ARRAY(size_t, grid_shape,100, 100);

      };

      struct hbr{
        SFERES_CONST size_t prev_behav_dim = 9;//Defining what the bd dimensions of the lower level are
        SFERES_CONST size_t num_indiv = 10; 
      };
  };
  double Params::nov::l = 0.01;

  // using namespace sferes;
  typedef Fit_hexa_omni<Params> fit_t;
  typedef sferes::gen::EvoFloat<9, Params> gen_t;
  typedef sferes::phen::Parameters_HBR<gen_t, fit_t, Params> phen_t;

  typedef sferes::qd::selector::Uniform<phen_t, Params> select_t;
  typedef sferes::qd::container::Grid<phen_t, Params> container_t;

  #if defined (GRAPHIC)
    typedef sferes::eval::Eval<Params> eval_t;
  #else
    typedef sferes::eval::Parallel<Params> eval_t;
  #endif


  typedef boost::fusion::vector<
      sferes::stat::BestFit<phen_t, Params>, 
      sferes::stat::QdContainerAngle<phen_t, Params>, 
      sferes::stat::QdProgressErr<phen_t, Params>, 
      sferes::stat::QdPrintMap<phen_t, Params>,
      sferes::stat::QdPrintMapUnc<phen_t, Params>,
      sferes::stat::QdMT<phen_t, Params>,
      sferes::stat::QdMapMTGrid<phen_t, Params>,
      sferes::stat::State<phen_t, Params>,
      sferes::stat::DeleteGenFiles<phen_t, Params>
      >
      stat_t; 


  
  typedef boost::fusion::vector<
          sferes::modif::Age<Params>,
          sferes::modif::GP_Update<phen_t,Params>
          >
          modifier_t;  

  typedef sferes::qd::QDLayer<phen_t, eval_t, stat_t, modifier_t, select_t, container_t, Params> qd_t;

  qd_t* layer_ptr;

}//layer2



#endif
