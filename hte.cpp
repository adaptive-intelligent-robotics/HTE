#include <csignal>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <assert.h>
#include <stdio.h>   
#include <math.h>
#include <chrono>

#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/control/pd_control.hpp>
#include <hexa_control.hpp>

#include <limbo/limbo.hpp>
#include <map_elites/binary_map.hpp> // to load in archive_file (behavioural repertoire) from MAPELITES

#include <mcts/uct.hpp>
#include <svg/simple_svg.hpp>
#include <boost/program_options.hpp>
#include <astar/a_star.hpp>
#include <algorithm>

// #ifdef GRAPHIC
// #include <robot_dart/graphics/graphics.hpp>
#include <robot_dart/gui/magnum/graphics.hpp>
#include <robot_dart/gui/magnum/windowless_graphics.hpp>
// #endif

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include"hbr_loading.hpp"
#include <hbr/Leg/fit_hexa_leg.hpp>
#include <hbr/Body_Acc/fit_hexa_body_gp.hpp>
#include <hbr/Omnidirectional/fit_mt_me.hpp>


#include <hbr/HBR/quality_diversity_hbr.hpp>
#include <valgrind/callgrind.h>

#ifdef HBR
    #define ARCHIVE_SIZE 2
#else
    #ifdef LOW_DIM
        #define ARCHIVE_SIZE 2
    #else
        #define ARCHIVE_SIZE 8
    #endif
#endif


using namespace limbo ; // for limbo objects and functions dont have to keep typing limbo::

template <typename T> // gaussaion_used in HexaState.move - 
inline T gaussian_rand(T m = 0.0, T v = 1.0)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());

    std::normal_distribution<T> gaussian(m, v);

    return gaussian(gen);
}// gaussian_rand

// b-a - function that obtains angle difference between the arrival_angle and the desired angle, used to compute the performance/fitness - used in HexaState
double angle_dist(double a, double b)
{
    double theta = b - a;
    while (theta < -M_PI)
        theta += 2 * M_PI;
    while (theta > M_PI)
        theta -= 2 * M_PI;
    return theta;
}// angle_dist 


struct Params {
    struct uct {
        MCTS_DYN_PARAM(double, c);
    };

    struct spw {
        MCTS_DYN_PARAM(double, a);
    };

    struct cont_outcome {
        MCTS_DYN_PARAM(double, b);
    };

    struct mcts_node {
        MCTS_DYN_PARAM(size_t, parallel_roots);
    };

    struct active_learning {
        MCTS_DYN_PARAM(double, k);
        MCTS_DYN_PARAM(double, scaling);
    };

    MCTS_DYN_PARAM(double, goal_x);
    MCTS_DYN_PARAM(double, goal_y);
    MCTS_DYN_PARAM(double, goal_theta);
    MCTS_DYN_PARAM(double, iterations);
    MCTS_DYN_PARAM(bool, learning);
    MCTS_DYN_PARAM(size_t, collisions);
    MCTS_PARAM(double, threshold, 1e-2);
    MCTS_PARAM(double, cell_size, 0.5);
    MCTS_PARAM(double, robot_radius, 0.25); //hexpod radius maximum value


    struct kernel_exp {
      BO_PARAM(double, l,0.03);
      BO_PARAM(double, sigma_sq, 0.01);
    }; //kernel_exp

    struct kernel : public defaults::kernel {
                  BO_PARAM(double, noise, 0.001);
    }; // kernel - from new version of limbo

    struct archiveparams { // used to extract and store the data from the archive file from MAPELITES 
        struct elem_archive {
	  double x, y, cos_theta, sin_theta; //extracts and store behaviroual descriptor and performance 
	  std::vector<double> controller; //to store the controller (motor parameters corresponding to the behaviorual descriptor
        }; //elem_archive

        struct classcomp {
            bool operator()(const std::vector<double>& lhs, const std::vector<double>& rhs) const
            {
	      
                assert(lhs.size() == ARCHIVE_SIZE && rhs.size() == ARCHIVE_SIZE);
                int i = 0;
                while (i < (ARCHIVE_SIZE - 1) && std::round(lhs[i] * 1000) == std::round(rhs[i] * 1000)) //lhs[i]==rhs[i])
                    i++;
                return std::round(lhs[i] * 1000) < std::round(rhs[i] * 1000); //lhs[i]<rhs[i];
            }
        }; //classcomp

        struct classcompequal {
            bool operator()(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) const
            {
                assert(lhs.size() == ARCHIVE_SIZE && rhs.size() == ARCHIVE_SIZE);
                int i = 0;
                while (i < (ARCHIVE_SIZE - 1) && std::round(lhs[i] * 1000) == std::round(rhs[i] * 1000)) //lhs[i]==rhs[i])
                    i++;
                return std::round(lhs[i] * 1000) == std::round(rhs[i] * 1000); //lhs[i]<rhs[i];
            }
        }; //classcompequal

      using archive_t = std::map<std::vector<double>, elem_archive, classcomp>;
      static archive_t archive;
    }; //archive_params
}; //Params


template <typename Params>
struct MeanArchive {
    MeanArchive(size_t dim_out = 4)
    {
    }
    template <typename GP>
    Eigen::VectorXd operator()(const Eigen::VectorXd& v, const GP&) const
    {
        Eigen::VectorXd r(4);
        std::vector<double> vv(v.size(), 0.0);
        Eigen::VectorXd::Map(&vv[0], v.size()) = v;
        typename Params::archiveparams::elem_archive elem = Params::archiveparams::archive[vv];
        r << elem.x, elem.y, elem.cos_theta, elem.sin_theta;
        return r;
    }
}; // MeanArchive


struct SimpleObstacle { // struct for generating obtaining the position of the obstacles from the map
    double _x, _y, _radius, _radius_sq;

    SimpleObstacle(double x, double y, double r = 0.1) : _x(x), _y(y), _radius(r), _radius_sq(r * r) {}
}; //SimpleObstacle

struct BoxObstacle { // struct for generating obtaining the position of the obstacles from the map
    double start_x, start_y, end_x, end_y, _height;
    Eigen::Vector3d size(){
        Eigen::Vector3d final_size;
        final_size << end_x-start_x,end_y-start_y,_height;
        return final_size;
    }
    Eigen::Vector6d pose(){
        Eigen::Vector6d final_pose;
        //center of box is used to place the box!
        final_pose << 0.0,0.0,0.0,(end_x+start_x)/2,(end_y+start_y)/2,0;
        return final_pose;
    }

    double width(){
        if(start_x<end_x)
            return (end_x-start_x);
        else
            return (start_x-end_x);
    }

    double height(){
        if(start_y<end_y)
            return (end_y-start_y);
        else
            return (start_y-end_y);
    }

    double x(){
        if(start_x<end_x){
            return start_x;
        }
        return end_x;
    }

    double y(){
        if(start_y<end_y){
            return start_y;
        }
        return end_y;
    }

    double c_x(){
        return (end_x+start_x)/2;
    }

    double c_y(){
        return (end_y+start_y)/2;
    }



    BoxObstacle(double x1, double y1, double x2, double y2,double height=0.9) : start_x(x1), start_y(y1), end_x(x2), end_y(y2), _height(height) {}
}; //BoxObstacle


using kernel_t = kernel::Exp<Params>; // kernel is under namespace limbo in limbo.hpp - limbo::kernel::Exp
using mean_t = MeanArchive<Params>; // MeanArchive is written above 
using GP_t = model::GP<Params, kernel_t, mean_t>; //model is under namespace limbo in limbo.hpp - limbo::model::GP 

struct Low_Params{
    struct kernel_exp {
      BO_PARAM(double, l, 0.1);
      BO_PARAM(double, sigma_sq, 0.02);
    }; //kernel_exp

    struct kernel : public defaults::kernel {
                  BO_PARAM(double, noise, 0.001);
    }; // kernel - from new version of limbo
    struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {
    };
    struct opt_rprop : public defaults::opt_rprop {
    };
    struct kernel_maternfivehalves : public defaults::kernel_maternfivehalves {
          BO_PARAM(double, noise, 0.02);
          BO_PARAM(double, sigma_sq, 0.05);
          BO_PARAM(double, l, 1.0);
      };
};


//Params for the Trial and Error Process between Layers
using kernel_low_t = kernel::MaternFiveHalves<Low_Params>;// kernel is under namespace limbo in limbo.hpp - limbo::kernel::Exp
using mean_low_t = mean::Data<Low_Params>;
using GP_low_t = model::GP<Low_Params, kernel_low_t, mean_low_t>; //model is under namespace limbo in limbo.hpp - limbo::model::GP 

bool collides(double x, double y, double r = Params::robot_radius());

namespace global {

    //Gaussian Process Definition for Planning with RTE
    GP_t gp_model(ARCHIVE_SIZE, 4);

    //Gaussian Process Definition for the Trial and Error Process between Layers
    GP_low_t gp_low_model(9, 3);

    std::shared_ptr<robot_dart::Robot> global_robot, simulated_robot; // simulated_robot is the clone of global_robot used for simulation
    Eigen::Vector3d robot_pose;
    std::vector<SimpleObstacle> obstacles;
    std::vector<BoxObstacle> box_obstacles;
    size_t height, width, entity_size, map_size, map_size_x, map_size_y;
    svg::Dimensions dimensions;
    std::shared_ptr<svg::Document> doc;
    size_t target_num;
    std::vector<std::string> dofs;

    robot_dart::RobotDARTSimu simu(0.001);

    // statistics
    std::ofstream robot_file, ctrl_file, iter_file, misc_file,dmg_file;
    double time_random_search = 0.0;
    int n_policies_called = 0,n_random_policies_called=0;
    int damaged_leg;
    int rep_nr=0;
} //namespace global 

// Stat GP - to save/write the gp.dat - used in reach_targets function
void write_gp(std::string filename)
{
    std::ofstream ofs;
    ofs.open(filename);
    typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;
    for (archive_it_t it = Params::archiveparams::archive.begin(); it != Params::archiveparams::archive.end(); it++) {
        Eigen::VectorXd desc = Eigen::VectorXd::Map(it->first.data(), it->first.size());
        Eigen::VectorXd mu;
        double sigma;
        std::tie(mu, sigma) = global::gp_model.query(desc);
        ofs << desc.transpose() << " "
            << mu.transpose() << " "
            << sigma << std::endl;
    }
    ofs.close();
} //write-gp


std::shared_ptr<robot_dart::Robot> box(size_t num, double x_pos, double y_pos, double x_dim, double y_dim, double height)
{
    // random pose
    Eigen::Vector6d pose = Eigen::Vector6d::Random();
    // make sure it is above the ground
    pose(0) = 0 ;
    pose(1) = 0 ;
    pose(2) = 0 ;
    pose(3) = x_pos ;
    pose(4) = y_pos ;
    pose(5) = 0 ;

    // random size
    Eigen::Vector3d size = Eigen::Vector3d::Random().array() * Eigen::Vector3d(0.1, 0.2, 0.1).array() + 0.3;
    size(0) = x_dim ;
    size(1) = y_dim ;
    size(2) = height ;

    //red coloured box - create_box function found in robot_dart robot.cpp file
    //return robot_dart::Robot::create_box(size, pose, "free", 1., dart::Color::Red(1.0), "box_" + std::to_string(num)); //"free" - the block can be pushed around and can fly
    return robot_dart::Robot::create_box(size, pose, "fixed", 1., dart::Color::Red(1.0), "box_" + std::to_string(num)); //"fixed" - fixed at its position. not free to move.
} //box


std::shared_ptr<robot_dart::Robot> sphere(size_t num = 0)
{
    // random pose
    Eigen::Vector6d pose = Eigen::Vector6d::Random();
    // make sure it is above the ground
    pose(5) = 1.0;
    // random size
    Eigen::Vector3d size = Eigen::Vector3d::Random()[0] * Eigen::Vector3d(0.2, 0.2, 0.2).array() + 0.3;

    // blue colored sphere - create_ellipsoid function found in robot_dart robot.cpp file
    return robot_dart::Robot::create_ellipsoid(size, pose, "free", 1., dart::Color::Blue(1.0), "sphere_" + std::to_string(num));
} // sphere

//collision version for boxes
bool collides(double x, double y,double r)
{
    double distance_x = 0,distance_y=0, cDist_sq = 0;
    for (size_t i = 0; i < global::box_obstacles.size(); i++) {
        
        distance_x = abs(x - (global::box_obstacles[i].c_x()));
        distance_y = abs(y - (global::box_obstacles[i].c_y()));

        if (distance_x > (global::box_obstacles[i].width()/2 + r)) { continue; }
        if (distance_y > (global::box_obstacles[i].height()/2 + r)) { continue; }
        if (distance_x <= (global::box_obstacles[i].width()/2)) { return true; } 
        if (distance_y <= (global::box_obstacles[i].height()/2)) { return true; }
        cDist_sq = std::pow((distance_x - (global::box_obstacles[i].width()/2)),2) + std::pow((distance_y - (global::box_obstacles[i].height()/2)),2);
        if(cDist_sq <= (r*r)){
            return true;
        }
    }
    return false;
} // collides

bool intersect_segment(double x1, double y1,double x2, double y2, double x3, double y3,double x4, double y4){
    double t = (((x1-x3)*(y3-y4))-((y1-y3)*(x3-x4)))/(((x1-x2)*(y3-y4))-((y1-y2)*(x3-x4)));
    double u = (((x1-x3)*(y1-y2))-((y1-y3)*(x1-x2)))/(((x1-x2)*(y3-y4))-((y1-y2)*(x3-x4)));
    if(u<1 &&  u>0 && t<1 && t>0){
        return true;
    }
    return false;
}

//collision version for boxes
bool collides_segment(const Eigen::Vector2d& start, const Eigen::Vector2d& end)
{
    for (size_t i = 0; i < global::box_obstacles.size(); i++) {

        bool l1  = intersect_segment(global::box_obstacles[i].start_x,global::box_obstacles[i].start_y,global::box_obstacles[i].start_x,global::box_obstacles[i].end_y, start(0),start(1), end(0),end(1));
        bool l2  = intersect_segment(global::box_obstacles[i].start_x,global::box_obstacles[i].start_y,global::box_obstacles[i].end_x,global::box_obstacles[i].start_y,  start(0),start(1), end(0),end(1));
        bool l3  = intersect_segment(global::box_obstacles[i].end_x,global::box_obstacles[i].end_y,global::box_obstacles[i].end_x,global::box_obstacles[i].start_y,  start(0),start(1), end(0),end(1));
        bool l4  = intersect_segment(global::box_obstacles[i].end_x,global::box_obstacles[i].end_y,global::box_obstacles[i].start_x,global::box_obstacles[i].end_y, start(0),start(1), end(0),end(1));
        
        if (l1||l2||l3||l4){
            return true;
        }
    }
    return false;
} // collides_box_segment

//collision version for boxes
bool collides(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double r = Params::robot_radius())
{
    Eigen::Vector2d dir = end - start;
    Eigen::Vector2d perp = Eigen::Vector2d(-dir(1), dir(0));
    Eigen::Vector2d A = start + perp * r;
    Eigen::Vector2d B = end + perp * r;
    Eigen::Vector2d C = end - perp * r;
    Eigen::Vector2d D = start - perp * r;
    return (collides(start(0), start(1),r) || collides(end(0), end(1),r) || collides_segment(A, B) || collides_segment(B, C) || collides_segment(C, D) || collides_segment(D, A));
} //collides

bool astar_collides(int x, int y, int x_new, int y_new) // used in Default Policy 
{
    Eigen::Vector2d s(x * Params::cell_size(), y * Params::cell_size());
    Eigen::Vector2d t(x_new * Params::cell_size(), y_new * Params::cell_size());
    return collides(s, t, Params::robot_radius() * 1.3);//We Multiply the Robot Radius with a Scaling Factor be on the safe side for collisions in A-Star
} //astar_collides 

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
      
      if(pitch_angle > M_PI_4 && pitch_angle < 3*M_PI_4){
        if(roll_angle > -M_PI_4 && roll_angle < M_PI_4){
          return true;
        }
        else
          return false;
      }
      else
        return false;
    }


template <typename State, typename Action>
struct DefaultPolicy { 
    Action operator()(const State* state, bool draw = false)
    {
        size_t N = 100;
        double dx = state->_x - Params::goal_x();
        double dy = state->_y - Params::goal_y();
        double d = dx * dx + dy * dy;
        if (d <= Params::cell_size() * Params::cell_size()) {
            Action best_action;
            double best_value = std::numeric_limits<double>::max();
            for (size_t i = 0; i < N; i++) {
                Action act = state->random_action();
                auto final = state->move(act, true);
                double dx = final._x - Params::goal_x();
                double dy = final._y - Params::goal_y();
                double val = dx * dx + dy * dy;
                if (collides(final._x, final._y))
                    val = std::numeric_limits<double>::max();
                if (best_action._desc.size() == 0 || val < best_value) {
		    best_value = val;
                    best_action = act;
                }
            }
            return best_action;
        }
        astar::AStar<> a_star;
        astar::Node ss(std::round(state->_x / Params::cell_size()), std::round(state->_y / Params::cell_size()), global::map_size_x, global::map_size_y);
        if (collides(ss._x * Params::cell_size(), ss._y * Params::cell_size()) || ss._x <= 0 || ss._x >= int(global::map_size_x) || ss._y <= 0 || ss._y >= int(global::map_size_y)) {
            astar::Node best_root = ss;
            bool changed = false;
            double val = std::numeric_limits<double>::max();
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    if (i == 0 && j == 0)
                        continue;
                    int x_new = ss._x + i;
                    int y_new = ss._y + j;
                    double dx = x_new * Params::cell_size() - state->_x;
                    double dy = y_new * Params::cell_size() - state->_y;
                    double v = dx * dx + dy * dy;
                    if (!collides(x_new * Params::cell_size(), y_new * Params::cell_size()) && x_new > 0 && x_new < int(global::map_size_x) && y_new > 0 && y_new < int(global::map_size_y) && v < val) {
                        best_root._x = x_new;
                        best_root._y = y_new;
                        val = v;
                        changed = true;
                    }
                }
            }

            if (!changed)
                state->random_action();

            ss = best_root;
        }
        astar::Node ee(std::round(Params::goal_x() / Params::cell_size()), std::round(Params::goal_y() / Params::cell_size()), global::map_size_x, global::map_size_y);
        auto path = a_star.search(ss, ee, astar_collides, global::map_size_x, global::map_size_y);
        if (path.size() < 2) {
	        // std::cout << "Error: A* path size less than 2: " << path.size() << ". Returning random action!" << std::endl;
            return state->random_action();
        }

        astar::Node best = path[1];

        // Find best action
        Eigen::Vector2d best_pos(best._x * Params::cell_size(), best._y * Params::cell_size());
        Eigen::Vector2d init_vec = Eigen::Vector2d(state->_x, state->_y);
        // First check if we have clean path
        if (path.size() >= 3 && (best_pos - init_vec).norm() < Params::cell_size()) {
            Eigen::Vector2d new_pos(path[2]._x * Params::cell_size(), path[2]._y * Params::cell_size());
            if (!collides(init_vec, new_pos, 1.5 * Params::robot_radius())) {
                best = path[2];
                best_pos = new_pos;
            }
        }

        // We are going to just use the best from sampled actions
        // Put best position better in space (to avoid collisions) if we are not in a safer region
        if (collides(best_pos(0), best_pos(1), 2.0 * Params::robot_radius())) {
            SimpleObstacle closest_obs(0, 0, 0);
            double closet_dist = std::numeric_limits<double>::max();
            for (auto obs : global::obstacles) {
                double dx = best_pos(0) - obs._x;
                double dy = best_pos(1) - obs._y;
                double v = dx * dx + dy * dy;
                if (v < closet_dist) {
                    closet_dist = v;
                    closest_obs = obs;
                }
            }
            Eigen::Vector2d dir_to_obs = best_pos - Eigen::Vector2d(closest_obs._x, closest_obs._y);
            Eigen::Vector2d new_best = best_pos;
            double step = Params::cell_size();
            size_t n = 0;
            bool b = false;
            do {
                new_best = best_pos.array() + (n + 1) * step * dir_to_obs.array();
                n++;
                b = collides(new_best(0), new_best(1), 2.0 * Params::robot_radius());
            } while (b && n < 10);

            if (n < 10)
                best_pos = new_best;
        }

        Action best_action;
        double best_value = std::numeric_limits<double>::max();
        std::vector<double> goal_action;
        N = 100;
        for (size_t i = 0; i < N; i++) {
            goal_action = {best_pos(0),best_pos(1)};

            //If we want to find the closest next action (deterministic) then this is similar to A-Star without MCTS
            // Action act = state->closest_action(goal_action);
            Action act = state->random_action();
            auto final = state->move(act, true);
            double dx = final._x - best_pos(0);
            double dy = final._y - best_pos(1);
            double val = dx * dx + dy * dy;
            if (collides(final._x, final._y))
	      {
                val = std::numeric_limits<double>::max();
	      }
            if (best_action._desc.size()==0 || val < best_value) {
                best_value = val;
                best_action = act;
            }
        }
        return best_action;
    }
};//Default Policy 

template <typename Params>
struct HexaAction {
    Eigen::VectorXd _desc;

    HexaAction() {}
    HexaAction(Eigen::VectorXd desc) : _desc(desc) {}

    bool operator==(const HexaAction& other) const
    {
        return (typename Params::archiveparams::classcompequal()(_desc, other._desc));
    }
};

template <typename Params>
struct HexaState {
    double _x, _y, _theta;
    static constexpr double _epsilon = 1e-3;

    HexaState()
    {
        _x = _y = _theta = 0;
    }

    HexaState(double x, double y, double theta)
    {
        _x = x;
        _y = y;
        _theta = theta;
    }

    bool valid(const HexaAction<Params>& act) const
    {
      if(act._desc.size()==0)
	{
	  std::cout<<"INVALID DESC"<<std::endl;
	  return false;
	}
        return true;
    }

    HexaAction<Params> next_action() const
    {
        global::n_policies_called +=1;
        return DefaultPolicy<HexaState<Params>, HexaAction<Params>>()(this);
    }

    //Function returns the action that is closest to the goal direction, useful if we don't want to randomly sample solutions for MCTS
    HexaAction<Params>  closest_action(std::vector<double> goal) const
    {
        typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;
        HexaAction<Params> act,best_action;
        double best_value = std::numeric_limits<double>::max();
        Eigen::VectorXd desc;

        for (archive_it_t possible_action =Params::archiveparams::archive.begin() ; possible_action != Params::archiveparams::archive.end(); possible_action++){
            desc = Eigen::VectorXd::Map(possible_action->first.data(), possible_action->first.size());
            act._desc = desc;
            
            auto final = this->move(act,true);;
            double dx = final._x - goal[0];
            double dy = final._y - goal[1];
            double val = dx * dx + dy * dy;
            if (collides(final._x, final._y))
	      {
                val = std::numeric_limits<double>::max();
	      }
            if (best_action._desc.size()==0 || val < best_value) {
                best_value = val;
                best_action = act;
            }
        }
        return best_action;
    }

    HexaAction<Params> random_action() const
    {
        global::n_random_policies_called +=1;

        static tools::rgen_int_t rgen(0, Params::archiveparams::archive.size() - 1);
        typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;
        
        HexaAction<Params> act;
        double time_running = 0.0;
        do {
            auto t1 = std::chrono::steady_clock::now();
            archive_it_t it = Params::archiveparams::archive.begin();
            std::advance(it, rgen.rand());
            time_running = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t1).count();
            Eigen::VectorXd desc = Eigen::VectorXd::Map(it->first.data(), it->first.size());
            act._desc = desc;

        } while (!valid(act));
        global::time_random_search += time_running;
        return act;
    }

    HexaState move(const HexaAction<Params>& action, bool no_noise = false) const
    {
        double x_new, y_new, theta_new;
        Eigen::VectorXd mu;
        double sigma;
        std::tie(mu, sigma) = global::gp_model.query(action._desc);
	
	if (!no_noise) {
            mu(0) = std::max(-1.5, std::min(1.5, gaussian_rand(mu(0), sigma)));
            mu(1) = std::max(-1.5, std::min(1.5, gaussian_rand(mu(1), sigma)));
            mu(2) = std::max(-1.0, std::min(1.0, gaussian_rand(mu(2), sigma)));
            mu(3) = std::max(-1.0, std::min(1.0, gaussian_rand(mu(3), sigma)));
        }
        double theta = std::atan2(mu(3), mu(2));
        double c = std::cos(_theta), s = std::sin(_theta);
        Eigen::MatrixXd tr(3, 3);
        tr << c, -s, _x, s, c, _y, 0, 0, 1;
        Eigen::Vector3d tr_pos;
        tr_pos << mu(0), mu(1), 1.0;
        tr_pos = tr * tr_pos;
        x_new = tr_pos(0);
        y_new = tr_pos(1);
        theta_new = _theta + theta;

        while (theta_new < -M_PI)
            theta_new += 2 * M_PI;
        while (theta_new > M_PI)
            theta_new -= 2 * M_PI;
        return HexaState(x_new, y_new, theta_new);
    }

    bool terminal() const
    {
        // Check if state is outside of bounds
        if (_x < 0.0 || _x >= global::map_size_x * Params::cell_size() || _y < 0.0 || _y >= global::map_size_y * Params::cell_size())
	  {
	    //std::cout<<"--A--"<<std::endl;
            return true;
	  }
        // Check if state is goal
        if (goal())
	  {
	    //std::cout<<"--B--"<<std::endl;
            return true;
	  }
        // Check if state is colliding
        if (collides(_x, _y))
	  {
	    //std::cout<<"--C--"<<std::endl;
            return true;
	  }
        return false;
    }

    bool goal() const
    {
        double dx = _x - Params::goal_x();
        double dy = _y - Params::goal_y();
        double threshold_xy = Params::cell_size() * Params::cell_size() / 4.0;
        if ((dx * dx + dy * dy) < threshold_xy)
            return true;
        return false;
    }

    bool operator==(const HexaState& other) const
    {
        double dx = _x - other._x;
        double dy = _y - other._y;
        double dth = std::abs(angle_dist(other._theta, _theta));
        return ((dx * dx + dy * dy) < (Params::cell_size() * Params::cell_size() / 4.0) && dth < 0.3);
    }
}; //HexaState

struct RewardFunction {
    template <typename State>
    double operator()(std::shared_ptr<State> from_state, HexaAction<Params> action, std::shared_ptr<State> to_state)
    {
        // Check if state is outside of bounds
        if (to_state->_x < 0.0 || to_state->_x >= global::map_size_x * Params::cell_size() || to_state->_y < 0.0 || to_state->_y >= global::map_size_y * Params::cell_size())
            return -1000.0;

        // Return values
        if (collides(to_state->_x, to_state->_y) || collides(Eigen::Vector2d(from_state->_x, from_state->_y), Eigen::Vector2d(to_state->_x, to_state->_y)))
            return -1000.0;
        if (to_state->goal())
            return 100.0;
        return 0.0;
    }
}; // RewardFunction


std::map<std::vector<double>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp> load_archive(std::string archive_name)
{

    std::map<std::vector<double>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp> archive;

    size_t lastindex = archive_name.find_last_of(".");
    std::string extension = archive_name.substr(lastindex + 1);

    std::cout << "Loading text file..." << std::endl;
    std::ifstream monFlux(archive_name.c_str());
    if (monFlux) {
        std::string line;
        while (std::getline(monFlux, line)) {
            std::istringstream iss(line);
            std::vector<double> numbers;
            double num;
            while (iss >> num) {
                numbers.push_back(num);
            }
            
            int init_i = 0;
            if (numbers.size() > 39)
                init_i = 1;

            Params::archiveparams::elem_archive elem;
            std::vector<double> candidate(ARCHIVE_SIZE);
            int size_data = 45;//36+8+1
            #ifdef HBR
            {
                size_data = 7;
                init_i = 1;
            }
            #endif
            for (int i = 0; i < size_data; i++) {
                if(ARCHIVE_SIZE == 8){
                    double data = numbers[init_i + i];
                    if (i == 0) {
                        candidate[i] = data;
                        elem.x = (data*3)-1.5 ;// The BD score is normalized so we need to put it back into Xreal x,y coordinates
                    }
                    if (i == 1) {
                            candidate[i] = data;
                            elem.y = (data*3)-1.5  ;// The BD score is normalized so we need to put it back into Xreal x,y coordinates
                        }
                    if (i<8){
                        candidate[i] = data;
                    }
                    if (i == 8) {
                        elem.cos_theta = std::cos(data) ;
                        elem.sin_theta = std::sin(data) ;  
                        }
                    if (i >= 9){
                        elem.controller.push_back(data);
                    }
                    if (elem.controller.size() == 36) {
                        archive[candidate] = elem;
                        }
                }
                else{
                    double data = numbers[init_i + i];
                    if (i == 0) {
                        #ifndef HBR
                            candidate[i] = data;
                            elem.x = (data*3)-1.5; // The BD score is normalized so we need to put it back into Xreal x,y coordinates
                        #else
                            candidate[i] = data;
                            elem.x = (data*3)-1.5;// The BD score is normalized so we need to put it back into Xreal x,y coordinates
                        #endif
                    }
                    if (i == 1) {
                            #ifndef HBR
                                candidate[i] = data;
                                elem.y = (data*3)-1.5;// The BD score is normalized so we need to put it back into Xreal x,y coordinates
                            #else
                                candidate[i] = data;
                                elem.y = (data*3)-1.5;// The BD score is normalized so we need to put it back into Xreal x,y coordinates
                            #endif
                        }
                    if (i == 2) {
                        elem.cos_theta = std::cos(data) ;
                        elem.sin_theta = std::sin(data) ;  
                        }
                    if (i >= 3){
                        elem.controller.push_back(data);
                    }

                    

                    #ifndef HBR
                    if (elem.controller.size() == 36) {
                        archive[candidate] = elem;
                        }
                    #else
                        if (elem.controller.size() == 4) {
                        archive[candidate] = elem;
                        }
                    #endif

                }   
            }
        }
    }
    else {
      std::cerr << "ERROR: Could not load the archive." << std::endl;
      return archive;
    }     
    std::cout << archive.size() << " elements loaded" << std::endl;
    return archive;
}

    
void execute(const Eigen::VectorXd& desc, double t, bool stat = true)
{
    /*This function executes a desired descriptor desc for a given time duration t with a flat repertoire.*/
    std::cout<< "Beginning execute function" << std::endl ;
    std::vector<double> d(desc.size(), 0.0);
    Eigen::VectorXd::Map(d.data(), d.size()) = desc;

    std::ofstream ofs;
    ofs.open("exp/hte/visualise_MCTS/execute.dat", std::ios_base::app);
    ofs << desc.transpose() << " " << std::endl;
    ofs.close();
    
    std::cout<<"EXECUTED DESC: "<<desc.transpose()<<std::endl;
    
    std::vector<double> ctrl = Params::archiveparams::archive[d].controller;

    for(double& d:ctrl){
      std::cout << d << "   " ;
      d = round( d * 1000.0 ) / 1000.0;// limite numerical issues
    }
    std::cout << std::endl ; 

    double* ptr = &ctrl[0];
    Eigen::Map<Eigen::VectorXd> new_ctrl(ptr, ctrl.size()); 

    
    
    if (stat) {
        // statistics - descriptor
        for (int i = 0; i < desc.size(); i++)
            global::ctrl_file << desc(i) << " ";
        global::ctrl_file << std::endl;
    }


    static auto init_trans = global::simu.robots().back()->skeleton()->getPositions();

    // Run the controller
    double ctrl_dt = 0.01;
    global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, new_ctrl,global::dofs)); 

    std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);
    // std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_broken(std::vector<int>(1,4));
    global::simu.run(t);
    
    
    global::simulated_robot->remove_controller(0);
					    
    Eigen::Vector3d final_pos;
    Eigen::Vector3d final_rot;
    double arrival_angle;
    double covered_distance;
    
    auto pose = global::simu.robots().back()->skeleton()->getPositions();
    Eigen::Matrix3d rot = dart::math::expMapRot({pose[0], pose[1], pose[2]});
    Eigen::Matrix3d init_rot = dart::math::expMapRot({init_trans[0], init_trans[1], init_trans[2]});
    Eigen::MatrixXd init_homogeneous(4, 4);
    init_homogeneous << init_rot(0, 0), init_rot(0, 1), init_rot(0, 2), init_trans[3], init_rot(1, 0), init_rot(1, 1), init_rot(1, 2), init_trans[4], init_rot(2, 0), init_rot(2, 1), init_rot(2, 2), init_trans[5], 0, 0, 0, 1;
    Eigen::MatrixXd final_homogeneous(4, 4);
    final_homogeneous << rot(0, 0), rot(0, 1), rot(0, 2), pose[3], rot(1, 0), rot(1, 1), rot(1, 2), pose[4], rot(2, 0), rot(2, 1), rot(2, 2), pose[5], 0, 0, 0, 1;
    Eigen::Vector4d pos = {init_trans[3], init_trans[4], init_trans[5], 1.0};
    pos = init_homogeneous.inverse() * final_homogeneous * pos;

    final_pos = pos.head(3);

    covered_distance = std::round(final_pos(0) * 100) / 100.0;

    // Angle computation
    final_rot = dart::math::matrixToEulerXYZ(init_rot.inverse() * rot);

    // roll-pitch-yaw
    arrival_angle = std::round(final_rot(2) * 100) / 100.0;


    std::cout <<"final_pos: " <<"x: "<< final_pos(0) << " y: "<< final_pos(1)<< "->"<< arrival_angle <<std::endl ;
    // If the robot tries to fall or collides
    size_t n = 0;
    while (covered_distance < -10000.0 && n < 10) {
        // try to stabilize the robot
        global::simulated_robot->skeleton()->setPosition(0, 0.0);
        global::simulated_robot->skeleton()->setPosition(1, 0.0);
        global::simulated_robot->skeleton()->setPosition(5, 0.2);
        global::simulated_robot->skeleton()->setVelocities(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getVelocities().size()));
        global::simulated_robot->skeleton()->setAccelerations(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getAccelerations().size()));
        global::simulated_robot->skeleton()->clearExternalForces();
        global::simulated_robot->skeleton()->clearInternalForces();

    global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, Eigen::VectorXd(36).setZero(),global::dofs)); // neutral position controller
	std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);
	global::simu.run(1.0) ; // global::simu.run(1.0, true, false);
	global::simulated_robot->remove_controller(0); 
        n++;
    }
    
    
    // reset to zero configuration for stability
    // global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, std::vector<double>(36, 0.0))); // nuetral position controller
    global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, Eigen::VectorXd(36).setZero(),global::dofs)); // nuetral position controller

    std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);
    global::simu.run(1.0);
    
    global::simulated_robot->remove_controller(0);

    
    double theta = pose(2);
    
    while (theta < -M_PI)
        theta += 2 * M_PI;
    while (theta > M_PI)
        theta -= 2 * M_PI;
    

    global::robot_pose << pose(3), pose(4), theta;
    
}// execute

void hbr_execute(const Eigen::VectorXd& desc, double t, int tot_step = 0, bool stat = true,bool use_children = false)
{
    /*This function executes a Behavioural Descriptor desc for a total number of steps t. Tot_step is the number of steps we have already taken in total during the deployment . 
    If use_childre is true, we use the children of a behaviour that has been found during training. This disables any selection mechanism for the secondary behaviours.*/
    std::cout<< "Beginning execute function" << std::endl ;
    std::vector<double> d(desc.size(), 0.0);
    Eigen::VectorXd::Map(d.data(), d.size()) = desc;

    std::ofstream ofs;
    ofs.open("exp/hte/visualise_MCTS/execute.dat", std::ios_base::app);
    ofs << desc.transpose() << " " << std::endl;
    ofs.close();
    
    std::cout<<"EXECUTED DESC: "<<desc.transpose()<<std::endl;
    
    std::vector<double> ctrl = Params::archiveparams::archive[d].controller;

   
    typedef boost::shared_ptr<layer2::phen_t> indiv_t;
    indiv_t indiv;


    
    indiv_t i1 = indiv_t(new layer2::phen_t());
    i1->random();
    i1->fit().set_desc(d); 
    auto p1 = layer2::layer_ptr->container().get_index(i1);
    indiv = layer2::layer_ptr->container().archive()(p1);

    ctrl.resize(9);
    for(size_t i=0;i<ctrl.size();i++)
    {
        ctrl[i] = round( ctrl[i] * 1000.0 ) / 1000.0;// limite numerical issues
    } 

    Eigen::VectorXd current_vel,current_acc;
    Eigen::VectorXd real_state(18),current_state(18);
   
    auto init_trans = global::simu.robots().back()->skeleton()->getPositions();

    double global_x = 0,global_y = 0,global_yaw = 0;
    double ctrl_dt = 0.01;

    Eigen::Matrix3d init_rot_beg = dart::math::expMapRot({0, 0, init_trans[2]});
    Eigen::MatrixXd init_start(4, 4);
    init_start << init_rot_beg(0, 0), init_rot_beg(0, 1), init_rot_beg(0, 2), init_trans[3], init_rot_beg(1, 0), init_rot_beg(1, 1), init_rot_beg(1, 2), init_trans[4], init_rot_beg(2, 0), init_rot_beg(2, 1), init_rot_beg(2, 2),init_trans[5], 0, 0, 0, 1;
    int leg_configuration=63;
    Eigen::Vector4d first_global_pos;

    for(int step=0;step<t;step++){

        Eigen::Vector3d _final_pos;
        Eigen::Vector3d _final_rot;
        double _arrival_angle;

        std::cout<<"######  STEP "<<step<<"#####"<<std::endl;

        auto init_position = global::simu.robots().back()->skeleton()->getPositions();
        current_vel = global::simu.robots().back()->skeleton()->getVelocities();
        current_acc = global::simu.robots().back()->skeleton()->getAccelerations();
        real_state << init_position.head(6),(current_vel).head(6),(current_acc).head(6);
        indiv->fit().set_state(real_state);//for the GP predictions

        Eigen::Matrix3d init_rot = dart::math::expMapRot({0, 0, init_position[2]});
        Eigen::MatrixXd init_homogeneous(4, 4);
        init_homogeneous << init_rot(0, 0), init_rot(0, 1), init_rot(0, 2), init_position[3], init_rot(1, 0), init_rot(1, 1), init_rot(1, 2), init_position[4], init_rot(2, 0), init_rot(2, 1), init_rot(2, 2), init_position[5], 0, 0, 0, 1;
        
        // Angle computation
        double _current_angle = std::round(dart::math::matrixToEulerXYZ(init_rot)(2)*100)/100;
        int index = step%3;
        
        //Our controller is an array of waypoints that we use.
        if(step<1){
            // global_x += ((((std::cos(ctrl[1])*ctrl[0])+std::sqrt(2))/(2*std::sqrt(2)))*0.6)-0.3;
            // global_y += ((((std::sin(ctrl[1])*ctrl[0])+std::sqrt(2))/(2*std::sqrt(2)))*0.6)-0.3;
            // global_yaw += (ctrl[3]*2)-1;

            global_x += (ctrl[index*3+0]*1)-0.5;
            global_y += (ctrl[index*3+1]*1)-0.5;
            global_yaw += (ctrl[index*3+2]*2)-1;


        }
        else if(step>=0){
        // else if(step>=3){
            global_x = (((indiv->fit().desc()[0])*3)-1.5)*1.0;
            global_y = (((indiv->fit().desc()[1])*3)-1.5)*1.0;
            if (step>0){
                global_yaw = (indiv->fit().angle());
            }
            else{
                global_yaw += (ctrl[index*3+2]*2)-1;
            }
                

        }
        else{
            // global_x += ((((std::cos(ctrl[2])*ctrl[0])+std::sqrt(2))/(2*std::sqrt(2)))*0.6)-0.3;
            // global_y += ((((std::sin(ctrl[2])*ctrl[0])+std::sqrt(2))/(2*std::sqrt(2)))*0.6)-0.3;
            // global_yaw += (ctrl[3]*2)-1;

            global_x += (ctrl[index*3+0]*1)-0.5;
            global_y += (ctrl[index*3+1]*1)-0.5;
            global_yaw += (ctrl[index*3+2]*2)-1;
        }

        while ( global_yaw < -M_PI)
            global_yaw += 2 * M_PI;
        while ( global_yaw > M_PI)
            global_yaw -= 2 * M_PI;

        // global_yaw = angle_dist(_current_angle,global_yaw);//calculating difference of angle we still have to do to get to the wanted angle

        Eigen::Vector4d local_pos = {global_x,global_y,0.0,1.0};
        
        local_pos = init_start*local_pos;
        
        std::cout<<"Asked Steps "<<(ctrl[index*3+0]*1)-0.5<<"  "<<(ctrl[index*3+1]*1)-0.5<<"  "<<(ctrl[index*3+2]*2)-1<<std::endl;

        Eigen::Vector4d global_pos = {global_x,global_y,0.0,1.0};
        int broken_leg_bit= -1;

        /*We can force the robot to use a specific leg by defining the leg_configuration. Here we initialize the leg and later on we replace the leg configuration by the one selected.
        Legs (Binary shows which leg shouldn't be used (0)): 47 (101111)(for leg 1 damage), 55 (110111) (5), 61 (111101) 62 (111110),31 011111, 59 111011 (0), 45 101101
        Change this if we want different variations*/
        if(step==0){
        switch(global::damaged_leg){
            case -1:
                leg_configuration = 63;
                break;
            case 0:
                leg_configuration = 31;
                break;
            case 1:
                leg_configuration = 47;
                break;
            case 2:
                leg_configuration = 55;
                break;
            case 3:
                leg_configuration = 59;
                break;
            case 4:
                leg_configuration = 61;
                break;
            case 5:
                leg_configuration = 62;
                break;
            case 6:
                leg_configuration = 45;
                break;

        }
        }



        bool render = true;
        //If we have a negative damaged leg, we use the children seen during training. This avoids any selection.
        if(global::damaged_leg<0){
            use_children = true;
        }

        std::cout<<"Position "<<local_pos.transpose()<<"  "<<global_pos.transpose()<<std::endl;        
        global_pos = init_homogeneous.inverse()*local_pos;
        
        std::vector<boost::shared_ptr<layer1::phen_t>> neigh_indiv;
        neigh_indiv = layer1::layer_ptr->pop();

        Eigen::VectorXd mu, best_mu, best_descriptor;
        double sigma, best_dist=10000, dist, best_sigma=100.0;
        boost::shared_ptr<layer1::phen_t> best_indiv,compare_indiv;

        typedef Eigen::Map<const Eigen::VectorXd> point_t;


        std::vector<double> tmp_descriptor = {global_pos(0),global_pos(1),global_yaw,1,1,1,1,1,1};
        point_t p_i1(tmp_descriptor.data(), tmp_descriptor.size());
        auto neigh_nn = layer1::layer_ptr->container().archive().knn(p_i1,1);
        neigh_indiv.clear();
        for(auto& nn: neigh_nn){
            neigh_indiv.push_back(nn.second);
        }


        for (auto& n : neigh_indiv){

            std::vector<double> d = n->fit().desc();
            double* ptr = &d[0];
            Eigen::Map<Eigen::VectorXd> descriptor_vector(ptr, d.size()); 

            Eigen::VectorXd test_legs(6),test_sample(9),mu(3),wanted_behav(3);
            std::vector<double> scores,actions;
            double sigma;
            std::vector<double>::iterator result;
            int best_index = -1;

            //We want to try out every secondary behaviour that we have and predict the error.
            for(int l=0;l<64;l++){
                auto leg_bits = std::bitset<6>(l);
                for(int i=0;i<leg_bits.size();i++){
                  descriptor_vector(i+3) = (double) leg_bits[leg_bits.size()-i-1];
                }

                std::tie(mu, sigma) = global::gp_low_model.query(descriptor_vector);


                //Scaling mu to represent the error we want to minimize
                mu(0) = std::exp(-4*std::max(0.0,mu(0)));
                mu(1) = std::exp(-4*std::max(0.0,mu(1)));
                mu(2) = std::exp(-3*std::max(0.0,mu(2)));


                scores.push_back(mu.norm());

                wanted_behav << (global_pos[0]+0.5)/1,(global_pos[1]+0.5)/1,(global_yaw+1)/2.0;

                std::bitset<6> bits;
                for(int i=0;i<bits.size();i++){
                    bits[bits.size()-i-1] = descriptor_vector[3+i];
                }

                //Check if our move is in this list. We didn't find any move for these repertoires (either impossible or very sparse number of skills and thus unusable)
                bool found = false;
                for(int i: std::vector<int>{0,1,2,4,8,16,32,3,5,6,7,24,40,48,56,17,10,12,33,20,34,14,49,36,9,18,28,35}){
                    if(bits == i){
                        found = true;
                        break;
                    }
                }
                if(found){
                    continue;
                }
                


                double current_step = 1+step+tot_step*3;
                double delta = 0.1;
                double beta = 2*std::log(36*1.5*std::pow(current_step,2)*std::pow(M_PI,2)/(6*delta));

                beta = std::sqrt(beta/2);

                dist = - mu.norm() - beta*sigma;


                if((dist<best_dist)){
                    best_dist = dist;
                    best_indiv = n;
                    best_sigma = sigma;
                    best_mu = mu;
                    best_descriptor = descriptor_vector;
                    std::cout<<"Error "<<best_mu.norm()<<"  And Sigma "<<best_sigma<<"  Features "<<best_descriptor.transpose()<<"  distance "<<best_dist<<" fitness"<<best_indiv->fit().value()<<std::endl;
                }
            }
            
        }
        
        std::bitset<6> bits;
        for(int i=0;i<bits.size();i++){
            bits[bits.size()-i-1] = best_descriptor[3+i];
        }
        

        // COMMENT OUT FOR PERFECT HBR
        #ifndef PERFECT_HTE
            leg_configuration = bits.to_ulong();// we overwrite the perfect leg_config with the selected one.
        #endif

        #ifdef RANDOM_HTE
            //Closed Intervall (including borders) for int uniform distribution
            static tools::rgen_int_t rgen_legs(0,  63);
            bool not_valid = true;
            int random_leg_config = -1;
            while (not_valid){
                random_leg_config = rgen_legs.rand();
                not_valid = false;
                for(int i: std::vector<int>{0,1,2,4,8,16,32,3,5,6,7,24,40,48,56,17,10,12,33,20,34,14,49,36,9,18,28,35}){
                    if(random_leg_config == i){
                        not_valid = true;
                        break;
                    }
                }
            }
            leg_configuration = random_leg_config;
        #endif
        
        std::cout<<"Predicted Action  "<<leg_configuration<<" with Error "<<best_mu.norm()<<"  And Sigma "<<best_sigma<<"  Features "<<best_descriptor.transpose()<<"  distance "<<best_dist<<std::endl;

        //Actual Damage Logging
        switch(global::damaged_leg){
            case -1:
                broken_leg_bit = 63;
                break;
            case 0:
                broken_leg_bit = 31;
                break;
            case 1:
                broken_leg_bit = 47;
                break;
            case 2:
                broken_leg_bit = 55;
                break;
            case 3:
                broken_leg_bit = 59;
                break;
            case 4:
                broken_leg_bit = 61;
                break;
            case 5:
                broken_leg_bit = 62;
                break;
            case 6:
                broken_leg_bit = 45;
                break;

        }
        global::dmg_file << step << "   ";
        auto leg_bits = std::bitset<6>(broken_leg_bit);
        for(int i=0;i<leg_bits.size();i++){
            global::dmg_file << (double) leg_bits[leg_bits.size()-i-1] << " ";
        }

        //Log the chosen action
        global::dmg_file << "   ";
        leg_bits = std::bitset<6>(leg_configuration);
        for(int i=0;i<leg_bits.size();i++){
            global::dmg_file << (double) leg_bits[leg_bits.size()-i-1] << " ";
        }
        global::dmg_file << "   " << std::endl;



        //Skill Execution
        std::cout<<"Position (Initial) "<<init_position.head(6).transpose()<<std::endl;
        std::cout<<"Position (Local) "<<global_pos.transpose()<<std::endl;
        std::cout<<"Angle (Local) "<<global_yaw<<std::endl;
        Eigen::VectorXd new_ctrl;
        boost::shared_ptr<layer1::phen_t> temp_indiv;
        std::tie(new_ctrl,temp_indiv) = indiv->fit().ff(global_pos,global_yaw,leg_configuration,render,use_children,step);// grabbing the controllers at the lowest level
        global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, new_ctrl,global::dofs)); 
        std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);
       
        auto inter_pose = global::simu.robots().back()->skeleton()->getPositions().head(6);
        double height_inter = inter_pose(5);
        // std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_broken(std::vector<int>(1,4));
        global::simu.run(1);
        global::simulated_robot->remove_controller(0);
        global::simu.world()->setTime(0.0);//IMPORTANT to reset time!
        

        //Pose Observations Calculations
        auto pose = global::simu.robots().back()->skeleton()->getPositions();
        Eigen::Matrix3d rot = dart::math::expMapRot({pose[0], pose[1], pose[2]});
        init_rot = dart::math::expMapRot({0, 0, init_position[2]});

        init_homogeneous << init_rot(0, 0), init_rot(0, 1), init_rot(0, 2), init_position[3], init_rot(1, 0), init_rot(1, 1), init_rot(1, 2), init_position[4], init_rot(2, 0), init_rot(2, 1), init_rot(2, 2), init_position[5], 0, 0, 0, 1;
        Eigen::MatrixXd final_homogeneous(4, 4);
        final_homogeneous << rot(0, 0), rot(0, 1), rot(0, 2), pose[3], rot(1, 0), rot(1, 1), rot(1, 2), pose[4], rot(2, 0), rot(2, 1), rot(2, 2), pose[5], 0, 0, 0, 1;
        Eigen::Vector4d tpos = {init_position[3], init_position[4], init_position[5], 1.0};
        Eigen::Vector4d pos = {pose[3], pose[4], pose[5], 1.0};
        

        pos = init_homogeneous.inverse()*pos; // We want our final position to be expressed with respect to the initial frame
        _final_pos = pos.head(3);
        // Angle computation
        _final_rot = dart::math::matrixToEulerXYZ(init_rot.inverse() * rot);
        // roll-pitch-yaw
        _arrival_angle = std::round(_final_rot(2) * 100) / 100.0;

        double x = _final_pos(0),y= _final_pos(1),yaw = _arrival_angle;


        Eigen::VectorXd observation(3),sample(9),legs(6);
        std::vector<double> temp_indiv_desc = temp_indiv->fit().desc();
        double x_error = 0.0, y_error = 0.0, yaw_error = 0.0;

        x_error = std::abs((temp_indiv_desc[0]-(x+0.5)/1))/std::pow(2*std::abs(temp_indiv_desc[0]-0.5)+0.001,1);//  + std::abs((temp_indiv_desc[0]-(global_pos(0)+0.5)/1))/std::pow(temp_indiv_desc[0]+0.001,2);
        y_error = std::abs((temp_indiv_desc[1]-(y+0.5)/1))/std::pow(2*std::abs(temp_indiv_desc[1]-0.5)+0.001,1);// + std::abs((temp_indiv_desc[1]-(global_pos(1)+0.5)/1))/std::pow(temp_indiv_desc[1]+0.001,2);
        yaw_error = std::abs((temp_indiv_desc[2]-(yaw+1.0)/2.0))/(std::pow(2*std::abs(temp_indiv_desc[2]-0.5)+0.001,1));// + std::abs((temp_indiv_desc[2]-(global_yaw+1.0)/2.0))/(std::pow(std::abs(temp_indiv_desc[2]-0.5)+0.001,1));


        std::cout<<"Executed Indiv "<<temp_indiv_desc[0]<<" "<<temp_indiv_desc[1]<<" "<<temp_indiv_desc[2]<<std::endl;
        std::cout<<"X Error: "<<x_error<<std::endl;
        std::cout<<"Y Error: "<<y_error<<std::endl;
        std::cout<<"Yaw Error: "<<yaw_error<<std::endl;

        observation << x_error,y_error,yaw_error;

        
        leg_bits = std::bitset<6>(leg_configuration);
        for(int i=0;i<leg_bits.size();i++){
            legs(i) = (double) leg_bits[leg_bits.size()-i-1];
            // std::cout<<(double) leg_bits[leg_bits.size()-i-1]<<" ";
        }
        if(use_children){
            std::cout<<"USING CHILDREN"<<std::endl;
            std::vector<double> d = indiv->fit().children()[step]->fit().desc();
            double* ptr = &d[0];
            Eigen::Map<Eigen::VectorXd> new_sample(ptr, d.size()); 
            sample = new_sample; //using children to train the GP
        }
        else{
            std::vector<double> d = temp_indiv->fit().desc();
            double* ptr = &d[0];
            Eigen::Map<Eigen::VectorXd> new_sample(ptr, d.size()); 
            sample = new_sample;
        }

        std::cout<<"USING GP"<<std::endl;
        global::gp_low_model.add_sample(sample,observation);
        // if(step%3==0)
        //     global::gp_low_model.optimize_hyperparams();
        std::cout<<"Trained GP"<<std::endl;

        
        std::cout<<"Actual Displacement BD:  "<<(x+0.5)/1<<"  "<<(y+0.5)/1<<"  "<<(yaw+1)/2<<std::endl;
        std::cout<<"Actual Displacement:  "<<x<<"  "<<y<<"  "<<yaw<<std::endl;
        std::cout<<"Adding Sample: "<<sample.transpose()<<" and Observation  "<<observation.transpose()<<std::endl;
    } 

    
    
    if (stat) {
        // statistics - descriptor
        for (int i = 0; i < desc.size(); i++)
            global::ctrl_file << desc(i) << " ";
        global::ctrl_file << std::endl;
    }

    //Observations after executing all the substeps of the Hierarchy				    
    Eigen::Vector3d final_pos;
    Eigen::Vector3d final_rot;
    double arrival_angle;
    double covered_distance;
    
    auto pose = global::simu.robots().back()->skeleton()->getPositions();
    Eigen::Matrix3d rot = dart::math::expMapRot({pose[0], pose[1], pose[2]});
    Eigen::Matrix3d init_rot = dart::math::expMapRot({init_trans[0], init_trans[1], init_trans[2]});
    Eigen::MatrixXd init_homogeneous(4, 4);
    init_homogeneous << init_rot(0, 0), init_rot(0, 1), init_rot(0, 2), init_trans[3], init_rot(1, 0), init_rot(1, 1), init_rot(1, 2), init_trans[4], init_rot(2, 0), init_rot(2, 1), init_rot(2, 2), init_trans[5], 0, 0, 0, 1;
    Eigen::MatrixXd final_homogeneous(4, 4);
    final_homogeneous << rot(0, 0), rot(0, 1), rot(0, 2), pose[3], rot(1, 0), rot(1, 1), rot(1, 2), pose[4], rot(2, 0), rot(2, 1), rot(2, 2), pose[5], 0, 0, 0, 1;
    Eigen::Vector4d pos = {init_trans[3], init_trans[4], init_trans[5], 1.0};
    pos = init_homogeneous.inverse() * final_homogeneous * pos;

    final_pos = pos.head(3);

    covered_distance = std::round(final_pos(0) * 100) / 100.0;

    // Angle computation
    final_rot = dart::math::matrixToEulerXYZ(init_rot.inverse() * rot);

    // roll-pitch-yaw
    arrival_angle = std::round(final_rot(2) * 100) / 100.0;


    std::cout <<"final_pos: " <<"x: "<< final_pos(0) << " y: "<< final_pos(1)<< "->"<< arrival_angle <<std::endl ;
    // If the robot tries to fall or collides
    size_t n = 0;
    while (covered_distance < -10000.0 && n < 10) {
        // try to stabilize the robot
        global::simulated_robot->skeleton()->setPosition(0, 0.0);
        global::simulated_robot->skeleton()->setPosition(1, 0.0);
        global::simulated_robot->skeleton()->setPosition(5, 0.2);
        global::simulated_robot->skeleton()->setVelocities(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getVelocities().size()));
        global::simulated_robot->skeleton()->setAccelerations(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getAccelerations().size()));
        global::simulated_robot->skeleton()->clearExternalForces();
        global::simulated_robot->skeleton()->clearInternalForces();

    global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, Eigen::VectorXd(36).setZero(),global::dofs)); // neutral position controller
	std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);
	global::simu.run(1.0) ;
	global::simulated_robot->remove_controller(0); 
        n++;
    }
    
    
    // reset to zero configuration for stability
    global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, Eigen::VectorXd(36).setZero(),global::dofs)); // neutral position controller

    std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);
    global::simu.run(1.0); //global::simu->run(1.0, true, false);
    
    global::simulated_robot->remove_controller(0);
    double theta = pose(2);//pose(2);
    
    while (theta < -M_PI)
        theta += 2 * M_PI;
    while (theta > M_PI)
        theta -= 2 * M_PI;
    global::robot_pose << pose(3), pose(4), theta;
    
}// hbr_execute


// radius, speed
std::tuple<double, double, double> get_x_y_theta(const Eigen::Vector3d& prev_pose, const Eigen::Vector3d& curr_pose)
{

    std::cout<<" displacement norm : "<<(prev_pose.head(2)-curr_pose.head(2)).norm()<<std::endl;
    double c = std::cos(prev_pose(2)), s = std::sin(prev_pose(2));
    Eigen::MatrixXd r(2, 2);
    r << c, -s, s, c;
    r.transposeInPlace();
    Eigen::VectorXd d(2);
    d << prev_pose(0), prev_pose(1);
    d = r * d;
    Eigen::MatrixXd tr(3, 3);
    tr << r(0, 0), r(0, 1), -d(0), r(1, 0), r(1, 1), -d(1), 0, 0, 1;
    Eigen::MatrixXd tr2(3, 3);
    c = std::cos(curr_pose(2));
    s = std::sin(curr_pose(2));
    tr2 << c, -s, curr_pose(0), s, c, curr_pose(1), 0, 0, 1;
    Eigen::Vector3d tr_pos;
    tr_pos << 0.0, 0.0, 1.0;
    tr_pos = tr * tr2 * tr_pos;
    std::ofstream ofs;
    ofs.open("exp/hte/visualise_MCTS/get_x_y.dat", std::ios_base::app);
    ofs << tr_pos(0) << " "
	<< tr_pos(1) << " "
	<< angle_dist(prev_pose(2), curr_pose(2)) << std::endl;
    ofs.close();

    std::cout<<" get_x_y_t : "<<tr_pos(0)<< ", "<< tr_pos(1)<<" -> "<< angle_dist(prev_pose(2), curr_pose(2))<<std::endl;
    std::cout<<" get_x_y_t_bd : "<<(tr_pos(0)+1.5)/3<< ", "<< (tr_pos(1)+1.5)/3<<" -> "<< (angle_dist(prev_pose(2), curr_pose(2))+1)/2<<std::endl;

    std::cout<<" computed displacement norm : "<<(tr_pos.head(2)).norm()<<std::endl;
    
    return std::make_tuple(tr_pos(0), tr_pos(1), angle_dist(prev_pose(2), curr_pose(2)));
} //get_x_y_theta

template <typename Params>
struct ActiveLearningValue {
    const double _epsilon = 1e-6;

    template <typename MCTSAction>
    double operator()(const std::shared_ptr<MCTSAction>& action)
    {
        return action->value() / (double(action->visits()) + _epsilon) + Params::active_learning::k() * Params::active_learning::scaling() * global::gp_model.sigma(action->action()._desc);
    }
}; // ActiveLeaningValue


void draw_obs_svg(svg::Document& doc)
{
    for (auto ob : global::obstacles) {
        svg::Circle circle(svg::Point(ob._y * global::entity_size, ob._x * global::entity_size), ob._radius * 2.0 * global::entity_size, svg::Fill(svg::Color::Red), svg::Stroke(1, svg::Color::Red));
        doc << circle;
    }
} //draw_obs_svg


void draw_target_svg(svg::Document& doc)
{
    svg::Circle circle_target(svg::Point(Params::goal_y() * global::entity_size, Params::goal_x() * global::entity_size), Params::cell_size() * global::entity_size, svg::Fill(), svg::Stroke(2, svg::Color::Green));
    doc << circle_target;
} //draw_target_svg


void draw_robot_svg(svg::Document& doc, const Eigen::Vector3d& pose)
{
    // Draw robot
    svg::Circle circle_init(svg::Point(pose(1) * global::entity_size, pose(0) * global::entity_size), Params::cell_size() / 2.0 * global::entity_size, svg::Fill(svg::Color::Blue), svg::Stroke(1, svg::Color::Black));
    doc << circle_init;

    svg::Circle circle_small(svg::Point(pose(1) * global::entity_size, pose(0) * global::entity_size), Params::cell_size() / 4.0 * global::entity_size, svg::Fill(svg::Color::Fuchsia), svg::Stroke(1, svg::Color::Black));
    doc << circle_small;

    // Draw direction
    Eigen::Vector2d dir(pose(0) + std::cos(pose(2)) * Params::cell_size() / 2.0, pose(1) + std::sin(pose(2)) * Params::cell_size() / 2.0);
    svg::Polyline polyline_clean(svg::Stroke(2.0, svg::Color::Fuchsia));
    polyline_clean << svg::Point(pose(1) * global::entity_size, pose(0) * global::entity_size);
    polyline_clean << svg::Point(dir(1) * global::entity_size, dir(0) * global::entity_size);
    doc << polyline_clean;
} //draw_robot_svg


void draw_line_svg(svg::Document& doc, const Eigen::Vector2d& from, const Eigen::Vector2d& to)
{
    svg::Polyline polyline_clean(svg::Stroke(1.0, svg::Color::Black));
    polyline_clean << svg::Point(from(1) * global::entity_size, from(0) * global::entity_size);
    polyline_clean << svg::Point(to(1) * global::entity_size, to(0) * global::entity_size);
    doc << polyline_clean;
}//draw line svg

std::tuple<bool, size_t, size_t> reach_target(const Eigen::Vector3d& goal_state, size_t max_iter = std::numeric_limits<size_t>::max(),std::string save_path = ".")
{
    // using Choose = mcts::GreedyValue;
    using Choose = ActiveLearningValue<Params>;

    Params::set_goal_x(goal_state(0));
    Params::set_goal_y(goal_state(1));
    Params::set_goal_theta(goal_state(2));
    global::target_num++;
    global::dimensions = svg::Dimensions(global::width, global::height);
    global::doc = std::make_shared<svg::Document>(save_path +"/plan_" + std::to_string(global::target_num) + ".svg", svg::Layout(global::dimensions, svg::Layout::TopLeft));

    // Initialize statistics
    global::robot_file.open(save_path + "/robot_" + std::to_string(global::target_num) + ".dat");
    global::ctrl_file.open(save_path + "/ctrl_" + std::to_string(global::target_num) + ".dat");
    global::iter_file.open(save_path +"/iter_" + std::to_string(global::target_num) + ".dat");
    global::misc_file.open(save_path +"/misc_" + std::to_string(global::target_num) + ".dat");
    global::dmg_file.open(save_path +"/dmg_" + std::to_string(global::target_num) + ".dat");
    

    // Draw obstacles
    draw_obs_svg(*global::doc);
    // Draw target
    draw_target_svg(*global::doc);
    // Draw robot
    draw_robot_svg(*global::doc, global::robot_pose);
    // Save doc
    global::doc->save();

    RewardFunction world;

    size_t n = 0;

    // statistics
    global::robot_file << "-1 " << global::robot_pose(0) << " " << global::robot_pose(1) << " " << global::robot_pose(2) << std::endl;

    bool collided = false;
    bool terminal = false;
    Params::set_collisions(0);

    // while (!terminal && !collided && (n < max_iter)) {
    while (!terminal && (n < max_iter)) {
        auto t1 = std::chrono::steady_clock::now();
        // Get last post from hexapod simulation/real robot
        HexaState<Params> init = HexaState<Params>(global::robot_pose(0), global::robot_pose(1), global::robot_pose(2));
        // DefaultPolicy<HexaState<Params>, HexaAction<Params>>()(&init, true);
        if(init.terminal() && !init.goal()){
            std::cout<<"Robot is in an obstacle! Taking closest point!"<<std::endl;
            double val = std::numeric_limits<double>::max();
            double best_x,best_y;
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    if (i == 0 && j == 0)
                        continue;
                    double x_new = global::robot_pose(0) + i* Params::cell_size();
                    double y_new = global::robot_pose(1) + j* Params::cell_size();
                    double dx = x_new - global::robot_pose(0);
                    double dy = y_new - global::robot_pose(1);
                    double v = dx * dx + dy * dy;
                    if (!collides(x_new , y_new) && x_new > 0 && x_new < int(global::map_size_x)* Params::cell_size() && y_new > 0 && y_new < int(global::map_size_y)* Params::cell_size() && v < val) {
                        best_x = x_new;
                        best_y = y_new;
                        val = v;
                    }
                }
            }
            init = HexaState<Params>(best_x, best_y, global::robot_pose(2));
            std::cout<<"Best Position "<<best_x<<" "<<best_y<<" , Old Position "<<global::robot_pose(0)<<" "<<global::robot_pose(1)<<std::endl;
        }
        
	
	    std::cout<< "initializing MCTS "<<init.terminal()<<std::endl ;
        // Run MCTS
        auto t2 = std::chrono::steady_clock::now();
        auto tree = std::make_shared<mcts::MCTSNode<Params, HexaState<Params>, mcts::SimpleStateInit<HexaState<Params>>, mcts::SimpleValueInit, mcts::UCTValue<Params>, mcts::UniformRandomPolicy<HexaState<Params>, HexaAction<Params>>, HexaAction<Params>, mcts::SPWSelectPolicy<Params>, mcts::ContinuousOutcomeSelect<Params>>>(init, 20);

	    std::cout<< "compute MCTS"<<std::endl ;
        
        tree->compute(world, Params::iterations());
	
        std::cout<< "computed!"<<std::endl ;

    auto mcts_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t2).count();
    
	auto time_running = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count();
    std::cout << "Children: " << tree->children().size() << std::endl;
    double sum = 0;
    double max = -std::numeric_limits<double>::max();
    double min = std::numeric_limits<double>::max();

    for (auto child : tree->children()) {
        double v = child->value() / double(child->visits());

    sum += v;

        if (v > max)
            max = v;
        if (v < min)
            min = v;
    }

    Params::active_learning::set_scaling((max - min) / Params::kernel_exp::sigma_sq());
    std::cout << "Active learning scaling: " << Params::active_learning::scaling() << std::endl;

    // Get best action/behavior
    auto best = tree->best_action<Choose>();
	if(!best)
	  std::cout<<"ERROR"<<std::endl;
        //std::cout << "val: " << best->value() / double(best->visits()) << std::endl;
        auto other_best = tree->best_action<mcts::GreedyValue>();
        //std::cout << "val without AL: " << other_best->value() / double(other_best->visits()) << std::endl;
        // std::cout << "avg: " << (sum / double(tree->children().size())) << std::endl;
        auto tmp = init.move(best->action(), true);
        std::cout << tmp._x << " " << tmp._y << " -> " << tmp._theta << std::endl;
        global::iter_file << n << " " << time_running / 1000.0 <<" "<<global::time_random_search/1e6<<" "<<global::n_policies_called<<" "<<global::n_random_policies_called<<" "<<tree->children().size()<<" "<<tree->max_depth()<<" " << best->value() / double(best->visits()) << " " << other_best->value() / double(other_best->visits()) << " " << (sum / double(tree->children().size())) << " " << tmp._x << " " << tmp._y << " " << tmp._theta << std::endl;
        
        global::n_policies_called = 0;//reset of number of action sampling calls
        global::n_random_policies_called = 0;//reset of number of default policy calls for rollouts
        global::time_random_search = 0.0;//reset of the time we need to find a random action in our archive
        std::cout << "Passed 1st round of MCTS" << std::endl ;
	
        // Execute in simulation
        Eigen::Vector3d prev_pose = global::robot_pose;

        #ifndef HBR
            execute(best->action()._desc, 3.0);
        #else
            bool use_children = false;          
            hbr_execute(best->action()._desc, 3.0,n,true,use_children);
        #endif


	    std::cout << "global::robot_pose "<< global::robot_pose(0) << " " << global::robot_pose(1) << " -> " << global::robot_pose(2) << std::endl;
        // Draw robot
        draw_robot_svg(*global::doc, global::robot_pose);
        // Draw line connecting steps
        draw_line_svg(*global::doc, Eigen::Vector2d(prev_pose(0), prev_pose(1)), Eigen::Vector2d(global::robot_pose(0), global::robot_pose(1)));
        global::doc->save();

        // std::cout << "Robot: " << global::robot_pose.transpose() << std::endl;
        global::robot_file << n << " " << global::robot_pose(0) << " " << global::robot_pose(1) << " " << global::robot_pose(2) << std::endl;

	
        global::misc_file << n << " ";
        if (Params::learning()) {
            // Update GP (add_sample)
            Eigen::VectorXd observation(3);
	    
            std::tie(observation(0), observation(1), observation(2)) = get_x_y_theta(prev_pose, global::robot_pose);
            Eigen::VectorXd data(4);

            data(0) = observation(0);
            data(1) = observation(1);
            data(2) = std::cos(observation(2));
            data(3) = std::sin(observation(2));

            global::gp_model.add_sample(best->action()._desc, data);
            
            global::misc_file << data(0) << " " << data(1) << " " << data(2) << " " << data(3) << " ";
        }
        global::misc_file << "3.0" << std::endl;

        // Check collisions/termination
        if (collides(global::robot_pose(0), global::robot_pose(1))) {
            collided = true;
            std::cout << "Collision!" << std::endl;
        }

        double dx = global::robot_pose(0) - Params::goal_x();
        double dy = global::robot_pose(1) - Params::goal_y();
        double threshold_xy = Params::cell_size() * Params::cell_size() / 4.0;
        if ((dx * dx + dy * dy) < threshold_xy) {
            terminal = true;
        }
        n++;
    }

    // Close statistics
    global::robot_file.close();
    global::ctrl_file.close();
    global::iter_file.close();
    global::misc_file.close();
    global::dmg_file.close();

    if (n < max_iter && !collided)
        return std::make_tuple(true, n, Params::collisions());

    if (n < max_iter)
        return std::make_tuple(true, n, Params::collisions());

    return std::make_tuple(false, n, Params::collisions());
} //reach_targets

void init_simu()
{
  //to do add damages

  //Damages

  //-----------------------------init_simu-------------------------------------------------
    std::cout<<"INIT Robot"<<std::endl;
    global::global_robot = std::make_shared<robot_dart::Robot>("exp/hte/resources/hexapod_v2.urdf");

    global::global_robot->set_position_enforced(true);

    // global::global_robot->set_actuator_types(dart::dynamics::Joint::SERVO);
    global::global_robot->set_actuator_types("servo");

    

    //Damages

    //need this for the controllers so that we can use the damages!!
    auto full_dofs = global::global_robot->dof_names(true, true, true);  //controllable + not controllable
    global::dofs = std::vector<std::string>(full_dofs.begin() + 6, full_dofs.end());  //extract only the controllable params, we need this for the damage to work!!

    // //Locking at certain Position!!

    if(global::damaged_leg>=0 && global::damaged_leg<=5){
        global::global_robot->skeleton()->getJoint("leg_"+std::to_string(global::damaged_leg)+"_1_2")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
        
        global::global_robot->set_actuator_type("locked", "leg_"+std::to_string(global::damaged_leg)+"_1_2");
        global::global_robot->set_actuator_type("locked", "leg_"+std::to_string(global::damaged_leg)+"_2_3");
        global::global_robot->set_actuator_type("locked", "body_leg_"+std::to_string(global::damaged_leg));
        // global::global_robot->set_actuator_type("passive", "leg_"+std::to_string(global::damaged_leg)+"_1_2");
        // global::global_robot->set_actuator_type("passive", "leg_"+std::to_string(global::damaged_leg)+"_2_3");
        // global::global_robot->set_actuator_type("passive", "body_leg_"+std::to_string(global::damaged_leg));
    }
    if(global::damaged_leg==6){
        global::global_robot->skeleton()->getJoint("leg_"+std::to_string(4)+"_1_2")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely

        global::global_robot->set_actuator_type("locked", "leg_"+std::to_string(4)+"_1_2");
        global::global_robot->set_actuator_type("locked", "leg_"+std::to_string(4)+"_2_3");
        global::global_robot->set_actuator_type("locked", "body_leg_"+std::to_string(4));

        global::global_robot->skeleton()->getJoint("leg_"+std::to_string(1)+"_1_2")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely

        global::global_robot->set_actuator_type("locked", "leg_"+std::to_string(1)+"_1_2");
        global::global_robot->set_actuator_type("locked", "leg_"+std::to_string(1)+"_2_3");
        global::global_robot->set_actuator_type("locked", "body_leg_"+std::to_string(1));
    }

    global::global_robot->skeleton()->enableSelfCollisionCheck();

    for(auto act: global::global_robot->actuator_types())
        std::cout<<act<<" ";
    std::cout<<std::endl;
    
    // std::cout<<"NumDOF "<<global::global_robot->skeleton()->getNumDofs()<<std::endl;
    global::simulated_robot = global::global_robot->clone(); // can put this line inside init_simu


    std::cout<<"End INIT Robot"<<std::endl;

}


void init_simu_world(const Eigen::Vector3d& start)
{


    // Clear previous maps
    //global::simu->clear_objects();

    // Add obstacles to simulation
    // add_ellipsoid is a function in hexapod_dart/robot dart
    // global::simu is a class/object in hexapod_dart/robot_dart
    // obstacles is a vector of the object of the class SimpleObstacles which is defined at the top of the script std::vector<SimpleObstacle> obstacles
    // obs global::obstacles which was formed and filled up in init_map
    Eigen::Vector3d size;
    Eigen::Vector6d pose;

    //Maze Map
    global::box_obstacles.push_back(BoxObstacle (-0.2,3.06,0.75,4.0));
    global::box_obstacles.push_back(BoxObstacle (2.2, -0.3,2.7, 0.625));
    global::box_obstacles.push_back(BoxObstacle (1.73, 1.84,2.73, 2.34));
    global::box_obstacles.push_back(BoxObstacle (2.73, 1.84,3.23, 2.84));
    global::box_obstacles.push_back(BoxObstacle (3.15, 1.84, 4.5, 4.04));

    global::box_obstacles.push_back(BoxObstacle (-0.2, -0.31,4.5, -0.3));
    global::box_obstacles.push_back(BoxObstacle (-0.21, -0.3, -0.2, 4.0));
    global::box_obstacles.push_back(BoxObstacle (4.5, -0.3, 4.51, 4.0));
    global::box_obstacles.push_back(BoxObstacle (-0.2, 4.0, 4.5, 4.01));


    global::height = 768;
    int r = 9;
    int c = 8;

    global::width = static_cast<size_t>(double(c) / double(r) * 768);

    //overwriting this given the fixed map we use
    
    global::map_size = (r > c) ? r - 1 : c - 1;
    global::map_size_x = r;
    global::map_size_y = c;
    global::entity_size = static_cast<size_t>(((global::height > global::width) ? global::width : global::height) / double(global::map_size * 0.5));

    std::cout<<global::width<<"  "<<global::map_size<<" "<<global::map_size_x<<"  "<<global::map_size_y<<std::endl;




    int num = 1;
    for (auto obs : global::box_obstacles) {
        std::cout<<obs.size().transpose()<<"  "<<obs.pose().transpose()<<std::endl;
        global::simu.add_robot(robot_dart::Robot::create_box(obs.size(), obs.pose(), "fixed", 1., dart::Color::Red(1.0), "box_" + std::to_string(num)));
        num++;
    }
    global::robot_pose = start;
    global::target_num = 0;
    std::cout << "init_simu_world complete" << std::endl;   
}


MCTS_DECLARE_DYN_PARAM(double, Params::uct, c);
MCTS_DECLARE_DYN_PARAM(double, Params::spw, a);
MCTS_DECLARE_DYN_PARAM(double, Params::cont_outcome, b);
MCTS_DECLARE_DYN_PARAM(double, Params::active_learning, scaling);
MCTS_DECLARE_DYN_PARAM(double, Params::active_learning, k);
MCTS_DECLARE_DYN_PARAM(double, Params, goal_x);
MCTS_DECLARE_DYN_PARAM(double, Params, goal_y);
MCTS_DECLARE_DYN_PARAM(double, Params, goal_theta);
MCTS_DECLARE_DYN_PARAM(double, Params, iterations);
MCTS_DECLARE_DYN_PARAM(bool, Params, learning);
MCTS_DECLARE_DYN_PARAM(size_t, Params, collisions);
MCTS_DECLARE_DYN_PARAM(size_t, Params::mcts_node, parallel_roots);



Params::archiveparams::archive_t Params::archiveparams::archive;
int main(int argc, char **argv)
{   
    namespace po = boost::program_options;
    boost::program_options::options_description desc;
    desc.add_options()("dir,d", po::value<std::string>(), "custom directory for gen files (when evolving)");
    desc.add_options()("leg,l", po::value<int>(), "leg to be damaged");
    desc.add_options()("rep,r", po::value<int>(), "directory of the gen files to use");

    
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    std::string save_path,archive_nr;
    int num = 0;

    if (vm.count("dir")) {
      save_path = vm["dir"].as<std::string>();
    }
    else{
        save_path = "./";
    }

    if (vm.count("rep")) {
      num = vm["rep"].as<int>();
      std::cout<<num<<std::endl;
      if(num<=2){
        #define ARCHIVE_SPEC
      }
      if (num < 15 && num > 0){
          archive_nr = std::to_string(num);
      }
        
      else{
        std::cout<<"INVALID Archive,USING DEFAULT ARCHIVE, (Valid ARCHIVES between 1 and 14)"<<std::endl;
        archive_nr = "";
      }

    }
    else{
        archive_nr = "";
    }

    if (vm.count("leg")) {
        global::damaged_leg = vm["leg"].as<int>();
        if (global::damaged_leg>6 || global::damaged_leg<0){
            std::cout<<"INVALID LEG, NO DAMAGE SET, (Valid Legs between 0 and 5 and 6 for double damage)"<<std::endl;
            global::damaged_leg = -1;
        }
    }
    else{
        global::damaged_leg = -1;
    }

    std::cout<<"SAVING TO "<<save_path<<std::endl;
    std::cout<<"Damaging Leg "<<global::damaged_leg<<std::endl;

    //------------------------------------------------LOAD ARCHIVE FILE (BEHAVIOURAL REPERTOIRE------------------------------------------------//
    std::string archive_file = "";

    #ifdef HBR
       
        std::string path = "";
        if(archive_nr.empty())
        {
            path = "exp/hte/resources/hbr_repertoires/hbr_gp_repertoire/Archive1";
            archive_file = path+"/archive_20000_hbr.dat";
            }
        else
        {
            path = "exp/hte/resources/hbr_repertoires/hbr_gp_repertoire/Archive" + archive_nr;
            archive_file = path+"/archive_20000_hbr.dat";
        }

        
         //Initialising all the layers in the layer namespace
        layer0::qd_t qd1;
        layer1::qd_t qd2;
        layer2::qd_t qd3;
        layer0::layer_ptr = &qd1;
        layer1::layer_ptr = &qd2;
        layer2::layer_ptr = &qd3;

        //trick because we readd everything to the Behevioral Repertoire
        layer0::Params::nov::l = layer0::Params::nov::l * (1 - layer0::Params::nov::eps);
        layer1::Params::nov::l = layer1::Params::nov::l * (1 - layer1::Params::nov::eps);
        layer2::Params::nov::l = layer2::Params::nov::l * (1 - layer2::Params::nov::eps);

        typedef boost::shared_ptr<typename layer2::phen_t> indiv_t3;
        typedef std::vector<indiv_t3> pop_t3;

        typedef boost::shared_ptr<typename layer1::phen_t> indiv_t2;
        typedef std::vector<indiv_t2> pop_t2;

        typedef boost::shared_ptr<typename layer0::phen_t> indiv_t1;
        typedef std::vector<indiv_t1> pop_t1;

        qd1.set_fit_proto(layer0::phen_t::fit_t());
        qd2.set_fit_proto(layer1::phen_t::fit_t());
        qd3.set_fit_proto(layer2::phen_t::fit_t());
        
        pop_t1 pop_leg = qd1.get_pop_from_gen_file(path+"/gen_l1");
        std::cout<<"Layer 1 Loaded: "<<layer0::layer_ptr->container().archive().size()<<std::endl;
        pop_t2 pop_body = qd2.get_pop_from_gen_file(path+"/gen_l2");
        std::cout<<"Layer 2 Loaded: "<<pop_body.size()<<"  "<<layer1::layer_ptr->container().archive().size()<<std::endl;
        pop_t3 pop_omni = qd3.get_pop_from_gen_file(path+"/gen_l3");
        std::cout<<"Layer 3 Loaded: "<<pop_omni.size()<<"  "<<layer2::layer_ptr->container().archive().size()<<std::endl;
        std::cout<<"Updating GPs"<<std::endl;
        sferes::modif::GP_Update<typename layer2::phen_t,layer2::Params> gp_modif;
        gp_modif.force(qd3);
        std::cout<<"Fitting GPs done"<<std::endl;

    #else
    if(ARCHIVE_SIZE==2){
        if(archive_nr.empty())
        {
             archive_file = "exp/hte/resources/flat_repertoires/flat_2D_repertoire/Archive1/archive_2400_2d.dat";
            }
        else
        {
            archive_file = "exp/hte/resources/flat_repertoires/flat_2D_repertoire/Archive"+ archive_nr+"/archive_2400_2d.dat";
        }
        
    }
    else{
        if(archive_nr.empty())
        {
            archive_file = "exp/hte/resources/flat_repertoires/flat_8D_repertoire/Archive1/archive_2400_8d.dat";
            }
        else
        {
            archive_file = "exp/hte/resources/flat_repertoires/flat_8D_repertoire/Archive"+ archive_nr+"/archive_2400_8d.dat";
        }
        
    }
    
    #endif
    if (!archive_file.empty()) {
        // Loading archive
        std::cout << "Loading archive..." << std::endl;
        Params::archiveparams::archive = load_archive(archive_file);
    }
    

    // Params::uct::set_c(150.0);
    Params::uct::set_c(100.0);
    Params::spw::set_a(0.5);
    Params::cont_outcome::set_b(0.6);
    if(ARRAY_DIM==8){
        Params::set_iterations(600);
    }
    else{
        Params::set_iterations(6000);
    }
    
    
    Params::mcts_node::set_parallel_roots(32);
    Params::active_learning::set_k(0.0);
    Params::set_learning(true);
    
    //----------------------------------------------INITIALISING STARTING POSITION AND ORIENTATION OF THE ROBOT----------------------------------------------------//
    Eigen::Vector3d robot_state; // starting position of robot
    std::vector<Eigen::Vector3d> goal_states;

    // Manually defining position and goal of robot.
    goal_states.clear();
    goal_states.push_back(Eigen::Vector3d(3.79, 0.424, 0));
    robot_state<< 2.32, 3.27,dart::math::constants<double>::pi();

    std::srand(std::time(NULL));
    init_simu();


    // Can put this in execute also 
    global::simulated_robot->skeleton()->setPosition(2, robot_state(2));
    global::simulated_robot->skeleton()->setPosition(3, robot_state(0));
    global::simulated_robot->skeleton()->setPosition(4, robot_state(1));
    global::simulated_robot->skeleton()->setPosition(5, 0.15); // z-axis -make sure the robot is not stuck in the floor
    
    //------------------------------------SETUP SIMULATION ENVIRONMENT---------------------------------------//

#ifdef GRAPHIC
    std::cout<<"Graphics Starting..."<<std::endl;
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(&global::simu);    
    
    std::cout<<"Graphics Set"<<std::endl;
    graphics->set_enable(true);    
    
    // graphics->record_video(save_path + std::string("/hexapod_rte.mp4"), 30);
    global::simu.set_graphics(graphics);
    std::static_pointer_cast<robot_dart::gui::magnum::Graphics>(global::simu.graphics())->look_at({6.0, 2.5, 5}, {2.5, 2.5, 0.});

#endif

    global::simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
    global::simu.add_floor(50);
    init_simu_world(robot_state);
    global::simu.add_robot(global::simulated_robot);

    std::cout << "Robot starting: " << robot_state.transpose() << "\nGoals: \n";
    for (auto g : goal_states)
        std::cout << g.transpose() << std::endl;
    std::cout << "--------------------------" << std::endl;

    
    if (Params::learning()) {
            write_gp(save_path + "/gp_0.dat");
    }

    
    std::ofstream results_file(save_path + "/results.dat");
    results_file << goal_states.size() << std::endl; //prints how many goal states there are to get to final goal 
    
    // Max trials are 100
    bool found = true;
    size_t n_iter, n_cols;
    
    for (size_t i = 0; i < goal_states.size(); i++) {
        if (!found) {

	  Eigen::Vector2d state;
          // Just as a safety, i will always be bigger than 0
          if (i > 0)
              state << goal_states[i - 1](0), goal_states[i - 1](1);
          else
              state << robot_state(0), robot_state(1);
	  
          global::simulated_robot->skeleton()->setPosition(0, 0.0);
          global::simulated_robot->skeleton()->setPosition(1, 0.0);
          global::simulated_robot->skeleton()->setPosition(2, robot_state(2)); //yaw angle - pitch 
          global::simulated_robot->skeleton()->setPosition(3, state(0)); // x coordinate 
          global::simulated_robot->skeleton()->setPosition(4, state(1)); //y coordinate 
          global::simulated_robot->skeleton()->setPosition(5, 0.2); // z-axis place slightly above the ground to ensure the legs are not stuck in the grounf
          global::simulated_robot->skeleton()->setVelocities(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getVelocities().size()));
          global::simulated_robot->skeleton()->setAccelerations(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getAccelerations().size()));
          global::simulated_robot->skeleton()->clearExternalForces();
          global::simulated_robot->skeleton()->clearInternalForces();

          global::robot_pose << state(0), state(1), robot_state(2);

	}
	std::cout<< "reach_target function"<<std::endl ;
    auto t1 = std::chrono::steady_clock::now();
	std::tie(found, n_iter, n_cols) = reach_target(goal_states[i],80,save_path);
    auto time_reach_target = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count();
	results_file << found << " " << n_iter << " " << n_cols <<" "<<time_reach_target/1000<< std::endl;
    }
    results_file.close();

    // Print the final position of the robot
    std::cout << global::simulated_robot->skeleton()->getPositions().head(6).tail(3).transpose() << std::endl;

    global::simulated_robot.reset();
    global::global_robot.reset();
    return 0;

    //g_robot is the clone - has becomes global::simulated_robot
}

    












