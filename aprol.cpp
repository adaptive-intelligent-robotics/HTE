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
#include <boost/filesystem.hpp>


#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/control/pd_control.hpp>
// #include <robot_dart/control/hexa_control.hpp>
#include <hexa_control.hpp>

#include <limbo/limbo.hpp>
#include <map_elites/binary_map.hpp> // to load in archive_file (behavioural repertoire) from MAPELITES

#include <mcts/uct.hpp>
#include <svg/simple_svg.hpp>
#include <boost/program_options.hpp>
#include <astar/a_star.hpp>
#include <algorithm>

#include <robot_dart/gui/magnum/graphics.hpp>
#include <robot_dart/gui/magnum/windowless_graphics.hpp>


#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include"hbr_loading.hpp"
#include <hbr/Leg/fit_hexa_leg.hpp>
#include <hbr/Body_Acc/fit_hexa_body_gp.hpp>

#ifdef BASE_HBR
  #include <hbr/Omnidirectional/fit_hexa_body.hpp>
#else
  #include <hbr/Omnidirectional/fit_mt_me.hpp>
#endif

#include <hbr/HBR/quality_diversity_hbr.hpp>

#define ARCHIVE_SIZE 2
#define NUM_THREADS 32

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
      static std::vector<archive_t> archives;
    }; //archive_params
}; //Params


template <typename Params,int* PtrIndex>
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
        typename Params::archiveparams::elem_archive elem = Params::archiveparams::archives[*PtrIndex][vv];
        // if(*PtrIndex ==1)
        //     std::cout<<"Index Map "<<*PtrIndex<<std::endl;
        r << elem.x, elem.y, elem.cos_theta, elem.sin_theta;
        // std::cout<<"Index Map "<<*PtrIndex<<"  "<<r.transpose()<<std::endl;
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


bool collides(double x, double y, double r = Params::robot_radius());

namespace global {

    

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

    std::vector<int> prior_map_indices,prior_map_trials;
    std::vector<double> prior_map_means;
    int latest_map_index = 0;
    int* index_ptr = &latest_map_index;
    std::vector<std::string> archive_files;

    int n_policies_called = 0;//reset of number of action sampling calls
    int n_random_policies_called = 0;//
    double time_random_search = 0.0;

    using kernel_t = kernel::Exp<Params>; // kernel is under namespace limbo in limbo.hpp - limbo::kernel::Exp
    using mean_t = MeanArchive<Params,&latest_map_index>; // MeanArchive is written above 
    using GP_t = model::GP<Params, kernel_t, mean_t>; //model is under namespace limbo in limbo.hpp - limbo::model::GP 
   
    // GP_t gp_model(ARCHIVE_SIZE, 4);

    std::vector<GP_t> gp_models;
    // statistics
    std::ofstream robot_file, ctrl_file, iter_file, misc_file;

    int damaged_leg;

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
        int prior_map_index = global::prior_map_indices.back();
        global::latest_map_index = prior_map_index;
        std::tie(mu, sigma) = global::gp_models[prior_map_index].query(desc);
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

// bool collides(double x, double y, double r)
// {
//     for (size_t i = 0; i < global::obstacles.size(); i++) {
//         double dx = x - global::obstacles[i]._x;
//         double dy = y - global::obstacles[i]._y;
//         if (std::sqrt(dx * dx + dy * dy) <= global::obstacles[i]._radius + r) {
// 	  //std::cout<< x <<" ; "<< y<<"  vs "<<  global::obstacles[i]._x<<" ; "<< global::obstacles[i]._y<<std::endl;
//             return true;
//         }
//     }
//     return false;
// } // collides

//collision version for boxes
bool collides(double x, double y,double r)
{
    double distance_x = 0,distance_y=0, cDist_sq = 0;
    for (size_t i = 0; i < global::box_obstacles.size(); i++) {
        
        distance_x = abs(x - (global::box_obstacles[i].c_x()));
        distance_y = abs(y - (global::box_obstacles[i].c_y()));

        // std::cout<<global::box_obstacles[i].c_x()<<"  "<<global::box_obstacles[i].c_y()<<" "<<x<<"  "<<y<<std::endl;
        // std::cout<<distance_x<<"  "<<distance_y<<"  "<<global::box_obstacles[i].width()/2<<"  "<<(global::box_obstacles[i].height()/2)<<std::endl;
        if (distance_x > (global::box_obstacles[i].width()/2 + r)) { continue; }
        if (distance_y > (global::box_obstacles[i].height()/2 + r)) { continue; }
        if (distance_x <= (global::box_obstacles[i].width()/2)) { return true; } 
        if (distance_y <= (global::box_obstacles[i].height()/2)) { return true; }
        cDist_sq = std::pow((distance_x - (global::box_obstacles[i].width()/2)),2) + std::pow((distance_y - (global::box_obstacles[i].height()/2)),2);
        if(cDist_sq <= (r*r)){
            // std::cout<<"Circle "<<cDist_sq<<" "<<(r*r)<<std::endl;
            // std::cout<<"Here : True 3"<<std::endl;
            return true;
        }
        // if( (x-radius < global::box_obstacles[i].x() + global::box_obstacles[i].width() &&
        //     x-radius > global::box_obstacles[i].x() &&
        //     y-radius < global::box_obstacles[i].y() + global::box_obstacles[i].height() &&
        //     y-radius > global::box_obstacles[i].y()) ||
        //     ( x+radius < global::box_obstacles[i].x() + global::box_obstacles[i].width() &&
        //     x+radius > global::box_obstacles[i].x() &&
        //     y+radius < global::box_obstacles[i].y() + global::box_obstacles[i].height() &&
        //     y+radius > global::box_obstacles[i].y()))
        //     {
        //     return true;
        // }
    }
    return false;
} // collides

// bool collides_segment(const Eigen::Vector2d& start, const Eigen::Vector2d& end)
// {
//     const double epsilon = 1e-6;
//     double Dx = end(0) - start(0);
//     double Dy = end(1) - start(1);

//     for (size_t i = 0; i < global::obstacles.size(); i++) {
//         double Fx = start(0) - global::obstacles[i]._x;
//         double Fy = start(1) - global::obstacles[i]._y;

//         double a = Dx * Dx + Dy * Dy;
//         double b = 2.0 * (Fx * Dx + Fy * Dy);
//         double c = (Fx * Fx + Fy * Fy) - global::obstacles[i]._radius_sq;

//         double discriminant = b * b - 4.0 * a * c;
//         if (discriminant > epsilon) {
//             discriminant = std::sqrt(discriminant);

//             double t1 = (-b - discriminant) / (2.0 * a);
//             double t2 = (-b + discriminant) / (2.0 * a);

//             if (t1 > epsilon && (t1 - 1.0) < epsilon) {
//                 return true;
//             }

//             if (t2 > epsilon && (t2 - 1.0) < epsilon) {
//                 return true;
//             }
//         }
//     }
//     return false;
// } // collides_segment

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




// bool collides(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double r = Params::robot_radius())
// {
//     Eigen::Vector2d dir = end - start;
//     Eigen::Vector2d perp = Eigen::Vector2d(-dir(1), dir(0));
//     Eigen::Vector2d A = start + perp * r;
//     Eigen::Vector2d B = end + perp * r;
//     Eigen::Vector2d C = end - perp * r;
//     Eigen::Vector2d D = start - perp * r;
//     return (collides(start(0), start(1), r) || collides(end(0), end(1), r) || collides_segment(A, B) || collides_segment(B, C) || collides_segment(C, D) || collides_segment(D, A));
// } //collides


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
    return collides(s, t, Params::robot_radius() * 1.3);
    // return collides_box(s, t);
} //astar_collides 


template <typename State, typename Action>
struct DefaultPolicy { 
    // Action operator()(const std::shared_ptr<State>& state)
    Action operator()(const State* state, bool draw = false)
    {
        global::n_policies_called +=1;
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
		    //std::cout<<"DEPOL:operator " <<std::endl;
		    best_value = val;
                    best_action = act;
                }
            }
	    //std::cout<<"DEPOL1 "<< best_action._desc.size()<<std::endl;

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
        // std::cout<<"Here1"<<std::endl;

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
                // if (b) {
                //     svg::Circle circle_target(svg::Point(new_best(1) * global::entity_size, new_best(0) * global::entity_size), Params::cell_size() * global::entity_size, svg::Fill(), svg::Stroke(2, svg::Color::Red));
                //     (*global::doc) << circle_target;
                // }
                // else {
                //     svg::Circle circle_target(svg::Point(new_best(1) * global::entity_size, new_best(0) * global::entity_size), Params::cell_size() * global::entity_size, svg::Fill(), svg::Stroke(2, svg::Color::Blue));
                //     (*global::doc) << circle_target;
                // }
            } while (b && n < 10);

            // svg::Polyline polyline_clean(svg::Stroke(1.0, svg::Color::Black));
            // polyline_clean << svg::Point(closest_obs._y * global::entity_size, closest_obs._x * global::entity_size);
            // polyline_clean << svg::Point(best_pos(1) * global::entity_size, best_pos(0) * global::entity_size);
            // (*global::doc) << polyline_clean;
            //
            // std::cout << "n: " << n << std::endl;

            if (n < 10)
                best_pos = new_best;
        }

        // if (draw) {
        //     svg::Circle circle_target(svg::Point(best_pos(1) * global::entity_size, best_pos(0) * global::entity_size), Params::cell_size() * global::entity_size, svg::Fill(), svg::Stroke(2, svg::Color::Green));
        //     (*global::doc) << circle_target;
        // }

        Action best_action;
        double best_value = std::numeric_limits<double>::max();
        for (size_t i = 0; i < N; i++) {
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
      
        // HexaState<Params> tmp = move(act);
        // // Check if state is outside of bounds
        // if (tmp._x < 0.0 || tmp._x > global::map_size * Params::cell_size() || tmp._y < 0.0 || tmp._y > global::map_size * Params::cell_size())
        //     return false;
        //
        // if (collides(tmp._x, tmp._y) || collides(Eigen::Vector2d(_x, _y), Eigen::Vector2d(tmp._x, tmp._y)))
        //     return false;
        return true;
    }

    HexaAction<Params> next_action() const
    {
        return DefaultPolicy<HexaState<Params>, HexaAction<Params>>()(this);
    }

    HexaAction<Params> random_action() const
    {
        global::n_random_policies_called +=1;
        tools::rgen_int_t rgen(0, Params::archiveparams::archives[global::latest_map_index].size() - 1);
        typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;
        HexaAction<Params> act;
        double time_running = 0.0;
        do {
            auto t1 = std::chrono::steady_clock::now();
            archive_it_t it = Params::archiveparams::archives[global::latest_map_index].begin();
            std::advance(it, rgen.rand());
            time_running = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t1).count();
            try{
            Eigen::VectorXd desc = Eigen::VectorXd::Map(it->first.data(), it->first.size());
            act._desc = desc;
            
            }
            catch (const std::bad_alloc& e) {
             std::cout << "Allocation failed: " << e.what()<<"  "<<it->first.size()<<" Map Index:  "<<global::latest_map_index<< '\n';
            }
        } while (!valid(act));
        global::time_random_search += time_running;
        return act;
    }

    HexaState move(const HexaAction<Params>& action, bool no_noise = false) const
    {
        double x_new, y_new, theta_new;
        Eigen::VectorXd mu;
        double sigma;
        int prior_map_index = global::prior_map_indices.back();
        global::latest_map_index = prior_map_index;
        std::tie(mu, sigma) = global::gp_models[prior_map_index].query(action._desc);

	/*
	std::ofstream ofs;
        ofs.open("exp/rte_v2_adapt/visualise_MCTS/move_gp.dat", std::ios_base::app);
	ofs << action._desc.transpose() << " "
            << mu.transpose() << " "
            << sigma << std::endl;
	ofs.close();
	*/
	
	if (!no_noise) {
            // std::cout << mu.transpose() << std::endl;
            mu(0) = std::max(-1.5, std::min(1.5, gaussian_rand(mu(0), sigma)));
            mu(1) = std::max(-1.5, std::min(1.5, gaussian_rand(mu(1), sigma)));
            mu(2) = std::max(-1.0, std::min(1.0, gaussian_rand(mu(2), sigma)));
            mu(3) = std::max(-1.0, std::min(1.0, gaussian_rand(mu(3), sigma)));
            //std::cout << "WITH NOISE - To: " << mu.transpose() << std::endl;
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

        ///std::cout << "(" << _x << "," << _y << "," << _theta << ") with (" << mu(0) << "," << mu(1) << "," << mu(2) << ") -> (" << x_new << "," << y_new << "," << theta_new << ")" << std::endl;
        return HexaState(x_new, y_new, theta_new);
    }

    bool terminal() const
    {
        // Check if state is outside of bounds
        if (_x < 0.0 || _x >= global::map_size_x * Params::cell_size() || _y < 0.0 || _y >= global::map_size_y * Params::cell_size())
	  {
	    //std::cout<<"--A--"<<std::endl;
	    //std::cout<<"_x "<< _x<<std::endl;
	    //std::cout<<"_y "<< _y<<std::endl;
	    //std::cout<<" global::map_size_x * Params::cell_size() "<<  global::map_size_x * Params::cell_size()<<std::endl;
	    //std::cout<<" global::map_size_y * Params::cell_size() "<<  global::map_size_y * Params::cell_size()<<std::endl;

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

    std::cout << "Loading text file..."<<archive_name.c_str() << std::endl;
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
            
            // if (numbers.size() < 39) //36+2+1
            //     continue;
            //std::cout << "HERE1" << std::endl;
            int init_i = 0;
            if (numbers.size() > 39)
                init_i = 1;

            // for(auto n: numbers){
            //     std::cout<<n<<"  ";
            // }
            // std::cout<<std::endl;
            //std::cout << "HERE2" << std::endl;
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
            // std::cout << "HERE3" << std::endl;
                if(ARCHIVE_SIZE == 8){
                    double data = numbers[init_i + i];
                    if (i == 0) {
                        candidate[i] = data;
                        elem.x = (data*3)-1.5 ;
                    }
                    if (i == 1) {
                            candidate[i] = data;
                            elem.y = (data*3)-1.5  ;
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
                            // candidate[i] = (data-0.5)*2;
                            // elem.x = (data-0.5)*2 ;
                            candidate[i] = data;
                            elem.x = (data*3)-1.5;
                        #else
                            candidate[i] = data;
                            elem.x = (data*3)-1.5;
                        #endif
                    }
                    if (i == 1) {
                            #ifndef HBR
                                // candidate[i] = (data-0.5)*2;
                                // elem.y = (data-0.5)*2 ;
                                candidate[i] = data;
                                elem.y = (data*3)-1.5;
                            #else
                                candidate[i] = data;
                                elem.y = (data*3)-1.5;
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
      std::cout << "ERROR: Could not load the archive." << std::endl;
      return archive;
    }     
    std::cout << archive.size() << " elements loaded" << std::endl;
    return archive;
}

//Calulcating the UCB means for all the maps and returning the posterior for all the maps
void update_ucb_posterior(Eigen::VectorXd desc,Eigen::VectorXd observation){
    double k = 5;
    double closeness = std::exp(-k * (desc-observation).norm() );
    global::prior_map_means[global::prior_map_indices.back()] = ((global::prior_map_trials[global::prior_map_indices.back()]-1)*global::prior_map_means[global::prior_map_indices.back()]+closeness)/(global::prior_map_trials[global::prior_map_indices.back()]);
}

Eigen::VectorXd get_ucb_posterior(){
    Eigen::VectorXd ucb(global::prior_map_trials.size()),posteriors(global::prior_map_trials.size());
    int total_trials = 0;
    total_trials = std::accumulate(global::prior_map_trials.begin(), global::prior_map_trials.end(), 0);

    for(int i=0;i<ucb.size();i++){
        ucb[i] = 0.01 * std::sqrt((2*std::log(total_trials))/global::prior_map_trials[i]);
    }

    Eigen::VectorXd means = Eigen::VectorXd::Map(global::prior_map_means.data(), global::prior_map_means.size());
    posteriors = (means+ucb)/(means+ucb).sum();
    return posteriors;
}

double gaussian_likelihood(Eigen::VectorXd mean, double var,Eigen::VectorXd goal){
    double likelihood = 0;

    //watch out for the normalization!! mu is usually not normalized
    for(int m=0;m<mean.size();m++){
        likelihood += -0.5*std::log(2*M_PI*var)-(std::pow(((mean[m]+1.5)/3)-goal[m],2)/(2.0*var + 1e-10));
    }
    // if((mean*3-1.5) == goal)
    //     std::cout<<"Likelihood "<<likelihood<<"  "<<mean.transpose()<<" "<<var<<std::endl;
    
    return std::exp(likelihood);
}
    

// typename Params::archiveparams::archive_t choose_prior(const Eigen::VectorXd& desc){
std::tuple<int,const Eigen::VectorXd> choose_prior(const Eigen::VectorXd& goal){

    //we can use goal to filter the individuals we want to use but our MCTS algorithm is already doing this.
    Eigen::VectorXd mu, desc, posteriors, chosen_desc,full_desc;
    std::vector<double>::iterator result;
    double sigma;
    int best_index = -1;
    posteriors = get_ucb_posterior();

    std::vector<double> scores,best_likelihoods, index_likelihoods;
    std::vector<double> prior_probs(Params::archiveparams::archives.size(),1/Params::archiveparams::archives.size());//prior probability to select one repertoire, uniform in our case
    for(int a = 0; a<Params::archiveparams::archives.size();a++){
        scores.clear();
        // int tmp = 0;
        global::latest_map_index = a;//we need to do this to use the right gp archive for the mean, needs to be fixed because it's not nice.
        for(auto indiv: Params::archiveparams::archives[a]){
            desc = Eigen::VectorXd::Map(indiv.first.data(), indiv.first.size());
            std::tie(mu, sigma) = global::gp_models[a].query(desc);

            // full_desc << desc(0),desc(1),indiv.second.cos_theta,indiv.second.sin_theta;


            // std::cout<<desc.transpose()<<"  "<<mu.transpose()<<"  "<<std::endl;
            // scores.push_back(gaussian_likelihood(mu, sigma,goal)*posteriors[a]);
            scores.push_back(gaussian_likelihood(mu.head(2), sigma,goal)*posteriors[a]);
            // if(gaussian_likelihood(mu.head(2), sigma, goal)>1)
            //     std::cout<<tmp<<"  Goal "<<goal.transpose()<<"  "<<desc.transpose()<<"  "<<mu.transpose()<<"  "<<posteriors[a]<<"  "<<scores.back()<<std::endl;
            // if(desc==goal)
            //      std::cout<<tmp<<"  EQUAL! "<<goal.transpose()<<"  "<<desc.transpose()<<"  "<<mu.transpose()<<"  "<<posteriors[a]<<"  "<<scores.back()<<"  "<<std::log(gaussian_likelihood(mu.head(2), sigma, goal))<<std::endl;
            // tmp++;
        }
        best_likelihoods.push_back(*std::max_element(scores.begin(),scores.end()));
        index_likelihoods.push_back(std::distance(scores.begin(), std::max_element(scores.begin(),scores.end())));
        std::cout<<"Map "<<a<<" Best Likelihood "<<best_likelihoods.back()<<" Posterior for Map: "<<posteriors[a]<<" Index "<<index_likelihoods.back()<<std::endl;
    }

    result = std::max_element(best_likelihoods.begin(),best_likelihoods.end());
    best_index = std::distance(best_likelihoods.begin(), result);


    //go fetch the best individual from the best map
    auto it = Params::archiveparams::archives[best_index].begin();
    std::advance(it, index_likelihoods[best_index]);  

    // auto best_individual = *it;
    chosen_desc = Eigen::VectorXd::Map(it->first.data(), it->first.size());

    global::prior_map_indices.push_back(best_index);//saving which repertoire we have used
    return std::tie(best_index,chosen_desc);  
}

/*
bool load_archive(const std::string& filename)
{
    Params::archiveparams::archive.clear();
    binary_map::BinaryMap b_map = binary_map::load(filename);

    for (auto v : b_map.elems) {
        std::vector<double> desc(v.pos.size(), 0.0);
        std::copy(v.pos.begin(), v.pos.end(), desc.begin());
        for (size_t i = 0; i < desc.size(); i++) {
            desc[i] /= (double)(b_map.dims[i]);
        }

        std::vector<double> ctrl(v.phen.size(), 0.0);
        std::copy(v.phen.begin(), v.phen.end(), ctrl.begin());
        assert(ctrl.size() == 36);

        Params::archiveparams::elem_archive elem; //saves the compoentns of the archive in to the Params::archiveparams
        elem.controller = ctrl;
        elem.x = -2.0 + (desc[ARCHIVE_SIZE - 2] * 4.0); //ARCHIVESIZE = 2 - low dim (behavioural descriptor is the x and y position) 
        elem.y = -2.0 + (desc[ARCHIVE_SIZE - 1] * 4.0);
        elem.cos_theta = std::cos(v.extra);
        elem.sin_theta = std::sin(v.extra);
        Params::archiveparams::archive[desc] = elem;
    }

    std::cout << "Loaded " << Params::archiveparams::archive.size() << " elements!" << std::endl;

    return true;
} //load_archive
*/

    
void execute(const Eigen::VectorXd& desc, double t, bool stat = true)
{
    std::cout<< "Beginning execute function" << std::endl ;

    std::ofstream ofs;
    ofs.open("exp/rte_v2_adapt/visualise_MCTS/execute.dat", std::ios_base::app);
    ofs << desc.transpose() << " " << std::endl;
    ofs.close();

    /*
    // HACK
    typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;

    archive_it_t it = Params::archiveparams::archive.begin();
    std::advance(it, 1000);
    Eigen::VectorXd desc2 = Eigen::VectorXd::Map(it->first.data(), it->first.size());
    Eigen::VectorXd::Map(d.data(), d.size()) = desc2;
    
    std::cout<<"EXECUTED DESC: "<<desc2.transpose()<<std::endl;
    //HACK
    */

    std::vector<double> d(desc.size(), 0.0);
    Eigen::VectorXd::Map(d.data(), d.size()) = desc;

    std::cout<<"EXECUTED DESC: "<<desc.transpose()<<std::endl;

    std::vector<double> ctrl = Params::archiveparams::archives[global::latest_map_index][d].controller;

    if(ctrl.empty()){
        std::cout<<"Index "<<global::latest_map_index<<std::endl;
        std::cout<<"Not Found"<<std::endl;
    }

    for(double& d:ctrl){
    //   std::cout << d << "   " ;
      d = round( d * 1000.0 ) / 1000.0;// limite numerical issues
    }
    // std::cout << std::endl ; 

    double* ptr = &ctrl[0];
    Eigen::Map<Eigen::VectorXd> new_ctrl(ptr, ctrl.size()); 

    
    
    if (stat) {
        // statistics - descriptor
        for (int i = 0; i < desc.size(); i++)
            global::ctrl_file << desc(i) << " ";
        global::ctrl_file << std::endl;
    }


    //#ifndef ROBOT
    // Resetting forces for stability
    //global::simulated_robot->skeleton()->setVelocities(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getVelocities().size()));
    //global::simulated_robot->skeleton()->setAccelerations(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getAccelerations().size()));
    //global::simulated_robot->skeleton()->clearExternalForces();
    //global::simulated_robot->skeleton()->clearInternalForces();

    static auto init_trans = global::simu.robots().back()->skeleton()->getPositions();
    // Run the controller
    // double ctrl_dt = 0.015;
    double ctrl_dt = 0.01;

    
        

    global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, new_ctrl,global::dofs)); 

    std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);

    // std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_broken(std::vector<int>(1,4));
    global::simu.run(t); //global::simu->run(1.0, true, false);
    
    
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

	// global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, std::vector<double>(36, 0.0))); // nuetral position controller
    global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, Eigen::VectorXd(36).setZero(),global::dofs)); // nuetral position controller
	std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);
	global::simu.run(1.0) ; // global::simu.run(1.0, true, false);
	global::simulated_robot->remove_controller(0); 
        n++;
    }
    
    
    // reset to zero configuration for stability
    // global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, std::vector<double>(36, 0.0))); // nuetral position controller
    global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, Eigen::VectorXd(36).setZero(),global::dofs)); // nuetral position controller

    std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);
    global::simu.run(1.0); //global::simu->run(1.0, true, false);
    
    global::simulated_robot->remove_controller(0);

    //#else // FOR REAL ROBOT ONLY 
    // global::hexa->move(ctrl, t, !global::first);
    //std::thread t1 = std::thread(&hexapod_ros::Hexapod::move, global::hexa, ctrl, t, !global::first);
    //std::thread t2 = std::thread(check_collision, t);
    //t1.join();
    //t2.join();
    //global::first = true;

    //Eigen::Vector3d robot = get_tf("odom", "/base_link");
    //Eigen::Vector3d pos;
    //pos << robot(0), robot(1), 1.0;
    //pos = global::transform * pos;
					    //#endif 
    //#ifndef ROBOT
    double theta = pose(2);//pose(2);
    //#else
    //double theta = robot(2) + global::orig_theta;
    //#endif
    
    while (theta < -M_PI)
        theta += 2 * M_PI;
    while (theta > M_PI)
        theta -= 2 * M_PI;
    //#ifndef ROBOT
    global::robot_pose << pose(3), pose(4), theta;
    //#else
    //global::robot_pose << pos(0), pos(1), theta;
    //#endif
    
}// execute

void hbr_execute(const Eigen::VectorXd& desc, double t, bool stat = true)
{
    std::cout<< "Beginning execute function" << std::endl ;
    std::vector<double> d(desc.size(), 0.0);
    Eigen::VectorXd::Map(d.data(), d.size()) = desc;

    std::ofstream ofs;
    ofs.open("exp/rte_v2_adapt/visualise_MCTS/execute.dat", std::ios_base::app);
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
    // indiv.fit()->ff()

    // ctrl.resize(4);
    ctrl.resize(9);
    for(size_t i=0;i<ctrl.size();i++)
    {
        ctrl[i] = round( ctrl[i] * 1000.0 ) / 1000.0;// limite numerical issues
        // if(i==1 || i==2){
        // ctrl[i] *= 2*M_PI;
        // }
        // if (i==0){
        // ctrl[i] *= std::sqrt(2);
        // }
        // std::cout<<ctrl[i]<<" ";
    } 
    std::cout<<std::endl;

    // double* ptr = &ctrl[0];
    // Eigen::Map<Eigen::VectorXd> new_ctrl(ptr, ctrl.size());
    Eigen::VectorXd current_vel,current_acc;
    Eigen::VectorXd real_state(18),current_state(18);
   
    auto init_trans = global::simu.robots().back()->skeleton()->getPositions();

    // double global_x = init_trans(3),global_y = init_trans(4),global_yaw = init_trans(2);
    double global_x = 0,global_y = 0,global_yaw = 0;
    double ctrl_dt = 0.01;

    Eigen::Matrix3d init_rot_beg = dart::math::expMapRot({0, 0, init_trans[2]});
    Eigen::MatrixXd init_start(4, 4);
    init_start << init_rot_beg(0, 0), init_rot_beg(0, 1), init_rot_beg(0, 2), init_trans[3], init_rot_beg(1, 0), init_rot_beg(1, 1), init_rot_beg(1, 2), init_trans[4], init_rot_beg(2, 0), init_rot_beg(2, 1), init_rot_beg(2, 2),init_trans[5], 0, 0, 0, 1;


    for(int step=0;step<t;step++){

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
        // std::cout<<init_homogeneous*local_pos<<std::endl;
        



        
        //Our controller is an array of waypoints that we use.
        if(step==-1){
            // global_x += ((((std::cos(ctrl[1])*ctrl[0])+std::sqrt(2))/(2*std::sqrt(2)))*0.6)-0.3;
            // global_y += ((((std::sin(ctrl[1])*ctrl[0])+std::sqrt(2))/(2*std::sqrt(2)))*0.6)-0.3;
            // global_yaw += (ctrl[3]*2)-1;

            global_x += (ctrl[index*3+0]*1)-0.5;
            global_y += (ctrl[index*3+1]*1)-0.5;
            global_yaw += (ctrl[index*3+2]*2)-1;


        }
        else if(step>=0){
            global_x = ((indiv->fit().desc()[0])*3)-1.5;
            global_y = ((indiv->fit().desc()[1])*3)-1.5;
            global_yaw = 0;

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
        // local_pos = init_homogeneous*local_pos;
        local_pos = init_start*local_pos;
        


        Eigen::Vector4d global_pos = {global_x,global_y,0.0,1.0};
        
        //47 (101111), 55 (110111), 61 (111101) 62 (111110),31 011111, 59 111011, 45 101101
        int leg_configuration = 45;//change this if we want different variations

        bool use_children = false, render = true;

        std::cout<<"Position "<<local_pos.transpose()<<"  "<<global_pos.transpose()<<std::endl;

        // global_pos = init_homogeneous.inverse()*global_pos; //We need to express the wanted movement in terms of the robot coordinates
        global_pos = init_homogeneous.inverse()*local_pos;
        // global_pos = init_rot_beg.inverse()*local_pos;
        // global_pos = {(ctrl[index*3+0]*1)-0.5,(ctrl[index*3+1]*1)-0.5,0.0,1.0};
        std::cout<<"Position (Initial) "<<init_position.head(6).transpose()<<std::endl;
        std::cout<<"Position (Local) "<<global_pos.transpose()<<std::endl;
        std::cout<<"Angle (Local) "<<global_yaw<<std::endl;
        Eigen::VectorXd new_ctrl;
        boost::shared_ptr<layer1::phen_t> temp_indiv;
        std::tie(new_ctrl,temp_indiv) = indiv->fit().ff(global_pos,global_yaw,leg_configuration,render,use_children,step);// grabbing the controllers at the lowest level
        global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, new_ctrl,global::dofs)); 
        std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);
        // std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_broken(std::vector<int>(1,4));
        global::simu.run(1); //global::simu->run(1.0, true, false);
        global::simulated_robot->remove_controller(0);
        global::simu.world()->setTime(0.0);//IMPORTANT!!!!!!
    } 

    
    
    if (stat) {
        // statistics - descriptor
        for (int i = 0; i < desc.size(); i++)
            global::ctrl_file << desc(i) << " ";
        global::ctrl_file << std::endl;
    }


    //#ifndef ROBOT
    // Resetting forces for stability
    //global::simulated_robot->skeleton()->setVelocities(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getVelocities().size()));
    //global::simulated_robot->skeleton()->setAccelerations(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getAccelerations().size()));
    //global::simulated_robot->skeleton()->clearExternalForces();
    //global::simulated_robot->skeleton()->clearInternalForces();

    

					    
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

	// global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, std::vector<double>(36, 0.0))); // nuetral position controller
    global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, Eigen::VectorXd(36).setZero(),global::dofs)); // nuetral position controller
	std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);
	global::simu.run(1.0) ; // global::simu.run(1.0, true, false);
	global::simulated_robot->remove_controller(0); 
        n++;
    }
    
    
    // reset to zero configuration for stability
    // global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, std::vector<double>(36, 0.0))); // nuetral position controller
    global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, Eigen::VectorXd(36).setZero(),global::dofs)); // nuetral position controller

    std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Ones(1) * ctrl_dt);
    global::simu.run(1.0); //global::simu->run(1.0, true, false);
    
    global::simulated_robot->remove_controller(0);

    //#else // FOR REAL ROBOT ONLY 
    // global::hexa->move(ctrl, t, !global::first);
    //std::thread t1 = std::thread(&hexapod_ros::Hexapod::move, global::hexa, ctrl, t, !global::first);
    //std::thread t2 = std::thread(check_collision, t);
    //t1.join();
    //t2.join();
    //global::first = true;

    //Eigen::Vector3d robot = get_tf("odom", "/base_link");
    //Eigen::Vector3d pos;
    //pos << robot(0), robot(1), 1.0;
    //pos = global::transform * pos;
					    //#endif 
    //#ifndef ROBOT
    double theta = pose(2);//pose(2);
    //#else
    //double theta = robot(2) + global::orig_theta;
    //#endif
    
    while (theta < -M_PI)
        theta += 2 * M_PI;
    while (theta > M_PI)
        theta -= 2 * M_PI;
    //#ifndef ROBOT
    global::robot_pose << pose(3), pose(4), theta;
    //#else
    //global::robot_pose << pos(0), pos(1), theta;
    //#endif
    
}// execute


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
    // std::cout << prev_pose.transpose() << " -> " << curr_pose.transpose() << " leads to: " << tr_pos.transpose() << std::endl;
    // Eigen::MatrixXd rr(2, 2);
    // c = std::cos(curr_pose(2));
    // s = std::sin(curr_pose(2));
    // rr << c, -s, s, c;
    // rr = r * rr;
    // double th = std::atan2(rr(1, 0), rr(0, 0));
    // // It's the same!
    // std::cout << th << " vs " << angle_dist(prev_pose(2), curr_pose(2)) << std::endl;
    std::ofstream ofs;
    ofs.open("exp/rte_v2_adapt/visualise_MCTS/get_x_y.dat", std::ios_base::app);
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
        int prior_map_index = global::prior_map_indices.back();
        global::latest_map_index = prior_map_index;
        return action->value() / (double(action->visits()) + _epsilon) + Params::active_learning::k() * Params::active_learning::scaling() * global::gp_models[prior_map_index].sigma(action->action()._desc);
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
    /*
#ifndef ROBOT
    // TO-DO: Fix visualization
    // Add target to simulation (arrow)
    VizParams::set_head(Eigen::Vector3d(goal_state(0), goal_state(1), 0.0));
    VizParams::set_tail(Eigen::Vector3d(goal_state(0), goal_state(1), 0.25));

    // Run dummy time to have something displayed
    global::simu->run(0.25);
#endif
    */
    global::target_num++;
    global::dimensions = svg::Dimensions(global::width, global::height);
    global::doc = std::make_shared<svg::Document>(save_path +"/plan_" + std::to_string(global::target_num) + ".svg", svg::Layout(global::dimensions, svg::Layout::TopLeft));

    // Initialize statistics
    global::robot_file.open(save_path + "/robot_" + std::to_string(global::target_num) + ".dat");
    global::ctrl_file.open(save_path + "/ctrl_" + std::to_string(global::target_num) + ".dat");
    global::iter_file.open(save_path +"/iter_" + std::to_string(global::target_num) + ".dat");
    global::misc_file.open(save_path +"/misc_" + std::to_string(global::target_num) + ".dat");

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
        auto tree = std::make_shared<mcts::MCTSNode<Params, HexaState<Params>, mcts::SimpleStateInit<HexaState<Params>>, mcts::SimpleValueInit, mcts::UCTValue<Params>, mcts::UniformRandomPolicy<HexaState<Params>, HexaAction<Params>>, HexaAction<Params>, mcts::SPWSelectPolicy<Params>, mcts::ContinuousOutcomeSelect<Params>>>(init, 20);

	    std::cout<< "compute MCTS"<<std::endl ;

        tree->compute(world, Params::iterations());
	
        std::cout<< "computed!"<<std::endl ;



	    auto time_running = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count();
        // std::cout << "Time in sec: " << time_running / 1000.0 << std::endl;

        std::cout << "Children: " << tree->children().size() << std::endl;
        double sum = 0;
        double max = -std::numeric_limits<double>::max();
        double min = std::numeric_limits<double>::max();
        //std::cout << "Child->value: "<< child->value() << std::endl;
	//std::cout << "Child->visits " << child->visits() << std::endl;
        for (auto child : tree->children()) {
            double v = child->value() / double(child->visits());
	    //std::cout << "value: " << child->value() << std::endl ;
	    //std::cout << "visit: " << child->visits() << std::endl ;
	    //std::cout << "max: " << max  << std::endl;
            //std::cout << "min: " << min  << std::endl;
	    //std::cout << "v: " << v  << std::endl;
	    sum += v;
	    //std::cout << "sum: " << sum  << std::endl;
            if (v > max)
                max = v;
            if (v < min)
                min = v;
        }

	//std::cout << "max: " << max  << std::endl;
	//std::cout << "min: " << min  << std::endl;
	//std::cout << "sigma_sq: " << Params::kernel_exp::sigma_sq()  << std::endl;
        Params::active_learning::set_scaling((max - min) / Params::kernel_exp::sigma_sq());
        std::cout << "Active learning scaling: " << Params::active_learning::scaling() << std::endl;

        // Get best action/behavior
        auto best = tree->best_action<Choose>();
	//std::cout<<"hereHH"<<std::endl;
	if(!best)
	  std::cout<<"ERROR"<<std::endl;
        //std::cout << "val: " << best->value() / double(best->visits()) << std::endl;
        auto other_best = tree->best_action<mcts::GreedyValue>();
        //std::cout << "val without AL: " << other_best->value() / double(other_best->visits()) << std::endl;
        // std::cout << "avg: " << (sum / double(tree->children().size())) << std::endl;
	//std::cout<<"hereHH2"<<std::endl;
        auto tmp = init.move(best->action(), true);
	//std::cout<<"hereHH3"<<std::endl;
        std::cout << tmp._x << " " << tmp._y << " -> " << tmp._theta << std::endl;
        global::iter_file << n << " " << time_running / 1000.0 <<" "<<global::time_random_search/1e6<<" "<<global::n_policies_called<<" "<<global::n_random_policies_called<<" "<<tree->children().size()<<" "<<tree->max_depth()<<" " << best->value() / double(best->visits()) << " " << other_best->value() / double(other_best->visits()) << " " << (sum / double(tree->children().size())) << " " << tmp._x << " " << tmp._y << " " << tmp._theta << std::endl;
        
        global::n_policies_called = 0;//reset of number of action sampling calls
        global::n_random_policies_called = 0;//reset of number of default policy calls for rollouts
        global::time_random_search = 0;
	std::cout << "Passed 1st round of MCTS" << std::endl ;
	
        // Execute in simulation/real robot
        Eigen::Vector3d prev_pose = global::robot_pose;

        //Running APROL to find the best action among all the prior maps
        std::cout<<"Proposed DESC: "<<best->action()._desc.transpose()<<std::endl;

        int prior_map_index = 0;
        Eigen::VectorXd final_desc;
        std::tie(prior_map_index,final_desc) = choose_prior(best->action()._desc); //checking which map to choose]
        global::latest_map_index = prior_map_index;
        std::cout<<"Chosen Map Index :"<<prior_map_index<<" aka Repertoire: "<<global::archive_files[prior_map_index]<<std::endl;
        Eigen::VectorXd full_desc(4); //with the angle theta

        std::vector<double> d(final_desc.size(), 0.0);
        Eigen::VectorXd::Map(d.data(), d.size()) = final_desc;
        full_desc << final_desc(0),final_desc(1),Params::archiveparams::archives[global::latest_map_index][d].cos_theta,Params::archiveparams::archives[global::latest_map_index][d].sin_theta;

        #ifndef HBR
            execute(final_desc, 3.0);
        #else
            hbr_execute(final_desc, 3.0);
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

	/*
	//hack
	auto hack =  get_x_y_theta(prev_pose, global::robot_pose);
	//hack
	*/
	
	//	std::cout << "here" << std::endl ;
        if (Params::learning()) {
            // Update GP (add_sample)
            Eigen::VectorXd observation(3),observation_2d(2);
	    //std::cout << "here"<< std::endl ;
            std::tie(observation(0), observation(1), observation(2)) = get_x_y_theta(prev_pose, global::robot_pose);
            observation_2d(0) = observation(0);
            observation_2d(1) = observation(1);

            Eigen::VectorXd data(4);
	    //std::cout<< "here" <<std::endl ;
            data(0) = observation(0);
            data(1) = observation(1);
            data(2) = std::cos(observation(2));
            data(3) = std::sin(observation(2));
	    //	    std::cout << "here" <<std::endl ;
            // Eigen::VectorXd test;
            // test = global::gp_model.mu(best->action()._desc);
            // std::cout << test(0) << " " << test(1) << " " << test(2) << " " << test(3) << std::endl;
            // global::gp_model.add_sample(best->action()._desc, data);

            int prior_map_index = global::prior_map_indices.back();
            //updating the gp for the map that we have last used
            global::latest_map_index = prior_map_index;

            global::gp_models[prior_map_index].add_sample(final_desc, data);
            
            update_ucb_posterior(full_desc,data);
            // update_ucb_posterior(final_desc,observation_2d);

            global::misc_file << data(0) << " " << data(1) << " " << data(2) << " " << data(3) << " ";
            // std::cout << observation(0) << " " << observation(1) << " " << std::cos(observation(2)) << " " << std::sin(observation(2)) << std::endl;
            // std::cout << observation(2) << std::endl;
            // std::vector<double> d;
            // for (size_t i = 0; i < best->action()._desc.size(); i++) {
            //     d.push_back(best->action()._desc[i]);
            // }
            // auto vvv = Params::archiveparams::archive[d];
            // std::cout << vvv.x << " " << vvv.y << " " << vvv.cos_theta << " " << vvv.sin_theta << " -> " << std::atan2(vvv.sin_theta, vvv.cos_theta) << std::endl;
            // std::cout << "----------------------------" << std::endl;
            // std::cout << observation(0) << " " << observation(1) << " " << observation(2) << std::endl;
            write_gp("gp_"+ std::to_string(prior_map_index) +"_" + std::to_string((global::gp_models[prior_map_index].samples().empty()) ? 0 : global::gp_models[prior_map_index].nb_samples()) + ".dat");
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

    if (n < max_iter && !collided)
        return std::make_tuple(true, n, Params::collisions());

    if (n < max_iter)
        return std::make_tuple(true, n, Params::collisions());

    return std::make_tuple(false, n, Params::collisions());
} //reach_targets



std::vector<Eigen::Vector3d> generate_targets(const Eigen::Vector2d& start, const Eigen::Vector2d& map_size, double dist, size_t N = 50)
{
    static tools::rgen_double_t rgen_x(Params::cell_size(), map_size(0));
    static tools::rgen_double_t rgen_y(Params::cell_size(), map_size(1));

    Eigen::Vector2d s = start;
    std::vector<Eigen::Vector3d> targets;
    for (size_t i = 0; i < N; i++) {
        Eigen::Vector2d t(rgen_x.rand(), rgen_y.rand());
        while (std::abs((s - t).norm() - dist) > Params::cell_size() / 5.0 || collides(t(0), t(1), 2.0 * Params::robot_radius())) {
            t << rgen_x.rand(), rgen_y.rand();
        }

        targets.push_back(Eigen::Vector3d(t(0), t(1), 0.0));
        s = t;
    }

    return targets;
} //generate_targets


//Eigen::Vector3d init_map(const std::string& map_string)
std::tuple<std::vector<Eigen::Vector3d>, Eigen::Vector3d> init_map(const std::string& map_string)
{
    // Init obstacles
    double path_width = 0.5;  //Params::cell_size();
    double i_x = 0, i_y = 0, i_th = 0; // i_x, i_y, i_th is the initial(starting) position and orientation of the robot
    std::map<int, Eigen::Vector3d> goals;
    bool goal_in_map = false;
    bool init_in_map = false;
    double max_x = 0, max_y = 0;
    int r = 0, c = 0;

    // for loop through the entire map (which has been made a string)
    for (size_t i = 0; i < map_string.size(); i++) {
        if (map_string[i] == '\n') {
            r++;
            if (i < map_string.size() - 1) {
                c = 0;
            }
            continue;
        }

        double x = path_width * r;
        double y = path_width * c;

        if (x > max_x)
            max_x = x;
        if (y > max_y)
            max_y = y;

        if (map_string[i] == '*') { // '*' represents the walls/obstacles/boundaries in the map
            global::obstacles.push_back(SimpleObstacle(x, y, path_width / 2.0));
        }
        else if (map_string[i] == '^') { // represents the starting orientiation of the robot - looking upwards (north)
            i_x = x;
            i_y = y;
            i_th = dart::math::constants<double>::pi();
            init_in_map = true;
        }
        else if (map_string[i] == 'v') { // represent the starting orientation of the robot - looking south
            i_x = x;
            i_y = y;
            i_th = 0;
            init_in_map = true;
        }
        else if (map_string[i] == '<') { // starting position and orientation of the robot - facing west (-pi)
            i_x = x;
            i_y = y;
            i_th = -dart::math::constants<double>::half_pi();
            init_in_map = true;
        }
        else if (map_string[i] == '>') { // starting position and orientation of the robot - facing east (+pi)
            i_x = x;
            i_y = y;
            i_th = dart::math::constants<double>::half_pi();
            init_in_map = true;
        }
        else if (map_string[i] != ' ') { // if not empty then the numbers or the rest of the string/figures and the goals
            int t = map_string[i] - '0';
            goals[t] = Eigen::Vector3d(x, y, 0);
            goal_in_map = true;
        }
        c++;
    }

    global::height = 768;
    global::width = static_cast<size_t>(double(c) / double(r) * 768);

    //overwriting this given the fixed map we use
    r = 9;
    c = 8;
    global::map_size = (r > c) ? r - 1 : c - 1;
    global::map_size_x = r;
    global::map_size_y = c;
    global::entity_size = static_cast<size_t>(((global::height > global::width) ? global::width : global::height) / double(global::map_size * 0.5));


    std::cout<<global::width<<"  "<<global::map_size<<" "<<global::map_size_x<<"  "<<global::map_size_y<<std::endl;

    if (!goal_in_map || !init_in_map) {
        std::cerr << "No goal or robot in the map." << std::endl;
        exit(1);
    }

    size_t N = 1;
    std::cout<<goals.size()<<"  "<<Eigen::Vector2d(goals[1](0), goals[1](1)).transpose()<<std::endl;


    std::cout << "init_map complete" << std::endl;

    std::vector<Eigen::Vector3d> g = generate_targets(Eigen::Vector2d(i_x, i_y),
						      Eigen::Vector2d((r - 1) * Params::cell_size(), (c - 1) * Params::cell_size()),
						      (Eigen::Vector2d(i_x, i_y) - Eigen::Vector2d(goals[1](0), goals[1](1))).norm(), N);

    
    return std::make_tuple(g, Eigen::Vector3d(i_x, i_y, i_th));
    //return Eigen::Vector3d(i_x, i_y, i_th);
} //init_map

void init_simu()
{
  //to do add damages

  //Damages

  //-----------------------------init_simu-------------------------------------------------
    std::cout<<"INIT Robot"<<std::endl;
    global::global_robot = std::make_shared<robot_dart::Robot>("exp/rte_v2_adapt/resources/hexapod_v2.urdf");

    global::global_robot->set_position_enforced(true);

    // global::global_robot->set_actuator_types(dart::dynamics::Joint::SERVO);
    global::global_robot->set_actuator_types("servo");

    

    //Damages

    //need this for the controllers so that we can use the damages!!
    auto full_dofs = global::global_robot->dof_names(true, true, true);  //controllable + not controllable
    global::dofs = std::vector<std::string>(full_dofs.begin() + 6, full_dofs.end());  //extract only the controllable params, we need this for the damage to work!!

    // //Locking at certain Position!!
    // global::global_robot->skeleton()->getJoint("leg_4_1_2")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
    // // global::global_robot->skeleton()->getJoint("leg_4_2_3")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
    // global::global_robot->set_actuator_type("locked", "leg_4_1_2");
    // global::global_robot->set_actuator_type("locked", "leg_4_2_3");
    // global::global_robot->set_actuator_type("locked", "body_leg_4");
    if(global::damaged_leg>=0 && global::damaged_leg<=5){
        global::global_robot->skeleton()->getJoint("leg_"+std::to_string(global::damaged_leg)+"_1_2")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
        //  global::global_robot->skeleton()->getJoint("leg_"+std::to_string(global::damaged_leg)+"_2_3")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
        // global::global_robot->skeleton()->getJoint("leg_4_2_3")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
        global::global_robot->set_actuator_type("locked", "leg_"+std::to_string(global::damaged_leg)+"_1_2");
        global::global_robot->set_actuator_type("locked", "leg_"+std::to_string(global::damaged_leg)+"_2_3");
        global::global_robot->set_actuator_type("locked", "body_leg_"+std::to_string(global::damaged_leg));
        // global::global_robot->set_actuator_type("passive", "leg_"+std::to_string(global::damaged_leg)+"_1_2");
        // global::global_robot->set_actuator_type("passive", "leg_"+std::to_string(global::damaged_leg)+"_2_3");
        // global::global_robot->set_actuator_type("passive", "body_leg_"+std::to_string(global::damaged_leg));
    }
    if(global::damaged_leg==6){
        global::global_robot->skeleton()->getJoint("leg_"+std::to_string(4)+"_1_2")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
        //  global::global_robot->skeleton()->getJoint("leg_"+std::to_string(global::damaged_leg)+"_2_3")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
        // global::global_robot->skeleton()->getJoint("leg_4_2_3")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
        global::global_robot->set_actuator_type("locked", "leg_"+std::to_string(4)+"_1_2");
        global::global_robot->set_actuator_type("locked", "leg_"+std::to_string(4)+"_2_3");
        global::global_robot->set_actuator_type("locked", "body_leg_"+std::to_string(4));

        global::global_robot->skeleton()->getJoint("leg_"+std::to_string(1)+"_1_2")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
        //  global::global_robot->skeleton()->getJoint("leg_"+std::to_string(global::damaged_leg)+"_2_3")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
        // global::global_robot->skeleton()->getJoint("leg_4_2_3")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
        global::global_robot->set_actuator_type("locked", "leg_"+std::to_string(1)+"_1_2");
        global::global_robot->set_actuator_type("locked", "leg_"+std::to_string(1)+"_2_3");
        global::global_robot->set_actuator_type("locked", "body_leg_"+std::to_string(1));
    }

    // global::global_robot->skeleton()->getJoint("leg_1_1_2")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
    // // global::global_robot->skeleton()->getJoint("leg_4_2_3")->setInitialPositions(tools::make_vector({-M_PI_2})); //removing leg entirely
    // global::global_robot->set_actuator_type("locked", "leg_1_1_2");
    // global::global_robot->set_actuator_type("locked", "leg_1_2_3");
    // global::global_robot->set_actuator_type("locked", "body_leg_1");

    // //Disabling Servo Motor entirely
    // global::global_robot->set_actuator_type("passive", "leg_4_1_2");
    // global::global_robot->set_actuator_type("passive", "leg_4_2_3");
    // global::global_robot->set_actuator_type("passive", "body_leg_4");



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


    //Obstacle(corner_1=(-0.2, 3.06), corner_2=(0.75, 4.0)),  # corner obstacle
        // Obstacle(corner_1=(2.2, -0.3), corner_2=(2.7, 0.625)),  # last obstacle
        // Obstacle(corner_1=(1.73, 1.84), corner_2=(2.73, 2.34)),  # Intermediate obstacle
        // Obstacle(corner_1=(2.73, 1.84), corner_2=(3.23, 2.84)),
        // Obstacle(corner_1=(3.15, 4.04), corner_2=(4.5, 1.84)),
    

    //Bryan & Luca Map
    global::box_obstacles.push_back(BoxObstacle (-0.2,3.06,0.75,4.0));
    global::box_obstacles.push_back(BoxObstacle (2.2, -0.3,2.7, 0.625));
    global::box_obstacles.push_back(BoxObstacle (1.73, 1.84,2.73, 2.34));
    global::box_obstacles.push_back(BoxObstacle (2.73, 1.84,3.23, 2.84));
    global::box_obstacles.push_back(BoxObstacle (3.15, 1.84, 4.5, 4.04));

    //Maxime Addition
    
    // global::box_obstacles.push_back(BoxObstacle (1.0,1.0,1.73,2.34));
    // global::box_obstacles.push_back(BoxObstacle (1.0,0.5,1.5,2.34));
    // global::box_obstacles.push_back(BoxObstacle (1.1,1.0,1.5,2.34));



    // boundaries [-0.2, 4.5, -0.3, 4.0] //center of box is used to place the box!
    // boxes_obstacles.push_back(BoxObstacle (0.0, 0.0,4.5, -0.3));
    // boxes_obstacles.push_back(BoxObstacle (0.0, 0.0, -0.2, 4.0));
    global::box_obstacles.push_back(BoxObstacle (-0.2, -0.31,4.5, -0.3));
    global::box_obstacles.push_back(BoxObstacle (-0.21, -0.3, -0.2, 4.0));
    global::box_obstacles.push_back(BoxObstacle (4.5, -0.3, 4.51, 4.0));
    global::box_obstacles.push_back(BoxObstacle (-0.2, 4.0, 4.5, 4.01));



    int num = 1;
    for (auto obs : global::box_obstacles) {
        std::cout<<obs.size().transpose()<<"  "<<obs.pose().transpose()<<std::endl;
        global::simu.add_robot(robot_dart::Robot::create_box(obs.size(), obs.pose(), "fixed", 1., dart::Color::Red(1.0), "box_" + std::to_string(num)));
        num++;
    }
    // for (auto obs : global::obstacles) {
    //     Eigen::Vector6d obs_pos;

    //     // new implementation using robot_dart - ask first
    //     obs_pos << 0, 0, 0, obs._x, obs._y, 0.0;
    //     global::simu.add_robot(robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(obs._radius * 2.05, obs._radius * 2.05, obs._radius * 2.05),obs_pos, "fixed",1., dart::Color::Blue(1.0)));
    //     //obs_pos << 0, 0, 0, obs._x, obs._y, obs._radius / 2.0;
    //     //global::simu.add_robot(robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(obs._radius * 2.05, obs._radius * 2.05, obs._radius * 2.05),obs_pos, "fixed",1., dart::Color::Blue(1.0)));
    //     //obs_pos << 0, 0, 0, obs._x, obs._y, obs._radius;
    //     //global::simu.add_robot(robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(obs._radius * 2.05, obs._radius * 2.05, obs._radius * 2.05),obs_pos, "fixed",1., dart::Color::Blue(1.0)));
    //     //obs_pos << 0, 0, 0, obs._x, obs._y, 3.0 * obs._radius / 2.0;
    //     //global::simu.add_robot(robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(obs._radius * 2.05, obs._radius * 2.05, obs._radius * 2.05),obs_pos, "fixed",1., dart::Color::Blue(1.0)));
    // }
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
//BO_DECLARE_DYN_PARAM(double, Params::kernel_exp, sigma_sq);
//BO_DECLARE_DYN_PARAM(double, Params::kernel_exp, l);


Params::archiveparams::archive_t Params::archiveparams::archive;
std::vector<Params::archiveparams::archive_t> Params::archiveparams::archives;
/*
void simulate(const std::vector<double> ctrl)  
{
    //initialise simulation (init_simu)
    double ctrl_dt = 0.015;
    global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, ctrl));
    std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controllers()[0])->set_h_params(std::vector<double>(1, ctrl_dt));
    //robot_dart::RobotDARTSimu simu(ctrl_dt);
    global::simu.run(7); // run the simulation for 5 seconds
    
    //obtain required data, fitness, descriptors
    // for final position of the robot. not need for this as descriptor is already the x and y position already b
    //auto pos=simu.robots().back()->skeleton()->getPositions().head(6).tail(3).cast <float>(); 
    g_robot.reset(); 
}
*/ 
/*
void visualise_behaviour(std::string path){
  Interactive_map<Params> imap;
  std::tuple<std::vector<std::vector<double>>, std::vector<double>, std::vector<std::vector<double>> > solutions = imap.visualise_map(path);
  size_t num_solutions = std::get<0>(solutions).size();

  for (int i = 0; i < num_solutions; i++) {
    std::cout << std::endl;
    std::cout << "Solution " << i + 1 << ": " << std::endl;

    std::cout << "Behaviour descriptor: ";
    for (double d : std::get<0>(solutions)[i])
      std::cout << d << " ";
    std::cout << endl;
    
    std::cout << "Performance: " << std::get<1>(solutions)[i];

    std::stringstream ss;
    std::vector<double> ctrl;
    for (double d : std::get<2>(solutions)[i]){
      ss << d << " ";
      ctrl.push_back(std::round(d * 100.0 ) / 100.0);
    }
    std::cout << std::endl;
    std::cout << "Controller: " << ss.str() << std::endl;
    simulate(ctrl); 
    	
  }
  
}
*/



int main(int argc, char **argv)
{   

    // if(DIM == 18)
    //     std::cout<<"HERE"<<std::endl;
    //     std::cout<<DIM<<std::endl;
    // if(DIM == 19)
    //     std::cout<<"HERE"<<std::endl;

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

    if (vm.count("rep")) {
      num = vm["rep"].as<int>();
      std::cout<<num<<std::endl;
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

    std::cout<<"SAVING TO "<<save_path<<std::endl;
    std::cout<<"Damaging Leg "<<global::damaged_leg<<std::endl;

  //------------------------------------------- READING IN THE MAPFILE(.TXT) AND CONVERTING INTO A STRING FORM---------------------------------------//
    std::string map_string = ""; //map_string used as input to the function init_map - u
    std::string map_file = "";

    map_file = "exp/rte_v2_adapt/test_maps/map_hard.txt" ;
    std::ifstream t(map_file); // t is now the object that containes the map_file - must #include <fstream> to use this
    if (!t.is_open() || !t.good()) { //is_open function used to check if a file is open. good: check whether state of stream is good. Therefore, if NOT (logical oprator for not is !) open or NOT good, return/print error
      std::cerr << "Exception while reading the map file: " << map_file << std::endl;
      return 1;
    } // if the file is open and good, read in file by converting the map file into string form

    map_string = std::string((std::istreambuf_iterator<char>(t)),std::istreambuf_iterator<char>());

    //------------------------------------------------LOAD ARCHIVE FILE (BEHAVIOURAL REPERTOIRE------------------------------------------------//
    std::string archive_file = "";

    
    namespace fs = boost::filesystem;
    // std::string path = "exp/rte_v2_adapt/resources/aprol";
    std::string path;
    if(archive_nr.empty())
        {
            path = "exp/rte_v2_adapt/resources/aprol_repertoires/aprol_repertoire/aprol_repertoire_1";
        }
    else{
            path = "exp/rte_v2_adapt/resources/aprol_repertoires/aprol_repertoire/aprol_repertoire_" + archive_nr;
    }
    std::cout<<"Files: "<<std::endl;
    for (const auto & entry : fs::directory_iterator(path)){
        std::cout << entry.path().string() << std::endl;
        // archive_file = "exp/rte_v2_adapt/resources/archive_files/archive_rte_19500.dat"; 
        archive_file = entry.path().string();//"exp/rte_v2_adapt/resources/archive_files/archive_2400_2d.dat";
        global::archive_files.push_back(archive_file);
    }
    
    if (!global::archive_files.empty()) {
        // Loading archive

        std::vector<Params::archiveparams::archive_t> archives;
        std::cout << "Loading archive..." << std::endl;
        for(int i=0; i<global::archive_files.size();i++){
            // Params::archiveparams::archive = load_archive(archive_file);
            archives.push_back(load_archive(global::archive_files[i])); 
            
        }
        Params::archiveparams::archives = archives;
    }

    //initializing all our vectors for the UCB algorithm
    global::prior_map_trials.resize(Params::archiveparams::archives.size(),1);
    global::prior_map_means.resize(Params::archiveparams::archives.size(),0.6);
    //initialize all our GPs, one for each map
    for (int i=0;i<Params::archiveparams::archives.size();i++){
        global::latest_map_index = i; //we need to use another archive for each mean prediction of the network
        global::gp_models.push_back(global::GP_t(ARCHIVE_SIZE, 4));
    }

    global::latest_map_index = 0;
    global::prior_map_indices.push_back(0);//Using Map 1 as first map
    global::prior_map_trials[0] += 1;
    
    std::cout<<"Initialized GPs for "<<Params::archiveparams::archives.size()<<" Maps"<<std::endl;

    Params::uct::set_c(100.0);
    Params::spw::set_a(0.5);
    Params::cont_outcome::set_b(0.6);
    if(ARRAY_DIM==8){
        Params::set_iterations(5000);
    }
    else{
        Params::set_iterations(6000);
    }
    
    
    // Params::mcts_node::set_parallel_roots(1);
    Params::mcts_node::set_parallel_roots(32);
    Params::active_learning::set_k(0.0);
    //Params::kernel_exp::set_sigma_sq(0.5);
    //Params::kernel_exp::set_l(0.03);
    Params::set_learning(true);
    
    //----------------------------------------------INITIALISING STARTING POSITION AND ORIENTATION OF THE ROBOT----------------------------------------------------//
    Eigen::Vector3d robot_state; // starting position of robot
    std::vector<Eigen::Vector3d> goal_states;
    std::tie(goal_states, robot_state) = init_map(map_string); //robot_state = init_map(map_string);

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

    //-----------------------------------INITIALIZE CONTROLLER OF THE ROBOT------------------------------------//
    /*  
    // g_robot->add_controller(std::make_shared<robot_dart::SimpleControl>());
    // Use the standard gait 36 motor parameters
    //std::vector<double> ctrl = {1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5};
    std::vector<double> ctrl = {0.73576, 0.540947, 0.725605, 0.556359, 0.449956, 0.50308, 0.995091, 0.943734, 0.670678, 0.11971, 0.552994, 0.776107, 0.798726, 0.719629, 0.323305, 0.472693, 0.792632, 0.242074, 0.799107, 0.503354, 0.328196, 0.314048, 0.418371, 0.492261, 0.991423, 0.0675634, 0.360108, 0.904968, 0.804568, 0.392786, 0.903756, 0.321555, 0.489208, 0.0764786, 0.0424173, 0.620316};
      
    double ctrl_dt = 0.015;
    global::simulated_robot->add_controller(std::make_shared<robot_dart::control::HexaControl>(ctrl_dt, ctrl));
    std::static_pointer_cast<robot_dart::control::HexaControl>(global::simulated_robot->controller(0))->set_h_params(std::vector<double>(1, ctrl_dt));
    */
    
    //------------------------------------SETUP SIMULATION ENVIRONMENT---------------------------------------//
    //robot_dart::RobotDARTSimu simu(0.001); // moved into namespace global

#ifdef GRAPHIC
      //   auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(&simu);
      //    graphics->set_enable(true);
      //    simu.set_graphics(graphics);
    // global::simu.set_graphics(std::make_shared<robot_dart::gui::magnum::Graphics>(global::simu.world(),640,480,false));
    // auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(&global::simu);
    std::cout<<"Graphics Starting..."<<std::endl;
    // auto graphics = std::make_shared<robot_dart::gui::magnum::WindowlessGraphics>(&global::simu);
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(&global::simu);
    // auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics<>>(global::simu.world(),640,480,false);
    
    
    std::cout<<"Graphics Set"<<std::endl;
    graphics->set_enable(true);
    // global::simu.set_graphics(std::make_shared<robot_dart::graphics::Graphics>(global::simu.world(),640,480,false));
    
    
    // graphics->record_video(save_path + std::string("/hexapod_rte.mp4"), 30);

    global::simu.set_graphics(graphics);

    std::static_pointer_cast<robot_dart::gui::magnum::Graphics>(global::simu.graphics())->look_at({6.0, 2.5, 5}, {2.5, 2.5, 0.});
    // std::static_pointer_cast<robot_dart::gui::magnum::WindowlessGraphics>(global::simu.graphics())->look_at({6.0, 2.5, 5}, {2.5, 2.5, 0.});

#endif

    global::simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
    global::simu.add_floor(50);
    init_simu_world(robot_state);

    // Build stairs - first argument is the name/label for the box, x_pos, y_pos, size_x, size_y, size_z
    //global::simu.add_robot(box(0, 2.0, 2, 0.4, 0.8, 0.3));
    //global::simu.add_robot(box(1, 2.4, 2, 0.4, 0.8, 0.6));
    //global::simu.add_robot(box(2, 2.8, 2, 0.4, 0.8, 0.9));

    global::simu.add_robot(global::simulated_robot);

    /*
    if(argc==2) {
      visualise_behaviour(argv[1]);
      global::simulated_robot.reset();
      return 0;
    }
    */
    // /*
    std::cout << "Robot starting: " << robot_state.transpose() << "\nGoals: \n";
    for (auto g : goal_states)
        std::cout << g.transpose() << std::endl;
    std::cout << "--------------------------" << std::endl;
    // */
    
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
	  
          // #ifndef ROBOT
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

          //global::simu->controller().set_parameters(std::vector<double>(36, 0.0));
	  //global::simu.run(1.0) ; //global::simu->run(1.0, true, false);
          // #endif

          global::robot_pose << state(0), state(1), robot_state(2);

	}
	std::cout<< "reach_target function"<<std::endl ;
    auto t1 = std::chrono::steady_clock::now();
	std::tie(found, n_iter, n_cols) = reach_target(goal_states[i], 80,save_path);
    auto time_reach_target = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count();
	results_file << found << " " << n_iter << " " << n_cols <<" "<<time_reach_target/1000<< std::endl;
    }
    results_file.close();
    
    

    
    //global::simu.run(10);


    // Print the final position of the robot
    std::cout << global::simulated_robot->skeleton()->getPositions().head(6).tail(3).transpose() << std::endl;

    global::simulated_robot.reset();
    global::global_robot.reset();
    return 0;

    //g_robot is the clone - has becomes globall::simulated_robot
}

    












