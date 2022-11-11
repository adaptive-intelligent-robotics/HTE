#include <iostream>
#include <algorithm>
#include <unistd.h>

#include <boost/process.hpp>
#include <boost/program_options.hpp>

#include <sferes/eval/parallel.hpp>
#include <sferes/gen/evo_float.hpp>
#include <sferes/modif/dummy.hpp>
#include <sferes/phen/parameters.hpp>
#include <sferes/run.hpp>
#include <sferes/stat/best_fit.hpp>
#include <sferes/stat/qd_container.hpp>
#include <sferes/stat/qd_selection.hpp>
#include <sferes/stat/qd_progress.hpp>

#include <sferes/fit/fit_qd.hpp>
#include <sferes/qd/container/archive.hpp>
#include <sferes/qd/container/kdtree_storage.hpp>
#include <sferes/qd/quality_diversity.hpp>
#include <sferes/qd/selector/value_selector.hpp>
#include <sferes/qd/selector/score_proportionate.hpp>

#include <robot_dart/gui/magnum/base_application.hpp>

#define NUM_THREADS 48


#include "./Leg/fit_hexa_leg.hpp"
#include "./Body_Acc/fit_hexa_body_gp.hpp"
#include "./Omnidirectional/fit_mt_me.hpp"
#include "HBR/quality_diversity_hbr.hpp"

namespace visu {

  struct Arguments {
    std::string path_gen_file;
    std::string path_gen_file_leg;
    
    std::string path_gen_file_l1;
    std::string path_gen_file_l2;
    std::string path_gen_file_l3;
    int full;
    size_t index;
    std::string name_video;
    int leg;
    int leg_config;
  };

  void get_arguments(const boost::program_options::options_description &desc, Arguments &arg, int argc, char **argv) {
    // For the moment, only returning number of threads
    boost::program_options::variables_map vm;
    boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options(
      desc).allow_unregistered().run();

    boost::program_options::store(parsed, vm);
    boost::program_options::notify(vm);
    arg.path_gen_file = vm["path"].as<std::string>();
    arg.path_gen_file_l1 = vm["path"].as<std::string>()+"_l1";
    arg.path_gen_file_l2 = vm["path"].as<std::string>()+"_l2";
    arg.path_gen_file_l3 = vm["path"].as<std::string>()+"_l3";
    arg.path_gen_file_leg = vm["leg_path"].as<std::string>();
    arg.index = vm["index"].as<size_t>();
    arg.name_video = vm["video"].as<std::string>();
    arg.leg = vm["leg"].as<size_t>();
    arg.full = vm["full"].as<size_t>();
    arg.leg_config = vm["leg_config"].as<size_t>();

  }
}

int main(int argc, char **argv) {
  boost::program_options::options_description desc;
  desc.add_options()("path", boost::program_options::value<std::string>(), "Set path of proj file (serialised gen file)");
  desc.add_options()("leg_path", boost::program_options::value<std::string>()->default_value(""), "Set path of proj file for legs (serialised gen file)");
  desc.add_options()("index", boost::program_options::value<size_t>(), "Index of behaviour to show in that file");
  desc.add_options()("video", boost::program_options::value<std::string>(), "Name video to save without the extension");
  desc.add_options()("leg", boost::program_options::value<size_t>()->default_value(-1), "Leg Number, if present then we do a leg simulation");
  desc.add_options()("full", boost::program_options::value<size_t>()->default_value(0), "Using all gen file if full>0");
  desc.add_options()("leg_config", boost::program_options::value<size_t>()->default_value(0), "Use n-nearest neighbour for run");


  visu::Arguments arg{};
  visu::get_arguments(desc, arg, argc, argv);

  srand(time(0));

  tbb::task_scheduler_init init(32);

  std::string name_video,name_trajectory;
  if(arg.leg<0)
    name_video = arg.name_video + '-' + std::to_string(arg.index) + ".mp4";
  else
    name_video = arg.name_video + '-' + std::to_string(arg.index)+ '-' + std::to_string(arg.leg) + ".mp4";

  layer0::qd_t qd1;
  layer1::qd_t qd2;
  layer2::qd_t qd3;

  layer0::layer_ptr = &qd1;
  layer1::layer_ptr = &qd2;
  layer2::layer_ptr = &qd3;


  sferes::stat::QdMapTraj<typename layer1::phen_t,layer1::Params> trajectory_stat;
  sferes::stat::QdMT<typename layer2::phen_t,layer2::Params> mt_stat;

  sferes::modif::GP_Update<typename layer2::phen_t,layer2::Params> gp_modif;
  
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

  // std::cout<<"HERE"<<std::endl;

  qd1.set_fit_proto(layer0::phen_t::fit_t());
  qd2.set_fit_proto(layer1::phen_t::fit_t());
  qd3.set_fit_proto(layer2::phen_t::fit_t());
  pop_t1 pop_leg = qd1.get_pop_from_gen_file(arg.path_gen_file_l1);

  //restore archive as well for the legs to get controllers

  std::cout<< layer0::layer_ptr->container().archive().size()<<std::endl;
  pop_t2 pop_body = qd2.get_pop_from_gen_file(arg.path_gen_file_l2);
  std::cout<<pop_body.size()<<std::endl;


  pop_t3 pop_omni = qd3.get_pop_from_gen_file(arg.path_gen_file_l3);
  std::cout<<pop_omni.size()<<std::endl;
  // std::pair<std::vector<Eigen::VectorXf>, double >
  if(arg.full>0){

    std::cout<<"Updating GPs"<<std::endl;
    gp_modif.force(qd3);
  
    std::cout<<pop_body[arg.index]->fit().get_samples().size()<<std::endl;

    std::cout<<"Fitting GPs done"<<std::endl;

    auto data = pop_omni[arg.index]->fit().simulate(*pop_omni[arg.index],true,name_video,arg.leg_config,true,3);
    std::vector<double> obs_desc = pop_omni[arg.index]->fit().virtual_desc(data);
    std::cout<<"Age: "<<pop_omni[arg.index]->fit().age()<<std::endl;
    std::cout<<"Steps: "<<pop_omni[arg.index]->fit().effective_steps()<<std::endl;

    for(int i=0;i< pop_omni[arg.index]->data().size();i++)
      std::cout<<pop_omni[arg.index]->data(i)<<" ";
    std::cout<<"\nBD"<<std::endl;

    for(int i=0;i< pop_omni[arg.index]->fit().desc().size();i++)
      std::cout<<pop_omni[arg.index]->fit().desc()[i]<<" ";
    std::cout<<" "<<std::endl;
      
    for(int i=0;i< pop_omni[arg.index]->fit().desc().size();i++)
      std::cout<<obs_desc[i]<<" ";
    
    double bd_error  = std::sqrt( std::pow(pop_omni[arg.index]->fit().desc()[0]-obs_desc[0],2) + 
                                                        std::pow(pop_omni[arg.index]->fit().desc()[1]-obs_desc[1],2));
    std::cout<<"\n"<<bd_error<<std::endl;

    for(auto conf: pop_omni[arg.index]->fit().obs_descs()){
      for(auto bd: conf){
        std::cout<<bd<<" ";
      }
      bd_error  = std::sqrt( std::pow(pop_omni[arg.index]->fit().desc()[0]-conf[0],2) + 
                                                        std::pow(pop_omni[arg.index]->fit().desc()[1]-conf[1],2));
      std::cout<<bd_error<<std::endl;
    }


    for(int i=0;i<pop_omni[arg.index]->fit().children().size();i++){
      for(auto d:pop_omni[arg.index]->fit().children()[i]->fit().desc()){
        std::cout<<d<<" ";
      }
      std::cout<<std::endl;
    }

    std::cout<<"Waypoints"<<std::endl;
    for(auto conf: pop_omni[arg.index]->fit().waypoints()){
      for(auto point: conf){
        std::cout<<point<<" ("<<(point+1.5)/3<<") ";
      }
      std::cout<<std::endl;
    }
    
  }
  else{
    Eigen::VectorXd init_vel(24),init_acc(24),state(18);//initializing the new velocity state
    init_vel.setZero(24);
    init_acc.setZero(24);

    pop_body[arg.index]->fit().simulate(*pop_body[arg.index],init_vel,init_acc,true,name_video);

    for(int i=0;i< pop_body[arg.index]->data().size();i++)
      std::cout<<pop_body[arg.index]->data(i)<<std::endl;
    std::cout<<"BD \n"<<std::endl;

    for(int i=0;i< pop_body[arg.index]->fit().desc().size();i++)
      std::cout<<pop_body[arg.index]->fit().desc()[i]<<std::endl;
      
    name_trajectory =  arg.name_video + '-' + std::to_string(arg.index) + ".png";
    trajectory_stat.write_progress(name_trajectory,pop_body[arg.index]);

  }

  return 0;
}

