#ifndef HBR_LOADING
#define HBR_LOADING

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

#include <hbr/Leg/fit_hexa_leg.hpp>
#include <hbr/Body_Acc/fit_hexa_body_gp.hpp>

#ifdef BASE_HBR
  #include <hbr/Omnidirectional/fit_hexa_body.hpp>
#else
  #include <hbr/Omnidirectional/fit_mt_me.hpp>
#endif

#include <hbr/HBR/quality_diversity_hbr.hpp>


bool load_hbr(std::string path,layer0::qd_t qd1,layer1::qd_t qd2,layer2::qd_t qd3)
{
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
    
    pop_t1 pop_leg = qd1.get_pop_from_gen_file(path+"_l1");
    std::cout<<"Layer 1 Loaded: "<<layer0::layer_ptr->container().archive().size()<<std::endl;
    pop_t2 pop_body = qd2.get_pop_from_gen_file(path+"_l2");
    std::cout<<"Layer 2 Loaded: "<<pop_body.size()<<"  "<<layer1::layer_ptr->container().archive().size()<<std::endl;
    pop_t3 pop_omni = qd3.get_pop_from_gen_file(path+"_l3");
    std::cout<<"Layer 3 Loaded: "<<pop_omni.size()<<"  "<<layer2::layer_ptr->container().archive().size()<<std::endl;
    // std::cout<<"Updating GPs"<<std::endl;
    sferes::modif::GP_Update<typename layer2::phen_t,layer2::Params> gp_modif;
    gp_modif.force(qd3);
    std::cout<<"Fitting GPs done"<<std::endl;

    return true;
}

#endif