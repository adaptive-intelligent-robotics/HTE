//| This file is a part of the sferes2 framework.
//| Copyright 2016, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

#include <iostream>

#define NUM_THREADS 32
#include <sys/stat.h>

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
#include <sferes/qd/container/grid.hpp>
#include <sferes/qd/container/archive.hpp>
#include <sferes/qd/container/kdtree_storage.hpp>
#include <sferes/qd/container/sort_based_storage.hpp>
#include <sferes/qd/quality_diversity.hpp>
#include <sferes/qd/selector/tournament.hpp>
#include <sferes/qd/selector/uniform.hpp>

#include "./Leg/fit_hexa_leg.hpp"
#include "./Body_Acc/fit_hexa_body_gp.hpp"
#include "./Omnidirectional/fit_mt_me.hpp"

#include "HBR/HBR_Seq.hpp"


using namespace sferes::gen::evo_float;

struct Params {
    struct nov {
        SFERES_CONST size_t deep = 3;
        SFERES_CONST double l = 0.01; 
        SFERES_CONST double k = 15; 
        SFERES_CONST double eps = 0.1;
    };
  
    struct pop {
        // number of initial random points
        SFERES_CONST size_t init_size = 1000;
        // size of a batch
        SFERES_CONST size_t size = 200;
        SFERES_CONST size_t nb_gen = 801;
        SFERES_CONST size_t dump_period = 100;
    };
    struct parameters {
      SFERES_CONST float min = 0.0;
      SFERES_CONST float max = 1.0;
    };
    struct evo_float {
        SFERES_CONST float cross_rate = 0.0f;
        SFERES_CONST float mutation_rate = 0.03f;
        SFERES_CONST float eta_m = 10.0f;
        SFERES_CONST float eta_c = 10.0f;
        SFERES_CONST mutation_t mutation_type = polynomial;
        SFERES_CONST cross_over_t cross_over_type = sbx;
    };
    struct qd {
        SFERES_CONST size_t dim = 2;
        SFERES_CONST size_t behav_dim = 2;
        SFERES_ARRAY(size_t, grid_shape, 100, 100);
    };
    struct hbr{
        SFERES_CONST size_t dim = 2;
    };
};

FIT_QD(DummyFit){

};

typedef DummyFit<Params> fit_th;
typedef sferes::gen::EvoFloat<8, Params> gen_th;
typedef sferes::phen::Parameters<gen_th, fit_th, Params> phen_th;
typedef sferes::qd::selector::Uniform<phen_th, Params> select_th;
typedef sferes::qd::container::KdtreeStorage <boost::shared_ptr<phen_th>, Params::qd::behav_dim > storage_th;
typedef sferes::qd::container::Archive<phen_th, storage_th, Params> container_th; 
#ifdef GRAPHIC
    typedef sferes::eval::Eval<Params> eval_th;
#else
    typedef sferes::eval::Parallel<Params> eval_th;
#endif
typedef boost::fusion::vector<sferes::stat::DeleteGenFiles<phen_th, Params>> stat_th; 
typedef sferes::modif::Dummy<> modifier_th;


#ifdef BD_ACC
    typedef boost::fusion::vector<layer0::qd_t,layer1::qd_t,layer2::qd_t> qd_layers;
#else
   typedef boost::fusion::vector<layer0::qd_t,layer1::qd_t> qd_layers;
#endif

typedef sferes::qd::HBR_Seq<qd_layers,phen_th, eval_th, stat_th, modifier_th, select_th, container_th, Params> hbr_t;


int main(int argc, char **argv) 
{    
    tbb::task_scheduler_init init(NUM_THREADS);

    // using namespace sferes;
    using namespace boost::fusion;

    // if(argc==37)
    //   {
	// visualise_behaviour<fit_t>(argc, argv);
	// global::global_robot.reset();
	// return 0;
    //   }

    layer0::qd_t qd1;
    layer1::qd_t qd2;

    layer0::layer_ptr = &qd1;
    layer1::layer_ptr = &qd2;

    qd1.set_name("Layer1");
    qd2.set_name("Layer2");
   

    sferes::run_ea(argc, argv, qd1);
    sferes::run_ea(argc, argv, qd2);

    #ifdef BD_ACC
    layer2::qd_t qd3;
    layer2::layer_ptr = &qd3;
    qd3.set_name("Layer3");
    sferes::run_ea(argc, argv, qd3);
    qd2.write();//saving trained GPs at end of full training
    // qd3.apply_modifier();
    // qd3.update_stats();

    #endif
   
    
    std::cout<<"LAYER 1" << std::endl;
    std::cout<<"best fitness:" << layer0::layer_ptr->stat<0>().best()->fit().value() << std::endl;
    std::cout<<"archive Size:" << layer0::layer_ptr->stat<1>().archive().size() << std::endl;

    std::cout<<"LAYER 2" << std::endl;
    std::cout<<"best fitness:" << layer1::layer_ptr->stat<0>().best()->fit().value() << std::endl;
    std::cout<<"archive Size:" << layer1::layer_ptr->stat<1>().archive().size() << std::endl;
    return 0;
}
