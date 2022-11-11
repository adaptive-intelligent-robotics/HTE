//| This file is a part of the sferes2 framework.
//| Copyright 2009, ISIR / Universite Pierre et Marie Curie (UPMC)
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

#ifndef HBR_SEQ_HPP_
#define HBR_SEQ_HPP_

#include <algorithm>
#include <limits>

#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <boost/fusion/algorithm/iteration/for_each.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/multi_array.hpp>
#include <boost/timer/timer.hpp>

#include <sferes/ea/ea.hpp>
#include <sferes/fit/fitness.hpp>
#include <sferes/stc.hpp>

#include <sferes/qd/container/cvt.hpp>
#include <sferes/qd/container/grid.hpp>
#include <sferes/qd/container/sort_based_storage.hpp>
#include <sferes/qd/selector/uniform.hpp>

// #include "qd_layer.hpp"

namespace sferes {
    namespace qd {
        using namespace boost::fusion;


        template<typename E>
        struct run_epoch {
        run_epoch(E &ea) : _ea(ea) {
        }
        E& _ea;
        template<typename T>
        void operator() (T & layer) const {
            
                
            layer.set_gen(_ea.gen());
            // std::cout<<"Epoch"<<std::endl;
            layer.epoch();
            // std::cout<<"Updating Stats"<<std::endl;
            layer.update_stats();
            
            if (_ea.gen() % _ea.dump_period == 0)
            {
               layer.write(_ea.gen());
            }
        }
        };

        template<typename E>
        struct rand_pop {
        rand_pop(E &ea) : _ea(ea) {
        }
        E& _ea;
        template<typename T>
        void operator() (T & layer) const {
            layer.set_res_dir(_ea.res_dir());
            layer.set_name("Layer"+std::to_string(_ea.Layer_Num));
            std::cout<< "RANDOM_POP_"<<_ea.Layer_Num<<std::endl;
            _ea.Layer_Num = _ea.Layer_Num+1;
            layer.random_pop();
        }
        };

        // template<typename E>
        // struct rand_pop {
        // rand_pop(E &ea) : _ea(ea) {
        // }
        // E& _ea;
        // template<typename T>
        // void operator() (T & layer) const {
        //     layer.set_res_dir(_ea.res_dir());
        //     layer.set_name("Layer"+_ea.Layer_Num);
        //     _ea.Layer_Num = _ea.Layer_Num+1;
        //     std::cout<< "RANDOM_POP_"<<_ea.Layer_Num<<std::endl;
        //     layer.random_pop();
        // }
        // };

        template <typename Layers,typename Phen, typename Eval, typename Stat, typename FitModifier,
            typename Selector, typename Container, typename Params, typename Exact = stc::Itself>
        class HBR_Seq: public ea::Ea<Phen, Eval, Stat, FitModifier, Params,
                  typename stc::FindExact<HBR_Seq<Layers,Phen, Eval, Stat, FitModifier, Selector,
                                              Container, Params, Exact>,
                      Exact>::ret> {
        public:
            typedef Phen phen_t;
            typedef Params params_t;
            typedef boost::shared_ptr<Phen> indiv_t;
            typedef typename std::vector<indiv_t> pop_t;
            typedef typename pop_t::iterator it_t;
            int Layer_Num = 1;//for initialization
            int dump_period = Params::pop::dump_period;

            HBR_Seq() {}

            // Random initialization of _parents and _offspring
            void random_pop()
            {
                parallel::init();

                boost::fusion::for_each(this->layers,rand_pop<typename stc::FindExact<HBR_Seq<Layers,Phen, Eval, Stat, FitModifier, Selector,Container, Params, Exact>,Exact>::ret>(stc::exact(*this)));
                
            }

            // Main Iteration of the QD algorithm
            void epoch()
            {
                boost::fusion::for_each(this->layers,run_epoch<typename stc::FindExact<HBR_Seq<Layers,Phen, Eval, Stat, FitModifier, Selector,Container, Params, Exact>,Exact>::ret>(stc::exact(*this)));
            }

            void final_eval()
            {
                boost::fusion::at_c<0>(layers).final_eval();
                boost::fusion::at_c<1>(layers).final_eval();
            }

            Layers layers;
            double max_curiosity=0.0f; 

        };

    } // namespace qd
} // namespace sferes
#endif
