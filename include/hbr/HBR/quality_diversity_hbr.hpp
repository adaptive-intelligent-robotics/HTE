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

#ifndef QD_HBR_HPP_
#define QD_HBR_HPP_

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
        struct max {
        max(E &ea) : _ea(ea) {
        }
        E& _ea;
        template<typename T>
        void operator() (T & layer) const {
            if(layer.curiosity()>=_ea.max_curiosity)
            {
                _ea.max_curiosity = layer.curiosity();
            }
        }
        };


        template<typename E>
        struct run_gen {
        run_gen(E &ea) : _ea(ea) {
        }
        E& _ea;
        template<typename T>
        void operator() (T & layer) const {
            if(layer.curiosity()==_ea.max_curiosity)
            {
                // std::cout<<layer.name()<<std::endl;
                // layer.set_gen(layer.gen()+1);
                layer.epoch();
                layer.update_stats();
            }
        }
        };

        template <typename Layers,typename Phen, typename Eval, typename Stat, typename FitModifier,
            typename Selector, typename Container, typename Params, typename Exact = stc::Itself>
        class QD_HBR: public ea::Ea<Phen, Eval, Stat, FitModifier, Params,
                  typename stc::FindExact<QD_HBR<Layers,Phen, Eval, Stat, FitModifier, Selector,
                                              Container, Params, Exact>,
                      Exact>::ret> {
        public:
            typedef Phen phen_t;
            typedef boost::shared_ptr<Phen> indiv_t;
            typedef typename std::vector<indiv_t> pop_t;
            typedef typename pop_t::iterator it_t;

            QD_HBR() {}

            // Random initialization of _parents and _offspring
            void random_pop()
            {
                parallel::init();

                //first we need the same directory for our layer results as the overall Hbr
                at_c<0>(layers).set_res_dir(this->_res_dir);
                at_c<1>(layers).set_res_dir(this->_res_dir);
               
                at_c<0>(layers).set_name("Layer1");
                at_c<1>(layers).set_name("Layer2");
                
                std::cout << "RANDOM_POP1"<<std::endl;
                boost::fusion::at_c<0>(layers).random_pop();
                std::cout << "RANDOM_POP2"<<std::endl;
                boost::fusion::at_c<1>(layers).random_pop();
                std::cout << "RANDOM_DONE"<<std::endl;

                // at_c<0>(layers).set_res_dir(this->_res_dir);//+"/Layer1");
                // at_c<1>(layers).set_res_dir(this->_res_dir);//+"/Layer2");
                // at_c<2>(layers).set_res_dir(this->_res_dir);//+"/Layer3");
                
            }

            // Main Iteration of the QD algorithm
            void epoch()
            {
                #if defined(CURIOSITY) || defined(BIAS_CURIOSITY) || defined(EVAL_VAR)
                {
                max_curiosity=at_c<0>(layers).curiosity();
                at_c<0>(layers).set_gen(this->_gen);
                at_c<1>(layers).set_gen(this->_gen);
               
                #if defined(EVAL_VAR)
                    if (this->gen() % Params::hbr::dump_period == 0)
                        {
                        std::cout<<"EVAL"<<std::endl;
                        at_c<0>(layers).eval();
                        at_c<1>(layers).eval();
                        }
                #endif
                if (this->gen()< Params::hbr::warm_up){
                    boost::fusion::at_c<0>(layers).epoch();
                    boost::fusion::at_c<0>(layers).update_stats();
                }
                else if ((this->gen()< 2*Params::hbr::warm_up)){
                    boost::fusion::at_c<1>(layers).epoch();
                    boost::fusion::at_c<1>(layers).update_stats();
                }
                else
                {
                    boost::fusion::for_each(this->layers,max<typename stc::FindExact<QD_HBR<Layers,Phen, Eval, Stat, FitModifier, Selector,Container, Params, Exact>,Exact>::ret>(stc::exact(*this)));
                    boost::fusion::for_each(this->layers,run_gen<typename stc::FindExact<QD_HBR<Layers,Phen, Eval, Stat, FitModifier, Selector,Container, Params, Exact>,Exact>::ret>(stc::exact(*this)));
                }

                
                }
                
                #else
                {
                at_c<0>(layers).set_gen(this->_gen);
                at_c<1>(layers).set_gen(this->_gen);

                boost::fusion::at_c<0>(layers).epoch();
                boost::fusion::at_c<1>(layers).epoch();
                boost::fusion::at_c<0>(layers).update_stats();
                boost::fusion::at_c<1>(layers).update_stats();
                }
                #endif


                if (this->_gen % Params::pop::dump_period == 0)
                {
                    at_c<0>(layers).write(this->_gen);
                    at_c<1>(layers).write(this->_gen);
                }
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
