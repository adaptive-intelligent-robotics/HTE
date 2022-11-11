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




#ifndef MODIFIER_GP_HPP
#define MODIFIER_GP_HPP

#include <sferes/stc.hpp>
#include <sferes/parallel.hpp>
#include <Eigen/Dense>
#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <boost/fusion/algorithm/iteration/for_each.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/multi_array.hpp>
#include <boost/timer/timer.hpp>
#include "../Body_Acc/fit_hexa_body_gp.hpp" //use second layer to access archive with gps instead of using children


namespace sferes {
  namespace modif {

    // template<typename Params>
    template<typename Phen, typename Params, typename Exact = stc::Itself>
    class GP_Update{
    // SFERES_CLASS(Closeness) {
    public:
      typedef std::vector<boost::shared_ptr<layer1::phen_t> > archive_t;
      

      template<typename Phen_t>
      struct _parallel_evaluate {
      typedef std::vector<boost::shared_ptr<Phen_t> > pop_t;
      typedef typename Phen_t::fit_t fit_t;
      pop_t _pop;
      bool h_opt=false;//Use if we want to optimize our GPs (EXPENSIVE!!!)


      ~_parallel_evaluate() { }
      _parallel_evaluate(pop_t& pop) : _pop(pop) {}
      // _parallel_evaluate(const _parallel_evaluate& ev) : _pop(ev._pop) {}
      void operator() (const parallel::range_t& r) const {
          for (size_t i = r.begin(); i != r.end(); ++i) {
              
              // std::cout<<i<<"\n"<<std::endl;
              assert(i < _pop.size());
              
              if(h_opt){
                  _pop[i]->fit().fit_gp(false);
                  _pop[i]->fit().gp.optimize_hyperparams();
                }

              _pop[i]->fit().fit_gp(true,true);//deleting previous values
          }
      }
      };
    
      template<typename Ea>
      void _update(Ea& ea)
      {
          std::cout << "Updating GPs..."<< ea.gen() << std::endl;
          bool h_opt=false;//Use if we want to optimize our GPs (EXPENSIVE!!!)
          // if (ea.gen() % Params::pop::dump_period == 0){
          //   h_opt=true;
          // }

          // for (auto it = ea.pop().begin(); it != ea.pop().end(); ++it) {
          //   (*it)->fit().update_gp_children(h_opt);
          // }
          for (auto it = (*layer1::layer_ptr).pop().begin(); it != (*layer1::layer_ptr).pop().end(); ++it) {
              if(h_opt){
                (*it)->fit().fit_gp(false);
                (*it)->fit().gp.optimize_hyperparams();
              }

              (*it)->fit().fit_gp(true);
            }
          std::cout << "Updated GPs..."<< ea.gen() << std::endl;

          if (ea.gen() % Params::pop::dump_period == 0){
            std::string prefix="GP_Params";
            std::cout << "writing..." << prefix << std::endl;
            std::string fname = ea.res_dir() + "/" + prefix + std::string(".dat");
            std::ofstream ofs(fname.c_str());
            size_t offset = 0;
            // for (auto it = ea.pop().begin(); it != ea.pop().end(); ++it) {
            //   for (auto child:(*it)->fit().children()){
            //     ofs << offset<<" " << (child)->fit().gp.kernel_function().h_params().transpose() <<"   "<< (child)->fit().gp.kernel_function().h_params().transpose() << std::endl;
            //     offset++;
            //   }
            for (auto it = (*layer1::layer_ptr).pop().begin(); it != (*layer1::layer_ptr).pop().end(); ++it) {
              ofs << offset<< "\t\t" << (*it)->fit().gp.kernel_function().h_params().transpose() <<"\t\t"<< (*it)->fit().gp.mean_observation().transpose()<<"\t\t"<<(*it)->fit().gp.nb_samples() << std::endl;
              offset++;
            }
          }

      }

      template<typename Ea>
      void force(Ea& ea) {
        _container.clear();
        for (auto it = (*layer1::layer_ptr).pop().begin(); it != (*layer1::layer_ptr).pop().end(); ++it)
            _container.push_back(*it);
        std::cout<<"Updating GPs "<<ea.pop().size()<<" "<<_container.size()<<std::endl;
        parallel::init();
        parallel::p_for(parallel::range_t(0, _container.size()),
                _parallel_evaluate<layer1::phen_t>(_container));
      }

      template<typename Ea>
      void apply(Ea& ea) {
        if (ea.gen() % Params::pop::dump_period == 0){

          // _update(ea);

          _container.clear();
          for (auto it = (*layer1::layer_ptr).pop().begin(); it != (*layer1::layer_ptr).pop().end(); ++it)
              _container.push_back(*it);
          std::cout<<"Updating GPs "<<ea.pop().size()<<" "<<_container.size()<<std::endl;
          parallel::init();
          parallel::p_for(parallel::range_t(0, _container.size()),
                  _parallel_evaluate<layer1::phen_t>(_container));
        } 


        if (ea.gen() % Params::pop::dump_period == 0){
            std::string prefix="GP_Params_";
            std::cout << "writing..." << prefix << std::endl;
            std::string fname = ea.res_dir() + "/" + prefix+ boost::lexical_cast<std::string>(ea.gen()) + std::string(".dat");
            std::ofstream ofs(fname.c_str());
            size_t offset = 0;
            // for (auto it = ea.pop().begin(); it != ea.pop().end(); ++it) {
            //   for (auto child:(*it)->fit().children()){
            //     ofs << offset<<" " << (child)->fit().gp.kernel_function().h_params().transpose() <<"   "<< (child)->fit().gp.kernel_function().h_params().transpose() << std::endl;
            //     offset++;
            //   }
            for (auto it = (*layer1::layer_ptr).pop().begin(); it != (*layer1::layer_ptr).pop().end(); ++it) {
              ofs << offset<< "\t\t" << (*it)->fit().gp.kernel_function().h_params().transpose() <<"\t\t"<< (*it)->fit().gp.mean_observation().transpose()<<"\t\t"<<(*it)->fit().gp.nb_samples() << std::endl;
              offset++;
            }
          }
      }
      // template<class Archive>
      // void serialize(Archive & ar, const unsigned int version) {
      //     ar & BOOST_SERIALIZATION_NVP(container);
      // }

      protected:
        archive_t _container;
    };
  }
}

#endif
