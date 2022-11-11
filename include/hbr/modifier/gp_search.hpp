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




#ifndef MODIFIER_GP_SEARCH_HPP
#define MODIFIER_GP_SEARCH_HPP

#include <sferes/stc.hpp>
#include <Eigen/Dense>
#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <boost/fusion/algorithm/iteration/for_each.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/multi_array.hpp>
#include <boost/timer/timer.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include "../Body_Acc/fit_hexa_body_gp.hpp" //use second layer to access archive with gps instead of using children


namespace sferes {
  namespace modif {

    template<typename Params>
    class GP_Search{
    public:

      std::vector<double> l_values,sigma_sq_values,noise_values;
      Eigen::Vector2d p;
      
      
    
      template<typename Ea>
      void _update(Ea& ea,double l,double sigma,double noise)
      { 

          for (auto it = (*layer1::layer_ptr).pop().begin(); it != (*layer1::layer_ptr).pop().end(); ++it) {

              p(0) = std::log(l);//first position is l value but we need it to be in the log space
              p(1) = std::log(std::sqrt(sigma));//second position is sigma value but we need it to be in the log space

              (*it)->fit().gp.kernel_function().set_h_params(p);
              (*it)->fit().gp.kernel_function().set_noise(noise);
              (*it)->fit().fit_gp(true);
            }
            
          ea.full_reeval(); //issue with function since it creates new individuals

          // for (auto it = ea.pop().begin(); it != ea.pop().end(); ++it) {

          //   // std::cout<<(*it)->fit().desc()[0]<<std::endl;
          //   (*it)->fit().eval(**it); // not in parallel
          //   // std::cout<<(*it)->fit().desc()[0]<<"\n"<<std::endl;

          // }
          
          double sum_quality=0.0;
          
          std::string prefix="Reeval_GP_Params";
          // std::cout << "writing..." << prefix << std::endl;
          std::string fname = ea.res_dir() + "/" + prefix + std::string(".dat");
          std::ofstream ofs(fname.c_str(), std::ofstream::out | std::ofstream::app);

          using namespace boost::accumulators;
          typedef accumulator_set<double, stats<tag::p_square_quantile,tag::min,tag::max> > accumulator_t;

          accumulator_t low_quant_fit(quantile_probability = 0.25);
          accumulator_t med_quant_fit(quantile_probability = 0.50);
          accumulator_t upp_quant_fit(quantile_probability = 0.75);

          for (auto it = ea.pop().begin(); it != ea.pop().end(); ++it) {
            low_quant_fit((*it)->fit().full_err());
            med_quant_fit((*it)->fit().full_err());
            upp_quant_fit((*it)->fit().full_err());
            sum_quality += (*it)->fit().value();
          }

          // for (auto it = (*layer1::layer_ptr).pop().begin(); it != (*layer1::layer_ptr).pop().end(); ++it) {
          //   ofs << offset<< "\t\t" << (*it)->fit().gp.kernel_function().h_params().transpose() <<"\t\t"<< (*it)->fit().gp.mean_observation().transpose()<<"\t\t"<<(*it)->fit().gp.nb_samples() << std::endl;
          //   offset++;
          // }
          ofs << ea.gen() << " " <<  ea.pop().size()
              << " " << l << " " << sigma << " " << noise
              << " " << sum_quality
              << "  " << p_square_quantile(low_quant_fit) << " " << p_square_quantile(med_quant_fit) << " " << p_square_quantile(upp_quant_fit)
              << std::endl;

          

      }

      template<typename Ea>
      void apply(Ea& ea) {
        this->l_values = {4,6,8};
        this->sigma_sq_values = {0.05,0.1};
        this->noise_values = {0.005,0.01,0.03,0.04};
        
        Eigen::VectorXd og_log_params = (*layer1::layer_ptr).pop()[0]->fit().gp.kernel_function().h_params();
        double og_noise = (*layer1::layer_ptr).pop()[0]->fit().gp.kernel_function().noise();

        if ((ea.gen() % 799 == 0 && ea.gen()>0 )){
          std::cout << "Writing GP Results..."<< ea.gen() << std::endl;
          for(int i=0;i<this->l_values.size();i++){
            for(int k=0;k<this->sigma_sq_values.size();k++){
              for(int n=0;n<this->noise_values.size();n++){
                _update(ea,this->l_values[i],this->sigma_sq_values[k],this->noise_values[n]);
              }
            }
          }

           _update(ea,std::exp(og_log_params(0)),std::pow(std::exp(og_log_params(1)),2),og_noise);  //original params are in the log space and since we transform in the update function we need to inverse the transform 
          
        }
      }
      // template<class Archive>
      // void serialize(Archive & ar, const unsigned int version) {
      //     ar & BOOST_SERIALIZATION_NVP(container);
      // }
    };
  }
}

#endif
