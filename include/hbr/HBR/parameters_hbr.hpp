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




#ifndef PARAMETERS_HBR_HPP_
#define PARAMETERS_HBR_HPP_

#include <vector>
#include <sferes/phen/indiv.hpp>
#include <boost/foreach.hpp>

namespace sferes {
  namespace phen {
    SFERES_INDIV(Parameters_HBR, Indiv) {

      template<typename G, typename F, typename P, typename E>
      friend std::ostream& operator<<(std::ostream& output, const Parameters_HBR< G, F, P, E >& e);
    public:
#ifdef EIGEN_CORE_H
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
      Parameters_HBR() : _params((*this)._gen.size()), _bias(Params::hbr::prev_behav_dim*Params::hbr::num_indiv,0.0){ }

      typedef float type_t;
      SFERES_CONST float max_p = Params::parameters::max;
      SFERES_CONST float min_p = Params::parameters::min;
      std::mutex mylock;

      void develop() {
        for (unsigned i = 0; i < _params.size(); ++i)
          _params[i] = this->_gen.data(i) * (max_p - min_p) + min_p;
      }
      float data(size_t i) const {
        assert(i < size());
        return _params[i];
      }
      size_t size() const {
        return _params.size();
      }
      const std::vector<float>& data() const {
        return _params;
      }
      // squared Euclidean distance
      float dist(const Parameters_HBR& params) const {
        assert(params.size() == size());
        float d = 0.0f;
        for (size_t i = 0; i < _params.size(); ++i) {
          float x = _params[i] - params._params[i];
          d += x * x;
        }
        return d;
      }
      void show(std::ostream& os) const {
        BOOST_FOREACH(float p, _params)
        os<<p<<" ";
        os<<std::endl;
      }
      const std::vector<float>& bias() const {
        return _bias;
      }
      void set_bias(std::vector<float> bias,bool mark_set=true)
      {
        assert(bias.size() == _bias.size());
        for (unsigned i = 0; i < _bias.size(); ++i)
          _bias[i] = bias[i];
        if(mark_set){
          this->_bias_set = true;
        }
      }

      void reset_bias()
      {
          this->_bias = std::vector<float>(Params::hbr::prev_behav_dim*Params::hbr::num_indiv,0.0);
          this->_bias_set = false;
      }

      float bias(size_t i) const {
        assert(i < size());
        return _bias[i];
      }

      bool bias_set() const {
        return _bias_set;
      }

      bool children_set() const {
        return _children_set;
      }
      const std::vector<float>& multi_task() const {
        return this->_multi_task;
      }
      void  set_multi_task(std::vector<float> task) {
        this->_multi_task = task;
      }



    protected:
      std::vector<float> _params;
      std::vector<float> _bias;
      bool _bias_set = false, _children_set=false;
      std::vector<float> _multi_task;

      
    };
    template<typename G, typename F, typename P, typename E>
    std::ostream& operator<<(std::ostream& output, const Parameters_HBR< G, F, P, E >& e) {
      for (size_t i = 0; i < e.size(); ++i)
        output <<" "<<e.data(i) ;
      return output;
    }
  }
}

#endif
