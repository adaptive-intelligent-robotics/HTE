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

#ifndef QD_Layer_HPP_
#define QD_Layer_HPP_

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

namespace sferes {
    namespace qd {
        // Main class
        template <typename Phen, typename Eval, typename Stat, typename FitModifier,
            typename Selector, typename Container, typename Params, typename Exact = stc::Itself>
        class QDLayer
            : public sferes::qd::QualityDiversity<Phen, Eval, Stat, FitModifier,Selector, Container, Params,
                  typename stc::FindExact<QDLayer<Phen, Eval, Stat, FitModifier, Selector,
                                              Container, Params, Exact>,
                      Exact>::ret>{
        public:
            typedef Phen phen_t;
            typedef boost::shared_ptr<Phen> indiv_t;
            typedef typename std::vector<indiv_t> pop_t;
            typedef typename pop_t::iterator it_t;
            typedef typename phen_t::fit_t fit_t;
         

            QDLayer(std::string name="Layer") {this->_name=name;}

            // Random initialization of _parents and _offspring
            void random_pop()
            {
                this->_counter+=1;
                // std::cout<<"!!!!!!Results Directory  :"<<this->_res_dir<<std::endl;
                // std::cout<<"!!!!!!Results Name  :"<<this->_name<<std::endl;
                this->set_res_dir(this->_res_dir+"/"+this->_name);
                
                parallel::init();

                this->_pop.clear();

                _offspring.resize(Params::pop::init_size);
                BOOST_FOREACH (indiv_t& indiv, this->_offspring) {
                    indiv = indiv_t(new Phen());
                    indiv->random();
                }
                std::cout<<"Eval"<<std::endl;
                this->_eval_pop(this->_offspring, 0, this->_offspring.size());
                std::cout<<"Modifier"<<std::endl;
                this->apply_modifier();
                std::cout<<"ADDING"<<std::endl;
                _add(_offspring, _added);
                std::cout<<"Done"<<std::endl;

                this->_parents = this->_offspring;
                _offspring.resize(Params::pop::init_size);

                BOOST_FOREACH (indiv_t& indiv, this->_offspring) {
                    indiv = indiv_t(new Phen());
                    indiv->random();
                }

                this->_eval_pop(this->_offspring, 0, this->_offspring.size());
                this->apply_modifier();
                _add(_offspring, _added);
                
                // std::cout<<_container.archive().size()<<std::endl;
                

                // if (!_offspring[0]->bias_set()){
                //         std::cout<<"TEST BIAS NOT SET! WEIRD"<<this->name()<<std::endl;
                //     }

                _container.get_full_content(this->_pop);
                std::cout<<this->_pop.size()<<std::endl;
            }

            // Main Iteration of the QD algorithm
            void epoch()
            {
                this->_counter+=1;
                _parents.resize(Params::pop::size);

                // Selection of the parents (will fill the _parents vector)
                _selector(_parents, *this); // not a nice API

                // CLEAR _offspring ONLY after selection, as it can be
                // used by the selector (via this->_offspring)
                _offspring.clear();
                _offspring.resize(Params::pop::size);

                // Generation of the offspring
                std::vector<size_t> a;
                misc::rand_ind(a, _parents.size());
                assert(_parents.size() == Params::pop::size);
                for (size_t i = 0; i < Params::pop::size; i += 2) {
                    boost::shared_ptr<Phen> i1, i2;
                    _parents[a[i]]->cross(_parents[a[i + 1]], i1, i2);
                    i1->mutate();
                    i2->mutate();

                    // // Crossover needs to be 0 otherwise this won't work
                    // #if defined(BIAS)|| defined(BIAS_CURIOSITY)
                    // i1->set_bias(_parents[a[i]]->bias(),false);
                    // i2->set_bias(_parents[a[i+1]]->bias(),false);
                    // #endif

                    i1->develop();
                    i2->develop();
                    _offspring[a[i]] = i1;
                    _offspring[a[i + 1]] = i2;
                }

                // Evaluation of the offspring
                this->_eval_pop(_offspring, 0, _offspring.size());
                this->apply_modifier();

                // Addition of the offspring to the container
                _add(_offspring, _added, _parents);
                
                set_curiosity(0.0);
                for(int i=0;i<_added.size();i++)
                {
                    if(_added[i]){
                        this->_curiosity += 1;
                    }
                    else
                    {
                        this->_curiosity -= 0.25;
                    }
                    
                }

                assert(_offspring.size() == _parents.size());

                this->_pop.clear();

                // Copy of the containt of the container into the _pop object.
                _container.get_full_content(this->_pop);
            }

            void chain_epoch()//pop_t& new_parents)
            {
                this->_counter+=1;
                // _parents = new_parents;
                // CLEAR _offspring ONLY after selection, as it can be
                // used by the selector (via this->_offspring)
                _offspring.clear();
                _offspring.resize(_parents.size());//Params::pop::size);

                // std::cout<<_parents.size()<<std::endl;
                // Generation of the offspring
                std::vector<size_t> a;
                misc::rand_ind(a, _parents.size());
                assert(_parents.size() == Params::pop::size);
                for (size_t i = 0; i < Params::pop::size; i += 2) {
                    // std::cout<<"MUTATIONS"<<std::endl;
                    boost::shared_ptr<Phen> i1, i2;
                    _parents[a[i]]->cross(_parents[a[i + 1]], i1, i2);
                    i1->mutate();
                    i2->mutate();

                    i1->develop();
                    i2->develop();
                    _offspring[a[i]] = i1;
                    _offspring[a[i + 1]] = i2;

                    #if defined(CHAIN_FIT)
                    {
                        //Only use parent(genome) fitness value when adding individual
                        i1->fit().parent_value = _parents[a[i]]->fit().value();
                        i2->fit().parent_value = _parents[a[i+1]]->fit().value();
                    }
                    #endif
                }

                // Evaluation of the offspring
                this->_eval_pop(_offspring, 0, _offspring.size());
                

                // Addition of the offspring to the container
                _add(_offspring, _added, _parents);
                this->apply_modifier();
                
                set_curiosity(0.0);
                for(int i=0;i<_added.size();i++)
                {
                    if(_added[i]){
                        this->_curiosity += 1;
                    }
                    else
                    {
                        this->_curiosity -= 0.25;
                    }
                    
                }

                assert(_offspring.size() == _parents.size());

                this->_pop.clear();

                // Copy of the containt of the container into the _pop object.
                _container.get_full_content(this->_pop);
            }

            void final_eval()
            {
                pop_t empty;
                this->_eval_pop(this->_pop, 0, this->_pop.size());
                _add(this->_pop, _added);
                unsigned orig_gen = this->gen();
                this->set_gen(100000);
                this->update_stats();
                this->write();
                this->set_gen(orig_gen);
            }

            void eval()
            {   
                this->_pop.clear();
                this->_container.get_full_content(this->_pop);
                // assert(this->_pop.size() == population.size());
                this->_eval_pop(this->_pop, 0, this->_pop.size());
                // this->apply_modifier();
                // std::cout<<"EVAL "<<this->_pop.size()<<"  "<<_container.archive().size()<<std::endl;
                this->_container.erase_content();
                this->_add(this->_pop, this->_added);
                // std::cout<<"EVAL "<<this->_pop.size()<<"  "<<_container.archive().size()<<std::endl;
                // this->update_stats();
            }

            void set_name(std::string x)
            {
                this->_name = x;
            }

            void set_curiosity(double x)
            {
                this->_curiosity = x;
            }

            void inc_curiosity(double x)
            {
                this->_curiosity += x;
            }

            pop_t get_pop_from_gen_file(const std::string& fname)
            {
                dbg::trace trace("ea", DBG_HERE);
                std::cout << "loading " << fname << std::endl;
                std::ifstream ifs(fname.c_str());
                if (ifs.fail()) {
                std::cerr << "Cannot open :" << fname
                            << "(does file exist ?)" << std::endl;
                exit(1);
                }
                #ifdef SFERES_XML_WRITE
                typedef boost::archive::xml_iarchive ia_t;
                #else
                typedef boost::archive::binary_iarchive ia_t;
                 #endif
                
                ia_t ia(ifs);
                sferes::stat::State<Phen, Params> stat_state;

                boost::fusion::for_each(this->_stat, sferes::ea::ReadStat_f<ia_t>(ia));
                stat_state = *boost::fusion::find<sferes::stat::State<Phen, Params>>(this->_stat);

                this->_pop = stat_state.pop();
                // Add back to container
                for (auto it = stat_state.pop().begin(); it != stat_state.pop().end(); ++it)
                {
                    
                    (*it)->develop();
                    // std::cout<<(*it)->fit().desc().size()<<std::endl;
                    (*it)->fit().alive();
                    if(!this->_container.add(*it)){
                        std::cout<<"ERROR ADDING"<<std::endl;
                        std::cout<<"Dead "<<(*it)->fit().dead()<<std::endl;
                        std::cout<<"Fitness"<<(*it)->fit().value()<<std::endl;
                    }
                }
                // std::cout<<"Archive Loading "<<this->_container.archive().size()<<std::endl;

                
                return stat_state.pop();
            }

            void full_reeval(){
                this->_eval_pop(this->_pop, 0, this->_pop.size());
            }
            
            const std::string& name() const { return _name; }
            
            const int& counter() const { return _counter; }

            const Container& container() const { return _container; }

            const fit_t& fit() const { return this->_fit_proto; }

            const pop_t& pop() const { return this->_pop; }
            pop_t& pop() { return this->_pop; }

            const pop_t& offspring() const { return _offspring; }
            pop_t& offspring() { return _offspring; }

            const pop_t& parents() const { return _parents; }
            pop_t& parents() { return _parents; }

            const Selector& selector() const { return _selector; }
            Selector& selector() { return _selector; }

            const std::vector<bool>& added() const { return _added; }
            std::vector<bool>& added() { return _added; }

            const double& curiosity() const { return _curiosity; }
            double& curiosity() { return _curiosity; }

        protected:
            // Add the offspring into the container and update the score of the individuals from the
            // container and both of the sub population (offspring and parents)
            void _add(pop_t& pop_off, std::vector<bool>& added, pop_t& pop_parents)
            {
                added.resize(pop_off.size());
                for (size_t i = 0; i < pop_off.size(); ++i)
                    added[i] = _add_to_container(pop_off[i], pop_parents[i]);
                _container.update(pop_off, pop_parents);
            }

            // Same function, but without the need of parent.
            void _add(pop_t& pop_off, std::vector<bool>& added)
            {
                added.resize(pop_off.size());
                for (size_t i = 0; i < pop_off.size(); ++i)
                    {
                    added[i] = _container.add(pop_off[i]);
                    // if(added[i]==false){
                    //     std::cout<<"DATA NOT ADDED ";
                    //     for (auto i: pop_off[i]->fit().desc())
                    //         std::cout<<i<<" ";
                    //     std::cout<<" "<<std::endl;
                    // }
                    }
                pop_t empty;
                std::cout<<"added now Updating"<<std::endl;
                _container.update(pop_off, empty);
            }

            void reeval_add(pop_t& pop_off, std::vector<bool>& added)
            {
                added.resize(pop_off.size());
                for (size_t i = 0; i < pop_off.size(); ++i)
                    {
                    if (pop_off[i]->fit().dead())
                        added[i] = false;
                    else{
                        added[i] = true;
                        _container.direct_add(pop_off[i]);
                    }
                    
                    }
                pop_t empty;
                _container.update(pop_off, empty);
            }

            // add to the container procedure.
            // TODO JBM: curiosity is hardcoded here...
            bool _add_to_container(indiv_t i1, indiv_t parent)
            {
                if (_container.add(i1)) {
                    parent->fit().set_curiosity(parent->fit().curiosity() + 1);
                    return true;
                }
                else {
                    parent->fit().set_curiosity(parent->fit().curiosity() - 0.5);
                    return false;
                }
            }

            // ---- attributes ----

            Selector _selector;
            Container _container;
            std::string _name;

            pop_t _offspring, _parents;
            std::vector<bool> _added;
            double _curiosity=0.0f;
            int _counter=0;
        };
    } // namespace qd
} // namespace sferes
#endif
