#ifndef STAT_QD_MULTITASK_HPP_
#define STAT_QD_MULTITASK_HPP_

#include <numeric>
#include <sferes/stat/stat.hpp>
#include <sferes/parallel.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

namespace sferes {
    namespace stat {
        SFERES_STAT(QdMT, Stat){
        public:
            typedef std::vector<boost::shared_ptr<Phen> > archive_t;

            
            template <typename EA>
            void _write_container(const std::string& prefix, const EA& ea,int max_steps = 16)
            {
            }
            template <typename E> void refresh(const E& ea)
            {
                if ((ea.gen() % (Params::pop::nb_gen-1) == 0) && (ea.gen()>0))//needs to be the same as the qd_multitask_eval.hpp file
                    _write_container(std::string("grid_multi_task"), ea);
            }
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version) {
                ar & BOOST_SERIALIZATION_NVP(_container);
            }
            const archive_t& archive() const { return _container; }
            archive_t& archive() { return _container; }
            
        protected:
            archive_t _container;
        }; // QdMT
    } // namespace stat
} // namespace sferes

#endif
