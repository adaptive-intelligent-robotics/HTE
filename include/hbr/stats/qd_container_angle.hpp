#ifndef STAT_QD_CONTAINER_ANGLE_HPP_
#define STAT_QD_CONTAINER_ANGLE_HPP_

#include <numeric>
#include <sferes/stat/stat.hpp>

namespace sferes {
    namespace stat {
        SFERES_STAT(QdContainerAngle, Stat){
        public:
            typedef std::vector<boost::shared_ptr<Phen> > archive_t;

            template <typename E> void refresh(const E& ea)
            {

                _container.clear();

                for (auto it = ea.pop().begin(); it != ea.pop().end(); ++it)
                    _container.push_back(*it);

                if (ea.gen() % Params::pop::dump_period == 0)
                    _write_container(std::string("archive_"), ea);

                // if (ea.gen() % Params::pop::dump_period == 0)
                // {
                //         std::cout<<"Making Video.."<<std::endl;
                //         ea.pop()[10]->fit().simulate(*ea.pop()[10],true,ea.res_dir() +"/hexapod_15_001111"+".mp4",15,false);
                //         ea.pop()[10]->fit().simulate(*ea.pop()[10],true,ea.res_dir() +"/hexapod_59_111011"+".mp4",59,false);
                //         ea.pop()[10]->fit().simulate(*ea.pop()[10],true,ea.res_dir() +"/hexapod_47_101111"+".mp4",47,false);
                //         // ea.pop()[10]->fit().simulate(*ea.pop()[10],true,ea.res_dir() +"/hexapod_15_001111"+".mp4",15,false);
                //         // if (ea.pop().size()>100)
                //         //     ea.pop()[100]->fit().simulate(*ea.pop()[100],true,ea.res_dir() +"/hexapod_59_111011"+".mp4",59,false);
                //         // if (ea.pop().size()>1000)
                //         //     ea.pop()[1000]->fit().simulate(*ea.pop()[1000],true,ea.res_dir() +"/hexapod_47_101111"+".mp4",47,false);
                //         std::cout<<"Done at "<<ea.res_dir() +"/hexapod"+".mp4"<<std::endl;
                // }
            }
            template <typename EA>
            void _write_container(const std::string& prefix, const EA& ea) const
            {
                std::cout << "writing..." << prefix << ea.gen() << std::endl;
                std::string fname = ea.res_dir() + "/" + prefix
                    + boost::lexical_cast<std::string>(ea.gen()) + std::string("_hbr.dat");

                std::ofstream ofs(fname.c_str());

                size_t offset = 0;
                ofs.precision(17);

                for (auto it = ea.pop().begin(); it != ea.pop().end(); ++it) {
                    ofs << offset << " ";
                    for (size_t dim = 0; dim < (*it)->fit().desc().size(); ++dim)
                        ofs << (*it)->fit().desc()[dim] << " ";
                    // ofs << " " << array(idx)->fit().value() << std::endl;
                    ofs << " " << (*it)->fit().angle() << " ";

                    // ofs << " " << (*it)->fit().value() << " ";
                    
                    // ofs << " " << (*it)->fit().age() <<  "         ";
                    
                    // ofs << " " << (*it)->fit().obs_err() << " " << (*it)->fit().orientation_err()<< " " << (*it)->fit().gp_uncertainty()<< "         ";


                    for (size_t dim = 0; dim < (*it)->size(); ++dim)
                        ofs << (*it)->data(dim) << " ";
                    ofs << std::endl;
                    ++offset;
                }
            }
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version) {
                ar & BOOST_SERIALIZATION_NVP(_container);
            }
            const archive_t& archive() const { return _container; }
        protected:
            archive_t _container;
        }; // QdContainerAngle
    } // namespace stat
} // namespace sferes

#endif
