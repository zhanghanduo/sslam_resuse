//
// Created by hd on 18-10-9.
//

#ifndef PROJECT_CEREALARCHIVER_H
#define PROJECT_CEREALARCHIVER_H

#include "mat_cerealisation.h"
#include "eigen_cerealisation.h"
#include <cereal/access.hpp>
#include <cereal/types/list.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/polymorphic.hpp>
#include <boost/dynamic_bitset.hpp>
#include <memory>
#include <fstream>
#include <cstring>

namespace cereal {

    template<class Archive, typename  Block, typename Alloc>
    inline void save(Archive& ar, boost::dynamic_bitset<Block, Alloc> const & bs)
    {
        size_t num_bits = bs.size();
        std::vector<Block> blocks(bs.num_blocks());
        to_block_range(bs, blocks.begin());

        ar (num_bits, blocks);
    }

    template<class Archive, typename  Block, typename Alloc>
    inline void load(Archive& ar, boost::dynamic_bitset<Block, Alloc> & bs)
    {
        size_t num_bits;
        std::vector<Block> blocks;
        ar (num_bits, blocks);

        bs.resize(num_bits);
        from_block_range(blocks.begin(), blocks.end(), bs);
        bs.resize(num_bits);
    }

    template<class Archive, typename _Scalar, int _Dim, int _Mode, int _Options>
    inline void save(Archive & ar,
                          const Eigen::Transform<_Scalar, _Dim, _Mode, _Options>& t)
    {
        save(ar, t.matrix());
    }

    template<class Archive, typename _Scalar, int _Dim, int _Mode, int _Options>
    inline void load(Archive & ar,
                     Eigen::Transform<_Scalar, _Dim, _Mode, _Options>& t)
    {
        load(ar, t.matrix());
    }

    template <class Archive, class _Scalar, int _Options>
    inline void serialize(Archive& ar, ::Eigen::Quaternion<_Scalar, _Options>& quat) {
        ar(make_nvp("w", quat.w()), make_nvp("x", quat.x()), make_nvp("y", quat.y()), make_nvp("z", quat.z()));
    }



}
#endif //PROJECT_CEREALARCHIVER_H
