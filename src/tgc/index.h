/**
 * @file
 * @brief 
 * 
 * @author David Valouch (valouch.david@fel.cvut.cz)
 * @date 2025-03-18
 * 
 * @copyright Copyright (c) 2025
 */

#pragma once
#ifndef TGC_INDEX_H_
#define TGC_INDEX_H_

#include "bolt/eigen/dense.h"
#include "bolt/base/option.h"

namespace tgc {

using Permutation = Eigen::PermutationMatrix<Eigen::Dynamic>;

/**
 * @brief Computes a permutation matrix P such that 
 * q_dest = P * q_src
 */
template<typename SourceRange, typename DestRange>
bolt::Option<Permutation> QPermutation(SourceRange src, DestRange dst)
{
    if(src.size() != dst.size()) return bolt::None;

    Eigen::PermutationMatrix<Eigen::Dynamic> P(src.size());

    int i = 0;
    for(auto v : src) {
        P.indices()(i++) = 
            std::find(dst.begin(), dst.end(), v) - dst.begin();
    }

    return bolt::Some(P);
}


} // namespace tgc

#endif // TGC_INDEX_H_