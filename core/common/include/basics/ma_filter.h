/**
 * @file ma_filter.h
 * @author HKUST Aerial Robotics Group
 * @brief moving average filter
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef SRC_MA_FILTER_H
#define SRC_MA_FILTER_H

#include <vector>
#include "common/basics/basics.h"

class MAFilter
{
public:
    MAFilter() : L_(3) {}
    MAFilter(int L) : L_(L) {}

    void set_L(const int L) { L_ = L; }

    void add(const decimal_t new_obs)
    {
        observations_.push_back(new_obs);
        if (static_cast<int>(observations_.size()) > kMaxSize_)
            observations_.erase(observations_.begin());
    }

    void reset() { observations_.clear(); }

    double get()
    {
        int num_elements = static_cast<int>(observations_.size());
        if (num_elements == 0) return 0.0;
        int cnt = 0;
        double accumulate = 0.0;
        for (int i = num_elements - 1; i >= 0; i--)
        {
            accumulate += observations_[i];
            cnt++;
            if (cnt == L_) break;
        }
        return accumulate / cnt;
    }

private:
    std::vector<double> observations_;
    const int kMaxSize_ = 10;
    int L_;
};

#endif //SRC_MA_FILTER_H
