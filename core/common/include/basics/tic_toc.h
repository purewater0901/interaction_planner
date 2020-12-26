#ifndef SRC_TIC_TOC_H
#define SRC_TIC_TOC_H

#include <chrono>
#include <cstdlib>
#include <ctime>

class TicToc
{
public:
    TicToc() { tic(); }

    /**
     * @brief start timing
     * @note the unit of time is in millisecond (ms)
     */
    void tic() { start = std::chrono::system_clock::now(); }

    /**
     * @brief get time elapsed
     * @note the unit of time is in millisecond (ms)
     */
    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

    /**
     * @brief Convert chrono::system_clock::time_point to double
     *
     * @param t
     * @return double
     */
    static double TimePointToDouble(
            const std::chrono::system_clock::time_point& t) {
        auto tt = std::chrono::duration<double>(t.time_since_epoch());
        return tt.count();
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

#endif //SRC_TIC_TOC_H
