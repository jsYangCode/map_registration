/**
 *
 * Created by js.yang on 30/8/2019 for map registration step by step
 *
 * */

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>
class  TicToc
{
    public:
        TicToc()
        {
           tic();
        }
        void tic()
        {
            start=std::chrono::system_clock::now();
        }
        double toc()
        {
            end=std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_time=end-start;
            return elapsed_time.count()*1000;
        }
    private:
    std::chrono::time_point<std::chrono::system_clock> start,end;
};