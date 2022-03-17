#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc
{
public:
  TicToc()
  {
    tic();
  }

  void tic()
  {
    lap_start = start = std::chrono::system_clock::now();
  }

  double toc()
  {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

  double lap()
  {
    auto lap_end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = lap_end - lap_start;
    double lap_time = elapsed_seconds.count() * 1000;
    lap_start = lap_end;
    return lap_time;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end, lap_start;
};