#pragma once

#include <chrono>
#include <iostream>
#include <sstream>

namespace timer {

class Timer {

  typedef std::chrono::high_resolution_clock clock;
  typedef std::chrono::milliseconds milliseconds;
  typedef std::chrono::microseconds microseconds;

public:
  Timer(std::string name = "timer", bool print_on_exit = true)
      : _name(name), _print_on_exit(print_on_exit) {
    reset();
  }

  ~Timer() {
    if (_print_on_exit)
      print();
  }

  inline void reset() { _start = clock::now(); }

  template <typename T> inline uint64_t elapsed() {
    auto dt = std::chrono::duration_cast<T>(clock::now() - _start);
    return dt.count();
  }

  inline uint64_t elapsed_ms() { return elapsed<milliseconds>(); }

  inline uint64_t elapsed_us() { return elapsed<microseconds>(); }

  inline std::string log(std::string label = "") {

    auto us = elapsed_us();
    auto ms = elapsed_ms();

    std::ostringstream ss;

    ss << "[ " << _name << " ] ";
    if (label != "")
      ss << "[ " << label << " ] ";

    ss << us << " us (" << ms << " ms)";

    return ss.str();
  }

  void print(std::string label = "") {

    std::string out = log(label);

    std::cout << out << std::endl;
  }

private:
  std::string _name;
  bool _print_on_exit = true;
  clock::time_point _start;
};

} // namespace timer
