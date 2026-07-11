#ifndef BOREAS_PROGRESS_BAR_HPP_
#define BOREAS_PROGRESS_BAR_HPP_

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <string>

namespace boreas
{

class ProgressBar
{
public:
  ProgressBar(size_t total, size_t bar_width = 40) : total_(total), bar_width_(bar_width) {}

  void update(size_t current)
  {
    if (total_ == 0) {
      return;
    }

    const size_t done = std::min(current, total_);
    const size_t remaining = total_ - done;
    const double percent = 100.0 * static_cast<double>(done) / static_cast<double>(total_);
    const size_t filled = static_cast<size_t>((percent / 100.0) * bar_width_);

    std::string bar(filled, '=');
    if (filled < bar_width_) {
      bar.push_back('>');
      bar.append(bar_width_ - filled - 1, ' ');
    }

    std::cout << "\r[" << bar << "] " << done << "/" << total_ << " done, " << remaining
              << " left (" << static_cast<int>(percent) << "%)   " << std::flush;
  }

  void finish(size_t current)
  {
    update(current);
    std::cout << std::endl;
  }

private:
  size_t total_;
  size_t bar_width_;
};

}  // namespace boreas

#endif  // BOREAS_PROGRESS_BAR_HPP_
