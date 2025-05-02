#ifndef MEAN_STDEV_AGGREGATOR_H
#define MEAN_STDEV_AGGREGATOR_H

#include <iostream>
#include <cmath>
#include <limits>

class MeanStdevAggregator {
 public:
  MeanStdevAggregator() : count_(0), mean_(0), second_mo_(0),
      min_(std::numeric_limits<double>::max()),
      max_(std::numeric_limits<double>::lowest())
  { }

  template<typename T>
  void insert(T const& i) {
    count_++;
    double delta = i - mean_;
    mean_ += delta/count_;
    second_mo_ += delta * ((double) i - mean_);
    if (i > max_)
      max_ = i;
    if (i < min_)
      min_ = i;
  }

  size_t count() const { return count_; }
  double min()   const { return min_; }
  double max()   const { return max_; }
  double mean()  const { return mean_; }
  double stdev() const { return std::sqrt(second_mo_ / (count_ - 1)); }
  double sem()   const { return std::sqrt(second_mo_ / (count_ * (count_ - 1))); }
  double ci95()  const { return 1.96 * sem(); }


  std::ostream& write(std::ostream& os) const {
    os << "# = " << count_;
    if (count_ > 0) {
      os << "  min = " << min_
         << "  mean = " << mean_
         << "  (+- " << ci95() << " 95ci)"
         << "  max = " << max_;
    }
    return os;
  }

 private:
  size_t count_;
  double mean_;
  double second_mo_;
  double min_;
  double max_;
};

inline std::ostream& operator<<(std::ostream& os, MeanStdevAggregator const& i) {
  return i.write(os);
}

#endif  // MEAN_STDEV_AGGREGATOR_H
