#ifndef JSON_REPORTER
#define JSON_REPORTER

#include <iostream>
#include <fstream>
#include <sstream>
#include <cassert>
#include <unordered_map>
#include <boost/variant.hpp>

#include "mean_stdev_aggregator.h"


class JsonReporter {
  using Variant = boost::variant<int, unsigned int, size_t, long, double, float, std::string>;
 public:
  JsonReporter(std::string const& filename) : filename_(filename) {
    insert("return code", "unknown");
  }

  JsonReporter() : JsonReporter("") { }
  ~JsonReporter() {
    save();
  }

  void setFilename(std::string const& filename) {
    assert(filename != "");
    filename_ = filename;
  }

  void save() const {
    if (filename_ == "") return;
    std::ofstream json;
    json.open(filename_.c_str());
    json << toString();
    json.close();
  }

  template<typename T>
  void insert(std::string const& key, T i) {
// #if not defined NDEBUG
//     if (data_.find(key) != data_.end()) {
//       std::cout << "\n\n[JsonReporter] Overwriting key'" << key << "'\n" << std::endl;
//       assert(false);
//     }
// #endif
    data_[key] = i;
  }

  template<typename T>
  bool overwrite(std::string const& key, T i) {
    auto it = data_.find(key);
    if (it == data_.end()) {
      data_[key] = i;
      return false;
    }
    it->second = i;
    return true;
  }

  bool remove(std::string const& key) {
    auto it = data_.find(key);
    if (it == data_.end()) return false;
    data_.erase(it);
    return true;
  }

  std::string toString() const {
    std::ostringstream ost;
    ost << "{\n";
    for (auto it = data_.begin(); it != data_.end(); ) {
      ost << "  \"" << it->first << "\": ";
      if (boost::get<std::string>(&it->second) != nullptr) {
        ost << "\"" << it->second << "\"";
      }
      else {
        ost << it->second;
      }
      ++it;
      if (it != data_.end())
        ost << ",";
      ost << "\n";
    }
    ost << "}\n";
    return ost.str();
  }

 private:
  std::string filename_;
  std::unordered_map<std::string, Variant> data_;
};


template<>
inline void JsonReporter::insert(std::string const& key, MeanStdevAggregator stats) {
  insert("count(" + key + ")", stats.count());
  if (stats.count() > 0) {
    insert("min(" + key + ")",  stats.min());
    insert("max(" + key + ")",  stats.max());
    insert("mean(" + key + ")", stats.mean());
    insert("ci95(" + key + ")", stats.ci95());
  }
  else {
    insert("min(" + key + ")",  "NaN");
    insert("max(" + key + ")",  "NaN");
    insert("mean(" + key + ")", "NaN");
    insert("ci95(" + key + ")", "NaN");
  }
}

#endif  // JSON_REPORTER
