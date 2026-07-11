#ifndef BOREAS_DATASET_INDEX_HPP_
#define BOREAS_DATASET_INDEX_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace boreas
{

using TimestampedPath = std::pair<int64_t, std::string>;

std::vector<TimestampedPath> index_dataset_directory(
  const std::string & directory, const std::string & data_path_prefix,
  const std::string & extension);

}  // namespace boreas

#endif  // BOREAS_DATASET_INDEX_HPP_
