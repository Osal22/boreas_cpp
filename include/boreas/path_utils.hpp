#ifndef BOREAS_PATH_UTILS_HPP_
#define BOREAS_PATH_UTILS_HPP_

#include <cstdint>
#include <string>

namespace boreas
{

int64_t timestamp_us_from_path(const std::string & path);

bool extract_timestamp_us_from_path(
  const std::string & path, const std::string & data_path_prefix,
  const std::string & extension, int64_t & timestamp_us);

}  // namespace boreas

#endif  // BOREAS_PATH_UTILS_HPP_
