#include <boreas/path_utils.hpp>

#include <filesystem>
#include <regex>
#include <stdexcept>

namespace boreas
{

int64_t timestamp_us_from_path(const std::string & path)
{
  return std::stoll(std::filesystem::path(path).stem().string());
}

bool extract_timestamp_us_from_path(
  const std::string & path, const std::string & data_path_prefix, const std::string & extension,
  int64_t & timestamp_us)
{
  std::string relative_path = path;
  const auto prefix_pos = relative_path.find(data_path_prefix);
  if (prefix_pos != std::string::npos) {
    relative_path.erase(prefix_pos, data_path_prefix.length());
  }

  const std::string pattern = "/(\\d+)\\." + extension + "$";
  std::regex number_regex(pattern);
  std::smatch match;
  if (!std::regex_search(relative_path, match, number_regex)) {
    return false;
  }

  std::string digits = match[1].str();
  try {
    timestamp_us = std::stoll(digits);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

}  // namespace boreas
