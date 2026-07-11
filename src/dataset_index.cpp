#include <boreas/dataset_index.hpp>
#include <boreas/path_utils.hpp>

#include <algorithm>
#include <filesystem>
#include <unordered_map>

namespace boreas
{

std::vector<TimestampedPath> index_dataset_directory(
  const std::string & directory, const std::string & data_path_prefix,
  const std::string & extension)
{
  std::unordered_map<int64_t, std::string> indexed_frames;

  for (const auto & entry : std::filesystem::directory_iterator(directory)) {
    const std::string frame_path = entry.path().string();
    int64_t timestamp_us{0};
    if (!extract_timestamp_us_from_path(frame_path, data_path_prefix, extension, timestamp_us)) {
      continue;
    }
    indexed_frames[timestamp_us] = frame_path;
  }

  std::vector<TimestampedPath> sorted_frames(indexed_frames.begin(), indexed_frames.end());
  std::sort(sorted_frames.begin(), sorted_frames.end());
  return sorted_frames;
}

}  // namespace boreas
