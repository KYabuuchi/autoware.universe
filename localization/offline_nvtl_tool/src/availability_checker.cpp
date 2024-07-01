#include "offline_nvtl_tool/topic_synchronizer.hpp"

bool check_ndt_availability(
  const std::vector<SynchronizedData> & synchronized_data_array, std::string & error_message)
{
  const int ndt_available_count = std::count_if(
    synchronized_data_array.begin(), synchronized_data_array.end(),
    [](const SynchronizedData & synchronized_data) {
      return synchronized_data.ndt_pose.has_value();
    });

  // If 80% of the data has NDT pose, we consider NDT pose is available.
  constexpr double TORRENCE_RATE = 0.8;
  const double ndt_is_available_rate =
    static_cast<double>(ndt_available_count) / static_cast<double>(synchronized_data_array.size());

  const bool ndt_is_available = ndt_is_available_rate > TORRENCE_RATE;

  if (!ndt_is_available) {
    error_message = "NDT pose is not available in more than 20% of the data." +
                    std::to_string(ndt_available_count) + " / " +
                    std::to_string(synchronized_data_array.size());
  }

  return ndt_is_available;
}