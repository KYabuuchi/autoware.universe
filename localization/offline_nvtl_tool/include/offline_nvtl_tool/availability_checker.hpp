#pragma once
#include "offline_nvtl_tool/topic_synchronizer.hpp"

bool check_ndt_availability(
  const std::vector<SynchronizedData> & synchronized_data_array, std::string & error_message);