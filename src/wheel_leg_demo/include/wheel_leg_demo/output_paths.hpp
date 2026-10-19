#pragma once
#include <cstdlib>
#include <filesystem>
#include <string>

namespace wheel_leg_demo {
// Runtime records belong to the user, not to the source/install tree.
inline std::string defaultOutputDir(const std::string& section) {
  const char* home = std::getenv("HOME");
  const auto base = home && *home
    ? std::filesystem::path(home) / ".local/state/wheel_leg_demo"
    : std::filesystem::temp_directory_path() / "wheel_leg_demo";
  return (base / section).string();
}
}
