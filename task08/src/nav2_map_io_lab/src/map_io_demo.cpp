#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "nav2_map_server/map_io.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fs = std::filesystem;

nav_msgs::msg::OccupancyGrid sampleMap()
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 3;
  map.info.height = 2;
  map.info.resolution = 0.1;
  map.info.origin.orientation.w = 1.0;
  map.data = {-1, 0, 100, 100, 0, -1};
  return map;
}

void printImageField(const fs::path & yaml_path)
{
  std::ifstream yaml(yaml_path);
  if (!yaml) {
    throw std::runtime_error("Cannot read saved YAML: " + yaml_path.string());
  }
  std::string line;
  while (std::getline(yaml, line)) {
    if (line.rfind("image:", 0) == 0) {
      std::cout << "YAML " << line << '\n';
      return;
    }
  }
  throw std::runtime_error("The YAML has no image field");
}

bool loadAndCheck(const fs::path & yaml_path)
{
  std::cout << "YAML path " << yaml_path << '\n';
  printImageField(yaml_path);
  const auto parameters = nav2_map_server::loadMapYaml(yaml_path.string());
  std::cout << "Resolved image " << parameters.image_file_name << '\n';

  nav_msgs::msg::OccupancyGrid loaded;
  const auto status = nav2_map_server::loadMapFromYaml(yaml_path.string(), loaded);
  std::cout << "LOAD_MAP_STATUS " << static_cast<int>(status) << '\n';
  if (status != nav2_map_server::LOAD_MAP_SUCCESS) {
    std::cerr << "Map loading failed\n";
    return false;
  }

  const auto expected = sampleMap();
  std::cout << "Loaded " << loaded.info.width << " x " << loaded.info.height
            << ", resolution " << loaded.info.resolution << ", cells";
  for (const auto cell : loaded.data) {
    std::cout << ' ' << static_cast<int>(cell);
  }
  std::cout << '\n';

  if (loaded.info.width != expected.info.width ||
    loaded.info.height != expected.info.height ||
    std::abs(loaded.info.resolution - expected.info.resolution) > 1e-6 ||
    loaded.data != expected.data)
  {
    std::cerr << "Loaded map differs from the 3 x 2 reference sample\n";
    return false;
  }
  std::cout << "Reference map verified\n";
  return true;
}

bool reproduce(bool flat)
{
  const std::string map_prefix = flat ? "inspection" : "maps/inspection";
  if (!flat) {
    fs::create_directories("maps");
  }

  nav2_map_server::SaveParameters parameters;
  parameters.map_file_name = map_prefix;
  parameters.image_format = "pgm";
  parameters.mode = nav2_map_server::MapMode::Trinary;
  parameters.free_thresh = 0.196;
  parameters.occupied_thresh = 0.65;

  if (!nav2_map_server::saveMapToFile(sampleMap(), parameters)) {
    std::cerr << "Map saving failed\n";
    return false;
  }
  std::cout << "Save succeeded\n";
  std::cout << "Image on disk " << fs::absolute(map_prefix + ".pgm") << '\n';
  return loadAndCheck(fs::absolute(map_prefix + ".yaml"));
}

int main(int argc, char ** argv)
{
  const char * usage = "Usage: map_io_demo reproduce [flat] | compatibility /absolute/path/map.yaml\n";
  if (argc < 2 || argc > 3) {
    std::cerr << usage;
    return 2;
  }

  rclcpp::init(argc, argv);
  int result = 1;
  try {
    std::cout << "Working directory " << fs::current_path() << '\n';
    const std::string mode(argv[1]);
    if (mode == "reproduce" && (argc == 2 || std::string(argv[2]) == "flat")) {
      result = reproduce(argc == 3) ? 0 : 1;
    } else if (mode == "compatibility" && argc == 3 && fs::path(argv[2]).is_absolute()) {
      result = loadAndCheck(fs::path(argv[2])) ? 0 : 1;
    } else {
      std::cerr << usage;
      result = 2;
    }
  } catch (const std::exception & error) {
    std::cerr << "Demo failed: " << error.what() << '\n';
  }
  rclcpp::shutdown();
  return result;
}
