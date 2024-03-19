#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_map_server/map_io.hpp"

#include "opencv2/opencv.hpp"

using namespace nav2_map_server;
using namespace cv;

class mappublish : public rclcpp::Node
{
public:
    mappublish(std::string& topic):Node("mappublish"){pubmap = create_publisher<nav_msgs::msg::OccupancyGrid>(topic,1);};
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubmap;
};

const char * USAGE_STRING{
  "Usage:\n"
  "  mappublish [arguments] [--ros-args ROS remapping args]\n"
  "\n"
  "Arguments:\n"
  "  -h/--help\n"
  "  -t <map_topic>\n"
  "  -f <mapname>\n"
  "\n"
  "NOTE: --ros-args should be passed at the end of command line"};

typedef enum
{
  COMMAND_MAP_TOPIC,
  COMMAND_MAP_FILE_NAME,
} COMMAND_TYPE;

struct cmd_struct
{
  const char * cmd;
  COMMAND_TYPE command_type;
};

typedef enum
{
  ARGUMENTS_INVALID,
  ARGUMENTS_VALID,
  HELP_MESSAGE
} ARGUMENTS_STATUS;

// Arguments parser
// Input parameters: logger, argc, argv
// Output parameters: map_topic, save_parameters
ARGUMENTS_STATUS parse_arguments(
  const rclcpp::Logger & logger, int argc, char ** argv,
  std::string & map_topic, std::string & filename)
{
  const struct cmd_struct commands[] = {
    {"-t", COMMAND_MAP_TOPIC},
    {"-f", COMMAND_MAP_FILE_NAME},
  };

  std::vector<std::string> arguments(argv + 1, argv + argc);
  std::vector<rclcpp::Parameter> params_from_args;


  size_t cmd_size = sizeof(commands) / sizeof(commands[0]);
  size_t i;
  for (auto it = arguments.begin(); it != arguments.end(); it++) {
    if (*it == "-h" || *it == "--help") {
      std::cout << USAGE_STRING << std::endl;
      return HELP_MESSAGE;
    }
    if (*it == "--ros-args") {
      break;
    }
    for (i = 0; i < cmd_size; i++) {
      if (commands[i].cmd == *it) {
        if ((it + 1) == arguments.end()) {
          RCLCPP_ERROR(logger, "Wrong argument: %s should be followed by a value.", it->c_str());
          return ARGUMENTS_INVALID;
        }
        it++;
        switch (commands[i].command_type) {
          case COMMAND_MAP_TOPIC:
            map_topic = *it;
            break;
          case COMMAND_MAP_FILE_NAME:
            filename = *it;
            break;
        }
        break;
      }
    }
    if (i == cmd_size) {
      RCLCPP_ERROR(logger, "Wrong argument: %s", it->c_str());
      return ARGUMENTS_INVALID;
    }
  }

  return ARGUMENTS_VALID;
}

int main(int argc, char ** argv)
{
  // ROS2 init
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("mappublish");

  // Parse CLI-arguments
  std::string filename;
  std::string map_topic = "map";
  switch (parse_arguments(logger, argc, argv, map_topic, filename)) {
    case ARGUMENTS_INVALID:
      rclcpp::shutdown();
      return -1;
    case HELP_MESSAGE:
      rclcpp::shutdown();
      return 0;
    case ARGUMENTS_VALID:
      break;
  }
    RCLCPP_INFO(logger,"FILE:%s,topic:%s",filename.c_str(),map_topic.c_str());
  // Call saveMapTopicToFile()
  int retcode=0;
  try {
    nav_msgs::msg::OccupancyGrid map;
    loadMapFromYaml(filename,map);

    auto nh = std::make_shared<mappublish>(map_topic);
    
    nh->pubmap->publish(map);
    rclcpp::spin(nh);
  } catch (std::exception & e) {
    RCLCPP_ERROR(logger, "Unexpected problem appear: %s", e.what());
    retcode = -1;
  }

  // Exit
  rclcpp::shutdown();
  return retcode;
}