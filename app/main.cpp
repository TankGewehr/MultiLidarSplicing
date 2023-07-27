#include "MultiLidarSplicing.h"

bool loadConfig(const std::string &filename,
                std::string &lidar_front_left,
                std::string &lidar_front_middle,
                std::string &lidar_front_right,
                std::string &lidar_rear_left,
                std::string &lidar_rear_middle,
                std::string &lidar_rear_right,
                std::string &frame_id,
                std::string &publish_topic)
{
    Json::Reader reader;
    Json::Value root;

    std::ifstream is(filename, std::ios::binary);
    if (!is.is_open())
    {
        std::cout << "Error opening file:" << filename << std::endl;
        return false;
    }

    if (reader.parse(is, root))
    {
        if (root["calibration_params_path"].isNull() || root["calibration_params_path"].type() != Json::objectValue || root["calibration_params_path"]["lidar_front_left"].isNull() || root["calibration_params_path"]["lidar_front_left"].type() != Json::stringValue || root["calibration_params_path"]["lidar_front_middle"].isNull() || root["calibration_params_path"]["lidar_front_middle"].type() != Json::stringValue || root["calibration_params_path"]["lidar_front_right"].isNull() || root["calibration_params_path"]["lidar_front_right"].type() != Json::stringValue || root["calibration_params_path"]["lidar_rear_left"].isNull() || root["calibration_params_path"]["lidar_rear_left"].type() != Json::stringValue || root["calibration_params_path"]["lidar_rear_middle"].isNull() || root["calibration_params_path"]["lidar_rear_middle"].type() != Json::stringValue || root["calibration_params_path"]["lidar_rear_right"].isNull() || root["calibration_params_path"]["lidar_rear_right"].type() != Json::stringValue)
        {
            std::cout << "Error calibration_params_path type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["frame_id"].isNull() || root["frame_id"].type() != Json::stringValue)
        {
            std::cout << "Error frame_id type:" << filename << std::endl;
            is.close();
            return false;
        }

        if (root["publish_topic"].isNull() || root["publish_topic"].type() != Json::stringValue)
        {
            std::cout << "Error publish_topic type:" << filename << std::endl;
            is.close();
            return false;
        }

        lidar_front_left = root["calibration_params_path"]["lidar_front_left"].asString();
        lidar_front_middle = root["calibration_params_path"]["lidar_front_middle"].asString();
        lidar_front_right = root["calibration_params_path"]["lidar_front_right"].asString();
        lidar_rear_left = root["calibration_params_path"]["lidar_rear_left"].asString();
        lidar_rear_middle = root["calibration_params_path"]["lidar_rear_middle"].asString();
        lidar_rear_right = root["calibration_params_path"]["lidar_rear_right"].asString();
        frame_id = root["frame_id"].asString();
        publish_topic = root["publish_topic"].asString();
    }

    is.close();
    return true;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./main <config_json_path> \n"
                     "example:\n"
                     "\t./bin/main ./config/config.json"
                  << std::endl;
        return 1;
    }
    std::string lidar_front_left, lidar_front_middle, lidar_front_right, lidar_rear_left, lidar_rear_middle, lidar_rear_right, frame_id, publish_topic;
    bool obstacle_enable, lane_enable;
    if (!loadConfig(argv[1], lidar_front_left, lidar_front_middle, lidar_front_right, lidar_rear_left, lidar_rear_middle, lidar_rear_right, frame_id, publish_topic))
        return 1;

    ros::init(argc, argv, "MultiLidarSplicing");
    MultiLidarSplicing app(
        lidar_front_left,
        lidar_front_middle,
        lidar_front_right,
        lidar_rear_left,
        lidar_rear_middle,
        lidar_rear_right,
        frame_id,
        publish_topic);
    app.run();
    return 0;
}
