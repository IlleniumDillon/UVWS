#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_map_server/map_io.hpp"

#include "opencv2/opencv.hpp"

using namespace nav2_map_server;
using namespace cv;

class mapfixer : public rclcpp::Node
{
public:
    mapfixer():Node("mapfixer"){pubmap = create_publisher<nav_msgs::msg::OccupancyGrid>("/map",1);};
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubmap;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto nh = std::make_shared<mapfixer>();
    //auto pubmap = rclcpp::create_publisher<nav_msgs::msg::OccupancyGrid>(nh,"/map",1);
    std::string path = "/home/jetson/Desktop/project/ros2_test/src/uvplanner/map/";
    std::string name = "map_3011_20240311.yaml";
    LoadParameters para = loadMapYaml(path+name);
    nav_msgs::msg::OccupancyGrid map;
    loadMapFromYaml(path+name,map);
    //pubmap->publish(map);
    uint8_t* dmap = new uint8_t[map.info.height*map.info.width];
    for(int i = 0; i < map.info.height; i++)
    {
        for(int j = 0; j < map.info.width; j++)
        {
            //RCLCPP_INFO(this->get_logger(),"%d",map.data.at(i*map.info.width+j));
            if(map.data.at(i*map.info.width+j) == -1||
                map.data.at(i*map.info.width+j) == 100)
            {
                dmap[j*map.info.height+i]=1;
            }
            else
            {
                dmap[j*map.info.height+i]=0;
            }
            
        }
    }
    Mat img(map.info.width,map.info.height,CV_8UC1,dmap);
    img *= 255;
    imshow("ori map",img);
    Mat structd = getStructuringElement(MORPH_ELLIPSE,Size(10,10));
    Mat res;
    dilate(img,res,structd);
    //res = 255-res;
    //rotate(res,res,ROTATE_90_COUNTERCLOCKWISE);
    imshow("res map",res);
    // char ok = 0;
    // while(ok!='y'||ok!='n')
    // {
    //     ok = waitKey();
    // }
    // if(ok == 'y')
    {
        name = "mapdilated.png";
        RCLCPP_INFO(rclcpp::get_logger("main"),"save");
        //imwrite(path+name,res);
        uint8_t* pmap = res.data;
        for(int i = 0; i < map.info.height; i++)
        {
            for(int j = 0; j < map.info.width; j++)
            {
                //RCLCPP_INFO(this->get_logger(),"%d",map.data.at(i*map.info.width+j));
                // if(map.data.at(i*map.info.width+j) == -1||
                //     map.data.at(i*map.info.width+j) == 100)
                // {
                //     dmap[j*map.info.height+i]=1;
                // }
                // else
                // {
                //     dmap[j*map.info.height+i]=0;
                // }
                map.data.at(i*map.info.width+j) = pmap[j*map.info.height+i]/255*100;
            }
        }
        //pubmap->publish(map);

        // SaveParameters spara;
        // spara.free_thresh = para.free_thresh;
        // spara.occupied_thresh = para.occupied_thresh;
        // spara.map_file_name = name;
        // spara.image_format=".pgm";
        // saveMapToFile(map,spara);
    }
    nh->pubmap->publish(map);
    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}