#include "ros/ros.h"
#include "pkg_zero/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_client");
    if (argc != 3)
    {
        ROS_ERROR("2 arguments are required!!");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<pkg_zero::AddTwoInts>("add_two_ints");

    pkg_zero::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    if (client.call(srv))
    {
        ROS_INFO("sum: %ld", (long int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("failed to call the service add_two_ints");
        return 1;
    }

    return 0;
}
