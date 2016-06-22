#include <string>

#include <barcode_reader/barcode_reader.hpp>
#include <param_utilities/param_utilities.hpp>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>

#include <boost/array.hpp>

int main(int argc, char *argv[]) {
    namespace br = barcode_reader;
    namespace pu = param_utilities;

    //
    // ROS initialization
    //

    ros::init(argc, argv, "barcode_reader");
    ros::NodeHandle handle;

    //
    // load parameters
    //

    br::BarcodeReader::Params params;
    params.image_topic = pu::param<std::string>("~image_topic", "image");
    params.image_transport = pu::param<std::string>("~image_transport", "compressed");
    params.barcode_topic = pu::param<std::string>("~barcode_topic", "barcode_image");
    {
        const double val(pu::param("~barcode_interval", 1.));
        params.barcode_interval = ros::Duration(val);
    }
    params.text_tickness = pu::param("~text_tickness", 2);
    {
        const boost::array<int, 3> default_val = {255, 255, 255};
        const boost::array<int, 3> val(pu::param("~text_rgb", default_val));
        params.text_color = CV_RGB(val[0], val[1], val[2]);
    }
    params.line_tickness = pu::param("~line_tickness", 3);
    {
        const boost::array<int, 3> default_val = {255, 0, 0};
        const boost::array<int, 3> val(pu::param("~line_rgb", default_val));
        params.line_color = CV_RGB(val[0], val[1], val[2]);
    }

    //
    // spin the node
    //

    br::BarcodeReader node(handle, params);

    ros::spin();

    return 0;
}
