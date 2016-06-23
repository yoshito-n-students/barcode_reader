#include <string>

#include <barcode_reader/barcode_reader.hpp>
#include <param_utilities/param_utilities.hpp>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>

#include <opencv2/core/core.hpp>

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
    params.use_interprocess = pu::param("~use_interprocess", false);
    params.image_transport = pu::param<std::string>("~image_transport", "compressed");
    params.barcode_topic = pu::param<std::string>("~barcode_topic", "barcode_image");
    params.scan_interval = ros::Duration(pu::param("~scan_interval", 1.));
    params.text_tickness = pu::param("~text_tickness", 2);
    {
        const cv::Vec3i rgb(pu::param<cv::Vec3i>("~text_rgb", cv::Vec3i(255, 255, 255)));
        params.text_color = CV_RGB(rgb[0], rgb[1], rgb[2]);
    }
    params.line_tickness = pu::param("~line_tickness", 3);
    {
        const cv::Vec3i rgb(pu::param<cv::Vec3i>("~line_rgb", cv::Vec3i(255, 0, 0)));
        params.line_color = CV_RGB(rgb[0], rgb[1], rgb[2]);
    }

    //
    // spin the node
    //

    br::BarcodeReader node(handle, params);

    ros::spin();

    return 0;
}
