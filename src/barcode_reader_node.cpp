#include <string>
#include <vector>

#include <barcode_reader/barcode_reader.hpp>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>

#include <opencv2/core/core.hpp>

cv::Scalar RGBParam(const std::string &key, const cv::Scalar &default_val) {
    namespace rp = ros::param;
    std::vector< int > val;
    rp::get(key, val);
    return val.size() == 3 ? cv::Scalar(val[2], val[1], val[0]) : default_val;
}

int main(int argc, char *argv[]) {
    namespace br = barcode_reader;
    namespace rp = ros::param;

    //
    // ROS initialization
    //

    ros::init(argc, argv, "barcode_reader");
    ros::NodeHandle nh;

    //
    // load parameters
    //

    br::BarcodeReader::Params params;
    params.image_transport = rp::param< std::string >("~image_transport", "raw");
    params.scan_interval = ros::Duration(rp::param("~scan_interval", 0.5));
    params.text_tickness = rp::param("~text_tickness", 2);
    params.text_color = RGBParam("~text_rgb", CV_RGB(255, 255, 255));
    params.line_tickness = rp::param("~line_tickness", 3);
    params.line_color = RGBParam("~line_rgb", CV_RGB(255, 0, 0));

    //
    // spin the node
    //

    br::BarcodeReader node(nh, params);

    ros::spin();

    return 0;
}
