#ifndef _BARCODE_READER_BARCODE_READER_HPP_
#define _BARCODE_READER_BARCODE_READER_HPP_

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <nodelet/nodelet.h>
#include <object_detection_msgs/Objects.h>
#include <object_detection_msgs/Point.h>
#include <object_detection_msgs/Points.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/timer.h>
#include <sensor_msgs/Image.h>

#include <boost/foreach.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

#include <zbar.h>

namespace barcode_reader {

class BarcodeReader : public nodelet::Nodelet {
public:
  BarcodeReader() {}

  virtual ~BarcodeReader() {
    // stop the timer first
    // or the timer may call the publisher after the publisher's destruction
    scan_timer_.stop();
  }

  virtual void onInit() {
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());

    // load params
    const std::vector< std::string > scanner_configs(
        pnh.param("scanner_configs", defaultScannerConfigs()));
    const ros::Duration scan_interval(pnh.param("scan_interval", 0.5));
    republish_image_ = pnh.param("republish_image", false);

    // enable QRcode detection with symbol positions
    BOOST_FOREACH (const std::string &config, scanner_configs) {
      if (scanner_.set_config(config) != 0) {
        NODELET_ERROR_STREAM("Faild to set scanner config: " << config);
      }
    }

    // start storing images to be scanned
    image_transport::ImageTransport it(nh);
    image_subscriber_ = it.subscribe("image_raw", 1, &BarcodeReader::saveImageMsg, this);

    // start scanning barcodes
    if (republish_image_) {
      image_publisher_ = it.advertise("image_out", 1, true);
    }
    barcode_publisher_ = nh.advertise< object_detection_msgs::Objects >("barcodes_out", 1, true);
    scan_timer_ = nh.createTimer(scan_interval, &BarcodeReader::scanImageMsg, this);
  }

private:
  static std::vector< std::string > defaultScannerConfigs() {
    std::vector< std::string > configs;
    // disable all detection
    configs.push_back("disable");
    // enable QR code detection
    configs.push_back("qrcode.enable");
    // detect with position
    configs.push_back("position");
    return configs;
  }

  void saveImageMsg(const sensor_msgs::ImageConstPtr &image_msg) {
    boost::lock_guard< boost::mutex > lock(mutex_);
    image_msg_ = image_msg;
  }

  void scanImageMsg(const ros::TimerEvent &) {
    // do nothing if no nodes sbscribe barcode image topic
    if (barcode_publisher_.getNumSubscribers() == 0) {
      return;
    }

    // pick the latest subscribed image message
    sensor_msgs::ImageConstPtr image_msg;
    {
      boost::lock_guard< boost::mutex > lock(mutex_);
      image_msg = image_msg_;
    }
    if (!image_msg) {
      NODELET_WARN("scanImageMsg: empty image message");
      return;
    }

    // scan the mono image
    cv_bridge::CvImageConstPtr mono_image(cv_bridge::toCvShare(image_msg, "mono8"));
    if (!mono_image) {
      NODELET_ERROR("scanImageMsg: image conversion error");
      return;
    }
    zbar::Image zbar_image(mono_image->image.cols, mono_image->image.rows, "Y800",
                           mono_image->image.data, mono_image->image.total());
    scanner_.scan(zbar_image);

    // pack a message of detected barcodes
    object_detection_msgs::Objects barcode_msg;
    barcode_msg.header = image_msg->header;
    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
         symbol != zbar_image.symbol_end(); ++symbol) {
      // set data
      barcode_msg.names.push_back(symbol->get_data());
      // set location
      object_detection_msgs::Points contour;
      for (int i = 0; i < symbol->get_location_size(); ++i) {
        object_detection_msgs::Point point;
        point.x = symbol->get_location_x(i);
        point.y = symbol->get_location_y(i);
        contour.points.push_back(point);
      }
      barcode_msg.contours.push_back(contour);
    }

    // publish the barcode image
    if (barcode_msg.names.empty()) {
      // no barcodes found
      return;
    }
    if (republish_image_) {
      image_publisher_.publish(image_msg);
    }
    barcode_publisher_.publish(barcode_msg);
  }

private:
  bool republish_image_;

  image_transport::Subscriber image_subscriber_;

  image_transport::Publisher image_publisher_;
  ros::Publisher barcode_publisher_;
  ros::Timer scan_timer_;

  sensor_msgs::ImageConstPtr image_msg_;
  boost::mutex mutex_;

  zbar::ImageScanner scanner_;
};
} // namespace barcode_reader

#endif /* _BARCODE_READER_BARCODE_READER_HPP_ */
