#ifndef _BARCODE_READER_BARCODE_READER_HPP_
#define _BARCODE_READER_BARCODE_READER_HPP_

#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/timer.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>

#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

#include <zbar.h>

namespace barcode_reader {

class BarcodeReader {
   public:
    struct Params {
        std::string image_topic;
        std::string image_transport;

        std::string barcode_topic;
        ros::Duration barcode_interval;

        int text_tickness;
        cv::Scalar text_color;
        int line_tickness;
        cv::Scalar line_color;
    };

   public:
    BarcodeReader(const ros::NodeHandle &handle, const Params &params)
        : handle_(handle), it_handle_(handle), params_(params) {
        // enable QRcode detection with symbol positions
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_POSITION, 1);

        //
        it_subscriber_ =
            it_handle_.subscribe(params_.image_topic, 1, &BarcodeReader::saveImageMsg, this,
                                 image_transport::TransportHints(params_.image_transport));

        //
        it_publisher_ = it_handle_.advertise(params_.barcode_topic, 1);

        //
        timer_ = handle_.createTimer(params_.barcode_interval, &BarcodeReader::scanImageMsg, this);
    }

    virtual ~BarcodeReader() {
        timer_.stop();
        it_subscriber_.shutdown();
    }

   private:
    void saveImageMsg(const sensor_msgs::ImageConstPtr &image_msg) {
        boost::lock_guard<boost::mutex> lock(mutex_);
        image_msg_ = image_msg;
    }

    void scanImageMsg(const ros::TimerEvent &) {
        //
        // 0. do nothing if no nodes sbscribe barcode image topic
        //

        if (it_publisher_.getNumSubscribers() == 0) {
            return;
        }

        //
        // 1. pick the latest subscribed image message
        //

        sensor_msgs::ImageConstPtr image_msg;
        {
            boost::lock_guard<boost::mutex> lock(mutex_);
            image_msg = image_msg_;
        }
        if (!image_msg) {
            ROS_WARN("Empty image message");
            return;
        }

        //
        // 2. scan the mono image
        //

        cv_bridge::CvImageConstPtr mono_image(cv_bridge::toCvShare(image_msg, "mono8"));
        zbar::Image zbar_image(mono_image->image.cols, mono_image->image.rows, "Y800",
                               mono_image->image.data, mono_image->image.size().area());
        scanner_.scan(zbar_image);

        //
        // 3. visualize detected barcodes on the raw image
        //

        cv_bridge::CvImagePtr barcode_image(cv_bridge::toCvCopy(image_msg, "bgr8"));

        // make the image darker
        barcode_image->image *= 0.5;

        if (zbar_image.symbol_begin() != zbar_image.symbol_end()) {  // if found
            // draw each barcode polygon and put data on the polygon
            for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
                 symbol != zbar_image.symbol_end(); ++symbol) {
                std::vector<cv::Point> points(symbol->get_location_size());
                cv::Rect rect(barcode_image->image.cols, barcode_image->image.rows, 0, 0);
                for (int i = 0; i < symbol->get_location_size(); ++i) {
                    const int x(symbol->get_location_x(i));
                    const int y(symbol->get_location_y(i));
                    points[i].x = x;
                    points[i].y = y;
                    if (rect.x > x) {
                        rect.x = x;
                    }
                    if (rect.y > y) {
                        rect.y = y;
                    }
                    if (rect.width < x) {
                        rect.width = x;
                    }
                    if (rect.height < y) {
                        rect.height = y;
                    }
                }
                cv::polylines(barcode_image->image, points, true, params_.line_color,
                              params_.line_tickness);
                rect.width -= rect.x;
                rect.height -= rect.y;
                putText(barcode_image->image, symbol->get_data(), rect);
            }
        } else {  // if not found
            putText(barcode_image->image, " No Barcode Found ",
                    cv::Rect(0, 0, barcode_image->image.cols, barcode_image->image.rows));
        }

        //
        // 4. publish the barcode image
        //

        it_publisher_.publish(barcode_image->toImageMsg());
    }

    void putText(cv::Mat &image, const std::string &text, const cv::Rect &rect) {
        const cv::Size size1(
            cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 1., params_.text_tickness, NULL));
        const double scale(std::min(static_cast<double>(rect.width) / size1.width,
                                    static_cast<double>(rect.height) / size1.height));
        cv::putText(image, text, cv::Point(rect.x, rect.y + rect.height / 2),
                    cv::FONT_HERSHEY_SIMPLEX, scale, params_.text_color, params_.text_tickness);
    }

   private:
    const Params params_;

    ros::NodeHandle handle_;
    ros::Timer timer_;

    image_transport::ImageTransport it_handle_;
    image_transport::Subscriber it_subscriber_;
    image_transport::Publisher it_publisher_;

    sensor_msgs::ImageConstPtr image_msg_;
    boost::mutex mutex_;

    zbar::ImageScanner scanner_;
};
}

#endif /* _BARCODE_READER_BARCODE_READER_HPP_ */
