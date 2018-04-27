# barcode_reader
A ROS nodelet to locate a barcode (ex. QR code) in a image

## System Dependencies
zbar
* `sudo apt-get install libzbar-dev`

object_detection_msgs
* https://github.com/yoshito-n-students/object_detection_msgs

## Subscribed Topics
image_raw (sensor_msgs/Image)

## Published Topics
image_out (sensor_msgs/Image)
* image in which barcodes are found
* never advertised and published unless ~republish_image is ture

barcodes_out (object_detection_msgs/Objects)
* data and location of detected barcodes

## Parameters
~scanner_configs (string array, default: \<detect QR code with position>)
* each string element is passed to zbar::ImageScanner::set_config()
* see zbar documentation for details

~scan_interval (double, default: 0.5)
* scan interval in seconds

~republish_image (bool, default: false)
* republish image if barcodes are found in it

~image_transport (string, default: "raw")
* transport type of the subscribed image topic

## Examples
see [launch/test.launch](launch/test.launch)