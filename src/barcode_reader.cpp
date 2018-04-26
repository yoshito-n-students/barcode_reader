#include <barcode_reader/barcode_reader.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(barcode_reader::BarcodeReader, nodelet::Nodelet);