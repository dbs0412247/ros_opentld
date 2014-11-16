#ifndef STEREOFRAME_H
#define STEREOFRAME_H

#include <QMainWindow>
#include <QtGui/QWidget>
#include <QtGui/QKeyEvent>
#include <QtCore/QObject>
#include <QtGui/QImage>
#include <QtCore/QRectF>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/connection.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tld_msgs/Target.h>
#include <tld_msgs/BoundingBox.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include <opencv2/core/core.hpp>

#include "ui_stereoFrame.h"

namespace Ui {
  class StereoFrame;
}

class StereoFrame : public QMainWindow
{
  Q_OBJECT

public:

  ros::NodeHandle n;
  // Left and Right bounding box publishers
  ros::Publisher pub_BBLeft;
  ros::Publisher pub_BBRight;
  // Left and Right keyboard command publishers
  ros::Publisher pub_CmdsLeft;
  ros::Publisher pub_CmdsRight;
  // Left and Right tracked object subscribers
  // ros::Subscriber sub_TrackedObjLeft;
  // ros::Subscriber sub_TrackedObjRight;
  message_filters::Subscriber<tld_msgs::BoundingBox> sub_TrackedObjLeft;
  message_filters::Subscriber<tld_msgs::BoundingBox> sub_TrackedObjRight;
  // An object to synchronize received tracked object for 3D position calculations
  message_filters::TimeSynchronizer<tld_msgs::BoundingBox, 
                                    tld_msgs::BoundingBox> sync_TrackedObj;
  // Left and Right tracking fps subscribers
  ros::Subscriber sub_FPSLeft;
  ros::Subscriber sub_FPSRight;
  // Left and Right image topic subscribers
  ros::Subscriber sub_ImageLeft;
  ros::Subscriber sub_ImageRight;
  // Left and Right CameraInfo subscribers
  ros::Subscriber sub_CameraInfoLeft;
  ros::Subscriber sub_CameraInfoRight;

  // Left and Right camera intrinsics and extrinsics
  cv::Mat matProjLeft, matProjRight;

  explicit StereoFrame(QWidget *parent = 0);
  ~StereoFrame();

protected:
  void keyPressEvent(QKeyEvent * event);

private:
  cv_bridge::CvImageConstPtr cv_ptr_Left;
  cv_bridge::CvImageConstPtr cv_ptr_Right;
  bool first_image_left;
  bool first_image_right;

  void imageReceivedCB_Left(const sensor_msgs::ImageConstPtr & msg);
  void camInfoReceivedCB_Left(const sensor_msgs::CameraInfo & msg);
  //void trackedObjectCB_Left(const tld_msgs::BoundingBoxConstPtr & msg);
  void fpsTrackerCB_Left(const std_msgs::Float32ConstPtr & msg);
  void imageReceivedCB_Right(const sensor_msgs::ImageConstPtr & msg);
  void camInfoReceivedCB_Right(const sensor_msgs::CameraInfo & msg);
  //void trackedObjectCB_Right(const tld_msgs::BoundingBoxConstPtr & msg);
  void fpsTrackerCB_Right(const std_msgs::Float32ConstPtr & msg);
  void trackedObjectCB_Sync(const tld_msgs::BoundingBoxConstPtr & msgLeft,
                            const tld_msgs::BoundingBoxConstPtr & msgRight);

signals:
  void sig_image_received_left(const QImage & image);
  void sig_image_received_right(const QImage & image);
  void sig_tracked_object_changed_left(const QRectF & bb);
  void sig_tracked_object_changed_right(const QRectF & bb);
  void sig_fps_tracker_changed_left(int fps);
  void sig_fps_tracker_changed_right(int fps);
  void sig_confidence_changed_left(int confidence);
  void sig_confidence_changed_right(int confidence);

public slots:
  void clear_background();
  void clear_and_stop_tracking();
  void toggle_learning();
  void alternating_mode();
  void export_model();
  void import_model();
  void reset();

private:
  Ui::StereoFrame *ui;
};

#endif // STEREOFRAME_H
