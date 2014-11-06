#include "stereo_frame.hpp"
#include "ui_stereoFrame.h"
#include <sensor_msgs/image_encodings.h>

#include <QtCore/QDebug>
#include <QtCore/QString>
#include <QtGui/QFileDialog>

namespace enc = sensor_msgs::image_encodings;

StereoFrame::StereoFrame(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::StereoFrame)
{
    ui->setupUi(this);
    // Signals for Left Graphics View
    QObject::connect(   this,
                        SIGNAL(sig_image_received_left(const QImage &)),
                        ui->graphicsView_Left,
                        SLOT(image_received(const QImage &)));
    QObject::connect(   this,
                        SIGNAL(sig_tracked_object_changed_left(const QRectF &)),
                        ui->graphicsView_Left,
                        SLOT(tracked_objet_changed(const QRectF &)));
    // Signals for Right Graphics View
    QObject::connect(   this,
                        SIGNAL(sig_image_received_right(const QImage &)),
                        ui->graphicsView_Right,
                        SLOT(image_received(const QImage &)));
    QObject::connect(   this,
                        SIGNAL(sig_tracked_object_changed_right(const QRectF &)),
                        ui->graphicsView_Right,
                        SLOT(tracked_objet_changed(const QRectF &)));
    // Signals for Left and Right FPS trackers
    QObject::connect(   this,
                        SIGNAL(sig_fps_tracker_changed_left(int)),
                        ui->lcdNumber_Left,
                        SLOT(display(int)));
    QObject::connect(   this,
                        SIGNAL(sig_fps_tracker_changed_right(int)),
                        ui->lcdNumber_Right,
                        SLOT(display(int)));
    // Signals for Left and Right Confidence 
    QObject::connect(   this,
                        SIGNAL(sig_confidence_changed_left(int)),
                        ui->progressBar_ConfLeft,
                        SLOT(setValue(int)));
    QObject::connect(   this,
                        SIGNAL(sig_confidence_changed_right(int)),
                        ui->progressBar_ConfRight,
                        SLOT(setValue(int)));
    // Signals for UI push buttons
    QObject::connect(ui->pushButton_ResetBackground,    SIGNAL(clicked()),this,SLOT(clear_background()));
    QObject::connect(ui->pushButton_ToggleLearning,     SIGNAL(clicked()),this,SLOT(toggle_learning()));
    QObject::connect(ui->pushButton_AlternatingMode,    SIGNAL(clicked()),this,SLOT(alternating_mode()));
    QObject::connect(ui->pushButton_ToggleTracking,     SIGNAL(clicked()),this,SLOT(clear_and_stop_tracking()));
    QObject::connect(ui->pushButton_ImportModel,        SIGNAL(clicked()),this,SLOT(import_model()));
    QObject::connect(ui->pushButton_ExportModel,        SIGNAL(clicked()),this,SLOT(export_model()));
    QObject::connect(ui->pushButton_Reset,              SIGNAL(clicked()),this,SLOT(reset()));
    
    sub_ImageLeft = n.subscribe("image_left", 1000, &StereoFrame::imageReceivedCB_Left, this);
    sub_TrackedObjLeft = n.subscribe("tracked_object_left", 1000, &StereoFrame::trackedObjectCB_Left, this);
    sub_FPSLeft = n.subscribe("fps_tracker_left", 1000, &StereoFrame::fpsTrackerCB_Left, this);
    pub_BBLeft = n.advertise<tld_msgs::Target>("tld_stereo_bb_left", 1000, true);
    pub_CmdsLeft = n.advertise<std_msgs::Char>("tld_stereo_cmds_left", 1000, true);

    sub_ImageRight = n.subscribe("image_right", 1000, &StereoFrame::imageReceivedCB_Right, this);
    sub_TrackedObjRight = n.subscribe("tracked_object_right", 1000, &StereoFrame::trackedObjectCB_Right, this);
    sub_FPSRight = n.subscribe("fps_tracker_right", 1000, &StereoFrame::fpsTrackerCB_Right, this);
    pub_BBRight = n.advertise<tld_msgs::Target>("tld_stereo_bb_right", 1000, true);
    pub_CmdsRight = n.advertise<std_msgs::Char>("tld_stereo_cmds_right", 1000, true);
  
    first_image_left = true;
    first_image_right = true;
}

StereoFrame::~StereoFrame() {
    delete ui;
}

void StereoFrame::keyPressEvent(QKeyEvent * event)  {
  switch (event->key())  {
    case Qt::Key_Return:
    case Qt::Key_Enter:
    case Qt::Key_Backspace:
      qDebug() << "Enter";
      if(!ui->graphicsView_Left->get_bb()->rect().isEmpty() && 
         !ui->graphicsView_Right->get_bb()->rect().isEmpty()) {
        // create left BB
        tld_msgs::Target msgLeft;
        msgLeft.bb.x = (int)ui->graphicsView_Left->get_bb()->rect().x();
        msgLeft.bb.y = (int)ui->graphicsView_Left->get_bb()->rect().y();
        msgLeft.bb.width = (int)ui->graphicsView_Left->get_bb()->rect().width();
        msgLeft.bb.height = (int)ui->graphicsView_Left->get_bb()->rect().height();
        msgLeft.bb.confidence = 1.0;
        cv_ptr_Left->toImageMsg(msgLeft.img);
        // create right BB
        tld_msgs::Target msgRight;
        msgRight.bb.x = (int)ui->graphicsView_Right->get_bb()->rect().x();
        msgRight.bb.y = (int)ui->graphicsView_Right->get_bb()->rect().y();
        msgRight.bb.width = (int)ui->graphicsView_Right->get_bb()->rect().width();
        msgRight.bb.height = (int)ui->graphicsView_Right->get_bb()->rect().height();
        msgRight.bb.confidence = 1.0;
        cv_ptr_Right->toImageMsg(msgRight.img);
        // publish BBs
        pub_BBLeft.publish(msgLeft);
        pub_BBRight.publish(msgRight);
      } break;
    case Qt::Key_Q:
      qDebug() << "Quitting";
      close();
      break;
    case Qt::Key_B:
      clear_background();
      break;
    case Qt::Key_C:
      clear_and_stop_tracking();
      break;
    case Qt::Key_L:
      toggle_learning();
      break;
    case Qt::Key_A:
      alternating_mode();
      break;
    case Qt::Key_E:
      export_model();
      break;
    case Qt::Key_I:
      import_model();
      break;
    case Qt::Key_R:
      reset();
      break;
    case Qt::Key_F5:
      first_image_left = true;
      first_image_right = true;
    default:
      event->ignore();
      break;
  } // end switch
}

// Left callbacks
void StereoFrame::imageReceivedCB_Left(const sensor_msgs::ImageConstPtr & msg) {
  if(first_image_left || ui->graphicsView_Left->get_correct_bb())  {
    try {
      if (enc::isColor(msg->encoding))
        cv_ptr_Left = cv_bridge::toCvShare(msg, enc::RGB8);
      else
        cv_ptr_Left = cv_bridge::toCvShare(msg, enc::MONO8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    QImage image;

    if (enc::isColor(msg->encoding)) {
      image = QImage(   (const unsigned char*)(cv_ptr_Left->image.data),
                        cv_ptr_Left->image.cols,
                        cv_ptr_Left->image.rows,
                        QImage::Format_RGB888);
      //image.rgbSwapped();
    } else {
      image = QImage(   (const unsigned char*)(cv_ptr_Left->image.data),
                        cv_ptr_Left->image.cols,
                        cv_ptr_Left->image.rows,
                        QImage::Format_Indexed8);
    }

    emit sig_image_received_left(image);

    if(first_image_left) first_image_left = false;
  }
}

void StereoFrame::trackedObjectCB_Left(const tld_msgs::BoundingBoxConstPtr & msg) {
  if(msg->width && msg->height)
    first_image_left = true;
  else
    first_image_left = false;

    QRectF rect(msg->x,msg->y,msg->width,msg->height);
    emit sig_tracked_object_changed_left(rect);
    emit sig_confidence_changed_left((int)(msg->confidence*100));
}

void StereoFrame::fpsTrackerCB_Left(const std_msgs::Float32ConstPtr & msg) {
        emit sig_fps_tracker_changed_left((int)msg->data);
}

// Right callbacks
void StereoFrame::imageReceivedCB_Right(const sensor_msgs::ImageConstPtr & msg) {
  if(first_image_right || ui->graphicsView_Right->get_correct_bb())  {
    try {
      if (enc::isColor(msg->encoding))
        cv_ptr_Right = cv_bridge::toCvShare(msg, enc::RGB8);
      else
        cv_ptr_Right = cv_bridge::toCvShare(msg, enc::MONO8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    QImage image;

    if (enc::isColor(msg->encoding)) {
      image = QImage(   (const unsigned char*)(cv_ptr_Right->image.data),
                        cv_ptr_Right->image.cols,
                        cv_ptr_Right->image.rows,
                        QImage::Format_RGB888);
      //image.rgbSwapped();
    } else {
      image = QImage(   (const unsigned char*)(cv_ptr_Right->image.data),
                        cv_ptr_Right->image.cols,
                        cv_ptr_Right->image.rows,
                        QImage::Format_Indexed8);
    }

    emit sig_image_received_right(image);

    if(first_image_right) first_image_right = false;
  }
}

void StereoFrame::trackedObjectCB_Right(const tld_msgs::BoundingBoxConstPtr & msg) {
  if(msg->width && msg->height)
    first_image_right = true;
  else
    first_image_right = false;

    QRectF rect(msg->x,msg->y,msg->width,msg->height);
    emit sig_tracked_object_changed_right(rect);
    emit sig_confidence_changed_right((int)(msg->confidence*100));
}

void StereoFrame::fpsTrackerCB_Right(const std_msgs::Float32ConstPtr & msg) {
        emit sig_fps_tracker_changed_right((int)msg->data);
}

void StereoFrame::clear_background()
{
        std_msgs::Char cmd;
        cmd.data = 'b';
        pub_CmdsLeft.publish(cmd);
        pub_CmdsRight.publish(cmd);
        qDebug() << "Clearing Background";
}

void StereoFrame::clear_and_stop_tracking()
{
        std_msgs::Char cmd;
        cmd.data = 'c';
        pub_CmdsLeft.publish(cmd);
        pub_CmdsRight.publish(cmd);
        qDebug() << "Clearing and stop tracking";
}

void StereoFrame::toggle_learning()
{
        std_msgs::Char cmd;
        cmd.data = 'l';
        pub_CmdsLeft.publish(cmd);
        pub_CmdsRight.publish(cmd);
        qDebug() << "Toggle learning";
}

void StereoFrame::alternating_mode()
{
        std_msgs::Char cmd;
        cmd.data = 'a';
        pub_CmdsLeft.publish(cmd);
        pub_CmdsRight.publish(cmd);
        qDebug() << "Alternating mode";
}

void StereoFrame::export_model() {
 
        std_msgs::Char cmd;
        cmd.data = 'e';

        QString res = QFileDialog::getSaveFileName(this, tr("Choose a left model file name"), "/", tr("All (*)"));
        if(!res.isNull())  {
                ros::NodeHandle nh;
                nh.setParam("/ros_tld_tracker_left_node/modelExportFile", res.toStdString());
        } else {
          // The user pressed cancel or closed the dialog.
        }
        pub_CmdsLeft.publish(cmd);
        qDebug() << "Exported left model : " << res;

        res = QFileDialog::getSaveFileName(this, tr("Choose a right model file name"), "/", tr("All (*)"));
        if(!res.isNull())  {
                ros::NodeHandle nh;
                nh.setParam("/ros_tld_tracker_right_node/modelExportFile", res.toStdString());
        } else {
          // The user pressed cancel or closed the dialog.
        }
        pub_CmdsRight.publish(cmd);
        qDebug() << "Exported right model : " << res;
}

void StereoFrame::import_model()
{
        std_msgs::Char cmd;
        cmd.data = 'i';

        QString res = QFileDialog::getOpenFileName(this, tr("Choose a left model file to open"), "/", tr("All (*)"));
        if(!res.isNull()) {
          ros::NodeHandle nh;
          nh.setParam("/ros_tld_tracker_left_node/modelImportFile", res.toStdString());
        } else {
          // The user pressed cancel or closed the dialog.
        }
        pub_CmdsLeft.publish(cmd);
        qDebug() << "Importing left model : " << res;

        res = QFileDialog::getOpenFileName(this, tr("Choose a right model file to open"), "/", tr("All (*)"));
        if(!res.isNull()) {
          ros::NodeHandle nh;
          nh.setParam("/ros_tld_tracker_right_node/modelImportFile", res.toStdString());
        } else {
          // The user pressed cancel or closed the dialog.
        }
        pub_CmdsRight.publish(cmd);
        qDebug() << "Importing right model : " << res;
}

void StereoFrame::reset()
{
        std_msgs::Char cmd;
        cmd.data = 'r';
        pub_CmdsLeft.publish(cmd);
        pub_CmdsRight.publish(cmd);
        qDebug() << "Reset";
}

