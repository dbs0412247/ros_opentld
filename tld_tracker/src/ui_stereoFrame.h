/********************************************************************************
** Form generated from reading UI file 'stereoframe-qt4.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
**
** This file is editted to provide support to BaseFrameGraphicsView
********************************************************************************/

#ifndef UI_STEREOFRAME_2D_QT4_H
#define UI_STEREOFRAME_2D_QT4_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGraphicsView>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "base_frame_graphics_view.hpp"

QT_BEGIN_NAMESPACE

class Ui_StereoFrame
{
public:
    QWidget *centralWidget;
    QLabel *label;
    QLabel *label_2;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QProgressBar *progressBar_ConfLeft;
    QProgressBar *progressBar_ConfRight;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout_2;
    QLCDNumber *lcdNumber_Left;
    QLCDNumber *lcdNumber_Right;
    QWidget *layoutWidget2;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_ToggleLearning;
    QPushButton *pushButton_ImportModel;
    QPushButton *pushButton_ExportModel;
    QPushButton *pushButton_AlternatingMode;
    QPushButton *pushButton_ToggleTracking;
    QPushButton *pushButton_ResetBackground;
    QPushButton *pushButton_Reset;
    QWidget *layoutWidget3;
    QHBoxLayout *horizontalLayout_3;
    BaseFrameGraphicsView *graphicsView_Left;
    BaseFrameGraphicsView *graphicsView_Right;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *StereoFrame)
    {
        if (StereoFrame->objectName().isEmpty())
            StereoFrame->setObjectName(QString::fromUtf8("StereoFrame"));
        StereoFrame->resize(1520, 551);
        centralWidget = new QWidget(StereoFrame);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(1341, 0, 141, 16));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(1356, 139, 111, 21));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(1380, 20, 62, 97));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setSpacing(10);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        progressBar_ConfLeft = new QProgressBar(layoutWidget);
        progressBar_ConfLeft->setObjectName(QString::fromUtf8("progressBar_ConfLeft"));
        progressBar_ConfLeft->setLayoutDirection(Qt::LeftToRight);
        progressBar_ConfLeft->setValue(24);
        progressBar_ConfLeft->setTextVisible(false);
        progressBar_ConfLeft->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(progressBar_ConfLeft);

        progressBar_ConfRight = new QProgressBar(layoutWidget);
        progressBar_ConfRight->setObjectName(QString::fromUtf8("progressBar_ConfRight"));
        progressBar_ConfRight->setLayoutDirection(Qt::LeftToRight);
        progressBar_ConfRight->setValue(24);
        progressBar_ConfRight->setTextVisible(false);
        progressBar_ConfRight->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(progressBar_ConfRight);

        layoutWidget1 = new QWidget(centralWidget);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(1305, 160, 211, 51));
        horizontalLayout_2 = new QHBoxLayout(layoutWidget1);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        lcdNumber_Left = new QLCDNumber(layoutWidget1);
        lcdNumber_Left->setObjectName(QString::fromUtf8("lcdNumber_Left"));

        horizontalLayout_2->addWidget(lcdNumber_Left);

        lcdNumber_Right = new QLCDNumber(layoutWidget1);
        lcdNumber_Right->setObjectName(QString::fromUtf8("lcdNumber_Right"));

        horizontalLayout_2->addWidget(lcdNumber_Right);

        layoutWidget2 = new QWidget(centralWidget);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(1320, 246, 181, 241));
        verticalLayout = new QVBoxLayout(layoutWidget2);
        verticalLayout->setSpacing(4);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        pushButton_ToggleLearning = new QPushButton(layoutWidget2);
        pushButton_ToggleLearning->setObjectName(QString::fromUtf8("pushButton_ToggleLearning"));

        verticalLayout->addWidget(pushButton_ToggleLearning);

        pushButton_ImportModel = new QPushButton(layoutWidget2);
        pushButton_ImportModel->setObjectName(QString::fromUtf8("pushButton_ImportModel"));

        verticalLayout->addWidget(pushButton_ImportModel);

        pushButton_ExportModel = new QPushButton(layoutWidget2);
        pushButton_ExportModel->setObjectName(QString::fromUtf8("pushButton_ExportModel"));

        verticalLayout->addWidget(pushButton_ExportModel);

        pushButton_AlternatingMode = new QPushButton(layoutWidget2);
        pushButton_AlternatingMode->setObjectName(QString::fromUtf8("pushButton_AlternatingMode"));

        verticalLayout->addWidget(pushButton_AlternatingMode);

        pushButton_ToggleTracking = new QPushButton(layoutWidget2);
        pushButton_ToggleTracking->setObjectName(QString::fromUtf8("pushButton_ToggleTracking"));

        verticalLayout->addWidget(pushButton_ToggleTracking);

        pushButton_ResetBackground = new QPushButton(layoutWidget2);
        pushButton_ResetBackground->setObjectName(QString::fromUtf8("pushButton_ResetBackground"));

        verticalLayout->addWidget(pushButton_ResetBackground);

        pushButton_Reset = new QPushButton(layoutWidget2);
        pushButton_Reset->setObjectName(QString::fromUtf8("pushButton_Reset"));

        verticalLayout->addWidget(pushButton_Reset);

        layoutWidget3 = new QWidget(centralWidget);
        layoutWidget3->setObjectName(QString::fromUtf8("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(10, 0, 1288, 482));
        horizontalLayout_3 = new QHBoxLayout(layoutWidget3);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        graphicsView_Left = new BaseFrameGraphicsView(layoutWidget3);
        graphicsView_Left->setObjectName(QString::fromUtf8("graphicsView_Left"));
        graphicsView_Left->setMinimumSize(QSize(640, 480));

        horizontalLayout_3->addWidget(graphicsView_Left);

        graphicsView_Right = new BaseFrameGraphicsView(layoutWidget3);
        graphicsView_Right->setObjectName(QString::fromUtf8("graphicsView_Right"));
        graphicsView_Right->setMinimumSize(QSize(640, 480));

        horizontalLayout_3->addWidget(graphicsView_Right);

        StereoFrame->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(StereoFrame);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1520, 25));
        StereoFrame->setMenuBar(menuBar);
        mainToolBar = new QToolBar(StereoFrame);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        StereoFrame->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(StereoFrame);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        StereoFrame->setStatusBar(statusBar);

        retranslateUi(StereoFrame);

        QMetaObject::connectSlotsByName(StereoFrame);
    } // setupUi

    void retranslateUi(QMainWindow *StereoFrame)
    {
        StereoFrame->setWindowTitle(QApplication::translate("StereoFrame", "StereoFrame", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("StereoFrame", "Trackers' Confidences", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("StereoFrame", "FPS of Trackers:", 0, QApplication::UnicodeUTF8));
        pushButton_ToggleLearning->setText(QApplication::translate("StereoFrame", "Toggle Learning", 0, QApplication::UnicodeUTF8));
        pushButton_ImportModel->setText(QApplication::translate("StereoFrame", "Import Model", 0, QApplication::UnicodeUTF8));
        pushButton_ExportModel->setText(QApplication::translate("StereoFrame", "Export Model", 0, QApplication::UnicodeUTF8));
        pushButton_AlternatingMode->setText(QApplication::translate("StereoFrame", "Alternating Mode", 0, QApplication::UnicodeUTF8));
        pushButton_ToggleTracking->setText(QApplication::translate("StereoFrame", "Start/Stop Tracking", 0, QApplication::UnicodeUTF8));
        pushButton_ResetBackground->setText(QApplication::translate("StereoFrame", "Reset Background", 0, QApplication::UnicodeUTF8));
        pushButton_Reset->setText(QApplication::translate("StereoFrame", "Reset", 0, QApplication::UnicodeUTF8));
        mainToolBar->setWindowTitle(QApplication::translate("StereoFrame", "TLD Stereo GUI", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class StereoFrame: public Ui_StereoFrame {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_STEREOFRAME_2D_QT4_H
