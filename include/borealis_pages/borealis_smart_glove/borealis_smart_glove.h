#pragma once
#include <ros/master.h>
#include <borealis_rviz/rviz.h>

#include <QAction>
#include <QImage>
#include <QList>
#include <QString>
#include <QSet>
#include <QSize>
#include <QWidget>
#include <QComboBox>
#include <QScrollArea>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QLabel>

#include <vector>
#include <memory>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>

namespace borealis_smart_glove {
    class BorealisSmartGlove : public QWidget {

        Q_OBJECT

        public:
            BorealisSmartGlove(ros::NodeHandlePtr rosNode);
            void selectGlovesTab();
            void exitPage();
            void enterPage();

            QVBoxLayout* setGloveSelectLayout();
            QFormLayout* setButtonLayout();
            void setRVizLayout();

        protected:
            QVBoxLayout* vertical_layout;
            QHBoxLayout* horizontal_layout;

            QVBoxLayout* vertical_rviz_layout;
            ros::NodeHandlePtr node;

            // Glove selection box
            QVBoxLayout* glove_vertical_layout;
            QFormLayout* button_layout;
            QPushButton* scan;
            QPushButton* disconnect;
            QComboBox* select_device;
            QComboBox* select_model;

            QLabel* click_to_scan;
            QLabel* click_to_disconnect;
            QLabel* select_device_to_connect;
            QLabel* select_model_to_use;
            
    };
}




