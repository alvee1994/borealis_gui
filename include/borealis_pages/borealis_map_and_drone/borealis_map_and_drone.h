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

#include <vector>
#include <memory>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>

namespace borealis_map_and_drone {
    class BorealisMapAndDrone : public QWidget {

        Q_OBJECT

        public:
            BorealisMapAndDrone(ros::NodeHandlePtr rosNode);
            void droneVideoLayout();
            void exitPage();
            void enterPage();

        protected:
            QHBoxLayout* horizontalLayout;
            QVBoxLayout* verticalLayout;
            ros::NodeHandlePtr node;
            
    };
}
