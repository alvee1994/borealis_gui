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

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>

namespace borealis_smart_glove {
    class BorealisSmartGlove : public QWidget {

        Q_OBJECT

        public:
            BorealisSmartGlove(ros::NodeHandlePtr rosNode, std::shared_ptr<borealis_rviz::RViz> rvizPtr);
            void selectGlovesTab();
            void typeNewModelName();

            QVBoxLayout* setGloveSelectLayout();
            void setRVizLayout();

        protected:
            QVBoxLayout* verticalLayout;
            QHBoxLayout* horizontalLayout;

            QVBoxLayout* verticalRVizLayout;
            std::shared_ptr<borealis_rviz::RViz> rvizPointer;
            ros::NodeHandlePtr node;

            // Glove selection box
            QVBoxLayout* gloveVerticalLayout;
            QPushButton* scan;
            QPushButton* disconnect;
            QComboBox* selectDevice;
            QComboBox* selectModel;
            
    };
}




