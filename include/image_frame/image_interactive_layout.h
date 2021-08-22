#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <ui_image_view.h>
#include <image_frame/ratio_layouted_frame.h>

#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/macros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

#include <opencv2/core/core.hpp>

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
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>



class ImageInteractiveLayout : public QVBoxLayout {

    Q_OBJECT

    public:
        ImageInteractiveLayout(QWidget* widget, ros::NodeHandlePtr rosNode);
        void setImageFrame();
        void setScrollWidget();
        void setBottomHorizontalLayout();
        void setTopHorizontalLayout();
        void setConnections();
        

        ~ImageInteractiveLayout();

        QVBoxLayout* vertical_layout_cover;
        QHBoxLayout* horizontal_layout_top;

        QComboBox* topics_combo_box;
        QPushButton* refresh_topics_button;
        QPushButton* save_as_image;

        QHBoxLayout* horizontal_layout_bottom;
        QScrollArea* scroll_area;
        QWidget* scroll_area_widget_contents;
        QHBoxLayout* horizontal_layout_image;

        borealis_gui::RatioLayoutedFrame* image_frame;

    public slots:
        void onTopicChanged(int index);
        void updateTopicList();
        // void saveImage();
        void onMouseLeft(int x, int y);
        void zoomInImage();

    protected:
        cv::Mat conversion_mat_;
        QWidget* parentWidget;
        // image_transport::ImageTransport it_;
        ros::NodeHandlePtr node;
        image_transport::Subscriber subscriber_;
        ros::Publisher pub_mouse_left_;
        bool pub_topic_custom_;

    private:
        void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
        void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
        
        
        QList<QString> getTopicList(const QSet<QString>& message_types, const QList<QString>& transports);
        QSet<QString> getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports);
        void selectTopic(const QString& topic);
        
        void onMousePublish(bool checked);
        void onPubTopicChanged();
        void callbackImage(const sensor_msgs::Image::ConstPtr& msg);

        
};


