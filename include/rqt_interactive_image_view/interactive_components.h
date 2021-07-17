#pragma once

#include <rqt_interactive_image_view/image_view.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_image_view.h>

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
        ImageInteractiveLayout();
        void initLayout(Ui::ImageViewWidget& theui, QWidget* widget, ros::NodeHandlePtr rosNode, rqt_interactive_image_view::ImageView* contextId);
        void setImageFrame();
        void setScrollWidget();
        void setBottomHorizontalLayout();
        void setTopHorizontalLayout();
        void setConnections(rqt_interactive_image_view::ImageView* contextId);

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

        rqt_interactive_image_view::RatioLayoutedFrame* image_frame;

    public slots:
        void onTopicChanged(int index);
        void updateTopicList();
        void saveImage();
        void onMouseLeft(int x, int y);

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


