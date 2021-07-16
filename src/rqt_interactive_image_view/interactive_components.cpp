#include <rqt_interactive_image_view/interactive_components.h>

namespace rqt_interactive_image_view{

void debugPrint(int lineNumber){
  std::cout << "this is line " << lineNumber << std::endl;
};

ImageInteractiveLayout::ImageInteractiveLayout(){
  std::cout << "initiated image interactive\n";
};

void ImageInteractiveLayout::initLayout(Ui::ImageViewWidget& theui, QWidget* widget, ros::NodeHandlePtr rosNode, ImageView* contextId){

  
    // toolbar widget -> qvboxlayout -> qhboxlayout -> [qcombobox, qpushbutton, qpushbutton]
    //                               -> qhboxlayout -> [scrollarea, qwidget, [qhboxlayout -> ratiolayoutedframe]]



  //
  node = rosNode;
  parentWidget = widget;
  vertical_layout_cover = new QVBoxLayout();

  setTopHorizontalLayout();
  setBottomHorizontalLayout();

  vertical_layout_cover->addLayout(horizontal_layout_top);
  vertical_layout_cover->addLayout(horizontal_layout_bottom);

  this->addLayout(vertical_layout_cover);
  theui.horizontalLayout->addLayout(this);

  setConnections(contextId);

};

void ImageInteractiveLayout::setConnections(ImageView* contextId){
  updateTopicList();
  this->topics_combo_box->setCurrentIndex(this->topics_combo_box->findText(""));
  connect(this->topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

  this->refresh_topics_button->setIcon(QIcon::fromTheme("view-refresh"));
  connect(this->refresh_topics_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));

  this->save_as_image->setIcon(QIcon::fromTheme("document-save-as"));
  connect(this->save_as_image, SIGNAL(pressed()), this, SLOT(saveImage()));

  this->image_frame->setInnerFrameMinimumSize(QSize(80, 60));
  this->image_frame->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));

  pub_topic_custom_ = false;

  this->image_frame->setOuterLayout(this->horizontal_layout_bottom);

  onMousePublish(true);
  connect(this->image_frame, SIGNAL(mouseLeft(int, int)), this, SLOT(onMouseLeft(int, int)));
  // this->image_frame->addAction(hide_toolbar_action_);
  // connect(hide_toolbar_action_, SIGNAL(toggled(bool)), this, SLOT(onHideToolbarChanged(bool)));
};

void ImageInteractiveLayout::setTopHorizontalLayout(){
  
  horizontal_layout_top = new QHBoxLayout();

  topics_combo_box = new QComboBox();
  topics_combo_box->setSizeAdjustPolicy(QComboBox::AdjustToContents);

  refresh_topics_button = new QPushButton();
  refresh_topics_button->setToolTip("Refresh Topics");
  refresh_topics_button->setIcon(QIcon::fromTheme("view-refresh"));

  save_as_image = new QPushButton();
  save_as_image->setToolTip("Save As Image");
  save_as_image->setIcon(QIcon::fromTheme("image-x-generic"));
  save_as_image->setIconSize(QSize(25,25));

  horizontal_layout_top->addWidget(topics_combo_box);
  horizontal_layout_top->addWidget(refresh_topics_button);
  horizontal_layout_top->addWidget(save_as_image);

  horizontal_layout_top->insertSpacing(-1,20);
  
};

void ImageInteractiveLayout::setBottomHorizontalLayout(){

  horizontal_layout_bottom = new QHBoxLayout();
  horizontal_layout_bottom->setSpacing(0);

  horizontal_layout_image = new QHBoxLayout();

  setImageFrame();  
  setScrollWidget();

  horizontal_layout_image->setMargin(1);
  horizontal_layout_image->addWidget(image_frame);

  horizontal_layout_bottom->addWidget(scroll_area);
}

void ImageInteractiveLayout::setImageFrame(){
  image_frame = new RatioLayoutedFrame(scroll_area_widget_contents);
  image_frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  image_frame->setMinimumSize(80, 60);
  image_frame->setContextMenuPolicy(Qt::ActionsContextMenu);
  image_frame->setFrameShape(QFrame::NoFrame);
  image_frame->setLineWidth(1);
};

void ImageInteractiveLayout::setScrollWidget(){
  scroll_area = new QScrollArea();
  scroll_area_widget_contents = new QWidget();

  scroll_area_widget_contents->setGeometry(0,0,767,650);
  scroll_area_widget_contents->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  scroll_area_widget_contents->setLayout(horizontal_layout_image);

  scroll_area->setWidget(scroll_area_widget_contents);
  scroll_area->setFrameShape(QFrame::NoFrame);
  scroll_area->setWidgetResizable(true);

};

void ImageInteractiveLayout::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const{
  QString topic = this->topics_combo_box->currentText();
  instance_settings.setValue("topic", topic);
};

void ImageInteractiveLayout::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings){
  QString topic = instance_settings.value("topic", "").toString();
  selectTopic(topic);
};

void ImageInteractiveLayout::selectTopic(const QString& topic)
{
  int index = this->topics_combo_box->findText(topic);
  if (index == -1)
  {
    // add topic name to list if not in
    QString label(topic);
    label.replace(" ", "/");
    this->topics_combo_box->addItem(label, QVariant(topic));
    index = this->topics_combo_box->findText(topic);
  }
  this->topics_combo_box->setCurrentIndex(index);
}

void ImageInteractiveLayout::updateTopicList(){
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");
  QSet<QString> message_sub_types;
  message_sub_types.insert("sensor_msgs/CompressedImage");

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it_(*node);
  std::vector<std::string> declared = it_.getDeclaredTransports();

  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++) {
    QString transport = it->c_str();

    QString prefix = "image_transport/";
    if (transport.startsWith(prefix)){
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QString selected = this->topics_combo_box->currentText();

  // fill combo box
  QList<QString> topics = getTopics(message_types, message_sub_types, transports).values();
  topics.append("");
  qSort(topics);
  this->topics_combo_box->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    this->topics_combo_box->addItem(label, QVariant(*it));
  }

  // restore previous selection
  selectTopic(selected);
}


void ImageInteractiveLayout::onPubTopicChanged()
{
  // pub_topic_custom_ = !(this->publish_click_location_topic_line_edit->text().isEmpty());
  // onMousePublish(this->publish_click_location_check_box->isChecked());
  onMousePublish(true);
}

void ImageInteractiveLayout::onTopicChanged(int index)
{
  conversion_mat_.release();
  subscriber_.shutdown();
  // reset image on topic change
  this->image_frame->setImage(QImage());

  QStringList parts = this->topics_combo_box->itemData(index).toString().split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty())
  {
    image_transport::ImageTransport it_(*node);
    image_transport::TransportHints hints(transport.toStdString());
    try {
      subscriber_ = it_.subscribe(topic.toStdString(), 1, &ImageInteractiveLayout::callbackImage, this, hints);
    } catch(image_transport::TransportLoadException& e) {
      QMessageBox::warning(parentWidget, tr("loading image transport plugin failed"), e.what());
    }
  }

  // onMousePublish(this->publish_click_location_check_box->isChecked());
  onMousePublish(true);
}

void ImageInteractiveLayout::onMousePublish(bool checked)
{
  std::string topicName;
  if(!subscriber_.getTopic().empty())
  {
    topicName = subscriber_.getTopic()+"_mouse_left";
  } else {
    topicName = "mouse_left";
  }
  

  if(checked){
    pub_mouse_left_ = node->advertise<geometry_msgs::Point>(topicName, 1000);
  } else {
    pub_mouse_left_.shutdown();
  }
}

void ImageInteractiveLayout::onMouseLeft(int x, int y)
{
  // if(this->publish_click_location_check_box->isChecked() && !this->image_frame->getImage().isNull())
  if(!this->image_frame->getImage().isNull())
  {
    geometry_msgs::Point clickCanvasLocation;
    // publish location in pixel coordinates
    clickCanvasLocation.x = round((double)x/(double)this->image_frame->width()*(double)this->image_frame->getImage().width());
    clickCanvasLocation.y = round((double)y/(double)this->image_frame->height()*(double)this->image_frame->getImage().height());
    clickCanvasLocation.z = 0;

    geometry_msgs::Point clickLocation = clickCanvasLocation;
    pub_mouse_left_.publish(clickLocation);
  }
}




QList<QString> ImageInteractiveLayout::getTopicList(const QSet<QString>& message_types, const QList<QString>& transports)
{
  QSet<QString> message_sub_types;
  return getTopics(message_types, message_sub_types, transports).values();
}




QSet<QString> ImageInteractiveLayout::getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports)
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    all_topics.insert(it->name.c_str());
  }

  QSet<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    if (message_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();


      // add raw topic
      topics.insert(topic);

      // add transport specific sub topics
      for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.insert(sub);
          //qDebug("ImageInteractiveLayout::getTopics() transport specific sub-topic '%s'", sub.toStdString().c_str());
        }
      }
    }

    if (message_sub_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();
      int index = topic.lastIndexOf("/");
      if (index != -1)
      {
        topic.replace(index, 1, " ");
        topics.insert(topic);
        //qDebug("ImageInteractiveLayout::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
      }
    }
  }
  return topics;
}

void ImageInteractiveLayout::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_ = cv_ptr->image;
      } else if (msg->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
      } 
      // else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
      //   // scale / quantify
      //   double min = 0;
      //   double max = this->max_range_double_spin_box->value();
      //   if (msg->encoding == "16UC1") max *= 1000;
      //   if (this->dynamic_range_check_box->isChecked())
      //   {
      //     // dynamically adjust range based on min/max in image
      //     cv::minMaxLoc(cv_ptr->image, &min, &max);
      //     if (min == max) {
      //       // completely homogeneous images are displayed in gray
      //       min = 0;
      //       max = 2;
      //     }
      //   }
      //   cv::Mat img_scaled_8u;
      //   cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));

      //   const auto color_scheme_index = this->color_scheme_combo_box->currentIndex();
      //   const auto color_scheme = this->color_scheme_combo_box->itemData(color_scheme_index).toInt();
      //   if (color_scheme == -1) {
      //     cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
      //   } else {
      //     cv::Mat img_color_scheme;
      //     cv::applyColorMap(img_scaled_8u, img_color_scheme, color_scheme);
      //     cv::cvtColor(img_color_scheme, conversion_mat_, CV_BGR2RGB);
      //   }
      // } 
      else {
        qWarning("ImageInteractiveLayout.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
        this->image_frame->setImage(QImage());
        return;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("ImageInteractiveLayout.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
      this->image_frame->setImage(QImage());
      return;
    }
  }
  
  // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
  this->image_frame->setImage(image);

  // if (!this->zoom_1_push_button->isEnabled())
  // {
  //   this->zoom_1_push_button->setEnabled(true);
  // }
  // Need to update the zoom 1 every new image in case the image aspect ratio changed,
  // though could check and see if the aspect ratio changed or not.
  // onZoom1(this->zoom_1_push_button->isChecked());
}

ImageInteractiveLayout::~ImageInteractiveLayout(){
  std::cout << "image interactive layout destructor " << this << "\n";
  // delete vertical_layout_cover;
  // delete horizontal_layout_top;

  // delete topics_combo_box;
  // delete refresh_topics_button;
  // delete save_as_image;

  // delete horizontal_layout_bottom;
  // delete scroll_area;
  // delete scroll_area_widget_contents;
  // delete horizontal_layout_image;

  // delete image_frame;
};
}