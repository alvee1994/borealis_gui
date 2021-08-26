/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef borealis_gui__BorealisGui_H
#define borealis_gui__BorealisGui_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_image_view.h>
#include <borealis_rviz/rviz.h>

#include <borealis_pages/borealis_map_and_drone/borealis_map_and_drone.h>
#include <borealis_pages/borealis_smart_glove/borealis_smart_glove.h>

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
#include <QTabWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QPushButton>
#include <QFormLayout>

#include <vector>
#include <memory>

namespace borealis_gui {

class BorealisGui
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  BorealisGui();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  void parseArguments(qt_gui_cpp::PluginContext& context, std::shared_ptr<borealis_rviz::RViz> rvizF);
  void setNavigationConnections();
  void setRvizTab();
  

  QFormLayout* setNavigationButtonLayout();

  QStackedWidget* stacked_widget;
  QWidget* widget_;
  image_transport::Subscriber subscriber_;
  cv::Mat conversion_mat_;


  // bool hide_menu_;
  // std::string display_config_;
  // bool ogre_log_;

public slots:
  void smartGlovePage();
  void mapAndDronePage();

private:

  QTabWidget* tab;
  std::shared_ptr<borealis_rviz::RViz> rviz_frame;
  ros::NodeHandlePtr node_handle_pointer;

  borealis_map_and_drone::BorealisMapAndDrone* map_and_drone_page;
  borealis_smart_glove::BorealisSmartGlove* smart_glove_page;
  
  QHBoxLayout* horizontal_layout;
  QVBoxLayout* vertical_layout;
  QFormLayout* navigation_button_layout;

  QPushButton* smart_glove_button;
  QPushButton* map_and_drone_button;

  enum RotateState {
    ROTATE_0 = 0,
    ROTATE_90 = 1,
    ROTATE_180 = 2,
    ROTATE_270 = 3,

    ROTATE_STATE_COUNT
  };

  // void syncRotateLabel();

  QString arg_topic_name;
  ros::Publisher pub_mouse_left_;

  bool pub_topic_custom_;

  QAction* hide_toolbar_action_;

  int num_gridlines_;

  RotateState rotate_state_;
};

}

#endif // borealis_gui__BorealisGui_H
