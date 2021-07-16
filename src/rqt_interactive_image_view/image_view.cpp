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

#include <rqt_interactive_image_view/image_view.h>
#include <rqt_interactive_image_view/interactive_components.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <memory>

namespace rqt_interactive_image_view {

ImageView::ImageView() 
: rqt_gui_cpp::Plugin()
, widget_(0)
, num_gridlines_(0)
, rotate_state_(ROTATE_0)
{
  setObjectName("ImageView");
}

void ImageView::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  hide_toolbar_action_ = new QAction(tr("Hide toolbar"), this);
  hide_toolbar_action_->setCheckable(false);

  // ros::NodeHandle node(getNodeHandle());
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>(getNodeHandle());
  image_transport::ImageTransport it(*node);

  ImageInteractiveLayout* mylayout_A = new ImageInteractiveLayout();
  mylayout_A->initLayout(ui_, widget_, node, this);

  ImageInteractiveLayout* mylayout_B = new ImageInteractiveLayout();
  mylayout_B->initLayout(ui_, widget_, node, this);

  ImageInteractiveLayout* mylayout_C = new ImageInteractiveLayout();
  mylayout_C->initLayout(ui_, widget_, node, this);
};


void ImageView::shutdownPlugin()
{
  subscriber_.shutdown();
  pub_mouse_left_.shutdown();
};
}

PLUGINLIB_EXPORT_CLASS(rqt_interactive_image_view::ImageView, rqt_gui_cpp::Plugin)
