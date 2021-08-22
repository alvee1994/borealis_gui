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

#include <main_gui.h>
#include <image_frame/image_interactive_layout.h>
#include <borealis_pages/borealis_map_and_drone/borealis_map_and_drone.h>
#include <borealis_pages/borealis_smart_glove/borealis_smart_glove.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/program_options.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>


namespace borealis_gui {

BorealisGui::BorealisGui() 
: rqt_gui_cpp::Plugin()
, widget_(0)
, num_gridlines_(0)
, rotate_state_(ROTATE_0)
{
  setObjectName("BorealisGui");
}

void BorealisGui::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QStackedWidget();
  
  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>(getNodeHandle());
  image_transport::ImageTransport it(*node);

  std::shared_ptr<borealis_rviz::RViz> rvizFrame = std::make_shared<borealis_rviz::RViz>();
  parseArguments(context, rvizFrame);
  rvizFrame->initPlugin(context);

  // borealis_map_and_drone::BorealisMapAndDrone* map_and_drone_page = new borealis_map_and_drone::BorealisMapAndDrone(node, rvizFrame);
  // widget_->addWidget(map_and_drone_page);

  borealis_smart_glove::BorealisSmartGlove* smart_glove_page = new borealis_smart_glove::BorealisSmartGlove(node, rvizFrame);
  widget_->addWidget(smart_glove_page);
  
};



void BorealisGui::parseArguments(qt_gui_cpp::PluginContext& context, std::shared_ptr<borealis_rviz::RViz> rvizF)
{
  namespace po = boost::program_options;

  const QStringList& qargv = context.argv();

  const int argc = qargv.count();

  // temporary storage for args obtained from qargv - since each QByteArray
  // owns its storage, we need to keep these around until we're done parsing
  // args using boost::program_options
  std::vector<QByteArray> argv_array;
  std::vector<const char *> argv(argc+1);
  argv[0] = ""; // dummy program name

  for (int i = 0; i < argc; ++i)
  {
    argv_array.push_back(qargv.at(i).toLocal8Bit());
    argv[i+1] = argv_array[i].constData();
  }

  po::variables_map vm;
  po::options_description options;
  options.add_options()
    ("display-config,d", po::value<std::string>(), "")
    ("topic,t", po::value<std::string>(), "")
    ("hide-menu,m", "")
    ("ogre-log,l", "");


  try
  {
    po::store(po::parse_command_line(argc+1, argv.data(), options), vm);
    po::notify(vm);

    if (vm.count("hide-menu"))
    {
      rvizF->hide_menu_ = true;
    }

    if (vm.count("display-config"))
    {
      rvizF->display_config_ = vm["display-config"].as<std::string>();
    }

    if (vm.count("ogre-log"))
    {
      rvizF->ogre_log_ = true;
    }
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Error parsing command line: %s", e.what());
  }
};


void BorealisGui::shutdownPlugin()
{
  // subscriber_.shutdown();
  // pub_mouse_left_.shutdown();
  std::cout << "shutting down plugin\n";
};
}

PLUGINLIB_EXPORT_CLASS(borealis_gui::BorealisGui, rqt_gui_cpp::Plugin)
