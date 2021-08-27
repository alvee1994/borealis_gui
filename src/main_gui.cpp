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
  widget_ = new QWidget();
  
  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  node_handle_pointer = boost::make_shared<ros::NodeHandle>(getNodeHandle());
  image_transport::ImageTransport it(*node_handle_pointer);

  rviz_frame = std::make_shared<borealis_rviz::RViz>();
  parseArguments(context, rviz_frame);
  rviz_frame->initPlugin(context);


  vertical_layout = new QVBoxLayout(widget_);
  horizontal_layout = new QHBoxLayout();
  stacked_widget = new QStackedWidget();
  tab = new QTabWidget();
  
  // instantiate the pages
  smart_glove_page = new borealis_smart_glove::BorealisSmartGlove(node_handle_pointer);
  stacked_widget->addWidget(smart_glove_page);
  map_and_drone_page = new borealis_map_and_drone::BorealisMapAndDrone(node_handle_pointer);
  stacked_widget->addWidget(map_and_drone_page);

  // the RViz tab
  setRvizTab();

  // add widgets and layouts to the main layout
  horizontal_layout->addWidget(stacked_widget);
  horizontal_layout->addWidget(tab, 2);
  vertical_layout->addLayout(horizontal_layout);
  vertical_layout->addLayout(setNavigationButtonLayout());

  // set the connections
  setNavigationConnections();
};

void BorealisGui::setRvizTab()
{
    tab->insertTab(0, rviz_frame->rvizFrameWidget_, "RVizTab");
    ImageInteractiveLayout* myTabLayout_D = new ImageInteractiveLayout(this->widget_, node_handle_pointer);
    tab->insertTab(1, myTabLayout_D->scroll_area, "image_viewer");
    
};

QFormLayout* BorealisGui::setNavigationButtonLayout(){
  smart_glove_button = new QPushButton("Smart Glove");
  map_and_drone_button = new QPushButton("Map and Drone");

  navigation_button_layout = new QFormLayout();
  navigation_button_layout->addRow(smart_glove_button, map_and_drone_button);

  return navigation_button_layout;
}

void BorealisGui::setNavigationConnections(){
  connect(this->map_and_drone_button, SIGNAL(pressed()), this, SLOT(mapAndDronePage()));
  connect(this->smart_glove_button, SIGNAL(pressed()), this, SLOT(smartGlovePage()));
}

// best to not to the following. have a way for pages to know if there are current
void BorealisGui::smartGlovePage(){
  this->map_and_drone_page->exitPage();
  this->smart_glove_page->enterPage();
  this->stacked_widget->setCurrentIndex(0);
}

void BorealisGui::mapAndDronePage(){
  this->smart_glove_page->exitPage();
  this->map_and_drone_page->enterPage();
  this->stacked_widget->setCurrentIndex(1);
}

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
