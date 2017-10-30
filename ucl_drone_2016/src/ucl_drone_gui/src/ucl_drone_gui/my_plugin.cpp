/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques
 *  \date 2016
 *
 */

#include "ucl_drone_gui/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace ucl_drone_gui
{
MyPlugin::MyPlugin() : rqt_gui_cpp::Plugin(), widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("ucl_drone_gui");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  roles_channel = getNodeHandle().resolveName("/drones_roles");
  roles_sub = getNodeHandle().subscribe(roles_channel, 1, &MyPlugin::rolesCb, this);

  // connect each button defined in my_plugin.ui to a method of  the current MyPlugin object
  connect(ui_.DroneSelect, SIGNAL(currentIndexChanged(const QString&)), this,
          SLOT(drone_select(const QString&)));
  connect(ui_.EmergencyButton, SIGNAL(clicked()), this, SLOT(emergency_publish()));
  connect(ui_.LandButton, SIGNAL(clicked()), this, SLOT(land_publish()));
  connect(ui_.TakeOffButton, SIGNAL(clicked()), this, SLOT(takeoff_publish()));
  connect(ui_.ResetPoseButton, SIGNAL(clicked()), this, SLOT(reset_pose_publish()));
  connect(ui_.addButton, SIGNAL(clicked()), this, SLOT(add_drone_to_list()));
  connect(ui_.removeButton, SIGNAL(clicked()), this, SLOT(remove_drone_from_list()));
}

void MyPlugin::add_drone_to_list()
{
  QString text = ui_.addEdit->text();
  if (text.isEmpty())
  {
    return;
  }
  if (drones_list.contains(text))
  {
    return;
  }
  drones_list += text;
  QStringList list = (QStringList() << text);
  ui_.DroneSelect->addItems(list);
  int i = drones_list.size() - 1;
  init_emergency_pub(i, true);
  init_land_pub(i, true);
  init_takeoff_pub(i, true);
  init_reset_pose_pub(i, true);
}

void MyPlugin::remove_drone_from_list()
{
  int i = ui_.DroneSelect->currentIndex();
  if (i < 0)
  {
    return;
  }
  drones_list.removeAt(i);
  ui_.DroneSelect->removeItem(i);

  emergency_pub.erase(emergency_pub.begin() + i);
  land_pub.erase(land_pub.begin() + i);
  takeoff_pub.erase(takeoff_pub.begin() + i);
  reset_pose_pub.erase(reset_pose_pub.begin() + i);
}

void MyPlugin::drone_select(const QString& str)
{
  ROS_INFO("MyPlugin::drone_select");
  ui_.BatteryPercentageLabel->setText("N/A%");
  navdata_sub.shutdown();
  std::string name = str.toUtf8().constData();
  std::string navdata_sub_channel = getNodeHandle().resolveName("/" + name + "/navdata");
  navdata_sub = getNodeHandle().subscribe(navdata_sub_channel, 1, &MyPlugin::navdataCb, this);
}

void MyPlugin::emergency_publish()
{
  ROS_INFO("MyPlugin::emergency_publish");
  std_msgs::Empty msg;
  int i = ui_.DroneSelect->currentIndex();
  if (i < 0)
  {
    return;
  }
  emergency_pub[i].publish(msg);
}

void MyPlugin::land_publish()
{
  ROS_INFO("MyPlugin::land_publish");
  std_msgs::Empty msg;
  int i = ui_.DroneSelect->currentIndex();
  if (i < 0)
  {
    return;
  }
  land_pub[i].publish(msg);
}

void MyPlugin::takeoff_publish()
{
  ROS_INFO("MyPlugin::takeoff_publish");
  std_msgs::Empty msg;
  int i = ui_.DroneSelect->currentIndex();
  if (i < 0)
  {
    return;
  }
  takeoff_pub[i].publish(msg);
}

void MyPlugin::reset_pose_publish()
{
  ROS_INFO("MyPlugin::reset_pose_publish");
  std_msgs::Empty msg;
  int i = ui_.DroneSelect->currentIndex();
  if (i < 0)
  {
    return;
  }
  reset_pose_pub[i].publish(msg);
}

void MyPlugin::rolesCb(const ucl_drone::DroneRoles::ConstPtr drones_rolesPtr)
{
  ROS_INFO("MyPlugin::rolesCb");
  if (ui_.DroneSelect->count() == 0)
  {
    emergency_pub.resize(drones_rolesPtr->roles.size());
    land_pub.resize(drones_rolesPtr->roles.size());
    takeoff_pub.resize(drones_rolesPtr->roles.size());
    reset_pose_pub.resize(drones_rolesPtr->roles.size());
    for (int i = 0; i < drones_rolesPtr->roles.size(); i++)
    {
      std::string name = drones_rolesPtr->roles[i].name;
      drones_list += (QStringList() << QString::fromUtf8(name.data(), name.size()));
    }
    drones_list.removeDuplicates();
    init_emergency_pub();
    init_land_pub();
    init_takeoff_pub();
    init_reset_pose_pub();
    ui_.DroneSelect->addItems(drones_list);
  }
}

void MyPlugin::init_emergency_pub(int i, bool only)
{
  ROS_INFO("MyPlugin::init_emergency_pub");
  if (drones_list.size() <= i)
    return;
  emergency_pub.resize(drones_list.size());
  std::string name = drones_list.at(i).toUtf8().constData();
  std::string emergency_pub_channel = getNodeHandle().resolveName("/" + name + "/emergency_toggle");
  emergency_pub[i] = getNodeHandle().advertise< std_msgs::Empty >(emergency_pub_channel, 1);
  if (!only && i + 1 < drones_list.size())
  {
    this->init_emergency_pub(i + 1, false);
  }
}

void MyPlugin::init_land_pub(int i, bool only)
{
  ROS_INFO("MyPlugin::init_land_pub");
  if (drones_list.size() <= i)
    return;
  land_pub.resize(drones_list.size());
  std::string name = drones_list.at(i).toUtf8().constData();
  std::string land_pub_channel = getNodeHandle().resolveName("/" + name + "/land");
  land_pub[i] = getNodeHandle().advertise< std_msgs::Empty >(land_pub_channel, 1);
  if (!only && i + 1 < drones_list.size())
  {
    this->init_land_pub(i + 1, false);
  }
}

void MyPlugin::init_takeoff_pub(int i, bool only)
{
  ROS_INFO("MyPlugin::init_takeoff_pub");
  if (drones_list.size() <= i)
    return;
  takeoff_pub.resize(drones_list.size());
  std::string name = drones_list.at(i).toUtf8().constData();
  std::string takeoff_pub_channel = getNodeHandle().resolveName("/" + name + "/takeoff");
  takeoff_pub[i] = getNodeHandle().advertise< std_msgs::Empty >(takeoff_pub_channel, 1);
  if (!only && i + 1 < drones_list.size())
  {
    this->init_takeoff_pub(i + 1, false);
  }
}

void MyPlugin::init_reset_pose_pub(int i, bool only)
{
  ROS_INFO("MyPlugin::init_reset_pose_pub");
  if (drones_list.size() <= i)
    return;
  reset_pose_pub.resize(drones_list.size());
  std::string name = drones_list.at(i).toUtf8().constData();
  std::string reset_pose_pub_channel = getNodeHandle().resolveName("/" + name + "/reset_pose");
  reset_pose_pub[i] = getNodeHandle().advertise< std_msgs::Empty >(reset_pose_pub_channel, 1);
  if (!only && i + 1 < drones_list.size())
  {
    this->init_reset_pose_pub(i + 1, false);
  }
}

void MyPlugin::navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdata)
{
  int percent = navdata->batteryPercent;
  // ROS_INFO("MyPlugin::navdataCb %d", percent);
  QString s = QString::number(percent) + "%";
  ui_.BatteryPercentageLabel->setText(s);
}

void MyPlugin::shutdownPlugin()
{
  roles_sub.shutdown();
  navdata_sub.shutdown();
  for (int i = 0; i < drones_list.size(); i++)
  {
    emergency_pub[i].shutdown();
    land_pub[i].shutdown();
    takeoff_pub[i].shutdown();
    reset_pose_pub[i].shutdown();
  }
  // TODO unregister all subscribers, publishers here
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  QString k1("drones_list");
  instance_settings.setValue(k1, drones_list);
  QString k2("drone_selected");
  instance_settings.setValue(k2, ui_.DroneSelect->currentIndex());
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k);

  QString k1("drones_list");
  drones_list = instance_settings.value(k1).toStringList();
  init_emergency_pub();
  init_land_pub();
  init_takeoff_pub();
  init_reset_pose_pub();
  ui_.DroneSelect->addItems(drones_list);
  QString k2("drone_selected");
  int drones_selected = instance_settings.value(k2).toInt();
  ui_.DroneSelect->setCurrentIndex(drones_selected);
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

}  // namespace
PLUGINLIB_DECLARE_CLASS(ucl_drone_gui, MyPlugin, ucl_drone_gui::MyPlugin, rqt_gui_cpp::Plugin)
