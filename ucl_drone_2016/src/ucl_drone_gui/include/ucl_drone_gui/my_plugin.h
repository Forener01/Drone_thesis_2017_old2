#ifndef ucl_drone_gui_my_plugin_H
#define ucl_drone_gui_my_plugin_H

#include <ardrone_autonomy/Navdata.h>
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <std_msgs/Empty.h>
#include <ucl_drone/DroneRole.h>
#include <ucl_drone/DroneRoles.h>
#include <ucl_drone_gui/ui_my_plugin.h>
#include <QDebug>
#include <QWidget>

namespace ucl_drone_gui
{
class MyPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();

  QStringList drones_list;

private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;

  ros::Subscriber roles_sub;
  std::string roles_channel;
  void rolesCb(const ucl_drone::DroneRoles::ConstPtr drones_rolesPtr);

  ros::Subscriber navdata_sub;
  void navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdata);

  std::vector< ros::Publisher > emergency_pub;
  void init_emergency_pub(int i = 0, bool only = false);

  std::vector< ros::Publisher > land_pub;
  void init_land_pub(int i = 0, bool only = false);

  std::vector< ros::Publisher > takeoff_pub;
  void init_takeoff_pub(int i = 0, bool only = false);

  std::vector< ros::Publisher > reset_pose_pub;
  void init_reset_pose_pub(int i = 0, bool only = false);

public slots:
  void drone_select(const QString& str);
  void emergency_publish();
  void land_publish();
  void takeoff_publish();
  void reset_pose_publish();
  void add_drone_to_list();
  void remove_drone_from_list();
};
}  // namespace ucl_drone_gui
#endif  // ucl_drone_gui_my_plugin_H
