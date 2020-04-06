#include<gazebo/common/Plugin.hh>
#include<ros/ros.h>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include<typeinfo>
#include<synchronizer/Combined.h>

namespace gazebo{

  class DataPublisher : public ModelPlugin
  {
    private: ros::NodeHandle* dpNode;
    private: ros::Publisher dpPub;
    private: event::ConnectionPtr updateConnection;
    private: physics::ModelPtr model;
  private: ignition::math::Quaternion<double> orientation;
    private: ignition::math::Vector3<double> accel_;

    public: virtual void Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
    {
      if(!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }
      this->model = _model;
      this->dpNode=new ros::NodeHandle("synchronizer");
      this->dpPub=this->dpNode->advertise<synchronizer::Combined>("Combined",100);
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&DataPublisher::OnUpdate, this));

    }
  public: void OnUpdate()
  {
    synchronizer::Combined msg;
    orientation = this->model->WorldPose().Rot();
    accel_=this->model->WorldLinearAccel();
    msg.angular={orientation.Roll()*(180/3.14),orientation.Pitch()*(180/3.14),orientation.Yaw()*(180/3.14)};
    msg.linear={accel_.X(),accel_.Y(),accel_.Z()};
    msg.depth={-(this->model->WorldPose().Pos().Z())};
    this->dpPub.publish(msg);
  }

  };
  GZ_REGISTER_MODEL_PLUGIN(DataPublisher)
}
