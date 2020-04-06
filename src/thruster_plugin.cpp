#include<thread>
#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<ros/subscribe_options.h>
#include<std_msgs/Float32.h>
#include<gazebo/common/Plugin.hh>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include<thruster_controller/ThrusterSpeeds.h>

#include<typeinfo>

namespace gazebo{
  class ThrusterPlugin : public ModelPlugin
  {
  private: std::unique_ptr<ros::NodeHandle> tpNode;
  private: ros::Subscriber tpSub;
  private: ros::CallbackQueue tpSubQueue;
  private: std::thread tpSubQueueThread;
  private: physics::ModelPtr model;
  private: physics::LinkPtr thrusters[8];
  private: std::string path="auv_demo::thruster_";
  private: std::string k;
  private: double initial,adjusted;
  private: event::ConnectionPtr updateEvent;
  private: double thrusterspeeds_[6]={1500.0,1500.0,1500.0,1500.0,1500.0,1500.0};
  private: double a=-51.348085781063;
  private: double b=0.10072267395657193;
  private: double c=-0.00006784574094879734;
  private: double d=1.5694002475642106e-8;

  public: virtual void Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
  {
    if(!ros::isInitialized()){
      int argc=0;
      char **argv=NULL;
      ros::init(argc,argv,"thruster_controller",ros::init_options::NoSigintHandler);
    }
    this->model=_model;
    for(int i=0;i<6;i++){
      k=std::to_string(i+1);
      this->thrusters[i]=_model->GetChildLink(path+k);
    }
    this->thrusters[6]=_model->GetChildLink(path+"p4");
    this->thrusters[7]=_model->GetChildLink(path+"p5");

    this->tpNode.reset(new ros::NodeHandle("thruster_controller"));
    ros::SubscribeOptions so=ros::SubscribeOptions::create<thruster_controller::ThrusterSpeeds>(
      "/thruster_speeds",1,
      boost::bind(&ThrusterPlugin::OnRosMsg,this,_1),
      ros::VoidPtr(), &this->tpSubQueue);
      this->tpSub = this->tpNode->subscribe(so);

      // Spin up the queue helper thread.
      this->tpSubQueueThread =
        std::thread(std::bind(&ThrusterPlugin::QueueThread, this));
        this->updateEvent = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ThrusterPlugin::OnFrame, this));
  }
  public: void OnRosMsg(const thruster_controller::ThrusterSpeeds::ConstPtr &_msg)
  {
    for(int i=0;i<6;i++){
//      std::cout<<"BEFORE:"<<i<<" = "<<_msg->data[i]<<std::endl;
      thrusterspeeds_[i]=(double)_msg->data[i];
//      std::cout<<"AFTER:"<<i<<" = "<<thrusterspeeds_[i]<<std::endl;
    }

  }
  /// \brief ROS helper function that processes messages
  private: void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->tpNode->ok())
    {
      this->tpSubQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  private: double AdjustForce(double initial){
    double adjusted=0.0;
/*    if(initial>=1470.0 && initial <= 1530.0){
        adjusted=0.0;
    }
    else if(initial > 1530.0){
      initial=initial-1530.0;
      adjusted=(initial/370.0)*2.36;
    }
    else{
      initial=initial-1470.0;
      adjusted=(initial/370.0)*1.85;
    }
    adjusted=adjusted*9.8;
    */
    if(initial>=1470.0 && initial <= 1530.0){
            adjusted=0.0;
        }
else{    adjusted=a+(b*initial)+(c*pow(initial,2.0))+(d*pow(initial,3.0)) ;
    adjusted = adjusted * 4.44822;}
    return adjusted;
  }

public: void OnFrame(){

  this->thrusters[0]->AddRelativeForce(ignition::math::Vector3d(0, 0, -AdjustForce(thrusterspeeds_[0])));
    this->thrusters[1]->AddRelativeForce(ignition::math::Vector3d(0, 0, -AdjustForce(thrusterspeeds_[1])));
    this->thrusters[2]->AddRelativeForce(ignition::math::Vector3d(0, 0, -AdjustForce(thrusterspeeds_[2])));
    this->thrusters[3]->AddRelativeForce(ignition::math::Vector3d(AdjustForce(thrusterspeeds_[3]), 0, 0));
    this->thrusters[6]->AddRelativeForce(ignition::math::Vector3d(AdjustForce(thrusterspeeds_[3]), 0, 0));
    this->thrusters[4]->AddRelativeForce(ignition::math::Vector3d(AdjustForce(thrusterspeeds_[4]), 0, 0));
    this->thrusters[7]->AddRelativeForce(ignition::math::Vector3d(AdjustForce(thrusterspeeds_[4]), 0, 0));
    this->thrusters[5]->AddRelativeForce(ignition::math::Vector3d(0, -AdjustForce(thrusterspeeds_[5]), 0));
}


  };
  GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin)
}
