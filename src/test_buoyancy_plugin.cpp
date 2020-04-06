#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class BuoyancyPlugin : public ModelPlugin
  {
    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateBuoyancyEvent;
    private: physics::LinkPtr hull;
  private: double volume,z,MaxUpthrust,upthrust,waterlevel;
private: double forceFactor,bounceDamp,floatHeight,uplift;
  public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      this->model=_model;
      this->hull=_model->GetChildLink("auv_demo::hull");
/*      volume=pow(.50,3.0);
      MaxUpthrust=volume*1000.0*9.8;
*/      waterlevel=0.0;
        floatHeight=1.7;
        bounceDamp=1.0;
      this->updateBuoyancyEvent = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BuoyancyPlugin::UpdateBuoyancy, this));
  }

public: void UpdateBuoyancy(){
  z=this->hull->WorldPose().Pos().Z();
/*  if(waterlevel-z >0.25){
    upthrust=MaxUpthrust;
  }
  else if(z-waterlevel > 0.25){
    upthrust=0.0;
  }
  else{
    upthrust= MaxUpthrust/2 + (waterlevel-z)*MaxUpthrust/0.25 ;
  }
  this->hull->AddRelativeForce(ignition::math::Vector3d(0,0,upthrust));
*/
forceFactor=1.0-((z-waterlevel)/floatHeight);
if(forceFactor>0.0){
  uplift=9.8*23.0*(forceFactor- this->model->WorldLinearVel().Z()*bounceDamp);
    this->hull->AddRelativeForce(ignition::math::Vector3d(0,0,uplift));
}
}
  };
  GZ_REGISTER_MODEL_PLUGIN(BuoyancyPlugin)
}
