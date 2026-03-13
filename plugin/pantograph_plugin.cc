#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <mutex>
#include <cmath>
#include <string>
#include <iostream>

namespace gazebo
{

class PantographPlugin : public ModelPlugin
{
public:
  PantographPlugin() : ModelPlugin()
  {
    this->targetTheta1 = 0.0;
    this->targetTheta2 = 0.0;
  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->model = _model;
    this->world = _model->GetWorld();

    this->joint1 = _model->GetJoint("Revolute_9");
    this->joint2 = _model->GetJoint("Revolute_10");

    if (!this->joint1 || !this->joint2)
    {
      gzerr << "[PantographPlugin] Could not find joints Revolute_9 / Revolute_10\n";
      return;
    }

    this->pid1.Init(80.0, 0.5, 8.0, 10.0, -10.0, 80.0, -80.0);
    this->pid2.Init(80.0, 0.5, 8.0, 10.0, -10.0, 80.0, -80.0);

    this->joint1->SetParam("fmax", 0, 80.0);
    this->joint2->SetParam("fmax", 0, 80.0);

    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->world->Name());

    std::string topic = std::string("/gazebo/") + this->world->Name()
                        + "/penplotter/joint_cmd";
    this->sub = this->node->Subscribe(topic,
                  &PantographPlugin::OnJointCmd, this);

    gzmsg << "[PantographPlugin] Listening on " << topic << "\n";

    this->updateConn = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PantographPlugin::OnUpdate, this));

    gzmsg << "[PantographPlugin] Loaded OK.\n";
  }

private:

  void OnUpdate()
  {
    double t1, t2;
    {
      std::lock_guard<std::mutex> lock(this->mtx);
      t1 = this->targetTheta1;
      t2 = this->targetTheta2;
    }

    double dt = this->world->Physics()->GetMaxStepSize();

    double cur1 = this->joint1->Position(0);
    double cur2 = this->joint2->Position(0);

    double err1 = t1 - cur1;
    double err2 = t2 - cur2;

    while (err1 >  M_PI) err1 -= 2*M_PI;
    while (err1 < -M_PI) err1 += 2*M_PI;
    while (err2 >  M_PI) err2 -= 2*M_PI;
    while (err2 < -M_PI) err2 += 2*M_PI;

    double force1 = this->pid1.Update(err1, dt);
    double force2 = this->pid2.Update(err2, dt);

    this->joint1->SetForce(0, force1);
    this->joint2->SetForce(0, force2);
  }

  void OnJointCmd(ConstGzStringPtr &_msg)
  {
    std::string data = _msg->data();
    double deg1, deg2;
    if (sscanf(data.c_str(), "%lf,%lf", &deg1, &deg2) != 2)
    {
      gzerr << "[PantographPlugin] Bad message: " << data << "\n";
      return;
    }

    std::lock_guard<std::mutex> lock(this->mtx);
    this->targetTheta1 = deg1 * M_PI / 180.0;
    this->targetTheta2 = deg2 * M_PI / 180.0;
  }

  physics::ModelPtr        model;
  physics::WorldPtr        world;
  physics::JointPtr        joint1, joint2;
  common::PID              pid1, pid2;
  event::ConnectionPtr     updateConn;
  transport::NodePtr       node;
  transport::SubscriberPtr sub;
  std::mutex               mtx;
  double                   targetTheta1, targetTheta2;
};

GZ_REGISTER_MODEL_PLUGIN(PantographPlugin)

} // namespace gazebo