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

    // Both joints PID controlled — targets are now geometrically compatible
    // because the IK bakes in the closed-loop constraint
    this->pid1.Init(80.0, 0.0, 8.0, 10.0, -10.0, 80.0, -80.0);
    this->pid2.Init(80.0, 0.0, 8.0, 10.0, -10.0, 80.0, -80.0);

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

    // ── ZERO POSE DIAGNOSTIC ──────────────────────────────────────────────
    gzmsg << "[PantographPlugin] ZERO POSE CHECK\n";
    gzmsg << "  joint1 (Revolute_9)  Position(0) = "
          << this->joint1->Position(0) << " rad = "
          << this->joint1->Position(0) * 180.0 / M_PI << " deg\n";
    gzmsg << "  joint2 (Revolute_10) Position(0) = "
          << this->joint2->Position(0) << " rad = "
          << this->joint2->Position(0) * 180.0 / M_PI << " deg\n";

    auto link_cam2  = this->model->GetLink("cam2_1");
    auto link_cam1  = this->model->GetLink("cam1_1");
    auto link_link1 = this->model->GetLink("link1_1");
    auto link_link2 = this->model->GetLink("link2_1");
    auto link_link3 = this->model->GetLink("link3_1");
    if (link_cam2)  gzmsg << "  cam2_1  world pose: " << link_cam2->WorldPose()  << "\n";
    if (link_cam1)  gzmsg << "  cam1_1  world pose: " << link_cam1->WorldPose()  << "\n";
    if (link_link1) gzmsg << "  link1_1 world pose: " << link_link1->WorldPose() << "\n";
    if (link_link2) gzmsg << "  link2_1 world pose: " << link_link2->WorldPose() << "\n";
    if (link_link3) gzmsg << "  link3_1 world pose: " << link_link3->WorldPose() << "\n";
    // ─────────────────────────────────────────────────────────────────────
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

    // Wrap to [-π, π]
    while (err1 >  M_PI) err1 -= 2*M_PI;
    while (err1 < -M_PI) err1 += 2*M_PI;
    while (err2 >  M_PI) err2 -= 2*M_PI;
    while (err2 < -M_PI) err2 += 2*M_PI;

    double force1 = this->pid1.Update(err1, dt);
    double force2 = this->pid2.Update(err2, dt);

    this->joint1->SetForce(0, force1);
    this->joint2->SetForce(0, force2);

    static int counter = 0;
    counter++;
    if (counter % 500 == 0)
    {
      gzmsg << "[PantographPlugin]"
            << "  t1=" << t1 * 180.0 / M_PI << "deg"
            << "  cur1=" << cur1 * 180.0 / M_PI << "deg"
            << "  t2=" << t2 * 180.0 / M_PI << "deg"
            << "  cur2=" << cur2 * 180.0 / M_PI << "deg\n";
    }
  }

  void OnJointCmd(ConstGzStringPtr &_msg)
  {
    std::string data = _msg->data();
    double deg1, deg2;

    // Expect two floats: "theta1_deg,theta2_deg"
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