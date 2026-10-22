// SPDX-License-Identifier: Apache-2.0
// Extension of the public MVSim 1.5 WorldElement API; the original vehicle XML,
// controller, collision fixtures and sensor definitions are left unchanged.
// Traffic starts on the first ROS command; zero/expired commands hold ego still.
#include <mvsim/Block.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/WorldElementBase.h>
#include <rapidxml/rapidxml.hpp>

#include <memory>
#include <fstream>
#include <stdexcept>
#include <string>

#include "sci_start_gate/initial_lock.hpp"

namespace sci_start_gate
{
namespace
{
std::string required_value(const rapidxml::xml_node<char>* root, const char* key)
{
  const auto* node = root ? root->first_node(key) : nullptr;
  if (!node || !node->value() || !*node->value())
    throw std::runtime_error(std::string("sci_start_gate requires <") + key + ">");
  std::string value(node->value(), node->value_size());
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
    throw std::runtime_error(std::string("sci_start_gate has an empty <") + key + ">");
  return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}
}  // namespace

class StartGate final : public mvsim::WorldElementBase
{
 public:
  static mvsim::WorldElementBase* Create(
      mvsim::World* world, const rapidxml::xml_node<char>* root)
  {
    return new StartGate(world, root);
  }

  StartGate(mvsim::World* world, const rapidxml::xml_node<char>* root)
      : WorldElementBase(world)
  {
    loadConfigFrom(root);
  }

  void loadConfigFrom(const rapidxml::xml_node<char>* root) override
  {
    vehicle_name_ = required_value(root, "vehicle_name");
    start_file_ = required_value(root, "start_file");
    if (start_file_.front() != '/')
      throw std::runtime_error("sci_start_gate <start_file> must be an absolute path");
    // World runs pre-step hooks in name order. Retiming must follow the normal
    // traffic_* block hooks, before Box2D and all post-step sensor updates.
    name_ = "zz_sci_start_" + vehicle_name_;
  }

  void simul_pre_timestep(const mvsim::TSimulContext& context) override
  {
    WorldElementBase::simul_pre_timestep(context);
    if (!lock_)
    {
      const auto& vehicles = context.world->getListOfVehicles();
      const auto it = vehicles.find(vehicle_name_);
      if (it == vehicles.end() || !it->second || !it->second->b2d_body())
        throw std::runtime_error("sci_start_gate cannot find vehicle body: " + vehicle_name_);
      vehicle_ = it->second;
      lock_ = std::make_unique<InitialLock>(*vehicle_->b2d_body());
    }
    std::ifstream signal(start_file_);
    int motion = 0;  // 0: waiting, 1: driving, 2: stopped after starting.
    signal >> motion;
    if (!started_ && (motion == 1 || motion == 2))
    {
      started_ = true;
      start_time_ = context.simul_time;
    }
    const bool moving = started_ && motion == 1;
    lock_->update(moving);
    if (!moving)
    {
      vehicle_->setRefVelocityLocal({0.0, 0.0, 0.0});
      for (size_t i = 0; i < vehicle_->getNumWheels(); ++i)
        vehicle_->getWheelInfo(i).setW(0.0);
    }

    // Reuse MVSim's native keyframes, with a clock frozen at zero until the
    // first command. Elapsed traffic time then advances independently of ego.
    auto traffic_context = context;
    traffic_context.simul_time = started_ ? context.simul_time - start_time_ : 0.0;
    for (const auto& block : context.world->getListOfBlocks())
      if (block.first.rfind("traffic_", 0) == 0)
        block.second->simul_pre_timestep(traffic_context);
  }

 protected:
  void internalGuiUpdate(
      const mrpt::optional_ref<mrpt::viz::Scene>&,
      const mrpt::optional_ref<mrpt::viz::Scene>&, bool) override {}

 private:
  std::string vehicle_name_;
  std::string start_file_;
  mvsim::VehicleBase::Ptr vehicle_;
  std::unique_ptr<InitialLock> lock_;
  bool started_ = false;
  double start_time_ = 0.0;
};

// libmvsim is a direct link dependency: its factory is initialized before this
// shared object's constructors. LD_PRELOAD loads this registration before XML.
struct RegisterStartGate
{
  RegisterStartGate()
  {
    mvsim::TClassFactory_worldElements::TClassData data;
    data.ptr_factory2 = &StartGate::Create;
    mvsim::classFactory_worldElements.do_register("sci_start_gate", data);
  }
};
static RegisterStartGate registration;
}  // namespace sci_start_gate
