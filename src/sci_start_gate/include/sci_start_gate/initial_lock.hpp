// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <box2d/box2d.h>

#include <stdexcept>

namespace sci_start_gate
{
inline void initialize_box2d_contact_runtime()
{
  // MVSim embeds a private static Box2D. This extension's static Box2D must
  // initialize its own contact dispatch table before destroying contacts
  // created by MVSim: Box2D initializes that table lazily on first creation.
  static const bool initialized = [] {
    b2World bootstrap({0.0f, 0.0f});
    b2BodyDef definition;
    definition.type = b2_dynamicBody;
    b2PolygonShape shape;
    shape.SetAsBox(0.1f, 0.1f);
    bootstrap.CreateBody(&definition)->CreateFixture(&shape, 1.0f);
    bootstrap.CreateBody(&definition)->CreateFixture(&shape, 1.0f);
    bootstrap.Step(0.001f, 1, 1);
    return true;
  }();
  (void)initialized;
}

class InitialLock
{
 public:
  explicit InitialLock(b2Body& body) : body_(body)
  {
    if (body_.GetType() != b2_dynamicBody)
      throw std::runtime_error("sci_start_gate requires a dynamic vehicle body");
    initialize_box2d_contact_runtime();
    body_.GetMassData(&original_mass_);
  }

  bool update(bool allow_motion)
  {
    const b2BodyType desired = allow_motion ? b2_dynamicBody : b2_staticBody;
    if (body_.GetType() == desired) return false;
    body_.SetType(desired);
    if (allow_motion)
    {
      // SetType(dynamic) derives a fresh mass from fixture densities; restore
      // MVSim's explicit chassis mass, center of mass and inertia instead.
      body_.SetMassData(&original_mass_);
      body_.SetAwake(true);
    }
    else
    {
      body_.SetLinearVelocity(b2Vec2_zero);
      body_.SetAngularVelocity(0.0f);
      body_.SetAwake(false);
    }
    return true;
  }

 private:
  b2Body& body_;
  b2MassData original_mass_{};
};
}  // namespace sci_start_gate
