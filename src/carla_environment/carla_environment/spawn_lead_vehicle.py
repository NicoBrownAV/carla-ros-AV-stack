#!/usr/bin/env python3
import carla, time, math


def spawn_lead(world, ego_actor, distance=25.0, model='vehicle.tesla.model3', role='lead'):
    bp_lib = world.get_blueprint_library()
    bp = bp_lib.find(model)
    bp.set_attribute('role_name', role)

    ego_tf = ego_actor.get_transform()
    fwd = ego_tf.get_forward_vector()  # unit vector in world frame
    lead_loc = carla.Location(
        x=ego_tf.location.x + fwd.x * distance,
        y=ego_tf.location.y + fwd.y * distance,
        z=ego_tf.location.z + 0.1
    )
    lead_tf = carla.Transform(lead_loc, ego_tf.rotation)

    actor = world.try_spawn_actor(bp, lead_tf)
    if actor is None:
        # fallback: nudge left/right if spawn blocked
        for dx in (1.0, -1.0, 2.0, -2.0):
            alt_loc = carla.Location(lead_loc.x + dx, lead_loc.y, lead_loc.z)
            actor = world.try_spawn_actor(bp, carla.Transform(alt_loc, ego_tf.rotation))
            if actor: break
    if actor is None:
        raise RuntimeError("Failed to spawn lead vehicle")

    # simple motion: enable autopilot via TrafficManager
    tm = world.get_trafficmanager()
    actor.set_autopilot(True, tm.get_port())
    tm.distance_to_leading_vehicle(actor, 1.5)
    tm.global_percentage_speed_difference(actor, -20)  # 20% faster than default

    return actor

