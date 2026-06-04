#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla spectator
"""

import math

import carla

from carla_ros_bridge.actor import Actor
from carla_ros_bridge.ego_vehicle import EgoVehicle


class Spectator(Actor):

    """
    Actor implementation details for spectators
    """

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        super(Spectator, self).__init__(uid=uid,
                                        name=name,
                                        parent=parent,
                                        node=node,
                                        carla_actor=carla_actor)

    def update(self, frame, timestamp):
        """
        Override to optionally follow the ego vehicle.
        """
        if self.node.parameters.get('spectator_follow_ego', False):
            for actor in self.node.actor_factory.actors.values():
                if isinstance(actor, EgoVehicle):
                    ego_transform = actor.carla_actor.get_transform()
                    yaw_rad = math.radians(ego_transform.rotation.yaw)
                    spectator_transform = carla.Transform(
                        carla.Location(
                            x=ego_transform.location.x - 8.0 * math.cos(yaw_rad),
                            y=ego_transform.location.y - 8.0 * math.sin(yaw_rad),
                            z=ego_transform.location.z + 5.0),
                        carla.Rotation(
                            pitch=-15.0,
                            yaw=ego_transform.rotation.yaw,
                            roll=0.0))
                    self.carla_actor.set_transform(spectator_transform)
                    break
        super(Spectator, self).update(frame, timestamp)
