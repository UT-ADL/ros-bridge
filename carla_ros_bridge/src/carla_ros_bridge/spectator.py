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

import carla_common.transforms as trans

from carla_ros_bridge.actor import Actor

from geometry_msgs.msg import Pose


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

        self.set_transform_subscriber = self.node.new_subscription(
            Pose,
            "/carla/spectator/set_transform",
            self.on_set_transform,
            qos_profile=10)

    def destroy(self):
        """
        Function (override) to destroy this object.
        """
        self.node.destroy_subscription(self.set_transform_subscriber)
        super(Spectator, self).destroy()

    def on_set_transform(self, pose):
        if self.carla_actor.is_alive:
            self.carla_actor.set_transform(trans.ros_pose_to_carla_transform(pose))
