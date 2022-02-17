#!/usr/bin/env python3
# Copyright 2021 John Harwell, All rights reserved.
#
#  This file is part of SIERRA.
#
#  SIERRA is free software: you can redistribute it and/or modify it under the
#  terms of the GNU General Public License as published by the Free Software
#  Foundation, either version 3 of the License, or (at your option) any later
#  version.
#
#  SIERRA is distributed in the hope that it will be useful, but WITHOUT ANY
#  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
#  A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License along with
#  SIERRA.  If not, see <http://www.gnu.org/licenses/

# Core packages

# 3rd party packages
import rospy
from std_msgs.msg import Empty

# Project packages


class SIERRATimekeeper():
    def __init__(self) -> None:
        rospy.init_node('sierra_timekeeper', anonymous=True)
        # Relative namespace so this works for robots and for master nodes
        length = rospy.search_param("sierra/experiment/length")
        self.name = rospy.get_caller_id()
        self.barrier_start = rospy.search_param("sierra/experiment/barrier_start")

        if self.barrier_start is not None:
            # Leading '/' -> This signal must come from the master in the global
            # namespace
            rospy.Subscriber('/sierra/experiment/start',
                             Empty,
                             self._callback)
            self.start = False
        else:
            self.start = True

        self.length = rospy.get_param(length)

        rospy.loginfo(
            f"{self.name}: {self.length} seconds, barrier_start={self.barrier_start}")

    def __call__(self) -> None:
        # Wait until we are given the signal to start the experiment so that all
        # nodes/robots/etc all start at about the same time.
        if self.barrier_start is not None:
            while not rospy.is_shutdown() and not self.start:
                rospy.sleep(1)

        rospy.loginfo(f"{self.name}: Experiment start")

        start = rospy.get_rostime().secs
        now = rospy.get_rostime().secs

        rate = rospy.Rate(1)  # 1hz
        while not rospy.is_shutdown():
            if now - start >= self.length:
                break
            now = rospy.get_rostime().secs

            rate.sleep()

        rospy.loginfo(f"{self.name}: Exit after {self.length} seconds")

    def _callback(self, data) -> None:
        rospy.loginfo(f"{self.name}: Received experiment start signal")
        self.start = True


if __name__ == '__main__':
    try:
        SIERRATimekeeper()()
    except rospy.ROSInterruptException:
        pass
