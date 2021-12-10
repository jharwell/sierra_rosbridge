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

# Project packages


class SIERRATimekeeper():
    def __init__(self) -> None:
        self.length = rospy.get_param("sierra/experiment/length")
        print(self.length)
        rospy.init_node('sierra_timekeeper', anonymous=True)

    def __call__(self) -> None:
        start = rospy.get_rostime().secs
        now = rospy.get_rostime().secs

        while not rospy.is_shutdown():
            if now - start >= self.length:
                break
            now = rospy.get_rostime().secs

            rate = rospy.Rate(1)  # 1hz
            rate.sleep()


if __name__ == '__main__':
    try:
        SIERRATimekeeper()()
    except rospy.ROSInterruptException:
        pass
