# Software License Agreement (BSD License)
#
# Copyright (c) 2021, Stephan Kunz
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
import launch_ros.actions
import launch.substitutions


def generate_launch_description():

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'executable',
            default_value='nmea_serial_driver',
            description='name of executable to use'
        ),
        launch.actions.DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                get_package_share_directory('nmea_navsat_driver'), 
                'config',
                'quectel_nmea_serial_driver.yaml'
            ),
            description='name of config file to use'
        ),
        launch_ros.actions.Node(
            package='nmea_navsat_driver',
            executable=launch.substitutions.LaunchConfiguration('executable'),
            output='screen',
            parameters=[launch.substitutions.LaunchConfiguration('config_file')]
        )
    ])
