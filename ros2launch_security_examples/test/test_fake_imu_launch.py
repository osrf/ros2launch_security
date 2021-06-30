# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource

import launch_testing
import launch_testing_ros

g_this_dir = os.path.dirname(os.path.abspath(__file__))


def generate_test_description():
    launch_description = LaunchDescription()

    launch_description.add_action(
        # bare minimum formatting for console output matching
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '{message}')
    )
    launch_description.add_action(
        IncludeLaunchDescription(AnyLaunchDescriptionSource(
            os.path.join(g_this_dir, '..', 'launch', 'fake_imu.launch.xml')
        ))
    )

    launch_description.add_action(launch_testing.util.KeepAliveProc())
    launch_description.add_action(launch_testing.actions.ReadyToTest())
    return launch_description, locals()


class TestFakeImuLaunch(unittest.TestCase):

    def test_processes_output(self, proc_output):
        """Test all processes output against expectations."""
        # Assumption is that this environment variable is always set by the test runner.
        rmw_under_test = os.environ['RMW_IMPLEMENTATION']
        assert rmw_under_test

        from launch_testing.tools.output import get_default_filtered_prefixes
        output_filter = launch_testing_ros.tools.basic_output_filter(
            filtered_prefixes=get_default_filtered_prefixes(),
            filtered_rmw_implementation=rmw_under_test
        )

        expected_outputs = [
            ('fake_imu', os.path.join(g_this_dir, 'expected_outputs', 'fake_imu')),
            ('imu_sink', os.path.join(g_this_dir, 'expected_outputs', 'imu_sink')),
        ]
        for process, expected_output_file in expected_outputs:
            proc_output.assertWaitFor(
                process=process,
                expected_output=launch_testing.tools.expected_output_from_file(
                    path=expected_output_file
                ),
                output_filter=output_filter,
                timeout=30,
            )


@launch_testing.post_shutdown_test()
class TestFakeImuLaunchAfterShutdown(unittest.TestCase):

    def test_last_process_exit_code(self, proc_info):
        """Test last process exit code."""
        launch_testing.asserts.assertExitCodes(proc_info, process='fake_imu')
        launch_testing.asserts.assertExitCodes(proc_info, process='imu_sink')
