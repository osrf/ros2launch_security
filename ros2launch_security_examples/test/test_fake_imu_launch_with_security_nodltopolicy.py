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
from pathlib import Path
from tempfile import NamedTemporaryFile, TemporaryDirectory

import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable

import launch_testing
import launch_testing_ros

from nodl._index import _get_nodes_from_package
from nodl_to_policy.policy import convert_to_policy
from sros2.api import _artifact_generation
from sros2.policy import dump_policy

g_this_dir = os.path.dirname(os.path.abspath(__file__))


def create_permissions(keystore_path: Path) -> None:
    # obtain `fake_imu.nodl.xml` and `imu_sink.nodl.xml`
    nodl_description = _get_nodes_from_package(
        package_name='ros2launch_security_examples')

    policy = convert_to_policy(nodl_description)

    # `_artifact_generation` requires a policy file path, so
    # the converted policy requires storing in a temporary file
    policy_file_path = NamedTemporaryFile().name
    with open(policy_file_path, 'w') as stream:
        dump_policy(policy, stream)

    # generates permission artifacts for all the entities in the policy file
    _artifact_generation.generate_artifacts(
        keystore_path, [], [Path(policy_file_path)])


def secure_launch_description() -> LaunchDescription:
    """
    Run `ros2 launch [...] --secure` plus create a keystore with permissions artifacts.

    In generating the permissions artifacts, `nodl_to_policy` is used to convert NoDL
    files in the package to corresponding access control policies. These policies are
    then supplied to `sros2.api._artifact_generation.generate_artifacts` to set up
    the appropriate DDS specific permissions files.
    """
    keystore_path = TemporaryDirectory().name

    launch_description = LaunchDescription()

    launch_description.add_action(
        # bare minimum formatting for console output matching
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '{message}')
    )
    launch_description.add_action(
        ExecuteProcess(
            cmd=[
                # TODO(wjwwood): instead of running `ros2 launch` with the `--secure`
                #   option, include the launch file normally, e.g. like the
                #   test_fake_imu_launch.py peer to this test, but this cannot
                #   be done until we integrate the ros2launch_security plugin
                #   into `launch_testing`, in addition to `ros2launch`.
                #   See: https://github.com/osrf/ros2launch_security/issues/5
                'ros2', 'launch',
                os.path.join(g_this_dir, '..', 'launch', 'fake_imu.launch.xml'),
                '--secure', keystore_path,  # provide a known keystore path to store artifacts in
            ],
            name='ros2_launch_fake_imu',
            output='screen',
        )
    )

    create_permissions(Path(keystore_path))  # to correctly initialize keystore, generate artifacts

    launch_description.add_action(launch_testing.util.KeepAliveProc())
    launch_description.add_action(launch_testing.actions.ReadyToTest())
    return launch_description


def generate_test_description():
    return secure_launch_description(), locals()


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
            # (Process, file containing expected output for the process)
            # None indicates, any process can produce the output.
            (None, os.path.join(g_this_dir, 'expected_outputs', 'fake_imu')),
            (None, os.path.join(g_this_dir, 'expected_outputs', 'imu_sink')),
            # TODO(wjwwood): right now, looking for the "Found security directory ..."
            #   messages output anywhere, but it would be better to assert that
            #   the output is coming from the right processes, but that would
            #   require figuring out how to integrate the `--secure` option for
            #   'ros2 launch' into launch_testing.
            #   See: https://github.com/osrf/ros2launch_security/issues/5
            (None, os.path.join(g_this_dir, 'expected_outputs', 'enclave_used')),
        ]
        for process, expected_output_file in expected_outputs:
            proc_output.assertWaitFor(
                process=process,
                expected_output=launch_testing.tools.expected_output_from_file(
                    path=expected_output_file
                ),
                output_filter=output_filter,
                timeout=10,
                # This is needed because ros2 launch channels things to stdout.
                stream='stdout',
            )


@launch_testing.post_shutdown_test()
class TestFakeImuLaunchAfterShutdown(unittest.TestCase):

    def test_last_process_exit_code(self, proc_info):
        """Test last process exit code."""
        launch_testing.asserts.assertExitCodes(proc_info, process=None)
