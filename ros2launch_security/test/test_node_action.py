# Copyright 2020 Canonical, Ltd.
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

"""Tests for the Security extension to Node Actions."""

import os
import pathlib
import tempfile
from typing import List
import unittest
from unittest.mock import patch

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import DeclareLaunchArgument
import launch_ros.actions

from sros2.keystore._keystore import create_keystore
from sros2.keystore._keystore import is_valid_keystore


class TestNodeActionSecurityExtension(unittest.TestCase):

    def _assert_launch_no_errors(self, actions):
        ld = LaunchDescription(actions)
        ls = LaunchService(debug=True)
        ls.include_launch_description(ld)
        assert 0 == ls.run()

    def _create_node(self, *, parameters=None, remappings=None):
        return launch_ros.actions.Node(
            package='demo_nodes_py', executable='talker_qos', output='screen',
            name='my_node', namespace='my_ns',
            exec_name='my_node_process',
            arguments=['--number_of_cycles', '1'],
            parameters=parameters,
            remappings=remappings,
        )

    def _prepare_secure_flags_and_keystore(self, actions: List, keystore: str) -> List:
        create_keystore(pathlib.Path(keystore))
        return [
            DeclareLaunchArgument('__secure', default_value=['true']),
            DeclareLaunchArgument('__keystore', default_value=[keystore]),
        ] + actions

    def _assert_has_enclave(self, node, node_name):
        assert '--enclave' in node.process_details['cmd']
        i = node.process_details['cmd'].index('--enclave')
        assert node_name == node.process_details['cmd'][i + 1]

    @patch('nodl.get_node_by_executable', autospec=True)
    def test_launch_node_secure_fully_qualified(self, get_node_by_executable):
        get_node_by_executable.return_value.name = 'foo'
        # First try with fully qualified node name specified
        with tempfile.TemporaryDirectory() as tmp:
            with patch.dict(
                os.environ,
                {
                    'ROS_SECURITY_KEYSTORE': tmp,
                    'ROS_SECURITY_STRATEGY': 'Enforce',
                    'ROS_SECURITY_ENABLE': 'true',
                },
            ):
                node_fixture = self._create_node()
                secure_node_action = self._prepare_secure_flags_and_keystore([node_fixture], tmp)
                self._assert_launch_no_errors(secure_node_action)
                self._assert_has_enclave(node_fixture, '/my_ns/my_node')
                assert is_valid_keystore(pathlib.Path(tmp))

    @patch('nodl.get_node_by_executable', autospec=True)
    def test_launch_node_secure_no_name(self, get_node_by_executable):
        # Now try filling in with node name from nodl
        get_node_by_executable.return_value.name = 'foo'
        with tempfile.TemporaryDirectory() as tmp:
            with patch.dict(
                os.environ,
                {
                    'ROS_SECURITY_KEYSTORE': tmp,
                    'ROS_SECURITY_STRATEGY': 'Enforce',
                    'ROS_SECURITY_ENABLE': 'true',
                },
            ):
                node_fixture = launch_ros.actions.Node(
                    package='demo_nodes_py',
                    executable='talker_qos',
                    output='screen',
                    namespace='my_ns',
                    exec_name='my_node_process',
                    arguments=['--number_of_cycles', '1'],
                )
                secure_node_action = self._prepare_secure_flags_and_keystore([node_fixture], tmp)
                self._assert_launch_no_errors(secure_node_action)
                self._assert_has_enclave(node_fixture, '/my_ns/foo')
                assert is_valid_keystore(pathlib.Path(tmp))

    @patch('nodl.get_node_by_executable', autospec=True)
    def test_launch_node_secure_no_namespace(self, get_node_by_executable):
        # Now try with no namespace
        get_node_by_executable.return_value.name = 'foo'
        with tempfile.TemporaryDirectory() as tmp:
            with patch.dict(
                os.environ,
                {
                    'ROS_SECURITY_KEYSTORE': tmp,
                    'ROS_SECURITY_STRATEGY': 'Enforce',
                    'ROS_SECURITY_ENABLE': 'true',
                },
            ):
                node_fixture = launch_ros.actions.Node(
                    package='demo_nodes_py',
                    executable='talker_qos',
                    output='screen',
                    exec_name='my_node_process',
                    arguments=['--number_of_cycles', '1'],
                )
                secure_node_action = self._prepare_secure_flags_and_keystore([node_fixture], tmp)
                self._assert_launch_no_errors(secure_node_action)
                self._assert_has_enclave(node_fixture, '/foo')
                assert is_valid_keystore(pathlib.Path(tmp))
