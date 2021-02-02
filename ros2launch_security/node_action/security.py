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

import pathlib
from typing import Dict
from typing import List
from typing import Union

from launch.launch_context import LaunchContext
from launch.substitutions import LocalSubstitution
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.actions.node import Node, NodeActionExtension

import nodl
import sros2.api._keystore


class SecurityNodeActionExtension(NodeActionExtension):

    def command_extension(self, context):
        if context.launch_configurations.get('__secure', None) is not None:
            cmd_extension = ['--enclave', LocalSubstitution("ros_specific_arguments['enclave']")]
            cmd_extension = [normalize_to_list_of_substitutions(x) for x in cmd_extension]
            return cmd_extension
        else:
            return []

    def execute(self, context, ros_specific_arguments, node_info):
        if context.launch_configurations.get('__secure', None) is not None:
            return self._setup_security(context, ros_specific_arguments, node_info)
        else:
            return ros_specific_arguments

    def _setup_security(
        self,
        context: LaunchContext,
        ros_specific_arguments: Dict[str, Union[str, List[str]]],
        node_info: NodeActionExtension.NodeInfo
    ) -> None:
        """Enable encryption, creating a key for the node if necessary."""
        nodl_node = nodl.get_node_by_executable(
            package_name=node_info.package,
            executable_name=node_info.executable
        )

        self.__enclave = node_info.name.replace(
            Node.UNSPECIFIED_NODE_NAME, nodl_node.name
        ).replace(Node.UNSPECIFIED_NODE_NAMESPACE, '')

        sros2.api._keystore.create_enclave(
            keystore_path=pathlib.Path(context.launch_configurations.get('__keystore')),
            identity=self.__enclave
        )

        ros_specific_arguments['enclave'] = self.__enclave
        return ros_specific_arguments
