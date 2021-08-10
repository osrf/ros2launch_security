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
from tempfile import TemporaryDirectory
from typing import Optional
from typing import Tuple

import launch
from ros2launch.option import OptionExtension
import sros2.keystore._keystore


class NoKeystoreProvidedError(Exception):
    """Exception raised when a keystore is not provided."""

    def __init__(self):
        super().__init__(('--no-create-keystore was specified and '
                          'no keystore was provided'))


class NonexistentKeystoreError(Exception):
    """Exception raised when keystore is not on disk."""

    def __init__(self, keystore_path: pathlib.Path):
        super().__init__(('--no-create-keystore was specified and '
                          f'the keystore "{keystore_path}" '
                          'does not exist'))


class InvalidKeystoreError(Exception):
    """Exception raised when provided keystore isn't valid."""

    def __init__(self, keystore_path: pathlib.Path):
        super().__init__(('--no-create-keystore was specified and '
                          f'the keystore "{keystore_path}" is not initialized. \n\t'
                          f'(Try running: ros2 security create_keystore {keystore_path})'))


class SecurityOption(OptionExtension):

    def add_arguments(self, parser, cli_name, *, argv=None):
        sec_args = parser.add_argument_group(
            title='security',
            description='Security-related arguments'
        )
        arg = sec_args.add_argument(
            '--secure',
            metavar='keystore',
            nargs='?',
            const='',
            help=('Launch with ROS 2 security features using the specified keystore directory. '
                  'Will set up an ephemeral keystore if one is not specified.'),
        )
        try:
            arg.completer = DirectoriesCompleter()  # argcomplete is optional
        except NameError:
            pass

        sec_args.add_argument(
            '--no-create-keystore',
            dest='create_keystore',
            action='store_false',
            help='Disable automatic keystore creation and/or initialization'
        )

    def prelaunch(
        self,
        launch_description: launch.LaunchDescription,
        args
    ) -> Tuple[launch.LaunchDescription, '_Keystore']:
        if args.secure is None:
            return (launch_description,)
        keystore_path = pathlib.Path(args.secure) if args.secure != '' else None
        keystore = _Keystore(keystore_path=keystore_path, create_keystore=args.create_keystore)

        launch_description = launch.LaunchDescription(
            [
                launch.actions.DeclareLaunchArgument(
                    name='__keystore', default_value=str(keystore.path)
                ),
                launch.actions.DeclareLaunchArgument(
                    name='__secure', default_value='true'
                ),
                launch.actions.SetEnvironmentVariable(
                    name='ROS_SECURITY_KEYSTORE', value=str(keystore.path)
                ),
                launch.actions.SetEnvironmentVariable(
                    name='ROS_SECURITY_STRATEGY',
                    value='Enforce'),
                launch.actions.SetEnvironmentVariable(name='ROS_SECURITY_ENABLE', value='true'),
            ]
            + launch_description.entities
        )

        return launch_description, keystore


class _Keystore:
    """
    Object that contains a keystore.

    If a transient keystore is created, contains the temporary directory, assuring
    it is destroyed alongside the _Keystore object.
    """

    def __init__(self, *, keystore_path: Optional[pathlib.Path], create_keystore: bool):
        if not create_keystore:
            if keystore_path is None:
                raise NoKeystoreProvidedError()
            if not keystore_path.exists():
                raise NonexistentKeystoreError(keystore_path)
            if not sros2.keystore._keystore.is_valid_keystore(keystore_path):
                raise InvalidKeystoreError(keystore_path)

        # If keystore path is blank, create a transient keystore
        if keystore_path is None:
            self._temp_keystore = TemporaryDirectory()
            self._keystore_path = pathlib.Path(self._temp_keystore.name)
        else:
            self._keystore_path = keystore_path
        self._keystore_path = self._keystore_path.resolve()
        # If keystore is not initialized, create a keystore
        if not self._keystore_path.is_dir():
            self._keystore_path.mkdir()
        if not sros2.keystore._keystore.is_valid_keystore(self._keystore_path):
            sros2.keystore._keystore.create_keystore(self._keystore_path)

    @property
    def path(self) -> pathlib.Path:
        return self._keystore_path

    def __str__(self) -> str:
        return str(self._keystore_path)
