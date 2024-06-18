#  Copyright 2020 Unity Technologies
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


class RosTcpEndpointError(Exception):
    """Base class for this package's custom exceptions"""
    pass


class TopicOrServiceNameDoesNotExistError(RosTcpEndpointError):
    """The topic or service name passed does not exist in the source destination dictionary."""
    pass
