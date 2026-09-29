# Copyright 2026 Medrobots Engineering
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

"""Offline regression checks for the production SLAM image dependency."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_production_image_installs_humble_slam_toolbox():
    dockerfile = (ROOT / "Dockerfile").read_text(encoding="utf-8")

    assert "    ros-humble-slam-toolbox \\\n" in dockerfile
