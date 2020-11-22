# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart,
# Tobias Kessler and Mansoor Nasir
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
from pathlib import Path


class Data:
  #xodr data
  _xodr_data = {}

  #track data
  _track_data = {}

  #params data
  _params_data = {}

  @staticmethod
  def xodr_data(name):
    if Data._xodr_data:
      return Data._xodr_data[name]

    data_dir = os.path.join(os.path.dirname(__file__), "../runtime/tests/data")
    files = [f for f in os.listdir(data_dir) if f.endswith(".xodr")]

    for file in files:
      Data._xodr_data[Path(file).stem] = os.path.join(data_dir, file)

    return Data._xodr_data[name]

  @staticmethod
  def track_data(name):
    if Data._track_data:
      return Data._track_data[name]

    data_dir = os.path.join(os.path.dirname(__file__), "../runtime/tests/data")
    files = [f for f in os.listdir(data_dir) if f.endswith(".csv")]

    for file in files:
      Data._track_data[Path(file).stem] = os.path.join(data_dir, file)

    return Data._track_data[name]

  @staticmethod
  def params_data(name):
    if Data._params_data:
      return Data._params_data[name]

    data_dir = os.path.join(os.path.dirname(__file__), "params")
    files = [f for f in os.listdir(data_dir) if f.endswith(".json")]

    for file in files:
      Data._params_data[Path(file).stem] = os.path.join(data_dir, file)

    return Data._params_data[name]
