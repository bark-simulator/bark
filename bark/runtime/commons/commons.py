# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


import time

def return_execution_time(func):
  def wrapper(*args, **kwargs):
    time_before_step = time.time()
    func(*args, **kwargs)
    step_duration = time.time() - time_before_step
    return step_duration

  return wrapper