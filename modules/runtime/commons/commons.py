# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import time

def return_execution_time(func):
  def wrapper(*args, **kwargs):
    time_before_step = time.time()
    func(*args, **kwargs)
    step_duration = time.time() - time_before_step
    return step_duration

  return wrapper