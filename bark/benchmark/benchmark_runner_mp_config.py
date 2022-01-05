# Copyright (c) 2021 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

__NUM_CPU_PER_ACTOR = 1

def SetNumCpuPerActor(num):
  global __NUM_CPU_PER_ACTOR
  __NUM_CPU_PER_ACTOR = num

def GetNumCpuPerActor():
  global __NUM_CPU_PER_ACTOR
  return __NUM_CPU_PER_ACTOR