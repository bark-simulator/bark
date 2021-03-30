# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


class AgentTrackInfo:
    def __init__(self, filename, track_id, start_time, end_time, first_ts_on_map=None, precision=-2):
        self._filename = filename
        self._track_id = track_id  # TODO: rename to agent_id
        self._start_time = int(round(start_time, precision))  # in ms
        self._end_time = int(round(end_time, precision))  # in ms

    def __str__(self):
        return 'id={} start_time={} end_time={}'.format(self._track_id, self._start_time, self._end_time)

    def GetFileName(self):
        # yields track filename
        return self._filename

    def GetTrackId(self):
        return self._track_id

    def GetStartTimeMs(self):
        return self._start_time

    def GetEndTimeMs(self):
        return self._end_time
