# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


class AgentTrackInfo:
    def __init__(self, filename, track_id, start_offset, end_offset, first_ts_on_map=None, precision=-2):
        self._filename = filename
        self._track_id = track_id  # TODO: rename to agent_id
        self._start_offset = int(round(start_offset, precision))  # TODO: rename to start_time
        self._end_offset = int(round(end_offset, precision))  # TODO: rename to end_time

    def __str__(self):
        return 'id={} start_offset={} end_offset={}'.format(self._track_id, self._start_offset, self._end_offset)

    def GetFileName(self):
        # yields track filename
        return self._filename

    def GetTrackId(self):
        return self._track_id

    def GetStartOffset(self):
        return self._start_offset

    def GetEndOffset(self):
        return self._end_offset
