# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from bark.runtime.scenario.interaction_dataset_processing.agent_track_info import AgentTrackInfo


class ScenarioTrackInfo:
    def __init__(self, track_filename, ego_track_info, xy_offset, start_ts=None, end_ts=None, prec=-2):
        self._track_filename = track_filename
        self._ego_track_info = ego_track_info
        self._xy_offset = xy_offset

        if start_ts is None:
            self._start_ts = int(round(ego_track_info.GetStartTimeMs(), prec))
        else:
            self._start_ts = int(round(start_ts, prec))

        if end_ts is None:
            self._end_ts = int(round(ego_track_info.GetEndTimeMs(), prec))
        else:
            self._end_ts = int(round(end_ts, prec))

        self._other_agents_track_infos = {}

    def __str__(self):
        str_out = 'start_ts={}ms end_ts={}ms ego: {}'.format(
            self.GetStartTimeMs(), self.GetEndTimeMs(), self._ego_track_info)
        for other in self.GetOtherTrackInfos().values():
            str_out = str_out + ', other: {}'.format(other)
        return str_out

    def AddTrackInfoOtherAgent(self, track_info_other):
        track_id = track_info_other.GetTrackId()
        self._other_agents_track_infos[track_id] = track_info_other
        if track_info_other.GetStartTimeMs() > self.GetEndTimeMs():
            raise ValueError("Agent cannot start after scenario end")

    def GetTrackFilename(self):
        return self._track_filename

    def GetEgoTrackInfo(self):
        return self._ego_track_info

    def GetXYOffset(self):
        return self._xy_offset

    def GetStartTimeMs(self):
        # time in ms
        return self._start_ts

    def GetEndTimeMs(self):
        # time in ms
        return self._end_ts

    def GetOtherTrackInfos(self):
        return self._other_agents_track_infos

    def GetTimeOffsetOfAgentInSec(self, agent_id):
        # Time Offset when Agent becomes valid in relation to start time of scenario
        if (agent_id == self._ego_track_info.GetTrackId()):
            return 0.0
        else:
            start_ts = self._other_agents_track_infos[agent_id].GetStartTimeMs(
            )
            timestamp_offset = float(start_ts - self.GetStartTimeMs()) / 1000.0
            return timestamp_offset

    def TimeSanityCheck(self):
        if self.GetStartTimeMs() < self.GetEgoTrackInfo().GetStartTimeMs():
            raise ValueError("ego agent {} starts after scenario".format(
                self.GetEgoTrackInfo().GetTrackId()))
        elif self.GetEndTimeMs() > self.GetEgoTrackInfo().GetEndTimeMs():
            raise ValueError("ego agent {} ends before scenario".format(
                self.GetEgoTrackInfo().GetTrackId()))
        else:
            return True
