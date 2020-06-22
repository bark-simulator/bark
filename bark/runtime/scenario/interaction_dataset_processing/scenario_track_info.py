# Copyright (c) 2020 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from bark.runtime.scenario.interaction_dataset_processing.agent_track_info import AgentTrackInfo


class ScenarioTrackInfo:
    def __init__(self, map_filename, track_filename, ego_track_info, start_ts=None, end_ts=None, precision=-2):
        self._map_filename = map_filename
        self._track_filename = track_filename
        self._ego_track_info = ego_track_info

        if start_ts is None:
            self._start_ts = int(round(ego_track_info.GetStartOffset(), precision))
        else:
            self._start_ts = int(round(start_ts, precision))

        if end_ts is None:
            self._end_ts = int(round(ego_track_info.GetEndOffset(), precision))
        else:
            self._end_ts = int(round(end_ts, precision))

        self._other_agents_track_infos = {}

    def __str__(self):
        str_out = 'start_ts={} end_ts={} ego: {}'.format(self.GetStartTs(), self.GetEndTs(), self._ego_track_info)
        for other in self.GetOtherTrackInfos().values():
            str_out = str_out + ', other: {}'.format(other)
        return str_out

    def AddTrackInfoOtherAgent(self, track_info_other):
        self._other_agents_track_infos[track_info_other.GetTrackId(
        )] = track_info_other

    def GetMapFilename(self):
        return self._map_filename

    def GetTrackFilename(self):
        return self._track_filename

    def GetEgoTrackInfo(self):
        return self._ego_track_info

    def GetStartTs(self):
        return self._start_ts

    def GetEndTs(self):
        return self._end_ts

    def GetOtherTrackInfos(self):
        return self._other_agents_track_infos

    def TimeSanityCheck(self):
        # for other in self.GetOtherTrackInfos().values():
        #     if self.GetStartTs() > other.GetStartOffset():
        #         raise ValueError(
        #             "Other agent {} starts before scenario".format(other.GetTrackId()))
        #     elif self.GetEndTs() < other.GetEndOffset():
        #         raise ValueError(
        #             "Other agent {} ends after scenario".format(other.GetTrackId()))

        if self.GetStartTs() < self.GetEgoTrackInfo().GetStartOffset():
            raise ValueError("ego agent {} starts after scenario".format(
                self.GetEgoTrackInfo().GetTrackId()))
        elif self.GetEndTs() > self.GetEgoTrackInfo().GetEndOffset():
            raise ValueError("ego agent {} ends before scenario".format(
                self.GetEgoTrackInfo().GetTrackId()))

        return True
