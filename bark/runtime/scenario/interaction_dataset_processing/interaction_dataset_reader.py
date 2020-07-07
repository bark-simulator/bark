# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from bark.core.world.agent import Agent
from bark.core.models.behavior import BehaviorStaticTrajectory, BehaviorMobil
from bark.core.models.dynamic import StateDefinition
from bark.core.world.goal_definition import GoalDefinition, GoalDefinitionPolygon
from bark.core.geometry import Point2d, Polygon2d, Norm0To2PI
from bark.runtime.commons.model_json_conversion import ModelJsonConversion
# Interaction dataset tools
from com_github_interaction_dataset_interaction_dataset.python.utils import dataset_reader
from com_github_interaction_dataset_interaction_dataset.python.utils import dict_utils
import numpy as np

from bark.runtime.scenario.interaction_dataset_processing.agent_track_info import AgentTrackInfo


def BarkStateFromMotionState(state, time_offset=0):
    bark_state = np.zeros(int(StateDefinition.MIN_STATE_SIZE))
    bark_state[int(StateDefinition.TIME_POSITION)] = (
        state.time_stamp_ms - time_offset) / 1000.0
    bark_state[int(StateDefinition.X_POSITION)] = state.x
    bark_state[int(StateDefinition.Y_POSITION)] = state.y
    bark_state[int(StateDefinition.THETA_POSITION)] = Norm0To2PI(state.psi_rad)
    bark_state[int(StateDefinition.VEL_POSITION)] = pow(
        pow(state.vx, 2) + pow(state.vy, 2), 0.5)
    return bark_state.reshape((1, int(StateDefinition.MIN_STATE_SIZE)))


def TrajectoryFromTrack(track, start=0, end=None):
    states = list(dict_utils.get_item_iterator(track.motion_states))
    if end is None:
        end = states[-1][0]
    filtered_motion_states = list(
        filter(lambda s: start <= s[0] <= end, states))
    n = len(filtered_motion_states)
    traj = np.zeros((n, int(StateDefinition.MIN_STATE_SIZE)))
    for i, state in enumerate(filtered_motion_states):
        # TODO: rename start to be the scenario start time!
        traj[i, :] = BarkStateFromMotionState(state[1], start)
    return traj


def ShapeFromTrack(track, wheelbase=2.7):
    offset = wheelbase / 2.0
    length = track.length
    width = track.width
    pose = [0.0, 0.0, 0.0]
    points = [[length / 2.0 + offset, -width / 2.0], [length / 2.0 + offset, width / 2.0], [-length / 2.0 + offset, width / 2.0],
              [-length / 2.0 + offset, -width / 2.0], [length / 2.0 + offset, -width / 2.0]]
    poly = Polygon2d(pose, points)
    return poly


def InitStateFromTrack(track, start):
    minimum_start = min(track.motion_states)
    if minimum_start > start:
        start = minimum_start
    state = track.motion_states[int(start)]
    return BarkStateFromMotionState(state, state.time_stamp_ms).reshape((int(StateDefinition.MIN_STATE_SIZE), 1))


def GoalDefinitionFromTrack(track, end):
    states = list(dict_utils.get_item_iterator(track.motion_states))
    motion_state = states[-1][1]
    bark_state = BarkStateFromMotionState(motion_state)
    goal_polygon = Polygon2d(np.array([0.0, 0.0, 0.0]),
                             [Point2d(-1.5, 0),
                              Point2d(-1.5, 8),
                              Point2d(1.5, 8),
                              Point2d(1.5, 0)])
    goal_polygon = goal_polygon.Translate(Point2d(bark_state[0, int(StateDefinition.X_POSITION)],
                                                  bark_state[0, int(StateDefinition.Y_POSITION)]))
    goal_definition = GoalDefinitionPolygon(goal_polygon)
    return goal_definition


def BehaviorFromTrack(track, params, start, end):
    return BehaviorStaticTrajectory(params, TrajectoryFromTrack(track, start, end))


class InteractionDatasetReader:
    def __init__(self):
        self._track_dict_cache = {}

    def TrackFromTrackfile(self, filename, track_id):
        if filename not in self._track_dict_cache:
            self._track_dict_cache[filename] = dataset_reader.read_tracks(filename)
        track = self._track_dict_cache[filename][track_id]
        # TODO: Filter track
        return track

    def AgentFromTrackfile(self, track_params, param_server, scenario_track_info, agent_id):
        if scenario_track_info.GetEgoTrackInfo().GetTrackId() == agent_id:
          agent_track_info = scenario_track_info.GetEgoTrackInfo()
        elif agent_id in scenario_track_info.GetOtherTrackInfos().keys():
          agent_track_info = scenario_track_info.GetOtherTrackInfos()[agent_id]
        else:
          raise ValueError("unknown agent id {}".format(agent_id))
        fname = agent_track_info.GetFileName()  # track_params["filename"]
        track_id = agent_track_info.GetTrackId()  # track_params["track_id"]
        agent_id = agent_track_info.GetTrackId()
        track = self.TrackFromTrackfile(fname, track_id)
        start_time = scenario_track_info.GetStartTs()
        end_time = scenario_track_info.GetEndTs()

        behavior_model = track_params["behavior_model"]
        model_converter = ModelJsonConversion()
        if behavior_model is None:
            # each agent need's its own param server
            behavior = BehaviorFromTrack(track, param_server.AddChild(
                "agent{}".format(agent_id)), start_time, end_time)
        else:
            behavior = model_converter.convert_model(behavior_model, param_server)
        try:
            initial_state = InitStateFromTrack(track, start_time)
        except:
            raise ValueError("Could not retrieve initial state of agent {} at t={}.".format(
                agent_id, start_time))
        bark_agent = Agent(
            initial_state,
            behavior,
            model_converter.convert_model(
                track_params["dynamic_model"], param_server),
            model_converter.convert_model(
                track_params["execution_model"], param_server),
            ShapeFromTrack(track, param_server["DynamicModel"]["wheel_base",
                                                                 "Distance between front and rear wheel center", 2.7]),
            param_server.AddChild("agent{}".format(agent_id)),
            GoalDefinitionFromTrack(track, end_time),
            track_params["map_interface"])
        # set agent id from track
        bark_agent.SetAgentId(track_id)
        return bark_agent
