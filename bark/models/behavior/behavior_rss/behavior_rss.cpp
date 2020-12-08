// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <memory>
#include <optional>
#include <tuple>

#include "bark/models/behavior/behavior_rss/behavior_rss.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

Trajectory BehaviorRSSConformant::Plan(
    double min_planning_time, const world::ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);

  const auto& lane_corr = observed_world.GetLaneCorridor();
  if (!initial_lane_corr_) {
    initial_lane_corr_ = lane_corr;
    behavior_safety_model_->SetInitialLaneCorridor(lane_corr);
    auto road_corr = observed_world.GetRoadCorridor();
    const auto& lane_corrs = road_corr->GetUniqueLaneCorridors();
    VLOG(4) << "Initial LaneCorridor used for safety behavior: " << *lane_corr
            << std::endl;

    // set the other lane corridor as target
    for (const auto& lc : lane_corrs) {
      VLOG(4) << "LaneCorridor: " << *lc << std::endl;
      if (lane_corr != lc) {
        VLOG(4) << "Setting LaneCorridor for nominal behavior: " << *lc
                << std::endl;
        auto nominal_behavior =
            std::dynamic_pointer_cast<BehaviorIDMLaneTracking>(
                nominal_behavior_model_);
        nominal_behavior->SetConstantLaneCorridor(lc);
      }
    }
  }

  if (switch_off_lat_limits_on_road_x_) {
    // we calculate a road polygon in which we do not apply the lateral limits
    std::vector<XodrRoadId> road_ids{roadx_id_};
    observed_world.GetMap()->GenerateRoadCorridor(
        road_ids, XodrDrivingDirection::FORWARD);
    auto rc = observed_world.GetMap()->GetRoadCorridor(
        road_ids, XodrDrivingDirection::FORWARD);
    roadx_polygon_ = rc->GetPolygon();
  }

  const float length_until_end =
      behavior_safety_model_->GetInitialLaneCorridor()->LengthUntilEnd(
          observed_world.CurrentEgoPosition());
  if (length_until_end <= minimum_safety_corridor_length_) {
    // Do not switch the lane corridor any more but only apply braking in the
    // current lane corridor
    behavior_safety_model_->SetInitialLaneCorridor(lane_corr);
  }

  if (!lane_corr) {
    VLOG(4) << "Agent " << observed_world.GetEgoAgentId()
            << ": Behavior status has expired!" << std::endl;
    SetBehaviorStatus(BehaviorStatus::EXPIRED);
    return GetLastTrajectory();
  }

  auto eval_res =
      boost::get<std::optional<bool>>(rss_evaluator_->Evaluate(observed_world));

#ifdef RSS
  auto rss_evaluator = std::dynamic_pointer_cast<EvaluatorRSS>(rss_evaluator_);
  if (rss_evaluator) {
    const auto& rss_response = rss_evaluator->GetRSSProperResponse();
    lon_response_ = rss_response.longitudinalResponse;
    lat_left_response_ = rss_response.lateralResponseLeft;
    lat_right_response_ = rss_response.lateralResponseRight;
    acc_restrictions_ = rss_response.accelerationRestrictions;
    VLOG(4) << "RSS Response: " << rss_response;
    safety_polygons_ = rss_evaluator->GetSafetyPolygons();
    ConvertRestrictions(min_planning_time, acc_restrictions_, observed_world);
  }
#endif

  if (!*eval_res) {
    VLOG(4) << "RSS is violated." << std::endl;
    behavior_rss_status_ = BehaviorRSSConformantStatus::SAFETY_BEHAVIOR;
    world_time_of_last_rss_violation_ = observed_world.GetWorldTime();
  } else {
    behavior_rss_status_ = BehaviorRSSConformantStatus::NOMINAL_BEHAVIOR;
  }

  Action last_action;
  dynamic::Trajectory last_traj;
  if (behavior_rss_status_ == BehaviorRSSConformantStatus::NOMINAL_BEHAVIOR ||
      no_safety_maneuver_) {
// execute normal
#ifdef RSS
    if (acc_restrictions_for_nominal_) {
      ApplyRestrictionsToModel(GetAccelerationLimitsVehicleCs(),
                               nominal_behavior_model_);
    }
#endif
    nominal_behavior_model_->Plan(min_planning_time, observed_world);
    last_action = nominal_behavior_model_->GetLastAction();
    last_traj = nominal_behavior_model_->GetLastTrajectory();
  } else {
    LOG(INFO) << "Executing safety behavior." << std::endl;
#ifdef RSS
    if (acc_restrictions_for_safety_) {
      ApplyRestrictionsToModel(GetAccelerationLimitsVehicleCs(),
                               behavior_safety_model_->GetBehaviorModel());
    }
#endif
    behavior_safety_model_->Plan(min_planning_time, observed_world);
    last_action = behavior_safety_model_->GetLastAction();
    last_traj = behavior_safety_model_->GetLastTrajectory();
  }
  SetLastTrajectory(last_traj);
  SetLastAction(last_action);
  return last_traj;
}

#ifdef RSS
void BehaviorRSSConformant::ConvertRestrictions(
    double delta_time,
    const ::ad::rss::state::AccelerationRestriction& rss_rest,
    const ObservedWorld& observed_world) {
  State last_state(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  auto history = observed_world.GetEgoAgent()->GetStateInputHistory();
  if (history.size() >= 2) {
    last_state = (history.end() - 2)->first;
  } else {
    last_state = observed_world.CurrentEgoState();
  }

  State ego_state = observed_world.CurrentEgoState();
  FrenetState ego_frenet(ego_state,
                         observed_world.GetLaneCorridor()->GetCenterLine());
  FrenetState last_ego_frenet(
      last_state, observed_world.GetLaneCorridor()->GetCenterLine());
  double acc_lon;
  auto last_action = GetLastAction();
  if (last_action.type() == typeid(Continuous1DAction)) {
    acc_lon = boost::get<Continuous1DAction>(last_action);
  } else if (last_action.type() == typeid(LonLatAction)) {
    acc_lon = boost::get<LonLatAction>(last_action).acc_lon;
  } else if (last_action.type() == typeid(Input)) {
    acc_lon = boost::get<Input>(last_action)(0);
  } else {
    LOG(FATAL) << "action type unknown: "
               << boost::apply_visitor(action_tostring_visitor(), last_action);
  }

  // Transform from streetwise to vehicle coordinate system
  double acc_lat_le_max, acc_lat_le_min, acc_lat_ri_max, acc_lat_ri_min;
  Polygon ego_polygon =
      observed_world.GetEgoAgent()->GetPolygonFromState(ego_state);
  if (switch_off_lat_limits_on_road_x_ &&
      Collide(ego_polygon, roadx_polygon_)) {
    // we do not apply the lateral limits
    acc_lat_le_max = 1000;  // could use default max and min values from rss lib
    acc_lat_le_min = -1000;
    acc_lat_ri_max = 1000;
    acc_lat_ri_min = -1000;
  } else {
    acc_lat_le_max = LatAccStreetToVehicleCs(rss_rest.lateralLeftRange.maximum,
                                             acc_lon, delta_time, ego_state,
                                             ego_frenet, last_ego_frenet);
    acc_lat_le_min = LatAccStreetToVehicleCs(rss_rest.lateralLeftRange.minimum,
                                             acc_lon, delta_time, ego_state,
                                             ego_frenet, last_ego_frenet);
    acc_lat_ri_max = LatAccStreetToVehicleCs(rss_rest.lateralRightRange.maximum,
                                             acc_lon, delta_time, ego_state,
                                             ego_frenet, last_ego_frenet);
    acc_lat_ri_min = LatAccStreetToVehicleCs(rss_rest.lateralRightRange.minimum,
                                             acc_lon, delta_time, ego_state,
                                             ego_frenet, last_ego_frenet);
  }
  AccelerationLimits acc_lim_vehicle_cs, acc_lim_street_cs;
  const float rss_vlat_threshold =
      0.01;  // @TODO parameter if this eps is too sensitive
  if (ego_frenet.vlat > rss_vlat_threshold) {
    VLOG(4) << "vel_lat_street = " << ego_frenet.vlat
            << ", Using left rss limits";
    // use left limits
    acc_lim_vehicle_cs.lat_acc_max = acc_lat_le_max;
    acc_lim_vehicle_cs.lat_acc_min = acc_lat_le_min;

    acc_lim_street_cs.lat_acc_max = rss_rest.lateralLeftRange.maximum;
    acc_lim_street_cs.lat_acc_min = rss_rest.lateralLeftRange.minimum;
  } else if (ego_frenet.vlat < -rss_vlat_threshold) {
    VLOG(4) << "vel_lat_street = " << ego_frenet.vlat
            << ", Using right rss limits";
    // use right limits
    acc_lim_vehicle_cs.lat_acc_max = acc_lat_ri_max;
    acc_lim_vehicle_cs.lat_acc_min = acc_lat_ri_min;

    acc_lim_street_cs.lat_acc_max = rss_rest.lateralRightRange.maximum;
    acc_lim_street_cs.lat_acc_min = rss_rest.lateralRightRange.minimum;
  } else {
    VLOG(4) << "vel_lat_street = " << ego_frenet.vlat << ", |vel_lat_street| < "
            << rss_vlat_threshold << " , Using both rss limits";
    // use both limits
    acc_lim_vehicle_cs.lat_acc_max = std::min(acc_lat_le_max, acc_lat_ri_max);
    acc_lim_vehicle_cs.lat_acc_min = std::max(acc_lat_le_min, acc_lat_ri_min);

    acc_lim_street_cs.lat_acc_max = std::min(
        rss_rest.lateralLeftRange.maximum, rss_rest.lateralRightRange.maximum);
    acc_lim_street_cs.lat_acc_min = std::max(
        rss_rest.lateralLeftRange.minimum, rss_rest.lateralRightRange.minimum);
  }

  acc_lim_vehicle_cs.lon_acc_max = rss_rest.longitudinalRange.maximum;
  acc_lim_vehicle_cs.lon_acc_min = rss_rest.longitudinalRange.minimum;

  acc_lim_street_cs.lon_acc_max = rss_rest.longitudinalRange.maximum;
  acc_lim_street_cs.lon_acc_min = rss_rest.longitudinalRange.minimum;
  VLOG(4) << "RSS Acceleration Restrictions in Road System: " << rss_rest;
  VLOG(4) << "Acceleration Limits in Road System: " << acc_lim_street_cs;
  VLOG(4) << "Acceleration Limits in Vehicle System: " << acc_lim_vehicle_cs;

  SetAccelerationLimitsVehicleCs(acc_lim_vehicle_cs);
  SetAccelerationLimitsStreetCs(acc_lim_street_cs);
}

void BehaviorRSSConformant::ApplyRestrictionsToModel(
    const AccelerationLimits& limits, std::shared_ptr<BehaviorModel> model) {
  VLOG(4) << "AccelerationLimits for model " << limits;
  std::shared_ptr<BehaviorIDMLaneTracking> behavior_idm_ptr =
      std::dynamic_pointer_cast<BehaviorIDMLaneTracking>(model);
  behavior_idm_ptr->SetAccelerationLimits(limits);
}

#endif

}  // namespace behavior
}  // namespace models
}  // namespace bark
