# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import logging, sys
from bark.ltl import RuleMonitor
from bark.world.evaluation import *
from bark.world.opendrive import XodrLaneType, XodrRoadMark, XodrRoadMarkType
from bark.geometry import Point2d
from modules.runtime.commons.parameters import ParameterServer


def make_rules(rule_params):
    rules = []
    for rule_json in rule_params:
        param_srv = ParameterServer()
        param_srv.convert_to_param(rule_json)
        rules.append(RuleMonitor(param_srv["formula", "Traffic rules described using LTL"],
                                      param_srv["weight", "Penalty for violating the rule.", 0.0],
                                      param_srv[
                                          "priority", "Index of the entry in the reward vector to add penalty.", 0]))

    return rules


def dump_rules(rules):
    rule_str = ""
    for agent in rules.keys():
        rule_str = rule_str + "Agent ID: {}".format(agent)
        for rule in rules[agent]:
            rule_str = rule_str + "\n{}".format(str(rule))
        rule_str = rule_str + "\n\n"
    return rule_str

def make_AgentBeyondPointLabelEvaluator(params):
    label_str = params["label_str"]
    point = params["point"]
    return AgentBeyondPointLabelEvaluator(label_str, Point2d(point[0], point[1]))

def make_EgoBeyondPointLabelEvaluator(params):
    label_str = params["label_str"]
    point = params["point"]
    return EgoBeyondPointLabelEvaluator(label_str, Point2d(point[0], point[1]))

def make_SafeDistanceLabelEvaluator(params):
    label_str = params["label_str"]
    reaction_delay = params["reaction_delay", "Delay until brakes are applied", 1.0]
    decel_ego = params["decel_ego", "Maximum brake acceleration for the ego agent", -8.0]
    decel_other = params["decel_other", "Maximum brake acceleration for the other agents", -8.0]
    to_rear = ["to_rear_agent", "Measure safe distance to rear agent", False]
    return SafeDistanceLabelEvaluator(label_str, reaction_delay, to_rear, decel_ego, decel_other)

def _make_default_label(evaluator_name, params):
    label_str = params["label_str"]
    return eval("{}('{}')".format(evaluator_name, label_str))

def make_labels(label_params):
    labels = []
    for label in label_params:
        try:
            labels.append(eval("make_{}({})".format(label["type"], label)))
        except:
            try:
                labels.append(_make_default_label(label["type"], label))
            except Exception as e:
                logging.error("Error during label creation: {}".format(e))
                sys.exit(1)
    return labels
