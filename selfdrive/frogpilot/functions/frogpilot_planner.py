import cereal.messaging as messaging

from openpilot.common.conversions import Conversions as CV

from openpilot.selfdrive.frogpilot.functions.frogpilot_functions import FrogPilotFunctions

class FrogPilotPlanner:
  def __init__(self, CP, params, params_memory):
    self.CP = CP
    self.params_memory = params_memory

    self.fpf = FrogPilotFunctions()

    self.v_cruise = 0

    self.update_frogpilot_params(params)

  def update(self, carState, controlsState, modelData, mpc, sm, v_cruise, v_ego):
    enabled = controlsState.enabled

    # Update the max allowed speed
    self.v_cruise = self.update_v_cruise(carState, controlsState, enabled, modelData, v_cruise, v_ego)

  def update_v_cruise(self, carState, controlsState, enabled, modelData, v_cruise, v_ego):
    # Adjust v_cruise to match the cluster
    v_cruise_cluster = controlsState.vCruiseCluster * CV.KPH_TO_MS if controlsState.vCruiseCluster != 0.0 else v_cruise
    v_cruise_diff = v_cruise_cluster - v_cruise
    v_cruise += v_cruise_diff

    return min(v_cruise)

  def publish(self, sm, pm, mpc):
    frogpilot_plan_send = messaging.new_message('frogpilotPlan')
    frogpilot_plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])
    frogpilotPlan = frogpilot_plan_send.frogpilotPlan

    pm.send('frogpilotPlan', frogpilot_plan_send)

  def update_frogpilot_params(self, params):
    self.is_metric = params.get_bool("IsMetric")

    custom_ui = params.get_bool("CustomUI")

    longitudinal_tune = params.get_bool("LongitudinalTune")
