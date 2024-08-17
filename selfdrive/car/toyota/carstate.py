from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.toyota.values import CAR, DBC, STEER_THRESHOLD


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["AGS_1"]['GEAR_SELECTOR']

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset = True
    self.accurate_steer_angle_seen = CP.hasZss
    self.angle_offset = 0.

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()
    ret.brakePressed = 0 != 0
    ret.brakeLights = bool(ret.brakePressed)
    ret.wheelSpeeds.fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.01    #Changed this from 0.001 to 0.1 to 0.01 bc longcontrol.py uses this to detect when car is stopped

    # Some newer models have a more accurate angle measurement in the TORQUE_SENSOR message. Use if non-zero
#    if abs(cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']) > 1e-3:
#      self.accurate_steer_angle_seen = True
#
#    if self.accurate_steer_angle_seen:
#      if self.CP.hasZss:
#        ret.steeringAngleDeg = cp.vl["SECONDARY_STEER_ANGLE"]['ZORRO_STEER'] - self.angle_offset
#      else:
#        ret.steeringAngleDeg = cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE'] - self.angle_offset
#
#      if self.needs_angle_offset:
#        angle_wheel = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION']
#        if abs(angle_wheel) > 1e-3:
#          self.needs_angle_offset = False
#          self.angle_offset = ret.steeringAngleDeg - angle_wheel
#    else:
#      ret.steeringAngleDeg = -(cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION'])
#
#    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]['STEER_RATE']

    if self.CP.carFingerprint == CAR.OLD_CAR: # STILL NEED TO CHECK THIS, AND STEERINGRATE
      ret.steeringAngleDeg = -(cp.vl["STEERING_STATUS"]['STEERING_ANGLE'])
    
    if self.CP.carFingerprint == CAR.OLD_CAR: # Steering rate sensor is code differently on CIVIC
      if cp.vl["STEERING_EPS_DATA"]['STEER_ANGLERATE'] == 0:
        ret.steeringRateDeg = (cp.vl["STEERING_EPS_DATA"]['STEER_RATEDEG'])

    
    ret.leftBlinker = cp.vl["SCM_FEEDBACK"]['LEFT_BLINKER']
    ret.rightBlinker = cp.vl["IKE_2"]['RIGHT_BLINKER']

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_DRIVER']
    ret.steeringTorqueEps = cp.vl["STEERING_STATUS"]['STEERING_TORQUE']
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerWarning = False

    if self.CP.carFingerprint == CAR.LEXUS_IS:
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]['MAIN_ON'] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]['SET_SPEED'] * CV.KPH_TO_MS
      self.low_speed_lockout = False
    else:
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]['MAIN_ON'] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]['SET_SPEED'] * CV.KPH_TO_MS
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]['LOW_SPEED_LOCKOUT'] == 2
    self.pcm_acc_status = cp.vl["PCM_CRUISE"]['CRUISE_STATE']

    ret.cruiseState.enabled = bool(cp.vl["CRUISE_STATUS"]['CRUISE_ON'])
    ret.cruiseState.nonAdaptive = cp.vl["PCM_CRUISE"]['CRUISE_STATE'] in [1, 2, 3, 4, 5, 6]
    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = 2 #standby for testing

    ret.epsDisabled = (True if ret.genericToggle == 0 else False)

    return ret

  @staticmethod
  def get_can_parser(CP):
    
    signals = [
      # sig_name, sig_address, default
      ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
      ("STEERING_TORQUE", "STEERING_STATUS", 0),
      ("STEERING_ANGLE", "STEER_TORQUE_SENSOR", 0),
      ("STEER_ANGLE", "STEERING_COMMAND", 0)
    ]

    checks = []

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = [
      ("FORCE", "PRE_COLLISION", 0),
      ("PRECOLLISION_ACTIVE", "PRE_COLLISION", 0)
    ]

    # use steering message to check if panda is connected to frc
    checks = [
      ("STEERING_LKA", 42)
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
