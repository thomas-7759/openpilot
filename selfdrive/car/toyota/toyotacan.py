def create_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Toyota Steer Command."""

  values = {
    "STEER_REQUEST": steer_req,
    "STEER_TORQUE_CMD": steer,
    "COUNTER": raw_cnt,
    "SET_ME_1": 1,
  }
  return packer.make_can_msg("STEERING_LKA", 0, values)

def create_new_steer_command(packer, mode, steer_delta, steer_tq, frame):
  """Creates a CAN message for the actuator STEERING_COMMAND"""
#  packer = CANPacker('ocelot_controls')
  values = {
    "SERVO_COUNTER": frame % 0xF,
    "STEER_MODE": mode,
    "STEER_ANGLE": steer_delta,
    "STEER_TORQUE": steer_tq,
  }
  msg = packer.make_can_msg("STEERING_COMMAND", 0, values)
  addr = msg[0]
  dat  = msg[2]

  values["SERVO_CHECKSUM"] = calc_checksum_8bit(dat, addr)

  return packer.make_can_msg("STEERING_COMMAND", 0, values) #bus 2 is the actuator CAN bus

def calc_checksum_8bit(work_data, msg_id): # 0xb8 0x1a0 0x19e 0xaa 0xbf
  checksum = msg_id
  for byte in work_data: #checksum is stripped from the data
    checksum += byte     #add up all the bytes

  checksum = (checksum & 0xFF) + (checksum >> 8); #add upper and lower Bytes
  checksum &= 0xFF #throw away anything in upper Byte
  return checksum

def create_accel_command(packer, accel, pcm_cancel, standstill_req, lead):
  # TODO: find the exact canceling bit that does not create a chime
  values = {
    "ACCEL_CMD": accel,
    "SET_ME_X01": 1,
    "DISTANCE": 0,
    "MINI_CAR": lead,
    "SET_ME_X3": 3,
    "SET_ME_1": 1,
    # "PERMIT_BRAKING": 1, #This was 082 version, added ""SET_ME_1": 1" because I think this is in my DBC? Did not check, maydbe gives problems?
    "RELEASE_STANDSTILL": not standstill_req,
    "CANCEL_REQ": pcm_cancel,
  }
  return packer.make_can_msg("ACC_CONTROL", 0, values)

def create_lead_command(packer, lead_rel_speed, lead_acceleration, lead_long_dist):
  values = {
    "LEAD_ACCELERATION": lead_acceleration,
    "LEAD_REL_SPEED": lead_rel_speed,
    "LEAD_LONG_DIST": lead_long_dist,
  }
  return packer.make_can_msg("LEAD_INFO", 0, values)


def create_acc_cancel_command(packer):
  values = {
    "GAS_RELEASED": 0,
    "CRUISE_ACTIVE": 0,
    "STANDSTILL_ON": 0,
    "ACCEL_NET": 0,
    "CRUISE_STATE": 0,
    "CANCEL_REQ": 1,
  }
  return packer.make_can_msg("PCM_CRUISE", 0, values)


def create_fcw_command(packer, fcw):
  values = {
    "FCW": fcw,
    "SET_ME_X20": 0x20,
    "SET_ME_X10": 0x10,
    "SET_ME_X80": 0x80,
  }
  return packer.make_can_msg("ACC_HUD", 0, values)


def create_ui_command(packer, steer, chime, left_line, right_line, left_lane_depart, right_lane_depart):
  values = {
    "RIGHT_LINE": 3 if right_lane_depart else 1 if right_line else 2,
    "LEFT_LINE": 3 if left_lane_depart else 1 if left_line else 2,
    "BARRIERS" : 3 if left_lane_depart else 2 if right_lane_depart else 0,
    "SET_ME_X0C": 0x0c,
    "SET_ME_X2C": 0x2c,
    "SET_ME_X38": 0x38,
    "SET_ME_X02": 0x02,
    "SET_ME_X01": 1,
    "SET_ME_X01_2": 1,
    "REPEATED_BEEPS": 0,
    "TWO_BEEPS": chime,
    "LDA_ALERT": steer,
  }
  return packer.make_can_msg("LKAS_HUD", 0, values)
