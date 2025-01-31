// global torque limit
const int TOYOTA_MAX_TORQUE = 1500;       // max torque cmd allowed ever

// rate based torque limit + stay within actually applied
// packet is sent at 100hz, so this limit is 1000/sec
const int TOYOTA_MAX_RATE_UP = 10;        // ramp up slow
const int TOYOTA_MAX_RATE_DOWN = 25;      // ramp down fast
const int TOYOTA_MAX_TORQUE_ERROR = 350;  // max torque cmd in excess of torque motor

// real time torque limit to prevent controls spamming
// the real time limit is 1500/sec
const int TOYOTA_MAX_RT_DELTA = 375;      // max delta torque allowed for real time checks
const uint32_t TOYOTA_RT_INTERVAL = 250000;    // 250ms between real time checks

// longitudinal limits
const int TOYOTA_MAX_ACCEL = 1500;        // 1.5 m/s2
const int TOYOTA_MIN_ACCEL = -3000;       // -3.0 m/s2

const int TOYOTA_ISO_MAX_ACCEL = 2000;        // 2.0 m/s2
const int TOYOTA_ISO_MIN_ACCEL = -3500;       // -3.5 m/s2

const int TOYOTA_STANDSTILL_THRSLD = 32;  // 1kph

// Roughly calculated using the offsets in openpilot +5%:
// In openpilot: ((gas1_norm + gas2_norm)/2) > 15
// gas_norm1 = ((gain_dbc*gas1) + offset1_dbc)
// gas_norm2 = ((gain_dbc*gas2) + offset2_dbc)
// In this safety: ((gas1 + gas2)/2) > THRESHOLD
const int TOYOTA_GAS_INTERCEPTOR_THRSLD = 845;
#define TOYOTA_GET_INTERCEPTOR(msg) (((GET_BYTE((msg), 0) << 8) + GET_BYTE((msg), 1) + (GET_BYTE((msg), 2) << 8) + GET_BYTE((msg), 3)) / 2) // avg between 2 tracks

const CanMsg TOYOTA_TX_MSGS[] = {{0x22E, 0, 5}, {0x164, 0,  8}};  // stepperservocan on bus 1 and civic cruise setting

AddrCheckStruct toyota_rx_checks[] = {
  {.msg = {{0x156, 0, 6, .check_checksum = false, .expected_timestep = 10000U}}}, // 156 is 342, steering eps data
  {.msg = {{0x17C, 0, 8, .check_checksum = false, .expected_timestep = 10000U}}}, // 17c is 380, powertrain data
  {.msg = {{0x1D0, 0, 8, .check_checksum = false, .expected_timestep = 30000U}}}, // 1d0 is 464, wheel speeds
  {.msg = {{0x224, 0, 8, .check_checksum = false, .expected_timestep = 25000U}, // idk
           {0x309, 0, 8, .check_checksum = false, .expected_timestep = 25000U}}}, // 309 is 777, car speed
};
const int TOYOTA_RX_CHECKS_LEN = sizeof(toyota_rx_checks) / sizeof(toyota_rx_checks[0]);

// global actuation limit states
int toyota_dbc_eps_torque_factor = 100;   // conversion factor for STEER_TORQUE_EPS in %: see dbc file

static uint8_t toyota_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);
  uint8_t checksum = (uint8_t)(addr) + (uint8_t)((unsigned int)(addr) >> 8U) + (uint8_t)(len);
  for (int i = 0; i < (len - 1); i++) {
    checksum += (uint8_t)GET_BYTE(to_push, i);
  }
  return checksum;
}

static uint8_t toyota_get_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int checksum_byte = GET_LEN(to_push) - 1;
  return (uint8_t)(GET_BYTE(to_push, checksum_byte));
}

static int toyota_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  bool valid = addr_safety_check(to_push, toyota_rx_checks, TOYOTA_RX_CHECKS_LEN,
                                 toyota_get_checksum, toyota_compute_checksum, NULL);

  if (valid && (GET_BUS(to_push) == 0)) {
    int addr = GET_ADDR(to_push);

    // get eps motor torque (0.66 factor in dbc)
    if (addr == 0x260) {
      int torque_meas_new = (GET_BYTE(to_push, 5) << 8) | GET_BYTE(to_push, 6);
      torque_meas_new = to_signed(torque_meas_new, 16);

      // scale by dbc_factor
      torque_meas_new = (torque_meas_new * toyota_dbc_eps_torque_factor) / 100;

      // update array of sample
      update_sample(&torque_meas, torque_meas_new);

      // increase torque_meas by 1 to be conservative on rounding
      torque_meas.min--;
      torque_meas.max++;
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    // exit controls on rising edge of gas press
    if (addr == 0x164) {
      // 5th bit is CRUISE_ACTIVE
      int cruise_engaged = GET_BYTE(to_push, 0) & 0x20;
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      cruise_engaged_prev = cruise_engaged;

      // sample gas pedal
      if (!gas_interceptor_detected) {
        gas_pressed = ((GET_BYTE(to_push, 0) >> 4) & 1) == 0;
      }
    }

    // sample speed
    if (addr == 0x158) {
      // first 2 bytes
      vehicle_moving = GET_BYTE(to_push, 0) | GET_BYTE(to_push, 1);
    }


    // E39 cars have brake_pressed on 0x1D2, corolla and rav4 on 0x224
    if ((addr == 0x224) || (addr == 0x17C)) {
      int byte = (addr == 0x224) ? 0 : 4;
      brake_pressed = ((GET_BYTE(to_push, byte) << 7) & 0x80) != 0;
    }

    // sample gas interceptor
    if (addr == 0x201) {
      gas_interceptor_detected = 1;
      int gas_interceptor = TOYOTA_GET_INTERCEPTOR(to_push);
      gas_pressed = gas_interceptor > TOYOTA_GAS_INTERCEPTOR_THRSLD;

      // TODO: remove this, only left in for gas_interceptor_prev test
      gas_interceptor_prev = gas_interceptor;
    }

    generic_rx_checks((addr == 0x2E4));
  }
  return valid;
}

static int toyota_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  if (!msg_allowed(to_send, TOYOTA_TX_MSGS, sizeof(TOYOTA_TX_MSGS)/sizeof(TOYOTA_TX_MSGS[0]))) {
    tx = 0;
  }

  if (relay_malfunction) {
    tx = 0;
  }

  // Check if msg is sent on BUS 0
  if (bus == 0) {

    // GAS PEDAL: safety check
    if (addr == 0x200) {
      if (!controls_allowed) {
        if (GET_BYTE(to_send, 0) || GET_BYTE(to_send, 1)) {
          tx = 0;
        }
      }
    }

    // ACCEL: safety check on byte 1-2
    if (addr == 0x343) {
      int desired_accel = (GET_BYTE(to_send, 0) << 8) | GET_BYTE(to_send, 1);
      desired_accel = to_signed(desired_accel, 16);
      if (!controls_allowed) {
        if (desired_accel != 0) {
          tx = 0;
        }
      }
      bool violation = (unsafe_mode & UNSAFE_RAISE_LONGITUDINAL_LIMITS_TO_ISO_MAX)?
        max_limit_check(desired_accel, TOYOTA_ISO_MAX_ACCEL, TOYOTA_ISO_MIN_ACCEL) :
        max_limit_check(desired_accel, TOYOTA_MAX_ACCEL, TOYOTA_MIN_ACCEL);

      if (violation) {
        tx = 0;
      }
    }

    // STEER: safety check on bytes 2-3
    if (addr == 0x2E4) {
      int desired_torque = (GET_BYTE(to_send, 1) << 8) | GET_BYTE(to_send, 2);
      desired_torque = to_signed(desired_torque, 16);
      bool violation = 0;

      uint32_t ts = TIM2->CNT;

      if (controls_allowed) {

        // *** global torque limit check ***
        violation |= max_limit_check(desired_torque, TOYOTA_MAX_TORQUE, -TOYOTA_MAX_TORQUE);

        // *** torque rate limit check ***
        violation |= dist_to_meas_check(desired_torque, desired_torque_last,
          &torque_meas, TOYOTA_MAX_RATE_UP, TOYOTA_MAX_RATE_DOWN, TOYOTA_MAX_TORQUE_ERROR);

        // used next time
        desired_torque_last = desired_torque;

        // *** torque real time rate limit check ***
        violation |= rt_rate_limit_check(desired_torque, rt_torque_last, TOYOTA_MAX_RT_DELTA);

        // every RT_INTERVAL set the new limits
        uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
        if (ts_elapsed > TOYOTA_RT_INTERVAL) {
          rt_torque_last = desired_torque;
          ts_last = ts;
        }
      }

      // no torque if controls is not allowed
      if (!controls_allowed && (desired_torque != 0)) {
        violation = 1;
      }

      // reset to 0 if either controls is not allowed or there's a violation
      if (violation || !controls_allowed) {
        desired_torque_last = 0;
        rt_torque_last = 0;
        ts_last = ts;
      }

      if (violation) {
        tx = 0;
      }
    }
  }

  return tx;
}

static void toyota_init(int16_t param) {
  controls_allowed = 0;
  relay_malfunction_reset();
  gas_interceptor_detected = 0;
  toyota_dbc_eps_torque_factor = param;
}

static int toyota_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  int bus_fwd = -1;
  if (!relay_malfunction) {
    if (bus_num == 0) {
      bus_fwd = 2;
    }
    if (bus_num == 2) {
      int addr = GET_ADDR(to_fwd);
      // block stock lkas messages and stock acc messages (if OP is doing ACC)
      // in TSS2, 0x191 is LTA which we need to block to avoid controls collision
      int is_lkas_msg = ((addr == 0x2E4) || (addr == 0x412) || (addr == 0x191));
      // in TSS2 the camera does ACC as well, so filter 0x343
      int is_acc_msg = (addr == 0x343);
      int block_msg = is_lkas_msg || is_acc_msg;
      if (!block_msg) {
        bus_fwd = 0;
      }
    }
  }
  return bus_fwd;
}

const safety_hooks toyota_hooks = {
  .init = toyota_init,
  .rx = toyota_rx_hook,
  .tx = toyota_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = toyota_fwd_hook,
  .addr_check = toyota_rx_checks,
  .addr_check_len = sizeof(toyota_rx_checks)/sizeof(toyota_rx_checks[0]),
};
