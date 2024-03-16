import math

# USER INPUTS
INITIAL_ROCKET_PITCH = 30 # Degrees
INITIAL_ROCKET_YAW = 30 # Degrees
PID_LOOP_TIME = 0.005 # Seconds
SERVO_LINKAGE_RATIO = 1 
TVC_PITCH_RANGE = 15 # Degrees
TVC_YAW_RANGE = 15 # Degrees
SERVO_DELAY_SEC = 0.03 # Seconds
PITCH_SENSOR_BIAS = 0 # Degrees
YAW_SENSOR_BIAS = 0 # Degrees
END_THRUST_TIME = 3 # Seconds
ROCKET_MASS = 10 * 1000# kg
ROCKET_RADIUS = 0.1 # m
MOTOR_MASS_START = 1.5 # kg
MOTOR_MASS_END = 0.5 # kg
THRUST = 500 # N
TVC_TO_COM = 0.5 # m
MOMENT_OF_INERTIA = (1/2) * ROCKET_MASS * ROCKET_RADIUS ** 2 # kg * m^2
TVC_DEG_PER_SECOND_MAX = 300 # Degrees per second
ROCKET_PITCH_RANGE = 15 # Degrees
ROCKET_YAW_RANGE = 15 # Degrees

# Constants
SIM_TIME_SECONDS = 10
DELTA_T_SECONDS = 0.1

INITIAL_TVC_PITCH = 0
INITIAL_TVC_YAW = 0




TVC_STEP_WAIT = math.ceil(SERVO_DELAY_SEC / DELTA_T_SECONDS)
DEG_TO_RAD = math.pi / 180
RAD_TO_DEG = 180 / math.pi

MAX_DEG_PER_TIME_STEP = TVC_DEG_PER_SECOND_MAX * DELTA_T_SECONDS



THRUST_STEP_NUM = math.floor(END_THRUST_TIME / DELTA_T_SECONDS)

MOTOR_MASS = [MOTOR_MASS_START]
MOTOR_MASS_DELTA = (MOTOR_MASS_START - MOTOR_MASS_END) / THRUST_STEP_NUM



CONSTANT_TORQUE_BIAS = 0.0001
TORQUE_NOISE_AMPLITUDE = 0

KP_TVC = 0.6
KI_TVC = 0.02
KD_TVC = 0.2
SERVO_PITCH_RANGE = SERVO_LINKAGE_RATIO * TVC_PITCH_RANGE
SERVO_YAW_RANGE = SERVO_LINKAGE_RATIO * TVC_YAW_RANGE

KP_POS = 3 
KI_POS = 1.2
KD_POS = 5  


# Variables
sim_steps = int(SIM_TIME_SECONDS / DELTA_T_SECONDS)
sensed_rocket_pitch = [INITIAL_ROCKET_PITCH]
sensed_rocket_yaw = [INITIAL_ROCKET_YAW]
sim_steps_between_pid_calls = int(math.ceil(PID_LOOP_TIME / DELTA_T_SECONDS))
tvc_pitch_commands = [0]
tvc_yaw_commands = [0]
rocket_pitch_setpoint = 0
rocket_yaw_setpoint = 0
setpoint_pitch = rocket_pitch_setpoint if abs(rocket_pitch_setpoint - INITIAL_ROCKET_PITCH) <= 180 else -(360 - rocket_pitch_setpoint)
setpoint_yaw = rocket_yaw_setpoint if abs(rocket_yaw_setpoint - INITIAL_ROCKET_YAW) <= 180 else -(360 - rocket_yaw_setpoint)
tvc_pitch = [INITIAL_ROCKET_PITCH]
tvc_yaw = [INITIAL_ROCKET_YAW]
torques_x = [0]
torques_y = [0]
y_position = [0]
accel_x = [0]
accel_y = [0]
velocity_x = [0]
velocity_y = [0]
actual_rocket_pitch = [INITIAL_ROCKET_PITCH]
actual_rocket_yaw = [INITIAL_ROCKET_YAW]

def init_pid(kp, ki, kd, range_val):
    last_err = 0
    first_step = True
    i_term = 0
    p_errs = [0]
    d_errs = [0]
    i_terms = [0, 0]

    def compute(input_val, setpoint_val, angular_vel=None):
        nonlocal last_err, first_step, i_term
        error = setpoint_val - input_val

        new_kp = kp
        new_kd = kd
        new_ki = ki

        i_term += new_ki * error * sim_steps_between_pid_calls * DELTA_T_SECONDS
        i_term = max(min(i_term, range_val), -range_val)

        if first_step:
            last_err = error
            first_step = False

        if angular_vel is not None:
            d_err = angular_vel / (sim_steps_between_pid_calls * DELTA_T_SECONDS)
        else:
            d_err = (error - last_err) / (sim_steps_between_pid_calls * DELTA_T_SECONDS)

        output = int(new_kp * error + i_term + new_kd * d_err)
        output = max(-range_val, min(output, range_val))

        last_err = error
        p_errs.append(error)
        d_errs.append(d_err)
        i_terms.append(i_term)

        if -output > range_val:
            return range_val
        if -output < -range_val:
            return -range_val
        return -output

    return {
        'p_errs': p_errs,
        'd_errs': d_errs,
        'i_terms': i_terms,
        'compute': compute
    }

# X = PITCH
# Y = YAW

TVCPID_PITCH = init_pid(KP_TVC, KI_TVC, KD_TVC, SERVO_PITCH_RANGE)
TVCPID_YAW = init_pid(KP_TVC, KI_TVC, KD_TVC, SERVO_YAW_RANGE)

X_POSITION_PID = init_pid(KP_POS, KI_POS, KD_POS, ROCKET_PITCH_RANGE)
Y_POSITION_PID = init_pid(KP_POS, KI_POS, KD_POS, ROCKET_YAW_RANGE)

def calculate_vane_thrust(tvc_angle):
    return -(3.8 * tvc_angle)

def limit_tvc_angle(new_tvc_angle, prev_tvc_angle):
    tvc_angle_sign = math.copysign(1, new_tvc_angle - prev_tvc_angle)
    angle_delta = abs(new_tvc_angle - prev_tvc_angle)

    if angle_delta > MAX_DEG_PER_TIME_STEP:
        allowed_angle = prev_tvc_angle + tvc_angle_sign * MAX_DEG_PER_TIME_STEP
        if allowed_angle > TVC_PITCH_RANGE:
            return TVC_PITCH_RANGE
        if allowed_angle < -TVC_PITCH_RANGE:
            return -TVC_PITCH_RANGE
        return allowed_angle
    else:
        return new_tvc_angle

def simulate():
    motor_mass_sum_loss = 0
    for i in range(1, sim_steps):
        prev_rocket_pitch = sensed_rocket_pitch[i - 1]
        prev_rocket_yaw = sensed_rocket_yaw[i - 1]

        x_thrust = 0
        y_thrust = 0

        if i % sim_steps_between_pid_calls == 0:
            tvc_pitch_commands.append(TVCPID_PITCH['compute'](prev_rocket_pitch, setpoint_pitch) / SERVO_LINKAGE_RATIO)
            tvc_yaw_commands.append(TVCPID_YAW['compute'](prev_rocket_yaw, setpoint_yaw) / SERVO_LINKAGE_RATIO)
        else:
            tvc_pitch_commands.append(tvc_pitch_commands[i - 1])
            tvc_yaw_commands.append(tvc_yaw_commands[i - 1])

        if i > TVC_STEP_WAIT:
            tvc_pitch.append(limit_tvc_angle(tvc_pitch_commands[i - TVC_STEP_WAIT], tvc_pitch[i - 1]))
            tvc_yaw.append(limit_tvc_angle(tvc_yaw_commands[i - TVC_STEP_WAIT], tvc_yaw[i - 1]))

            x_thrust = calculate_vane_thrust(tvc_pitch[i])

            y_thrust = calculate_vane_thrust(tvc_yaw[i])

        else:
            tvc_pitch.append(INITIAL_TVC_PITCH)
            tvc_yaw.append(INITIAL_TVC_YAW)

            x_thrust = calculate_vane_thrust(tvc_pitch[i])
            y_thrust = calculate_vane_thrust(tvc_yaw[i])
        

        # ALL THE CALCULATION BELOW HAVE ERROR. I WILL FIX IT LATER, not important for TVC, but important for SIMULATION
            
        torque_x = (x_thrust * TVC_TO_COM - CONSTANT_TORQUE_BIAS)
        torques_x.append(torque_x)

        torque_y = (y_thrust * TVC_TO_COM - CONSTANT_TORQUE_BIAS)
        torques_y.append(torque_y)


        accel_x.append(torque_x / MOMENT_OF_INERTIA)
        accel_y.append(torque_y / MOMENT_OF_INERTIA)

        velocity_x.append(velocity_x[i - 1] + accel_x[i - 1] * DELTA_T_SECONDS)
        velocity_y.append(velocity_y[i - 1] + accel_y[i - 1] * DELTA_T_SECONDS)

        exact_pitch = RAD_TO_DEG * (actual_rocket_pitch[i - 1] * DEG_TO_RAD + velocity_x[i - 1] * DELTA_T_SECONDS)
        exact_yaw = RAD_TO_DEG * (actual_rocket_yaw[i - 1] * DEG_TO_RAD + velocity_y[i - 1] * DELTA_T_SECONDS)

        actual_rocket_pitch.append(exact_pitch)
        actual_rocket_yaw.append(exact_yaw)

        sensed_rocket_pitch.append(exact_pitch + PITCH_SENSOR_BIAS)
        sensed_rocket_yaw.append(exact_yaw + YAW_SENSOR_BIAS)

        if i < THRUST_STEP_NUM:
            motor_mass_sum_loss += MOTOR_MASS_DELTA

        MOTOR_MASS.append(MOTOR_MASS_START - motor_mass_sum_loss)

        print("-------------------")
        print(f"Time: {i * DELTA_T_SECONDS}")
        print("\n")
        print(f"TVC Pitch: {tvc_pitch[i]}")
        print(f"Pitch Thrust: {x_thrust}")
        print(f"Sensed Rocket Pitch: {sensed_rocket_pitch[i]}")
        print(f"Actual Rocket Pitch: {actual_rocket_pitch[i]}")
        print("\n")
        print(f"TVC Yaw: {tvc_yaw[i]}")
        print(f"Yaw Thrust: {y_thrust}")
        print(f"Sensed Rocket Yaw: {sensed_rocket_yaw[i]}")
        print(f"Actual Rocket Yaw: {actual_rocket_yaw[i]}")
        print("\n")
        print(f"MOTOR_MASS: {MOTOR_MASS[i]}")
        print("-------------------")

simulate()