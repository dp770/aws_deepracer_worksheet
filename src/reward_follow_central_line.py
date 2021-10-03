import math

# constants
MAX_SPEED = 4.0
MAX_STEERING = 30.0
MAX_STEPS_TO_DECAY_PENALTY = 10
MAX_STEPS_TO_PROGRESS_RATIO = 2.5
TRACK_WIDTH_FREE_ZONE = 0.05
TRACK_WIDTH_SAFE_ZONE = 0.15
CNT_DISTANCE_SENSITIVITY_EXP = 3.00  # higher number gives more freedom on the track, can cause zig-zags
ACTION_SPEED_SENSITIVITY_EXP = 3.00  # higher number increases penalty for low speed
ACTION_STEER_SENSITIVITY_EXP = 0.70  # higher number decreases penalty for high steering
TOTAL_PENALTY_ON_OFF_TRACK = 0.99  # maximum penalty in percentage of total reward on being off track
TOTAL_PENALTY_ON_HIGH_STEERING = 0.30  # maximum penalty in percentage of total reward on high steering
REWARD_WEIGHT_ON_TRACK = 5.00
REWARD_WEIGHT_MAX_SPEED = 2.50
REWARD_WEIGHT_MIN_STEER = 1.50
REWARD_WEIGHT_PROG_STEP = 1.00

# static
was_off_track_at_step = -MAX_STEPS_TO_DECAY_PENALTY


# Returns distance between two points in meters
def calc_distance(prev_point, next_point):
    delta_x = next_point[0] - prev_point[0]
    delta_y = next_point[1] - prev_point[1]
    return math.sqrt(delta_x * delta_x + delta_y * delta_y)


# Reward function expected by AWS DeepRacer API
def reward_function(params):
    # re-initialize was_off_track_at_step
    global was_off_track_at_step
    steps = params['steps']
    if steps < was_off_track_at_step:
        was_off_track_at_step = -MAX_STEPS_TO_DECAY_PENALTY
    if not params['all_wheels_on_track']:
        was_off_track_at_step = steps

    # Calculate penalty for wheels being or have recently been off track
    wheels_off_track_penalty = min(steps - was_off_track_at_step, MAX_STEPS_TO_DECAY_PENALTY) / (
            1.0 * MAX_STEPS_TO_DECAY_PENALTY)

    # Speed penalty to keep the car moving fast
    speed = params['speed']  # Range: 0.0:4.0
    speed_ratio = speed / MAX_SPEED
    reward_max_speed = REWARD_WEIGHT_MAX_SPEED * pow(speed_ratio, ACTION_SPEED_SENSITIVITY_EXP)

    # Steering penalty to discourage zig-zags
    steering = params['steering_angle']  # Range: -30:30
    steering_ratio = abs(steering / MAX_STEERING)
    reward_min_steering = REWARD_WEIGHT_MIN_STEER * (1.0 - pow(steering_ratio, ACTION_STEER_SENSITIVITY_EXP))

    # Reward on close distance to the racing line
    track_width = params['track_width']
    dislocation = params['distance_from_center']
    free_zone = track_width * TRACK_WIDTH_FREE_ZONE
    safe_zone = track_width * TRACK_WIDTH_SAFE_ZONE
    on_track_ratio = 0.0
    if dislocation <= free_zone:
        on_track_ratio = 1.0
    elif dislocation <= safe_zone:
        on_track_ratio = 1.0 - pow(dislocation / safe_zone, CNT_DISTANCE_SENSITIVITY_EXP)
    reward_on_track = on_track_ratio * REWARD_WEIGHT_ON_TRACK

    # Reward on good progress per step
    progress = params['progress']
    reward_prog_step = REWARD_WEIGHT_PROG_STEP * min(1.0, MAX_STEPS_TO_PROGRESS_RATIO * (progress / steps))

    reward_total = reward_on_track + reward_max_speed + reward_min_steering + reward_prog_step
    reward_total -= reward_total * (1.0 - on_track_ratio) * TOTAL_PENALTY_ON_OFF_TRACK
    reward_total -= reward_total * steering_ratio * TOTAL_PENALTY_ON_HIGH_STEERING
    reward_total *= wheels_off_track_penalty

    return float(0.0001 + reward_total)
