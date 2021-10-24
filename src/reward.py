import math

# constants
MAX_SPEED = 4.0
MAX_STEERING = 30.0
MAX_DIRECTION_DIFF = 30.0
MAX_STEPS_TO_DECAY_PENALTY = 0      # Value of zero or below disables penalty for having wheels off track
MAX_STEPS_TO_PROGRESS_RATIO = 2.0
TRACK_WIDTH_FREE_ZONE = 0.015
TRACK_WIDTH_SAFE_ZONE = 0.035
FOLLOWING_CENTRAL_LINE_RATIO = 0.02  # number in range of [0, 1]. Zero forces to follow smoothed line, 1 - central line
CNT_DISTANCE_SENSITIVITY_EXP = 3.00  # higher number gives more freedom on the track, can cause zig-zags
ACTION_SPEED_SENSITIVITY_EXP = 3.00  # higher number increases penalty for low speed
ACTION_STEER_SENSITIVITY_EXP = 0.70  # higher number decreases penalty for high steering
DIR_STEERING_SENSITIVITY_EXP = 2.00  # lower number accelerates penalty increase for not following track direction
TOTAL_PENALTY_ON_OFF_TRACK = 0.98  # maximum penalty in percentage of total reward on being off track
TOTAL_PENALTY_ON_OFF_DIR_STEER = 0.30  # maximum penalty in percentage of total reward on off directional steering
TOTAL_PENALTY_ON_HIGH_STEERING = 0.25  # maximum penalty in percentage of total reward on high steering
REWARD_WEIGHT_ON_TRACK = 35
REWARD_WEIGHT_PROG_STEP = 25
REWARD_WEIGHT_DIR_STEER = 15
REWARD_WEIGHT_MAX_SPEED = 10
REWARD_WEIGHT_MIN_STEER = 5
MAX_TOTAL_REWARD = REWARD_WEIGHT_ON_TRACK + REWARD_WEIGHT_PROG_STEP + REWARD_WEIGHT_DIR_STEER + \
                   REWARD_WEIGHT_MAX_SPEED + REWARD_WEIGHT_MIN_STEER

# static
smoothed_central_line = None
was_off_track_at_step = -MAX_STEPS_TO_DECAY_PENALTY
previous_steps_reward = MAX_TOTAL_REWARD

# Range [-180:+180]
def calc_slope(prev_point, next_point):
    return math.degrees(math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]))


# Range [0:180]
def calc_direction_diff(steering, heading, track_direction):
    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = steering + heading - track_direction
    if direction_diff > 180.0:
        direction_diff = direction_diff - 360.0
    if direction_diff < -180.0:
        direction_diff = direction_diff + 360.0
    return abs(direction_diff)


# Returns distance between two points in meters
def calc_distance(prev_point, next_point):
    delta_x = next_point[0] - prev_point[0]
    delta_y = next_point[1] - prev_point[1]
    return math.sqrt(delta_x * delta_x + delta_y * delta_y)


def smooth_central_line(center_line, max_offset, pp, p, c, n, nn, iterations=72, skip_step=1):
    smoothed_line = center_line
    for i in range(0, iterations):
        smoothed_line = smooth_central_line_internal(center_line, max_offset, smoothed_line, pp, p, c, n, nn, skip_step)
    return smoothed_line


def smooth_central_line_internal(center_line, max_offset, smoothed_line, pp, p, c, n, nn, skip_step):
    length = len(center_line)
    new_line = [[0.0 for _ in range(2)] for _ in range(length)]
    for i in range(0, length):
        wpp = smoothed_line[(i - 2 * skip_step + length) % length]
        wp = smoothed_line[(i - skip_step + length) % length]
        wc = smoothed_line[i]
        wn = smoothed_line[(i + skip_step) % length]
        wnn = smoothed_line[(i + 2 * skip_step) % length]
        new_line[i][0] = pp * wpp[0] + p * wp[0] + c * wc[0] + n * wn[0] + nn * wnn[0]
        new_line[i][1] = pp * wpp[1] + p * wp[1] + c * wc[1] + n * wn[1] + nn * wnn[1]
        while calc_distance(new_line[i], center_line[i]) >= max_offset:
            new_line[i][0] = (0.98 * new_line[i][0]) + (0.02 * center_line[i][0])
            new_line[i][1] = (0.98 * new_line[i][1]) + (0.02 * center_line[i][1])
    return new_line


# Calculate distance between current point and closest point on line between prev_point and next_point
def calc_distance_from_line(curr_point, prev_point, next_point):
    distance_cp_to_pp = calc_distance(curr_point, prev_point)  # b
    distance_cp_to_np = calc_distance(curr_point, next_point)  # a
    distance_pp_to_np = calc_distance(prev_point, next_point)  # c
    # cos A = (b^2 + c^2 - a^2) / 2bc
    angle_pp = math.acos((distance_cp_to_pp * distance_cp_to_pp + distance_pp_to_np * distance_pp_to_np
                          - distance_cp_to_np * distance_cp_to_np) / (2 * distance_cp_to_pp * distance_pp_to_np))
    # b / sin(Pi/2) = d / sin(A)
    return distance_cp_to_pp * math.sin(angle_pp)


def ema(prev, new, period):
    k = 2.0 / (1.0 + period)
    return k * new + (1.0 - k) * prev


# Reward function expected by AWS DeepRacer API
def reward_function(params):
    track_width = params['track_width']
    waypoints = params['waypoints']
    # initialize central line
    global smoothed_central_line
    if smoothed_central_line is None:
        max_offset = track_width * ((1.0 - FOLLOWING_CENTRAL_LINE_RATIO) / 2.0)
        smoothed_central_line = smooth_central_line(waypoints, max_offset, 0.10, 0.05, 0.70, 0.05, 0.10, skip_step=2)
        print("track_waypoints:",
              "track_width =", track_width, ", original =", waypoints, ", smoothed =", smoothed_central_line)

    # re-initialize was_off_track_at_step
    global was_off_track_at_step
    steps = params['steps']
    if steps < was_off_track_at_step:
        was_off_track_at_step = -MAX_STEPS_TO_DECAY_PENALTY
    if not params['all_wheels_on_track']:
        was_off_track_at_step = steps

    global previous_steps_reward
    if steps <= 2:
        previous_steps_reward = MAX_TOTAL_REWARD

    # Calculate penalty for wheels being or have recently been off track
    wheels_off_track_penalty = 1.0
    if MAX_STEPS_TO_DECAY_PENALTY > 0:
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

    # Reward on directional move to the next milestone
    wp_length = len(smoothed_central_line)
    wp_indices = params['closest_waypoints']
    curr_point = [params['x'], params['y']]
    prev_point = smoothed_central_line[wp_indices[0]]
    next_point_1 = smoothed_central_line[(wp_indices[1] + 1) % wp_length]
    next_point_2 = smoothed_central_line[(wp_indices[1] + 2) % wp_length]
    next_point_3 = smoothed_central_line[(wp_indices[1] + 3) % wp_length]
    track_direction_1 = calc_slope(prev_point, next_point_1)
    track_direction_2 = calc_slope(prev_point, next_point_2)
    track_direction_3 = calc_slope(prev_point, next_point_3)

    heading = params['heading']  # Range: -180:+180
    direction_diff_ratio = (
            0.50 * min((calc_direction_diff(steering, heading, track_direction_1) / MAX_DIRECTION_DIFF), 1.00) +
            0.30 * min((calc_direction_diff(steering, heading, track_direction_2) / MAX_DIRECTION_DIFF), 1.00) +
            0.20 * min((calc_direction_diff(steering, heading, track_direction_3) / MAX_DIRECTION_DIFF), 1.00))
    dir_steering_ratio = 1.0 - pow(direction_diff_ratio, DIR_STEERING_SENSITIVITY_EXP)
    reward_dir_steering = REWARD_WEIGHT_DIR_STEER * dir_steering_ratio

    # Reward on close distance to the racing line
    free_zone = track_width * TRACK_WIDTH_FREE_ZONE
    safe_zone = track_width * TRACK_WIDTH_SAFE_ZONE
    dislocation = calc_distance_from_line(curr_point, prev_point, next_point_1)
    on_track_ratio = 0.0
    if dislocation <= free_zone:
        on_track_ratio = 1.0
    elif dislocation <= safe_zone:
        on_track_ratio = 1.0 - pow(dislocation / safe_zone, CNT_DISTANCE_SENSITIVITY_EXP)
    reward_on_track = on_track_ratio * REWARD_WEIGHT_ON_TRACK

    # Reward on good progress per step
    progress = params['progress']
    reward_prog_step = REWARD_WEIGHT_PROG_STEP * min(1.0, MAX_STEPS_TO_PROGRESS_RATIO * (progress / steps))

    reward_total = reward_on_track + reward_max_speed + reward_min_steering + reward_dir_steering + reward_prog_step
    reward_total -= reward_total * (1.0 - on_track_ratio) * TOTAL_PENALTY_ON_OFF_TRACK
    reward_total -= reward_total * (1.0 - dir_steering_ratio) * TOTAL_PENALTY_ON_OFF_DIR_STEER
    reward_total -= reward_total * steering_ratio * TOTAL_PENALTY_ON_HIGH_STEERING
    reward_total *= wheels_off_track_penalty

    print("rewards:" + (20 * "{:.4f}," + "{:.4f}").format(
        wheels_off_track_penalty, reward_on_track, reward_max_speed, reward_min_steering, reward_dir_steering,
        reward_prog_step,
        dislocation, track_direction_1, track_direction_2, track_direction_3, direction_diff_ratio,
        waypoints[wp_indices[0]][0], waypoints[wp_indices[0]][1], prev_point[0], prev_point[1],
        next_point_1[0], next_point_1[1], next_point_2[0], next_point_2[1], next_point_3[0], next_point_3[1]))

    previous_steps_reward = ema(previous_steps_reward, reward_total, 5)
    return float(0.0001 + previous_steps_reward)
