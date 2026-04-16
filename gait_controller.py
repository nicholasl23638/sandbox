import numpy as np
import math
import gait_utils
import leg_ik

def catbot_crawl(vel, cycle_period, turning=False, generated_leg_gait_func=gait_utils.configure_vel_to_gait_func(step_length=[-5, -2],step_width=[-2, 2],step_height=[-10, -8])):
    """
    inputs:
        vel: [x, y] vector
        t_mod: current timestamp modified to account for slowing down & speeding up "dt" to make gait slower when leg is on floor, faster when in air
        legs: {FL_leg: [hip motor, t_a motor, t_l motor], FR_leg: [...], BL_leg: [...], BR_leg: [...]}
        turning: if True, make robot rotate instead of strafing with ang_vel = vel. NOTE: this means we can't rotate while strafing. NOTE: vel in this case = vel inputs of FR_leg
        generated_leg_gait_func: function to turn robot vel inputs into leg 
    """
    ret = []

    leg_names = ["FL_leg", "FR_leg", "BL_leg", "BR_leg"]
    cycle_offset_crawl = [0, math.pi/2, math.pi, 3*math.pi/2] # time offset per leg to make legs out of sync with one another, thus enabling crawl
    hip_offset = 3.5

    if turning:
        # TODO: test that this is properly mapping to right legs
        turn_vel = [
            [-vel[0], vel[1]],
            [vel[0], vel[1]],
            [-vel[0], -vel[1]],
            [vel[0], -vel[1]]
            ]

    for i in range(4):
        leg = leg_names[i]

        # get current leg pos given timestamp, vel

        if not turning:
            x_pos, y_pos, z_pos = generated_leg_gait_func(vel, cycle_period, cycle_offset_crawl[i])
        else:
            x_pos, y_pos, z_pos = generated_leg_gait_func(turn_vel[i], cycle_period, cycle_offset_crawl[i])

        print("pose before offsets:", x_pos, y_pos, z_pos)
        # get leg angles given target pos with leg offsets
        if leg == "FL_leg" or leg == "BL_leg":
            hip_deg, t_a_deg, t_l_deg = leg_ik.calculate_leg_ik([x_pos, y_pos + hip_offset, z_pos], L_0=hip_offset)
        else:
            hip_deg, t_a_deg, t_l_deg = leg_ik.calculate_leg_ik([x_pos, y_pos - hip_offset, z_pos], L_0=-hip_offset)

        ret.append(np.array([hip_deg, t_a_deg, t_l_deg]))

    return ret
        

jeff = gait_utils.GhostMotor(1)
test_legs = {"FL_leg": [jeff, jeff, jeff], "FR_leg": [jeff, jeff, jeff], "BL_leg": [jeff, jeff, jeff], "BR_leg": [jeff, jeff, jeff]}

print(catbot_crawl([1, 0.5], 4))

