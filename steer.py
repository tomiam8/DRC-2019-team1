
def steer(midpoints, height, width):
    # midpoints is a list of midpoints :)
    # returns the value that should be sent to arduino

    # if no midpoints found don't turn?? reconsider this later
    if len(midpoints) == 0:
        return 90

    kp = 90 / 70
    kd = 0

    # one midpoint == only proportional
    if len(midpoints) == 1:
        x, y = midpoints[0]
        # theta = x from centre to midpoint / y from centre to midpoint
        theta = (x - width/2) / (height - y)
        theta += 90

        # -20 acts as a stabiliser
        return kp * (theta - 20)

    # more than one midpoint == proportional and differential
    if len(midpoints) == 2:
        x1, y1 = midpoints[0]
        x2, y2 = midpoints[1]

        theta1 = (x1 - width/2)) / (height - y1)
        theta1 += 90
        theta2 = (x2 - width/2)) / (height - y2)
        theta2 += 90

        change_in_theta = (theta2 - theta1) / (y1 - y2)

        return kp * (theta1 - 20) + kd * (change_in_theta - 20 / height)

    if len(midpoints) == 3:
        x1, y1 = midpoints[0]
        x2, y2 = midpoints[1]
        x3, y3 = midpoints[2]

        theta1 = (x1 - width/2)) / (height - y1)
        theta1 += 90
        theta2 = (x2 - width/2)) / (height - y2)
        theta2 += 90
        theta3 = (x3 - width/2)) / (height - y3)
        theta3 += 90

        change_in_theta1 = (theta2 - theta1) / (y1 - y2)
        change_in_theta2 = (theta3 - theta2) / (y2 - y3)
        change_in_theta_average = (change_in_theta1 + change_in_theta2) / 2

        # return kp * (theta1 - 20) + kd * (change_in_theta1 - 20 / height) + kd * (change_in_theta2 - 20 / height)
        return kp * (theta1 - 20) + kd * (change_in_theta_average - 20 / height)

    # if something wrong tho don't steer
    return 90


def speed(midpoints, height, width, steer):

    # if no midpoints found don't move?? reconsider this later
    if len(midpoints) == 0:
        return 90

    kp = -0.333
    min_speed = 80
    max_speed_proportional = 50
    max_speed_integration = 40
    ki = 0

    proportional_speed = min_speed + kp * steer
    # one midpooint == only proportional
    if len(midpoints) == 1:

        return proportional_speed

    # more than one midpoint == proportional and integration

    if len(midpoints) == 2:
        x1, y1 = midpoints[0]
        x2, y2 = midpoints[1]

        change_in_y = y1 - y2

        ki = max_speed_integration / (max_speed_proportional * change_in_y)

        return ki * change_in_y * proportional_speed

    if len(midpoints) == 3:
        x1, y1 = midpoints[0]
        x2, y2 = midpoints[1]
        x3, y3 = midpoints[2]

        change_in_y_1 = y1 - y2
        change_in_y_2 = y3 - y2
        change_in_y_average = (change_in_y1 + change_in_y2) / 2

        # ki_1 = max_speed_integration / (max_speed_proportional * change_in_y1)
        # ki_2 = max_speed_integration / (max_speed_proportional * change_in_y2)

        ki = max_speed_integration / (max_speed_proportional * change_in_y_average)

        # return ki_1 * change_in_y1 * proportional_speed + ki_2 * change_in_y2 * proportional_speed
        return ki * change_in_y_average * proportional_speed

    # if something wrong tho don't move
    return 90
