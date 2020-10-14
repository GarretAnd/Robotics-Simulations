import math

current_y = 1.5  # Defining Constants
goal_y = 2
sample = .4
k_p = 1
vel = goal_y - current_y
angle = 0
step = 1

while current_y < goal_y:
    error = goal_y - current_y  # Getting current error
    u = k_p * error

    omega = u * 2 * math.pi  # Converting to radians

    arc_angle = omega * sample  # Calculating Arc Values
    arc_length = sample * vel
    delta_y = arc_length

    if arc_angle + angle > math.pi/2:
        arc_angle = 0

    if arc_angle != 0:
        arc_radius = arc_length / arc_angle
        delta_y = (arc_radius * math.cos(angle)) - (arc_radius * math.cos(arc_angle + angle))

    current_y = current_y + delta_y  # Find change in y and adjust current position/angle
    angle = angle + arc_angle

    print("The current step is: " + str(step) + "  The current Y Position is: " + str(current_y) +
          "  The Error was: " + str(error))
    step = step + 1

print("The total time taken is: " + str((step-1)*sample) + " seconds")

# Output:
# The current step is: 1  The current Y Position is: 1.609973360937722  The Error was: 0.5
# The current step is: 2  The current Y Position is: 1.809973360937722  The Error was: 0.39002663906227797
# The current step is: 3  The current Y Position is: 2.009973360937722  The Error was: 0.190026639062278
# The total time taken is: 1.2000000000000002 seconds
