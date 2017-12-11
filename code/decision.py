import math
import numpy as np

def find_leftmost_aboveavg_dist(Rover):
    if len(Rover.nav_angles) > 0:
        mean_dist = np.mean(Rover.nav_dists)
        min_dist = 10
        offset = 0
        it = np.nditer(Rover.nav_angles, flags=['multi_index'])
        while not it.finished:
            if Rover.nav_dists[it.multi_index] >= min_dist + offset:
                return it[0] * 180/np.pi
            it.iternext()
    return 0

def rover_stuck(Rover):
    return Rover.throttle > 0 and math.fabs(Rover.avg_vel) < 0.05

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    # Toggle for rock finding behavior
    rock_finding = False

    # If the rover is stuck
    if rover_stuck(Rover):
        Rover.stuck_timer += 1
        print("stuck for ", Rover.stuck_timer)
    else:
        Rover.stuck_timer = 0

    if Rover.picking_up:
        Rover.rock_angles = None
        Rover.rock_dists = None
    # If in a state where want to pickup a rock send pickup command
    elif Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    elif Rover.stuck_timer > 100:
        Rover.mode = 'reverse'
        Rover.stuck_timer = 0
        rock_finding = False
    # If there is a visible rock
    elif rock_finding and Rover.rock_angles is not None:
        mean_angle = np.mean(Rover.rock_angles * 180/np.pi)
        Rover.steer = np.clip(mean_angle, -7, 7)
        if np.mean(Rover.rock_dists) < 10:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            print("I break for rocks.")
        else:
            Rover.brake = 0
            if Rover.vel > 0.2:
                Rover.throttle = -0.2
            else:
                Rover.throttle = 0.1
            print("A ROCK! JACKPOT")
    # Check if we have nav vision data to make decisions with
    elif Rover.nav_angles is not None:
        mean_angle = np.mean(Rover.nav_angles * 180/np.pi)
        #leftmost_aboveavg_angle = find_leftmost_aboveavg_dist(Rover)
        #door_angle = Rover.left_wall_angle * 180 / np.pi
        #print("mean_angle", mean_angle)
        #print("leftmost", leftmost_aboveavg_angle)

        # Check for Rover.mode status
        if Rover.mode == 'forward':
            #rock_finding = True
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(mean_angle + 10, -15, 15)
                #print("forward steer: ", Rover.steer)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                print("state changing to stop")
                # set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                #print("braking")
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                #print("moving slowly")
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    #print("force steering")
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    #print("found navigable terrain")
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(mean_angle, -15, 15)
                    #print("from stop steer: ", Rover.steer)
                    Rover.mode = 'forward'
        elif Rover.mode == 'reverse':
            print('in reverse')
            if Rover.reverse_timer <= 50:
                print('backing up')
                Rover.brake = 0
                if Rover.vel < -0.2:
                    Rover.throttle = 0.1
                else:
                    Rover.throttle = -0.1
                Rover.reverse_timer += 1
                Rover.steer = -15
            else:
                Rover.reverse_timer = 0
                Rover.mode = 'stop'

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        print("just do something!")
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover
