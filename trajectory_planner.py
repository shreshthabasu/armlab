"""!
! Trajectory planner.

TODO: build a trajectory generator and waypoint planner so it allows your state machine to iterate through the plan at
the desired command update rate.
"""
import numpy as np
import time


class TrajectoryPlanner():
    """!
    @brief      This class describes a trajectory planner.
    """

    def __init__(self, rexarm):
        """!
        @brief      Constructs a new instance.

        @param      rexarm  The rexarm
        """
        self.idle = True
        self.rexarm = rexarm
        self.initial_wp = None
        self.final_wp = None
        self.dt = 0.05  # command rate

    def set_initial_wp(self):
        """!
        @brief      TODO: Sets the initial wp to the current position.
        """
        self.initial_wp = self.rexarm.get_positions()
        pass

    def set_final_wp(self, waypoint):
        """!
        @brief      TODO: Sets the final wp.

        @param      waypoint  The waypoint
        """
        self.final_wp = waypoint
        pass

    def go(self, max_speed=1.0):
        """!
        @brief      TODO Plan and execute the trajectory.

        @param      max_speed  The maximum speed
        """
        T = self.calc_time_from_waypoints(self.initial_wp, self.final_wp, max_speed)
        plan = self.generate_quintic_spline(self.initial_wp, self.final_wp, T)
        speed_plan = [((np.array(plan[i + 1]) - np.array(plan[i])) / self.dt).tolist() for i in range(len(plan) - 1)]
        speed_plan = [([0] * self.rexarm.num_joints)] + speed_plan
        #print(speed_plan)
        for i in range(7):
            speed_plan.append(speed_plan[-1])
        for i in range (len(plan)):
            if((len(plan)-i)<8):
                waypoint = plan[-1]
            else:
                waypoint = plan[i+7]
            full_wp = [0.0] * self.rexarm.num_joints
            full_wp[0:len(waypoint)] = waypoint
            self.rexarm.set_positions(full_wp)
            # self.rexarm.set_speeds(speed_plan[i + 1])
            time.sleep(self.dt)

    def stop(self):
        """!
        @brief      TODO Stop the trajectory planner
        """
        # if self.final_wp == self.rexarm.get_postions():
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed=1.0):
        """!
        @brief      TODO Calculate the time to get from initial to final waypoint.

        @param      initial_wp  The initial wp
        @param      final_wp    The final wp
        @param      max_speed   The maximum speed

        @return     The amount of time to get to the final waypoint.
        """
        max_dist = max([abs(final_wp[i] - initial_wp[i]) for i in range(len(final_wp))])
        return (max_dist / max_speed) * 1.2

    def generate_quintic_spline(self, initial_wp, final_wp, T):
        """!
        @brief      TODO generyate a qintic spline

        @param      initial_wp  The initial wp
        @param      final_wp    The final wp
        @param      T           Amount of time to get from initial to final waypoint

        @return     The plan as num_steps x num_joints np.array
        """
        M = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 2, 0, 0, 0],
                      [1, T, T**2, T**3, T**4, T**5],
                      [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
                      [0, 0, 2, 6*T, 12*T**2, 20*T**3]])
        b = np.array([initial_wp,
                      np.zeros(self.rexarm.num_joints),
                      np.zeros(self.rexarm.num_joints),
                      final_wp,
                      np.zeros(self.rexarm.num_joints),
                      np.zeros(self.rexarm.num_joints)])
        a = np.matmul(np.linalg.inv(M), b)
        plan = [initial_wp]
        for i in np.linspace(0, T, T/self.dt):
            x = np.array([1, i, i**2, i**3, i**4, i**5]).T
            plan.append(np.matmul(x, a).tolist())

        return plan

    def execute_plan(self, plan, look_ahead=8):
        """!
        @brief      TODO: Execute the planed trajectory.

        @param      plan        The plan
        @param      look_ahead  The look ahead
        """

        pass
