import argparse
import math
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class Waypoint():

    def __init__(self, north, east, altitude, heading_degrees=0):
        self.north = north
        self.east = east
        self.altitude = altitude
        self.heading = math.radians(heading_degrees)

    def is_close_to(self, north, east, altitude, within):
        is_close_north = abs(north-self.north) <= within
        is_close_east = abs(east-self.east) <= within
        is_close_altitude = abs(altitude-self.altitude) <= within
        return is_close_north and is_close_east and is_close_altitude

    def __str__(self) -> str:
        return f"Waypoint[N={self.north}, E={self.east}, A={self.altitude}, H={self.heading}]"


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.in_mission = True
        self.check_state = {}
        
        self.target_altitude = 5.0
        self.all_waypoints = []
        self.calculate_box()

        # initial state
        self.flight_state = States.MANUAL

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if self.is_at_target_height():
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if self.is_at_waypoint():
                self.waypoint_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and
            abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            return

        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()

    def is_at_target_height(self):
        # coordinate conversion
        altitude = -1.0 * self.local_position[2]
        return altitude > 0.95 * self.target_position[2]

    def is_at_waypoint(self):
        if not self.next_waypoint:
            return False
        altitude = -1.0 * self.local_position[2]
        return self.next_waypoint.is_close_to(self.local_position[0],
                                              self.local_position[1],
                                              altitude,
                                              within=0.20)

    def calculate_box(self):
        """        
        Populates all waypoints for the flight.
        """
        self.all_waypoints.append(Waypoint(5, 0, self.target_altitude))
        self.all_waypoints.append(Waypoint(5, 5, self.target_altitude))
        self.all_waypoints.append(Waypoint(0, 5, self.target_altitude))
        self.all_waypoints.append(Waypoint(0, 0, self.target_altitude))

    def arming_transition(self):
        """
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        self.target_position[2] = self.target_altitude
        self.takeoff(self.target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        if len(self.all_waypoints) > 0:
            self.flight_state = States.WAYPOINT
            self.next_waypoint = self.all_waypoints.pop(0)
            
            print(f"next waypoint: {self.next_waypoint}")
            self.target_position = np.array([self.next_waypoint.north,
                                             self.next_waypoint.east,
                                             self.next_waypoint.altitude])
            self.cmd_position(self.next_waypoint.north,
                              self.next_waypoint.east,
                              self.next_waypoint.altitude,
                              self.next_waypoint.heading)
        else:
            time.sleep(1) # give a second for the drone to stabilize
            self.landing_transition()


    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    
    drone = BackyardFlyer(conn)

    time.sleep(2)
    drone.start()