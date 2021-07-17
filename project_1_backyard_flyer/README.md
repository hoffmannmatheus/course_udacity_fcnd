
# Project 1 - Backyard Flyer

## Mission 
The vehicle should fly in a square shape and land within 1m of the starting location.

## Implementation Notes
The flight is done at 5m high, with waypoints corresponing to a square with 5m sides. 
Note that I've added a 0.5 second delay before starting waypoint transitions to make the flight a bit smoother. This is because the drone could reach a transition point with excess momentum and immediately start moving towards the next waypoint, resulting in an unwanted curved tragectory. Maybe there's a better way to solve this (like decelerating the drone before it gets to each waypoint?), but it wasn't obvious from the API.