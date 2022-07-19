# Objectives
1. Perception Using Laser Range Finder: Visulise walls "visible" to the robot from the data obtained from the laser range finder
2. Implement Bug2 algoritm to avoid obstacles in the way

## Perception (perception.py node)
- subsribed to laserscan data, converted detected points into cartezian coordinates
- transfered this points to RANSAC algorithm which gives two points defining detected wall
- transfered these points to marker message to publish a line
- visulised the line using rviz

## Bug2 (bug2.py)
- start point and goal/destination point were given, rosparameter is used to define goal coordinates 
which can be changed from bug2.launch file
- Implemented Bug2 algorithm: robot reaches to the given destination from the start point by 
circumnavigating obstacles in the way
- till the goal is reached, the robot can be either GOALSEEK state or WALLFOLLOW state

### Tools used: rviz, stage simulator
### Concepts: Local Path planning using Bug2, marker message, rosparameter
