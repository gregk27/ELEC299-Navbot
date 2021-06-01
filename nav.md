# Robot Configuration
## Coordinate System
The coordiante system used is:
```
^ y
│
│  ╭─̬╮ heading
│  ╰─╯
└────────> x
```
## Sensor Configuration
**Ultrasonic:** Mounted front-centre, facing right<br/>
Used to maintain horizontal separation from obstacle

**Right IR:** Mounted front-left, facing forward. Range ~5cm<br/>
Used to detect obstacle ahead

**Left IR:** Mounted front-right, facing 45° right. Range ~5cm<br/>
Used to maitain detection of obstacle while turning

**Downward IR:** Mounted on udnerside, downward facing<br/>
Used to detect end marker on floor

# Navigation Procedures
This procedure outlines a simplified bug 0 algorithm which will be employed for navigation
Cases are listed in order of priority

## Case 4: Downward sensor detects marker
Indicates the target has been reached
 - Stop all navigation

## Case 3: Left sensor obstacle
Indicates that an obstacle is directly ahead
 - Pivot left

## Case 2: Ultrasonic distance < 20cm
Indicates driving alonside obstacle
 - Drive heading to maintain 15cm separation from obstacle

## Case 1: Right Sensor obstace
Indicates obstacle is 45° off right, but not in ultrasonic FoV
 - Pivot left

## Case 0: No detections
Inidicates that robot should continue to goal
 - Calculate heading to goal
 - Drive heading
