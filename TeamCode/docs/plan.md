# FTC 7215 Turning Gears implementation plan

## Poses and Pose Mapping

### default poses

- Red Left (ex. -60, -60)
- Red Right (ex. 60, -60)

### default headings

- Red Left: 180deg
- Red Right: 0deg

### mapped poses

- Blue Left (ex. 60, 60); Red Left x *-1, y* -1, heading +180
- Blue Right (ex. -60, 60); Red Right x *-1, y* -1, heading * +180

### mapped headings

- Blue Left: 0deg; Red Left rotated 180deg
- Blue Right: 180deg; Red Right rotated 180deg

Alliance design:

RED is default pose; returns IdentityPoseMap
BLUE is mapped pose; returns transformed PoseMap

## To-Dos

### Servos

### Autonomous
- Launcher
  - 
- Intake

### Vision

- tag IDs
