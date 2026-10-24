# Poses and Pose Mapping

## default poses:
- Blue Left (ex. 60, 60)
- Blue Right (ex. -60, 60)

## default headings:
- Blue Left: 0deg
- Blue Right: 180deg
- 
## mapped poses:
- Red Left (ex. -60, -60); Blue Left x * -1, y * -1
- Red Right (ex. 60, -60); Blue Right x * -1, y * -1

## mapped headings:
- Red Left: 180deg; Blue Left rotated 180deg
- Red Right: 0deg; Blue Right rotated 180deg

Alliance design:

BLUE is default pose; returns IdentityPoseMap
RED is mapped pose; returns transformed PoseMap

