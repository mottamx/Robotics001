from robodk.robolink import *   # RoboDK API
from robodk.robomath import *   # Robot toolbox
RDK = Robolink()
robot = RDK.Item('Fanuc LR Mate 200iD')
robot.setJoints([0,0,0,0,0,0])

target = RDK.Item('Target 1')
robot.MoveJ(target) #Movimiento joint

RDK.ShowMessage("Movimiento Joint a target")

movements = [
    (-150,-100,0), #A
    (150,-100,0), #B
    (150,-100,200), #C
    (-150,-100,200),#D
    (-150,100,200), #E  
    (150,100,200), #F
    (150,100,0), #G
    (-150,100,0), #H
    (-150,-100,0), #A
    (-150,-100,200),#D
    (150,-100,200), #C
    (150,100,200), #F
    (-150,100,200), #E
    (-150,100,0), #H
    (150,100,0), #G
    (150,-100,0), #B
    (0,0,100), #Half
]
for i in movements:
    nextVert = target.Pose()*transl(i)
    robot.MoveL(nextVert)

for i, movement in enumerate(movements):
    if i != len(movements) - 1:  # Saltar el último movimiento
        new_x = movements[i][0]/2
        new_y = movements[i][1]/2
        if movements[i][2] == 0:
            new_z = 25
        else:
            new_z = movements[i][2]-25
        new_point = (new_x, new_y, new_z)
        nextVertHalf = target.Pose()*transl(new_point)
        robot.MoveL(nextVertHalf)

msg = "No problems"
RDK.ShowMessage(msg)