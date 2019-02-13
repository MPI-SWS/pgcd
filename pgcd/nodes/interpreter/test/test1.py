import sys
print(sys.version)

import parser as rp
import executor as nv


def main():
    algorithm1 = '''
        a = sin(30);
        position = {"x": 1, "y": 2};
        pose = {"x": 0, "y": 1};
        if (pose.x - position.x > 0.01 || pose.y - position.y > 0.01)
        {
            skip
        } 
        else 
        {
            skip
        };
        angles = {"yaw": 0, "pitch":3.14159/4, "roll":0};
        position.y = 2*2 + sin(30);
        # receive(m_Idle){(msg_MoveToPosition, position, { m_MoveToPosition(position) })};
        # send(id_arm, msg_Rotate, angles)
    '''
    visitor = nv.Executor('comp1')
    visitor.execute(algorithm1)

main()
