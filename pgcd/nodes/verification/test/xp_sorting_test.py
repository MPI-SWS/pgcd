
import spec.conf
from compatibility import *
from utils.geometry import *
from cart import CartSquare
from arm import Arm
from franka import FrankaEmikaPanda
from static_process import CubeProcess
from refinement import *
from vectorize import *
from mpmath import mp
from experiments_setups import World, XpTestHarness
from copy import deepcopy
from choreography.projection import Projection
import interpreter.parser as parser
import spec.conf
import time


def world():
    # mounting pts:  x      y     z        θ     comp
    w = World(  (    0,   0.7, 0.18,       0), # arm
                ( -0.6,     0,    0,       0), # carrier
                ( -0.2,  -1.1,    0, mp.pi/2), # franka
                (-0.95,     0,    0,       0), # producer
                ( 0.15, -0.05,    0,       0)) # sensor
    # specs
    arm       = Arm("arm", w, 0,  -2.2689280275926285, 2.2689280275926285, 0)
    carrier   = CartSquare("carrier", w, 1)
    franka    = FrankaEmikaPanda("franka", w, 2)
    producer  = CubeProcess("producer", 0, 0, 0, 0, 0.1, 0.2, 0.05, w, 3)
    sensor    = CubeProcess("sensor", 0, 0, 0, 0, 0.1, 0.1, 0.2, w, 4)
    return w

def choreo():
    return ''' Sorting =
        def start = (producer: Wait(1), arm: Idle(), franka: Idle(), carrier: Idle(), sensor: Idle()); x0
            x0 + x17 = x1
            x1 = producer -> carrier: Ok() ; x2
            x2 = producer -> arm: Ok(); x3
            x3 = producer -> franka: Ok(); x4
            x4 = producer -> sensor: Ok(); fork_producer
            fork_producer = { fpx > -0.81 } x5 ||   # all - producer
                            { fpx < -0.82 } x10a    # producer
            x5 = { (fpx > -0.8) && (fpx < 0.1) && (fpy > -0.2) && (fpy < 0.2) } x6a ||  # carrier
                 { (fpx > -0.4) && (fpy > 0.3) } x7a ||                                 # arm
                 { (fpx > -0.8) && (fpy < -0.3) } x8a ||                                # franka
                 { (fpx >  0.1) && (fpy > -0.15) && (fpy < 0.15) } x9a                  # sensor
            #  carrier move, the other stay
            x6a = (carrier: MoveCart(0, 0, 0, 0.5)) ; x6b
            x6b = (carrier: SetAngleCart(rad(-90))) ; x6c
            x6c = (carrier: StrafeCart(0.5, 0, rad(-90), -0.1)) ; x6z
            x7a = (arm: Idle()) ; x7z
            x8a = (franka: HomePos(0, 0, 0, 0, 0, 0, 0)) ; x8b
            x8b = (franka: Idle()) ; x8z
            x9a = (sensor: Idle()) ; x9z
            x6z || x7z || x8z || x9z = x11
            # carrier meets with the sensor and informs the arm+franka
            x11 = carrier -> sensor: Ok() ; choice_sensor
            # make the choice local at the sensor
            choice_sensor = [sensor_dummy == 0] x13a + [sensor_dummy != 0] x14a
            # go to franka side
            x13a = sensor -> carrier: Green() ; x13b
            x13b = carrier -> franka: Ok() ; x13c
            x13c = carrier -> arm: Done() ; x13d
            x13d = { (fpx > -0.8) && (fpy < 0.2) && ( (fpy < -0.2) || (fpx < 0.14) ) } x13d1a || # franka + carrier
                   { (fpx > -0.8) && (fpy > 0.3)  } x13d2a ||                                    # arm
                   { (fpx >  0.14) && (fpy > -0.15) && (fpy < 0.15) } x13d3a                     # sensor
            x13d1a = { (fpx > -0.8) && (fpy < -0.3) && ( (fpy < -0.6) || (fpz > 0.25) ) } frank_fast1 ||
                     { (fpx > -0.8) && (fpx < 0.14) && (fpy < 0.2) && (fpy > -0.5) && (fpz < 0.2) } carrier_slow_green1
            #frank_fast1 = (franka: SetJoints(0, 0, 0, 0, 0, 0, 0, 0.178310,0.635300,-0.449920,-2.122150,2.866786,2.016097,1.141317)); frank_fast2
            frank_fast1 = (franka: SetJoints(0, 0, 0, 0, 0, 0, 0, 0.178310,0.635300,-0.449920,-0.234,2.866786,2.016097,1.141317)); frank_fast2
            frank_fast2 = (franka: Idle()); frank_fast3
            carrier_slow_green1 = (carrier: MoveCart(0.6, 0, rad(-90), 0.4)); carrier_slow_green2
            frank_fast3 || carrier_slow_green2 = x13d1b
            x13d1b = carrier -> franka: Ok() ; x13d1c
            x13d1c = (franka: Grasp(0.02), carrier: Idle()) ; x13d1d
            #x13d1d = (franka: SetJoints(0.178310,0.635300,-0.449920,-2.122150,2.866786,2.016097,1.141317, 0, 0, 0, 0, 0, 0, 0), carrier: Idle()); x13d1e
            x13d1d = (franka: SetJoints(0.178310,0.635300,-0.449920,-0.234,2.866786,2.016097,1.141317, 0, 0, 0, 0, 0, 0, 0), carrier: Idle()); x13d1e
            x13d1e = franka -> carrier: Ok() ; fork_franka_carrier
            fork_franka_carrier = { (fpx > -0.8) && (fpy < -0.3) && ( (fpy < -0.6) || (fpz > 0.25) ) } x13d1f1a ||            # franka
                                  { (fpx > -0.8) && (fpx < 0.14) && (fpy < 0.2) && (fpy > -0.5) && (fpz < 0.2) } x13d1f2a     # carrier
            #x13d1f1a = (franka: SetJoints(0, 0, 0, 0, 0, 0, 0, 0.926170,-1.693679,1.469714,-2.709620,1.511592,1.437029,0.573354)); x13d1f1b
            x13d1f1a = (franka: SetJoints(0, 0, 0, 0, 0, 0, 0, 0.926170,-0.908281,1.469714,-0.35425,1.511592,1.437029,0.573354)); x13d1f1b
            x13d1f1b = (franka: Open()); x13d1f1c
            x13d1f1c = (franka: HomePos(0.926170,-0.908281,1.469714,-0.35425,1.511592,1.437029,0.573354)); x13d1f1d
            x13d1f1d = (franka: Idle()); x13d1f1z
            x13d1f2a = (carrier: MoveCart(0.6, -0.4, rad(-90), -0.4)); x13d1f2b
            x13d1f2b = (carrier: StrafeCart(0.6, 0.0, rad(-90), 0.1)); x13d1f2c
            x13d1f2c = (carrier: SetAngleCart(0)); x13d1f2d
            x13d1f2d = (carrier: MoveCart(0.5, 0.0, 0, -0.5)); x13d1f2z
            x13d2a = (arm: Idle()) ; x13d2z
            x13d3a = (sensor: Idle()) ; x13d3z
            x13d1f1z || x13d1f2z = join_franka_carrier
            join_franka_carrier || x13d2z || x13d3z  = x13z
            # go to arm side
            x14a = sensor -> carrier: Red() ; x14b
            x14b = carrier -> arm: Ok() ; x14c
            x14c = carrier -> franka: Done() ; x14d
            x14d = { (fpx > -0.8) && (fpy > -0.2) && ( (fpy > 0.2) || (fpx < 0.14) ) } x14d1a ||    # arm + carrier
                   { (fpx > -0.8) && (fpy < -0.3) } x14d2a ||                                       # franka
                   { (fpx >  0.14) && (fpy > -0.2) && (fpy < 0.2) } x14d3a                          # sensor
            x14d1a = (arm: Rotate(0, 0, 0, rad(90), 0, rad(90)), carrier: MoveCart(0.6, 0, rad(-90), -0.4)); x14d1b
            x14d1b = carrier -> arm: Ok() ; x14d1c
            x14d1c = (arm: Rotate(rad(90), 0, rad(90), rad(90), 0, rad(150)), carrier: Idle()); x14d1d
            x14d1d = (arm: Grip(), carrier: Idle()) ; x14d1e
            x14d1e = (arm: Rotate(rad(90), 0, rad(150), rad(45), rad(-210), rad(150)), carrier: Idle()) ; x14d1f
            x14d1f = (arm: OpenGripper(), carrier: Idle()) ; x14d1g
            x14d1g = arm -> carrier: Ok() ; fork_arm_carrier
            fork_arm_carrier = { (fpx > -0.8) && (fpy > 0.2) && ( (fpy > 0.5) || (fpz > 0.17) ) } x14d1h1a ||             # arm
                               { (fpx > -0.8) && (fpx < 0.14) && (fpy < 0.5) && (fpy > -0.2) && (fpz < 0.17) } x14d1h2a   # carrier
            x14d1h1a = (arm: Rotate(rad(45), rad(-210), rad(150), 0, 0, 0)); x14d1h1b
            x14d1h1b = (arm: Idle()); x14d1h1z
            x14d1h2a = (carrier: MoveCart(0.6, 0.4, rad(-90), 0.4)); x14d1h2b
            x14d1h2b = (carrier: StrafeCart(0.6, 0.0, rad(-90), 0.1)); x14d1h2c
            x14d1h2c = (carrier: SetAngleCart(0)); x14d1h2d
            x14d1h2d = (carrier: MoveCart(0.5, 0.0, 0, -0.5)); x14d1h2z
            x14d2a = (franka: Idle()) ; x14d2z
            x14d3a = (sensor: Idle()) ; x14d3z
            x14d1h1z || x14d1h2z = join_arm_carrier
            join_arm_carrier || x14d2z || x14d3z  = x14z
            # merge franka + arm
            x13z + x14z = merge_sensor
            x10a = (producer: Idle()) ; x10z
            # join, notify the producer, and repeat
            merge_sensor || x10z = join_producer
            join_producer = carrier -> producer: Ok() ; x17
        in [
          (franka_a == 0.0) && (franka_b == 0.0) && (franka_c == 0.0) && (franka_d == 0.0) && (franka_e == 0.0) && (franka_f == 0.0) && (franka_g == 0.0) &&
          (carrier_theta == 0) && (carrier_x == 0) && (carrier_y == 0) &&
          (arm_a == 0) && (arm_b == 0) && (arm_c == 0)
        ] start
    '''

def code_arm():
    return '''
while (true) {
    receive(producer, idle) {
        case Ok() => {
            receive(carrier, idle) {
                case Ok() => {
                    rotate( -90, 0, 90 );
                    receive(carrier, idle) {
                        case Ok() => {
                            rotate( -90, 0, 150 );
                            grip();
                            rotate(-45, 210, 150);
                            openGripper();
                            send(carrier, Ok);
                            rotate(0, 0, 0);
                        }
                    }
                }
                case Done() => {
                    skip;
                }
            }
        }
        case Done() => {
            stop( );
            exit( 0 );
        }
    }
}
    '''

def code_franka():
    return '''
while (true) {
    receive(producer, idle) {
        case Ok() => {
            homePos( );
            receive(carrier, idle) {
                case Ok() => {
                    # grab position
                    setJoints( 0.178310,0.635300,-0.449920,-2.122150,2.866786,2.016097,1.141317 );
                    receive(carrier, idle) {
                        case Ok() => {
                             grasp(0.02);
                             setJoints( 0, 0, 0, 0, 0, 0, 0 );
                             send(carrier, Ok);
                             # drop position
                             setJoints( 0.926170,-1.693679,1.469714,-2.709620,1.511592,1.437029,0.573354 );
                             open( );
                             homePos( );
                        }
                    }
                }
                case Done() => {
                    skip;
                }
            }
        }
        case Done() => {
            stop( );
            exit( 0 );
        }
    }
}
    '''

def code_carrier():
    return '''
while (true) {
    receive(producer, idle) {
        case Ok() => {
            moveCart( 500 );
            setAngleCart( -90 );
            strafeCart( -100 );
            send(sensor,Ok);
            receive(sensor, idle) {
                case Green() => {
                    send(franka, Ok);
                    send(arm, Done);
                    moveCart(400);
                    send(franka, Ok);
                    receive(franka, idle) {
                        case Ok() => {
                            moveCart(-400);
                            strafeCart( 100 );
                            setAngleCart( 0 );
                            moveCart( -500 );
                        }
                    }
                }
                case Red() => {
                    send(arm, Ok);
                    send(franka, Done);
                    moveCart(-340);
                    send(arm, Ok);
                    receive(arm, idle) {
                        case Ok() => {
                            moveCart( 340 );
                            strafeCart( 100 );
                            setAngleCart( -0 );
                            moveCart( -500 );
                        }
                    }
                }
            }
            send(producer, Ok);
        }
        case Done() => {
            stop( );
            exit( 0 );
        }
    }
}
    '''

def code_sensor():
    return '''
sensor_dummy = 0;
while (true) {
    receive(producer, idle) {
        case Ok() => {
            receive(carrier, idle) {
                 case Ok() => {
                     if (sensor_dummy == 0) send(carrier, Green);
                     else send(carrier, Red);
                     sensor_dummy = 1;
                 }
            }
        }
        case Done() => {
            exit( 0 );
        }
    }
}
    '''

def code_producer():
    return '''
Wait(1);
while (true) {
    send(carrier, Ok);
    send(arm, Ok);
    send(franka, Ok);
    send(sensor, Ok);
    receive(carrier, idle) {
        case Ok() => skip;
    }
}
    '''



class XpSortingTest(XpTestHarness):

    def test_sorting(self):
        w = world()
        ch = choreo()
        contracts = []
        progs = { "arm": code_arm(),
                  "franka": code_franka(),
                  "carrier": code_carrier(),
                  "sensor": code_sensor(),
                  "producer": code_producer()}
        self.check(ch, w, contracts, progs)

if __name__ == '__main__':
    unittest.main()
