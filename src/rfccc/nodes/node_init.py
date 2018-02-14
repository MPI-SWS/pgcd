import roslaunch
import rospy

package = 'rfccc'
broadcaster = 'dynamic_component.py'
broadcaster_fix = 'fixed_component.py'
listener = 'listener.py'
node1 = roslaunch.core.Node(package, broadcaster, name="turtle1", respawn=False, output="screen")
node2 = roslaunch.core.Node(package, broadcaster_fix, name="arm_fix", respawn=False, output="screen")
node5 = roslaunch.core.Node(package, broadcaster_fix, name="tool_fix", respawn=False, output="screen")


launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process1 = launch.launch(node1)
process2 = launch.launch(node2)
process5 = launch.launch(node5)

input()
