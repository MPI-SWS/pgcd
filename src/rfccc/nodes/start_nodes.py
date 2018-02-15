import roslaunch

package = 'rfccc'
broadcaster = 'component.py'
broadcaster_fix = 'fixed_component.py'
node1 = roslaunch.core.Node(package, broadcaster, name="turtle1", respawn=False, output="screen")
node2 = roslaunch.core.Node(package, broadcaster_fix, name="arm_fix", respawn=False, output="screen")
node3 = roslaunch.core.Node(package, broadcaster_fix, name="tool_fix", respawn=False, output="screen")


launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

launch.launch(node1)
launch.launch(node2)
launch.launch(node3)

input()
