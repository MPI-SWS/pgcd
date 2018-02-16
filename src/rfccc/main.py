import roslaunch

package = 'rfccc'
broadcaster_turt = 'turtle_comp.py'
broadcaster_comp = 'component.py'
node1 = roslaunch.core.Node(package, broadcaster_turt, name="turtle1", respawn=False, output="screen")
node2 = roslaunch.core.Node(package, broadcaster_comp, name="arm_fix", respawn=False, output="screen")
node3 = roslaunch.core.Node(package, broadcaster_comp, name="tool_fix", respawn=False, output="screen")


launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

launch.launch(node1)
launch.launch(node2)
launch.launch(node3)

input()
