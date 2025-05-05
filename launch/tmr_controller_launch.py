from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    node1 = Node(package='tmr_controller',
                          executable='odometry',
                            name="odometry",
                          )
        
    node2 = Node(package='tmr_controller',
                       executable='path_generator',
                       name="path_generator",
                       )
    
    node3 = Node(package='tmr_controller',
                       executable='vision',
                       name="vision",
                       )
    
    node4 = Node(package='tmr_controller',
                        executable='controller',
                        name="controller",
                        )
    
    node5 = Node(package='tmr_controller',
                        executable='move',
                        name="move",
                        )
    node6 = Node(package='tmr_controller',
                        executable='mapping_points',
                        name="mapping_points",
                        )
    
    node7 = Node(package='tmr_controller',
                        executable='arm_routine',
                        name="arm_routine",
                        )
    
    
    
    l_d = LaunchDescription([node1, node2, node3, node4, node5, node6, node7])

    return l_d