import numpy as np
import rospy
import matplotlib.pyplot as plt

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid

from legibot.static_map import obstacles


obstacles_np = np.array(obstacles)
x0, y0 = np.min(obstacles_np[:, 0]), np.min(obstacles_np[:, 1])
x1, y1 = np.max(obstacles_np[:, 0]), np.max(obstacles_np[:, 1])
grid_size = (x1 - x0, y1 - y0)
grid_resolution = 0.2

grid = np.zeros((int(grid_size[0] / grid_resolution),
                 int(grid_size[1] / grid_resolution)), dtype=np.int8)

transform = lambda xy: ((xy[0] - x0) / grid_resolution, (xy[1] - y0) / grid_resolution)


for obstacle in obstacles:
    xc, yc, r = obstacle
    xc, yc = transform((xc, yc))
    for x_ in range(round(xc - r / grid_resolution), round(xc + r / grid_resolution)):
        for y_ in range(round(yc - r / grid_resolution), round(yc + r / grid_resolution)):
            if 0 <= x_ < grid.shape[0] and 0 <= y_ < grid.shape[1] and np.linalg.norm((x_ - xc, y_ - yc)) -1 <= r / grid_resolution:
                grid[x_, y_] = 100

initial_pos = (-3.0, 3.0)
goal_pos = (-3.7, -8.5)

plt.imshow(grid, cmap='gray')
plt.show()

ros_node = rospy.init_node('A_star', anonymous=True)
rate = rospy.Rate(10)  # 10hz

pub_initial_pos = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
pub_goal_pos = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
pub_map = rospy.Publisher('/map', OccupancyGrid, queue_size=10)


grid_msg = OccupancyGrid()
grid_msg.header.frame_id = "map"
grid_msg.info.resolution = grid_resolution
grid_msg.info.width = grid.shape[0]
grid_msg.info.height = grid.shape[1]

grid_msg.info.origin.position.x = x0
grid_msg.info.origin.position.y = y0
grid_msg.info.origin.position.z = 0

grid_msg.info.origin.orientation.x = 0
grid_msg.info.origin.orientation.y = 0
grid_msg.info.origin.orientation.z = 0
grid_msg.info.origin.orientation.w = 1

grid_msg.data = grid.T.flatten().astype(np.int8)

initial_pos_msg = PoseWithCovarianceStamped()
initial_pos_msg.header.frame_id = "map"
initial_pos_msg.pose.pose.position = Point(initial_pos[0], initial_pos[1], 0)

goal_pos_msg = PoseStamped()
goal_pos_msg.header.frame_id = "map"
goal_pos_msg.pose.position = Point(goal_pos[0], goal_pos[1], 0)


while not rospy.is_shutdown():
    rospy.set_param('/astar/InflateRadius', 0.5)
    pub_map.publish(grid_msg)
    pub_initial_pos.publish(initial_pos_msg)
    pub_goal_pos.publish(goal_pos_msg)
    rate.sleep()
    print("Published")
