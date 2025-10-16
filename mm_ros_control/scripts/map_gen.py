import rospy
from gazebo_msgs.srv import SpawnModel
import random
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import GetWorldProperties, DeleteModel

def get_model_list():
    rospy.wait_for_service('/gazebo/get_world_properties')
    try:
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        world_properties = get_world_properties()
        return world_properties.model_names
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return []

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_service(model_name)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return
    
def clear_all_models():
    model_list = get_model_list()  # 获取当前场景中的所有模型
    for model_name in model_list:
        delete_model(model_name)  # 删除每个模型

def generate_box_sdf(size=[1.0, 1.0, 1.0], pose=[0.0, 0.0, 0.0], rpy=[0.0, 0.0, 0.0], color="Gazebo/Red"):
    sdf_template = f"""
    <sdf version="1.6">
      <model name="box">
        <static>true</static>
        <link name="link">
          <pose>{pose[0]} {pose[1]} {pose[2]} {rpy[0]} {rpy[1]} {rpy[2]}</pose>
          <collision name="collision">
            <geometry>
              <box>
                <size>{size[0]} {size[1]} {size[2]}</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>{size[0]} {size[1]} {size[2]}</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>{color}</name>
              </script>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """
    return sdf_template

def check_valid_obs(obstacles, new_obs, min_dist=1.0):
    for obs in obstacles:
        dist = ((obs[0]-new_obs[0])**2 + (obs[1]-new_obs[1])**2)**0.5
        if dist < min_dist:
            return False
    return True

# rospy.wait_for_service('/gazebo/spawn_sdf_model')
clear_all_models()
spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
num_blocks =20
x_min, x_max = 0.0, 6.0
y_min, y_max = -2.0, 2.0
size = [0.3, 0.3, 0.3]
# for i in range(num_blocks):
#     print("Spawning block {}".format(i))
#     x = random.uniform(x_min, x_max)
#     y = random.uniform(y_min, y_max)
#     sdf = generate_box_sdf(size, color="Gazebo/White")
#     spawn_model(model_name="block{}".format(i), model_xml=sdf,
#                 robot_namespace="", initial_pose=Pose(position=Point(x, y, 0)), reference_frame="world")
obstacles_1 = []
obstacles_2 = []
for i in range(num_blocks):
    print("Spawning block {}".format(i))
    
    floating = random.choice([True, False])
    if i < num_blocks / 2:
        while True:
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            if check_valid_obs(obstacles_1, [x, y], min_dist=1.0):
                obstacles_1.append([x, y])
                break
        z = random.uniform(0.5, 1)
        size = [random.uniform(1, 1.5), 0.3, 0.3]
        rpy = [0, 0, random.uniform(0, 3.14 / 2)]
        color = "Gazebo/White"
    else:
        while True:
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            if check_valid_obs(obstacles_2, [x, y], min_dist=1.3):
                obstacles_2.append([x, y])
                break
        size = [random.uniform(0.2, 0.3), random.uniform(0.2, 0.3), random.uniform(0.5, 1.2)]
        z = size[2] / 2.0
        rpy = [0, 0, 0]
        color = "Gazebo/White"
    sdf = generate_box_sdf(size, pose=[x, y, z], rpy=rpy, color=color)
    spawn_model(model_name="block{}".format(i), model_xml=sdf,
                robot_namespace="", initial_pose=Pose(position=Point(0, 0, 0)), reference_frame="world")

