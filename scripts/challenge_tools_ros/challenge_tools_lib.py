import csv
import sys

class ReactPose:
    def __init__(self, pos=[0., 0., 0.], quat=[0., 0., 0., 1.]):
        self.pos = pos
        self.quat = quat # order is xyzw

    def __repr__(self):
        return f"Pose(pos={self.pos}, quat={self.quat})"

class ReactTime:
    def __init__(self, sec: int, nsec: int):
        self.sec = sec
        self.nsec = nsec

    def __repr__(self):
        return f"Time(sec={self.sec}, nsec={self.nsec})"

    def __eq__(self, other):
        if isinstance(other, ReactTime):
            return (self.sec, self.nsec) == (other.sec, other.nsec)
        return False  # Not a ReactTime instance

    def __ge__(self, other):
        print("ReactTime ge TODO: properly debug/test this")
        t_this_long = self.sec*10e9 + self.nsec
        t_other_long = other.sec*10e9 + other.nsec
        return t_this_long >= t_other_long

    def __le__(self, other):
        print("ReactTime le TODO: properly debug/test this")
        t_this_long = self.sec*10e9 + self.nsec
        t_other_long = other.sec*10e9 + other.nsec
        return t_this_long <= t_other_long

    def __hash__(self):
        return hash((self.sec, self.nsec))


class ChallengeToolsLib:
    def __init__(self):
        x = 0

    def convert_line_to_pose(self, line):
        xyz = [0., 0., 0.]
        quat = [0., 0., 0., 0.]
        xyz[0], xyz[1], xyz[2] = line[3], line[4], line[5]
        # order is xyzw
        quat[0], quat[1], quat[2], quat[3] = line[6], line[7], line[8], line[9]

        pose = ReactPose()
        pose.pos = xyz
        pose.quat = quat
        return pose


    def load_csv(self, csv_filename):
        # parse the file in the closed_loop_trajectory.csv format
        with open(csv_filename, mode ='r')as file:
            csvFile = csv.reader(file)

            lines = []
            for line in csvFile:
                #print (line[0])
                if (line[0].startswith("#")):
                    xxxx = 0
                    #print ("hash")
                else:
                    #print(line)
                    this_line = []
                    for this_item in line[0:]:
                        this_line.append(float(this_item))

                    #print(type(line))
                    lines.append(this_line)

        return lines


    def load_csv_as_poses_list(self, csv_filename):

        lines = self.load_csv(csv_filename)
        poses = {}
        for line in lines:
            poses_as_list = self.convert_line_to_pose(line)
            this_time = ReactTime( int(line[1]) , int (line[2]))
            # counter is 0 and is lost
            poses[this_time] = poses_as_list
        return poses


    def publish_trajectory(self, pub, these_poses):
        if 'Path' not in sys.modules:
            from nav_msgs.msg import Path

        poses_ros = []
        for pose in these_poses:
            ps = self.pack_PoseStamped(pose[1])
            poses_ros.append(ps)

        msg = Path()
        msg.header.stamp.sec = 0 #time.sec
        msg.header.stamp.nanosec = 0# time.nsec
        msg.header.frame_id = self.fixed_frame
        msg.poses = poses_ros
        pub.publish(msg)


    def pack_PoseStamped(self, pose):
        if 'PoseStamped' not in sys.modules:
            from geometry_msgs.msg import PoseStamped

        #print (pose)
        xyz = pose.pos# [:3] # first 3 [of 7]
        quat = pose.quat# [3:] # last 4 [of 7]

        msg = PoseStamped()
        #msg.header = msg.header
        msg.pose.position.x = xyz[0]
        msg.pose.position.y = xyz[1]
        msg.pose.position.z = xyz[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        return msg


    def write_csv_poses(self, csv_filename, poses):
        poses_file = open(csv_filename, 'w')
        poses_file.write("# counter, sec, nsec, x, y, z, qx, qy, qz, qw\n")
        poses_file.flush()

        for index, pose_t in enumerate(poses):
            this_time = pose_t[0]
            pose = pose_t[1]
            line_out = str(index) + ", " + str(int(this_time.sec)) + ", " + str(int(this_time.nsec))
            line_out = line_out + ", " + str(pose.pos[0])
            line_out = line_out + ", " + str(pose.pos[1])
            line_out = line_out + ", " + str(pose.pos[2])
            line_out = line_out + ", " + str(pose.quat[0])
            line_out = line_out + ", " + str(pose.quat[1])
            line_out = line_out + ", " + str(pose.quat[2])
            line_out = line_out + ", " + str(pose.quat[3])
            line_out = line_out + "\n"
            poses_file.write(line_out)
            poses_file.flush()    