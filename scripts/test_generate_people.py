#!/usr/bin/python

import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, Vector3, Point32
import math
from people_msgs.msg import PositionMeasurementArray, Person, People
from easy_markers.generator import MarkerGenerator, Marker
from sensor_msgs.msg import PointCloud
from tf import TransformListener

def add(v1, v2):
    return Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)

class PeopleGenerator(object):
    def __init__(self):
        rospy.init_node('test_generate_people')

        self.gen = MarkerGenerator()
        self.gen.type = Marker.ARROW
        self.gen.ns = 'velocities'
        self.gen.lifetime = .5

        self.xs = [-30, -27, -24, -21, -18, -15, -12]
        self.ymean = -5.14
        self.ywidth = 7
        self.thetas = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]

        self.theta_vel = 0.001
        self.timestep = 0.01

        self.mpub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.ppub = rospy.Publisher('/people', People, queue_size=10)
        self.cpub = rospy.Publisher('/people_cloud', PointCloud, queue_size=10)
        self.tl = TransformListener(10)

        while not rospy.is_shutdown():
            rospy.sleep(self.timestep)
            self.update()

    def update(self):
        old_ys = [self.ymean + self.ywidth*math.sin(theta) for theta in self.thetas]
        self.thetas = [theta + self.theta_vel for theta in self.thetas]
        ys = [self.ymean + self.ywidth*math.sin(theta) for theta in self.thetas]
        vels = [(y2-y1)/self.timestep for (y1, y2) in zip(old_ys, ys)]

        pc = PointCloud()
        people = People()
        people.header = std_msgs.msg.Header()
        people.header.stamp = rospy.Time.now()
        people.header.frame_id = "/map"
        pc.header = std_msgs.msg.Header()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = "/map"
        pc.header.seq = 1
        for (x, y, vel) in zip(self.xs, ys, vels):
            p = Person()
            p.name = "Nils"
            p.position = Point(x, y, 0)
            p.velocity = Point(0, vel, 0)
            p.reliability = 1
            people.people.append(p)

            self.gen.scale = [.1, .3, 0]
            self.gen.color = [1, 1, 1, 1]
            pos = Vector3(x, y, 0)
            m = self.gen.marker(points=[pos, add(pos, Vector3(0, vel, 0))])
            m.header = std_msgs.msg.Header()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "/map"
            self.mpub.publish(m)

            p32 = Point32(x, y, 0.365)
            pc.points.append(p32)

        #now = rospy.Time.now();
        found = True
        try:
            #found = True
            self.tl.waitForTransform("base_laser_link", "map", pc.header.stamp, rospy.Duration(0.01))
            #stamp = self.tl.getLatestCommonTime("base_laser_link", "map")
            pc = self.tl.transformPointCloud("/base_laser_link", pc)
        except:
            found = False
            print "Failed to find transform"

        self.ppub.publish(people)
        if found:
            #for i in range(0, 60):
            #    f = (i - 30.0)/10.0;
            #    pc.points.append(Point32(-3.0, f, -0.365))
            #    pc.points.append(Point32(3.0, f, -0.365))
            #    pc.points.append(Point32(f, -3.0, -0.365))
            #    pc.points.append(Point32(f, 3.0, -0.365))
            pc.points = [p for p in pc.points if p.x > 0]
            print len(pc.points)
            self.cpub.publish(pc)

if __name__ == '__main__':
    server = PeopleGenerator()
