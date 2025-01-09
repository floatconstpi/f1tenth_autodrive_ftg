import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class ftg(Node):
    def __init__(self):
        # Create node and init pubs/subs
        super().__init__('ftgnode')
        self.get_logger().info('Awaiting subscriptions, ensure bridge is running')
        # AutoDrive lidar
        self.lidarsub = self.create_subscription(
            LaserScan,
            '/autodrive/f1tenth_1/lidar',
            self.lidarcallback,
            10)
        # AutoDrive steering and throttle - if IRL use Ackermann accordingly
        self.steerpub = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.throttlepub = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.lidarsub  

    def lidarcallback(self, msg):
        # Get raw Lidar data off topic
        ranges = msg.ranges
        # Go thru neigbor avging 
        procranges = self.preprocesslidar(ranges)
        # Find the largest possible gap & its midpoint
        start, end = self.findmaxgap(procranges)
        midgap = self.findmidpointofgap(start, end)
        # Calculate output throttle/steering
        steer = self.calculatesteeringangle(midgap, len(ranges))
        throttle = self.calculatethrottle(steer, procranges)
        # Publish steering/throttle accordingly
        self.publishsteering(steer)
        self.publishthrottle(throttle)

    def preprocesslidar(self, ranges):
        # Each range is the avg of the range and the one to left and right
        prange = ranges[:]
        for i in range(1, len(ranges) - 1):
            # Avging here
            prange[i] = (ranges[i-1] + ranges[i] + ranges[i+1]) / 3
        return prange

    def findmaxgap(self, ranges):
        # Finding largest gap as it's most probably whats needed
        maxgap = 0
        startgap = 0
        endgap = 0
        curr = 0
        for i in range(len(ranges)):
            if ranges[i] > 1.23:  # If more than 1.23m, it's a gap
                curr += 1
                if curr > maxgap:
                    # Move the gap indices if new one
                    maxgap = curr
                    endgap = i
                    startgap = i - maxgap + 1
            else:
                # Oops, not a gap
                curr = 0
        # Return start/end of the largest gap
        return startgap, endgap

    def findmidpointofgap(self, start, end):
        # Midpoint of gap withstart/end index
        midpoint = (start + end) // 2
        return midpoint

    def calculatesteeringangle(self, bestpoint, totalpoints):
        # Calculates angle value from [0,1] for steering and return it
        angleincrement = (3.14159) / totalpoints
        angle = (bestpoint - totalpoints / 2) * angleincrement
        # Gap for steering +- based on topic
        maxsteeringangle = 1.0 
        # Error avoidance
        angle = max(min(angle, maxsteeringangle), -maxsteeringangle)
        # Debug!
        self.get_logger().info(f'Steering out: {angle}')
        return angle

    def calculatethrottle(self, steeringangle, ranges):
        # Calculating throttle based on data and steer angle
        targetpercentage = 0.05 # the center 5% of the lidar i.e. what it sees in front
        centermid = len(ranges) // 2 # middle of index
        centeravg = sum(ranges[int(centermid - len(ranges) * targetpercentage / 2): int(centermid + len(ranges) * targetpercentage / 2)]) / len(ranges[int(centermid - len(ranges) * targetpercentage / 2): int(centermid + len(ranges) * targetpercentage / 2)])
        mindistance = min(ranges) # lowest distance found
        # if the car straight and no obstacles, base throttle multi is 0.5
        if centeravg > 7.0 and abs(steeringangle) < 0.1:
            # For cheeky infs
            if centeravg > 10:
                centeravg = 10.0
            # Calculating based on center avg
            basethrottle= 0.5 * (centeravg + 0.01) / 10.0
        else:
            # Probably not straight, change reactively based on distance
            basethrottle =  0.15 if mindistance > 0.6 else 0.3 if mindistance > 0.6 else 0.2 if mindistance > 0.4 else 0.15 if mindistance > 0.12 else 0.04 if mindistance > 0.09 else 0.01
        # Reduce throttle based on the steering angle and a decay 
        decay = 0.9
        throttle = decay * basethrottle * (1 - abs(steeringangle) + 0.01 / 1.0)
        # Debug!
        self.get_logger().info(f'Throttle out: {throttle}')
        return throttle

    def publishsteering(self, angle):
        # Sending the steering angle to the actuators
        msg = Float32()
        msg.data = angle
        self.steerpub.publish(msg)
        self.get_logger().info(f'Steering successfully published: {angle}')

    def publishthrottle(self, value):
        # Sending the throttle value to the actuators
        msg = Float32()
        msg.data = value
        self.throttlepub.publish(msg)
        self.get_logger().info(f'Throttle successfully published: {value}')

    def destroynode(self):    
        super().destroy_node()

def main(args=None):
    # Init the node and spin
    rclpy.init(args=args)
    ftgnode = ftg()
    try:
        # Start!
        rclpy.spin(ftgnode)
    except KeyboardInterrupt:
        # Quit and attempt to send zeroing info
        ftgnode.get_logger().info('Keyboard interrupt')
        # Preparing 0 msg
        msg = Float32()
        msg.data = 0.0
        # Send!
        ftgnode.throttlepub.publish(msg)
        ftgnode.steerpub.publish(msg)
    finally:
        # In all cases, quit it once not needed
        ftgnode.destroynode()
        rclpy.shutdown()

if __name__ == '__main__':
    # Start the main function
    main()
