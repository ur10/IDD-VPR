from scipy.spatial.transform import Rotation as R
import numpy as np
import rospy
import rospy
import tf

r = R.from_matrix([[-0.975156,  -0.208783,  0.0740251],
				  [-0.0902202,  0.0691309, -0.99352],
				  [  0.202313, -0.975516,  -0.0862499]])



if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br1 = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    quat = r.as_quat()
    while not rospy.is_shutdown():
        br1.sendTransform((0.0, 0.0, 0.0),
                         (quat[0], quat[1], quat[2], quat[3]),
                         rospy.Time.now(),
                         "os_sensor",
                         "map")
        print('publishing transform')
        rate.sleep()