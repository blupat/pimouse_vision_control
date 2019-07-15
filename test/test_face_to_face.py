#!/usr/bin/env python
import unittest, rostest
import rosnode, rospy, sys, time

class VisionTest(unittest.TestCase):
    def get_file_freq(self, lr):
        with open('/dev/rtmotor_raw_' + lr + '0', 'r') as f:
            s = f.readline().rstrip()
            if s != "":
                return int(s)
            else:
                return 0
    
    def test_put_freq(self):
        count, l_turn_count, r_turn_count = 0, 0, 0
        start = rospy.Time.now().to_sec()
        
        prev_lhz, prev_rhz = 0, 0
        rate = rospy.Rate(10)
        while rospy.Time.now().to_sec() - start < 10.0:
            lhz = self.get_file_freq('l')
            rhz = self.get_file_freq('r')
            
            if (lhz == prev_lhz) and (rhz == prev_rhz):
                continue
            
            if lhz > rhz:
                r_turn_count += 1
            if lhz < rhz:
                l_turn_count += 1
            
            prev_lhz, prev_rhz = lhz, rhz
            
            count += 1
            rate.sleep()
        
        r_turn_rate = 1.0 * r_turn_count / count
        l_turn_rate = 1.0 * l_turn_count / count
        
        turn = None
        if (r_turn_rate > 0.6) and (l_turn_rate < 0.2):
            turn = 'right'
        if (r_turn_rate < 0.2) and (l_turn_rate > 0.6):
            turn = 'left'
        
        self.assertFalse(turn is None, "direction is not fixed. L:" + str(l_turn_rate) + " R:" + str(r_turn_rate))
        self.assertEqual(turn, sys.argv[1], "wrong direction. L:" + str(l_turn_rate) + " R:" + str(r_turn_rate))
        
if __name__ == '__main__':
    rospy.init_node('test_face_to_face.py')
    rostest.rosrun('pimouse_vision_control', 'test_face_to_face', VisionTest)
