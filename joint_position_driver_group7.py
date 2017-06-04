#!/usr/bin/env python

# Copyright (c) 2013-2017, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SDK Joint Position Example: keyboard
"""
import argparse
import math
import rospy
import redis
import intera_interface
from intera_interface import Lights
import intera_external_devices

from intera_interface import CHECK_VERSION

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

def blink(light_name, head_display):
    l = Lights()
    for i in range(0, 5):
    	head_display.display_image('hit1.jpg', False, 1000000)
    	rospy.sleep(.05)
    	head_display.display_image('hit2.jpg', False, 1000000)
    	rospy.sleep(.05)

def map_keyboard(side, redisInstance):

	headObj = intera_interface.Head()
        head_display = intera_interface.HeadDisplay()
        min_q = [-3.0503, -3.8095, -3.0426, -3.0439, -2.9761, -2.9761, -4.7124]
	max_q = [3.0503, 2.2736, 3.0426, 3.0439, 2.9761, 2.9761, 4.7124]
	home_q = [-0.11243359375, -1.15601367188, -0.00805859375, 2.16251171875, 0.0014658203125, 0.47171875, 3.31875488281]
	float_q = home_q
	limb = intera_interface.Limb(side)
	joints = limb.joint_names()

	def set_j(limb, joints, q):
		joint_command = {joints[0]: q[0], joints[1]: q[1], joints[2]: q[2], joints[3]: q[3], joints[4]: q[4], joints[5]: q[5], joints[6]: 1.74236968882}
		limb.set_joint_positions(joint_command)

	done = False
	print("Controlling joints. Press ? for help, Esc to quit.")
	while not done and not rospy.is_shutdown():
		
		# Read q from REDIS
		flag = False
		target_state = redisInstance.get('target_key')
		if target_state == '1':
			blink('head_blue_light', head_display)
                target_position = redisInstance.get('cs225a::robot::sawyer::tasks::tg_pos')
		redis_angles = redisInstance.get('cs225a::robot::sawyer::sensors::q')
   		q_pub = " ".join(str(limb.joint_angle(joints[i])) for i in range(0, 7))
		redisInstance.set('cs225a::robot::sawyer::tasks::lag_q', q_pub)
		if target_position is not None:
			x = target_position.split(" ")
			angle_head = math.atan2(float(x[1]), float(x[0]))
			headObj.set_pan(angle_head, speed=1, timeout=0)
			zprime = float(x[2]) - 0.4
			angle_vertical = angle([float(x[0]), float(x[1]), 0], [float(x[0]), float(x[1]), zprime])
			angle_vertical = math.copysign(angle_vertical, zprime)
			pic_index = int(round(angle_vertical*10))
			if pic_index > 5: 
				pic_index = 5
			elif pic_index < -5:
				pic_index = -5 
			if target_state == '0':
                        	head_display.display_image('sawyer_eyes/'+str(pic_index)+'.jpg', False, 1000000)

		if redis_angles is not None: # Check if read from redis correctly
			q = redis_angles.split(" ")
			if len(q) is 7: # Check if 7 forces read
				for i in range(0, 7):
					angle_float = float(q[i])
					if math.isnan(angle_float) or angle_float < min_q[i] or angle_float > max_q[i]: 
						flag = True
					else:
						float_q[i] = angle_float
                                
			else :
				flag = True
		else:
			flag = True
		if flag:
			q = home_q
        
		# Write q to Robot
        	set_j(limb, joints, float_q)
		# print q

        	# Check for escape characters
        	c = intera_external_devices.getch()

        	if c:
            		#catch Esc or ctrl-c
            		if c in ['\x1b', '\x03']:
                		done = True
                		rospy.signal_shutdown("Example finished.")

def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """

    redisInstance = redis.StrictRedis(host='localhost', port=6379, db=0)
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("sdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    map_keyboard(args.limb, redisInstance)
    print("Done.")


if __name__ == '__main__':
    main()
