#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import LoadController
from controller_manager_msgs.srv import UnloadController

pos = 0

def joy_listener():

    # start node
    rospy.init_node("joy_controller", anonymous=True)

    # subscribe to joystick messages on topic "joy"
    rospy.Subscriber("joy", Joy, joy_callback, queue_size=1)

    # keep node alive until stopped
    rospy.spin()


# called when joy cmd_joy message is received
def joy_callback(data):
    global pos
    new_position = Float64()
    position = rospy.get_param("position")
    velocity = rospy.get_param("velocity")
    button_A = data.buttons[0]
    button_back = data.buttons[6]
     

    if (position):  
        # start publishers of /markhor/flipper_xx_position_controller/command
        flipper_fl_pub = rospy.Publisher("/markhor/flipper_fl_position_controller/command", 
                                    Float64, queue_size=1)
        flipper_fr_pub = rospy.Publisher("/markhor/flipper_fr_position_controller/command", 
                                    Float64, queue_size=1)   
        flipper_rl_pub = rospy.Publisher("/markhor/flipper_rl_position_controller/command", 
                                    Float64, queue_size=1)   
        flipper_rr_pub = rospy.Publisher("/markhor/flipper_rr_position_controller/command", 
                                    Float64, queue_size=1)     

        pos += ( data.axes[1] / 300 )                     
        new_position.data = pos

        if button_A:
            flipper_fl_pub.publish(new_position)
            flipper_fr_pub.publish(new_position)
            flipper_rl_pub.publish(new_position)
            flipper_rr_pub.publish(new_position)
        
    # elif we use velocity controller
    if (velocity and button_A):
        # start publishers of /markhor/flipper_xx_velocity_controller/command
        flipper_fl_pub = rospy.Publisher("/markhor/flipper_fl_velocity_controller/command", 
                                    Float64, queue_size=1)
        flipper_fr_pub = rospy.Publisher("/markhor/flipper_fr_velocity_controller/command", 
                                    Float64, queue_size=1)   
        flipper_rl_pub = rospy.Publisher("/markhor/flipper_rl_velocity_controller/command", 
                                    Float64, queue_size=1)   
        flipper_rr_pub = rospy.Publisher("/markhor/flipper_rr_position_controller/command", 
                                    Float64, queue_size=1)   
       
        new_position.data = data.axes[1]

        flipper_fl_pub.publish(new_position)
        flipper_fr_pub.publish(new_position)
        flipper_rl_pub.publish(new_position)
        flipper_rr_pub.publish(new_position)

    rospy.loginfo("new_position = %f", new_position.data)    
    

    # Swapping controller
    if (button_back):
        if (position):
            rospy.wait_for_service('/controller_manager/switch_controller')
            rospy.wait_for_service('/controller_manager/unload_controller')
            rospy.wait_for_service('/controller_manager/load_controller')

            try:
                swap = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
                unload = rospy.ServiceProxy('/controller_manager/unload_controller',LoadController)
                load = rospy.ServiceProxy('/controller_manager/load_controller', UnloadController)

                rospy.set_param("position", 0)
                rospy.set_param("velocity", 1)

                if (swap([],
                         ['/markhor/flipper_fl_position_controller, \
                           /markhor/flipper_fr_position_controller, \
                           /markhor/flipper_rl_position_controller, \
                           /markhor/flipper_rr_position_controller'],1,False,0) ):
                    rospy.loginfo("Stopped position controller!") 
                 
                  
                if (unload('/markhor/flipper_fl_position_controller') &
                    unload('/markhor/flipper_fr_position_controller') &
                    unload('/markhor/flipper_rl_position_controller') &
                    unload('/markhor/flipper_rr_position_controller') ):
                    rospy.loginfo("Unloaded position controller!") 

                if (load('/markhor/flipper_fl_velocity_controller') &
                    load('/markhor/flipper_fr_velocity_controller') &
                    load('/markhor/flipper_rl_velocity_controller') &
                    load('/markhor/flipper_rr_velocity_controller') ):   
                    rospy.loginfo("Load velocity controller!")       

                if (swap(['/markhor/flipper_fl_velocity_controller, \
                           /markhor/flipper_fr_velocity_controller, \
                           /markhor/flipper_rl_velocity_controller, \
                           /markhor/flipper_rr_velocity_controller'],
                           [],1,False,0) ):   
                    rospy.loginfo("Started velocity controller!")   

                rospy.sleep(rospy.Duration(2))             

            except rospy.ServiceException as e:  
                print("Service call failled: %s", e)
                

        if (velocity):
            rospy.wait_for_service('/controller_manager/switch_controller')
            rospy.wait_for_service('/controller_manager/unload_controller')
            rospy.wait_for_service('/controller_manager/load_controller')

            try:
                swap = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
                unload = rospy.ServiceProxy('/controller_manager/unload_controller',LoadController)
                load = rospy.ServiceProxy('/controller_manager/load_controller', UnloadController)

                rospy.set_param("position", 1)
                rospy.set_param("velocity", 0)

                if (swap([],
                         ['/markhor/flipper_fl_velocity_controller, \
                           /markhor/flipper_fr_velocity_controller, \
                           /markhor/flipper_rl_velocity_controller, \
                           /markhor/flipper_rr_velocity_controller'],1,False,0) ):
                   rospy.loginfo("Stopped velocity controller!") 
                  
                if (unload('/markhor/flipper_fl_velocity_controller') &
                    unload('/markhor/flipper_fr_velocity_controller') &
                    unload('/markhor/flipper_rl_velocity_controller') &
                    unload('/markhor/flipper_rr_velocity_controller') ):
                    rospy.loginfo("Unloaded velocity controller!") 

                if (load('/markhor/flipper_fl_position_controller') &
                    load('/markhor/flipper_fr_position_controller') &
                    load('/markhor/flipper_rl_position_controller') &
                    load('/markhor/flipper_rr_position_controller') ):   
                    rospy.loginfo("Load position controller!") 

                if(swap(['/markhor/flipper_fl_position_controller, \
                          /markhor/flipper_fr_position_controller, \
                          /markhor/flipper_rl_position_controller, \
                          /markhor/flipper_rr_position_controller'],
                          [],1,False,0) ):  
                    rospy.loginfo("Started position controller!")     

                rospy.sleep(rospy.Duration(2)) 

            except rospy.ServiceException as e:  
                print("Service call failled: %s", e)        



if __name__ == '__main__':
    try:                               
        joy_listener() 
                              
    except rospy.ROSInterruptException:
        pass


# TODO --> move by velocity then press a button to take the avg position
# TODO --> homing
# TODO --> 3 mode ( 4 , 2 ,1)
# TODO --> preset 
# TODO --> pwm en percent value (255 = 100)
# TODO --> ratio de read ( reduc 40x et tick par tour ~1350)
# TODO --> 



# Done
# --> Build up pos then press a to accept
# --> Remove joy_twist to get full freedom
# --> Can swap controller with setparam and load/unload 
