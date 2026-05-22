import serial
import rospy
from pneumaticbox_traj_playback.msg import goals

# Initialize serial port
# Ensure the port name '/dev/cu.usbmodem11301' is correct for your Arduino
try:
    arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)
except serial.SerialException as e:
    print('Fatal Error: Could not open serial port 1')
    # If rospy is available, log as fatal, otherwise just print and exit.
    try:
        rospy.logfatal("Fatal Error: Could not open serial port 2")
    except NameError: # rospy might not be imported if this fails very early
        pass
    exit() # Exit if the serial port cannot be opened
def pressure_goal_callback(msg):
    """
    Callback for /pressure_goal topic.
    Extracts 'grip' and writes it to serial.
    """
    try:
        grip_value = msg.grip # Assuming the message has a 'grip' attribute
        
        if arduino.is_open:
            arduino.write(grip_value.encode())
            rospy.loginfo('Sent grip value: %s to serial port.',grip_value)
        else:
            rospy.logwarn('Serial port is not open. Cannot write.')
            
    except AttributeError:
        rospy.logerr('Error: Message on /pressure_goal does not have grip attribute.')
    except serial.SerialException as e:
        rospy.logerr('Serial write error')
    except Exception as e:
        print(e)
        rospy.logerr('Unexpected error in callback')

if __name__ == "__main__":
    try:
        rospy.init_node('serial_grip_writer', anonymous=True)
        rospy.loginfo('Serial grip writer node started.')

        if not arduino.is_open:
            rospy.logwarn('Serial port {arduino.port} was not open. Attempting to reopen.')
            try:
                arduino.open()
                rospy.loginfo('Serial port {arduino.port} reopened successfully.')
            except serial.SerialException as e:
                rospy.logfatal('Failed to reopen serial port {arduino.port}. Shutting down.')
                exit()
        else:
            rospy.loginfo('Using serial port: {arduino.port}')

        # Subscribe to the /pressure_goal topic
        rospy.Subscriber('/pressure_goal', goals, pressure_goal_callback)
        rospy.loginfo('Subscribed to /pressure_goal topic. Listening for messages...')

        rospy.spin()  # Keep the node running

    except rospy.ROSInterruptException:
        rospy.loginfo('ROS node shutdown requested.')
    except Exception as e:
        rospy.logfatal('An unhandled exception occurred')
    finally:
        if 'arduino' in globals() and arduino.is_open:
            arduino.close()
            rospy.loginfo('Serial port closed.')
        else:
            rospy.loginfo('Serial port was already closed or not initialized.')
