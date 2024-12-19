import rospy
import threading
import sys

from sensor_msgs.msg import Joy
from can2RNET import *
from time import sleep, time

# Global variables for joystick state
joystick_X = 0
joystick_Y = 0
rnet_threads_running = True


def signal_handler(sig, frame):
    """
    Signal handler to stop the wheelchair controller.

    :param sig: signal number
    :param frame: current stack frame
    """
    print("Interrupt signal received. Stopping wheelchair controller...")
    global rnet_threads_running
    rnet_threads_running = False
    rospy.signal_shutdown("Node closed...")
    sys.exit(0)


def dec_to_hex(dec, hexlen):
    """
    Convert dec to hex with leading 0s and no '0x' prefix.

    :param dec: decimal number to convert
    :param hexlen: length of the hex string

    :returns: hex string
    """
    h=hex(int(dec))[2:]
    l=len(h)
    if h[l-1]=="L":
        l-=1  #strip the 'L' that python int sticks on
    if h[l-2]=="x":
        h= '0'+hex(int(dec))[1:]
    return ('0' * hexlen + h)[l:l + hexlen]


def joy_callback(msg):
    """
    Callback for the /joy_input topic to update joystick_X and joystick_Y.

    :param msg: Joy message containing joystick axes
    """
    global joystick_X, joystick_Y
    try:
        joystick_X = msg.axes[0]
        joystick_Y = msg.axes[1]
    except IndexError:
        rospy.logerr("Joystick axes index out of range")


def send_joystick_canframe(can_socket, joy_id):
    """
    Publishes joystick CAN frame every 10ms.

    :param can_socket: socket for sending CAN messages
    :param joy_id: Joystick ID, used as a reference for sending joystick CAN messages
    """
    mintime = 0.01
    nexttime = time() + mintime

    while rnet_threads_running and not rospy.is_shutdown():
        joyframe = joy_id + '#' + dec_to_hex(joystick_X, 2) + dec_to_hex(joystick_Y, 2)
        print("Sending joystick_X: ", joystick_X, " joystick_Y: ", joystick_Y)
        cansend(can_socket, joyframe)

        nexttime += mintime
        t = time()
        if t < nexttime:
            sleep(nexttime - t)
        else:
            nexttime += mintime


def disable_rnet_joystick(can_socket):
    """
    Send JSM error exploit CAN message to disable on-board joystick.

    :param can_socket: socket for sending CAN messages

    :returns: True if successful, False otherwise
    """
    print("Waiting for JSM heartbeat...")
    canwait(can_socket,"03C30F0F:1FFFFFFF")

    try:
        for _ in range(0,3):
            cansend(can_socket,'0c000000#')
        return True
    except:
        print("Error sending JSM exploit message.")
        return False


def wait_rnet_joystick_frame(can_socket):
    """
    Waits for a frame send by joystick. Extracts the frame ID and returns it.
    If no frame is found within the timeout, 'Err!' is returned.

    :param can_socket: socket for sending and receiving CAN messages

    :return: Joystick frame extendedID
    :throws: TimeoutError
    """
    frameid = ''
    timeout = time() + 0.2 # wait 200ms until timeout

    while frameid[0:3] != '020':  # joystick frame ID (no extended frame)
        cf, _ = can_socket.recvfrom(16) # Blocking if no CANBUS traffic
        candump_frame = dissect_frame(cf)
        frameid = candump_frame.split('#')[0]
        
        if time() > timeout:
            print("Joystick frame wait timed out...")
            return TimeoutError
        
    return frameid


def joystick_spoofing(can_socket):
    """
    Handles R-Net joystick spoofing.

    :param can_socket: socket for sending CAN messages
    """
    global rnet_threads_running

    rospy.loginfo("Waiting for R-net joystick frame...")

    try:
        joystick_ID = wait_rnet_joystick_frame(can_socket)
        rospy.loginfo(f"Found R-net joystick frame: {0}")
        if not disable_rnet_joystick(can_socket):
            rospy.logerr("Failed to disable R-net joystick. Aborting...")
            return

        # Start thread to send joystick CAN frames
        spoof_thread = threading.Thread(
            target=send_joystick_canframe, 
            args=(can_socket, joystick_ID), 
            daemon=True
        )
        spoof_thread.start()

        rospy.loginfo("Joystick spoofing started.")

    except TimeoutError:
        rospy.logerr("No R-net joystick frame seen within timeout. Aborting...")


def play_beep(cansocket):
    """
    Plays a short beep sound on the wheelchair.

    :param cansocket: socket for sending CAN messages

    :return: None
    """
    cansend(cansocket,"181c0100#0260000000000000")


def play_song(cansocket):
    """
    Plays a song on the wheelchair. 

    :param cansocket: socket for sending CAN messages

    :return: None
    """
    cansend(cansocket,"181C0100#2056080010560858")
    sleep(.77)
    cansend(cansocket,"181C0100#105a205b00000000")


def main():
    global rnet_threads_running

    # Initialize ROS node
    rospy.init_node("wheelchair_controller")
    rospy.Subscriber("/joy_input", Joy, joy_callback)

    can_socket = opencansocket(0)

    try:
        joystick_spoofing(can_socket)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down node...")
    finally:
        rnet_threads_running = False
        can_socket.close()
        rospy.loginfo("CAN socket closed.")


if __name__ == "__main__":
    main()
