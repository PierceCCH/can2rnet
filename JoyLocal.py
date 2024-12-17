#!/python3
# joystick based on: https://www.kernel.org/doc/Documentation/input/joystick-api.txt

#Requires: socketCan, can0 interface

# This file is part of can2RNET.
#
# can2RNET is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# can2RNET is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

import sys, os, array, threading
from time import *
from fcntl import ioctl
from can2RNET import *

debug = True

class Gamepad:
    """
    Translates USB gamepad joystick X/Y-position to R-Net compatible format
    """

    # TODO: Modify to fit current gamepad

    axis_map = []
    button_map = []
    xthreshold = 8 * 0x10000 / 128
    ythreshold = 8 * 0x10000 / 128

    joystick_x = 0
    joystick_y = 0

    axis_states = {}
    button_states = {}

    # These constants were borrowed from linux/input.h
    axis_names = {
        0x00 : 'x',
        0x01 : 'y',
        0x02 : 'z',
        0x03 : 'rx',
        0x04 : 'ry',
        0x05 : 'rz',
        0x06 : 'trottle',
        0x07 : 'rudder',
        0x08 : 'wheel',
        0x09 : 'gas',
        0x0a : 'brake',
        0x10 : 'hat0x',
        0x11 : 'hat0y',
        0x12 : 'hat1x',
        0x13 : 'hat1y',
        0x14 : 'hat2x',
        0x15 : 'hat2y',
        0x16 : 'hat3x',
        0x17 : 'hat3y',
        0x18 : 'pressure',
        0x19 : 'distance',
        0x1a : 'tilt_x',
        0x1b : 'tilt_y',
        0x1c : 'tool_width',
        0x20 : 'volume',
        0x28 : 'misc',
    }

    button_names = {
        0x120 : 'trigger',
        0x121 : 'thumb',
        0x122 : 'thumb2',
        0x123 : 'top',
        0x124 : 'top2',
        0x125 : 'pinkie',
        0x126 : 'base',
        0x127 : 'base2',
        0x128 : 'base3',
        0x129 : 'base4',
        0x12a : 'base5',
        0x12b : 'base6',
        0x12f : 'dead',
        0x130 : 'a',
        0x131 : 'b',
        0x132 : 'c',
        0x133 : 'x',
        0x134 : 'y',
        0x135 : 'z',
        0x136 : 'tl',
        0x137 : 'tr',
        0x138 : 'tl2',
        0x139 : 'tr2',
        0x13a : 'select',
        0x13b : 'start',
        0x13c : 'mode',
        0x13d : 'thumbl',
        0x13e : 'thumbr',

        0x220 : 'dpad_up',
        0x221 : 'dpad_down',
        0x222 : 'dpad_left',
        0x223 : 'dpad_right',

        # XBox 360 controller uses these codes.
        0x2c0 : 'dpad_left',
        0x2c1 : 'dpad_right',
        0x2c2 : 'dpad_up',
        0x2c3 : 'dpad_down',
    }

    def init_joystick(self):
        """
        Initialize the joystick device and return the file descriptor for the device.
        Out of all joystick devices found, the first one is selected.

        :return: file descriptor for the joystick device
        """
        js_list = []
        print('---- Available gamepad devices ----')
        for fn in os.listdir('/dev/input'):
            if fn.startswith('js'):
                js_list.append(fn)
                print(f'\n/dev/input/{fn}')

        try:
            fn = js_list[0]
            print('Opening %s...' % fn)
            jsdev = open(fn, 'rb')
        except IOError:
            print ('No joystick at ' + fn)
            return ('')

        # Get device name
        buf = bytearray([0] * 64)
        ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        print(f'Device name: {buf}')

        # Get number of axes
        buf = array.array('B', [0] )
        ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
        num_axes = buf[0]

        # Get number of buttons
        buf = array.array('B', [0] )
        ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Get axis map
        buf = array.array('B', [0] * 0x40)
        ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

        # Get button map
        buf = array.array('H', [0] * 200)
        ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

        # Build axis map
        for axis in buf[:num_axes]:
            axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0

        # Build button map
        for btn in buf[:num_buttons]:
            btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0

        print (f'{num_axes} Axes found: {',' .join(self.axis_map)}')
        print (f'{num_buttons} buttons found: {', '.join(self.button_map)}')

        return (jsdev)

    def usb_joystick_read_thread(self, jsdev):
        """
        Read USB joystick input and update joystick_x and joystick_y values.

        :param jsdev: file descriptor for the joystick device

        :return: None
        """
        global joystick_x
        global joystick_y
        global rnet_threads_running

        while rnet_threads_running:
            try:
                evbuf = jsdev.read(8)
                _, jvalue, jtype, jnumber = struct.unpack('IhBB', evbuf)

                if jtype & 0x02:
                    axis = self.axis_map[jnumber]
                    if (axis == 'x'):
                        if abs(jvalue) > self.xthreshold:
                            joystick_x = 0x100 + int(jvalue * 100 / 128) >> 8 &0xFF
                        else:
                            joystick_x = 0
                    elif (axis == 'y'):
                        if abs(jvalue) > self.ythreshold:
                            joystick_y = 0x100 - int(jvalue * 100 / 128) >> 8 &0xFF
                        else:
                            joystick_y = 0

            except:
                print("Error reading gamepad")
                joystick_x = 0
                joystick_y = 0
                rnet_threads_running = False


def dec2hex(dec,hexlen):
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
    return ('0'*hexlen+h)[l:l+hexlen]


def RNET_JSMerror_exploit(cansocket, joy_id):
    """
    Send JSM error exploit CAN message to disable on-board joystick.

    :param cansocket: socket for sending CAN messages

    :returns: Joystick ID, used as a reference for sending joystick CAN messages
    """
    print("Waiting for JSM heartbeat...")
    canwait(cansocket,"03C30F0F:1FFFFFFF")

    t=time()+0.20

    print("Waiting for joy frame...")
    joy_id = wait_rnet_joystick_frame(cansocket,t)
    print("Using joy frame: "+joy_id)

    for _ in range(0,3):
        cansend(cansocket,'0c000000#')

    return(joy_id)

# TODO: Replace this with ROS publisher
def send_joystick_canframe(can_socket, joy_id):
    """
    Publishes joystick CAN frame every 10ms. Runs on a separate thread.

    :param can_socket: socket for sending CAN messages
    :param joy_id: Joystick ID, used as a reference for sending joystick CAN messages

    :return: None
    """
    mintime = 0.01
    nexttime = time() + mintime
    
    while rnet_threads_running:
        joyframe = joy_id + '#' + dec2hex(joystick_x, 2) + dec2hex(joystick_y, 2)
        cansend(can_socket,joyframe)

        nexttime += mintime
        t = time()

        if t < nexttime:
            sleep(nexttime - t)
        else:
            nexttime += mintime

# TODO: Replace this with ROS publisher
def inject_rnet_joystick_frame(can_socket, joy_id):
    """
    Waits for joyframe and injects another spoofed frame ASAP. Runs on a separate thread.

    :param can_socket: socket for sending and receiving CAN messages
    :param joy_id: Joystick ID, used as a reference for sending joystick CAN messages

    :return: None
    """
    rnet_joystick_frame_raw = build_frame(joy_id + "#0000")
    
    while rnet_threads_running:
        cf, _ = can_socket.recvfrom(16)
        if cf == rnet_joystick_frame_raw:
            cansend(can_socket, joy_id + '#' + dec2hex(joystick_x, 2) + dec2hex(joystick_y, 2))


def wait_rnet_joystick_frame(can_socket, start_time):
    """
    Waits for a frame send by joystick. Extracts the frame ID and returns it.
    If no frame is found within the timeout, 'Err!' is returned.

    :param can_socket: socket for sending and receiving CAN messages
    :param start_time: start time for the wait

    :return: Joystick frame extendedID or 'Err!'
    """
    frameid = ''

    while frameid[0:3] != '020':  # joystick frame ID (no extended frame)
        cf, _ = can_socket.recvfrom(16) # Blocking if no CANBUS traffic
        candump_frame = dissect_frame(cf)
        frameid = candump_frame.split('#')[0]
        if time() > start_time:
            print("Joystick frame wait timed out...")
            return('Err!')
        
    return(frameid)


#Set speed_range: 0% - 100%
def RNETsetSpeedRange(cansocket,speed_range):
    if speed_range>=0 and speed_range<=0x64:
        cansend(cansocket,'0a040100#'+dec2hex(speed_range,2))
    else:
        print('Invalid RNET SpeedRange: ' + str(speed_range))


def RNETshortBeep(cansocket):
    """
    Plays a short beep sound on the wheelchair.

    :param cansocket: socket for sending CAN messages

    :return: None
    """
    cansend(cansocket,"181c0100#0260000000000000")


def RNETplaysong(cansocket):
    """
    Plays a song on the wheelchair. 

    :param cansocket: socket for sending CAN messages

    :return: None
    """
    cansend(cansocket,"181C0100#2056080010560858")
    sleep(.77)
    cansend(cansocket,"181C0100#105a205b00000000")


# TODO: Possibly remove when ROS publisher is implemented
def watch_and_wait():
    """
    Monitors joystick position and active threads. Prints joystick position and active threads every 0.5s.

    :return: None
    """
    started_time = time()
    while threading.active_count() > 0 and rnet_threads_running:
        sleep(0.5)
        print(str(round(time()-started_time,2))+'\tX: '+dec2hex(joystick_x,2)+'\tY: '+dec2hex(joystick_y,2)+ '\tThreads: '+str(threading.active_count()))


def kill_rnet_threads():
    """
    Sets global variable rnet_threads_running to False, stopping all threads.

    :return: None
    """
    global rnet_threads_running
    rnet_threads_running = False


def check_usb_gamepad_center():
    """
    Checks if the joystick is centered. If not, waits until the joystick is centered.

    :return: None
    """
    print('waiting for joystick to be centered')
    while (joystick_x !=0 or joystick_y !=0):
        print('joystick not centered')


def selectControlExploit(can_socket):
    user_selection = int(input("Select exploit to use: \n \n 1. Disable R-Net Joystick temporarily. (Lower latency) \n 2. Allow R-Net Joystick (Latency))\n"))
    
    start_time = time() + .20 # TODO: Many parts of the code use this. Replace with variable local to function
    print('Waiting for RNET-Joystick frame...')

    # Wait for RNET-Joystick frame to be sent by the joystick
    rnet_joystick_id = wait_rnet_joystick_frame(can_socket, start_time)
    if rnet_joystick_id == 'Err!':
        print('No RNET-Joystick frame seen within minimum time... Aborting...')
        return
    
    print('Found RNET-Joystick frame: ' + rnet_joystick_id)

    if user_selection == 1:
        # Send JSM error exploit to disable on-board joystick
        print("\nYou chose to disable the R-Net Joystick temporary. Restart the chair to enable it again.")
        rnet_joystick_id = RNET_JSMerror_exploit(can_socket, rnet_joystick_id)
    elif user_selection == 2:
        print("\nYou chose to allow the R-Net Joystick. The joystick will be used to control the chair.")

    # Initialize chair to lowest speed setting
    chair_speed_range = 00
    RNETsetSpeedRange(can_socket, chair_speed_range)

    sendjoyframethread = threading.Thread(
        target=send_joystick_canframe,
        args=(can_socket,rnet_joystick_id),
        daemon=True
    )
    sendjoyframethread.start()


if __name__ == "__main__":
    global rnet_threads_running
    global joystick_x
    global joystick_y
    rnet_threads_running = True
    can_socket = opencansocket(0)

    #init usb joystick
    x360 = Gamepad()
    usb_joystick_dev = x360.init_joystick()
    joystick_x = 0
    joystick_y = 0

    if usb_joystick_dev != '':
        print('Using USB joystick @ ' + str(usb_joystick_dev).split("'")[1])
        usb_joystick_read_thread = threading.Thread(
            target=x360.usb_joystick_read_thread,
            args=(usb_joystick_dev,),
            daemon=True)
        usb_joystick_read_thread.start()

        check_usb_gamepad_center()
        selectControlExploit(can_socket)

        sleep(0.5)
        watch_and_wait()
        kill_rnet_threads()
    else:
        print('No Joystick found.')
        kill_rnet_threads()

    print("Exiting")
