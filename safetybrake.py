"""
To prevent the drone from hitting objects, this program takes an input for distance to an object in front of the drone, and prevents the drone from hitting object by aggressively braking, and then maintaining said distance at desired value.
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

from ctypes import *

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Example showing how to set and clear vehicle channel-override information.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

#----named pipe variables
PIPE_ACCESS_DUPLEX = 0x3
PIPE_TYPE_MESSAGE = 0x4
PIPE_READMODE_MESSAGE = 0x2
PIPE_WAIT = 0
PIPE_UNLIMITED_INSTANCES = 255
BUFSIZE = 4096
NMPWAIT_USE_DEFAULT_WAIT = 0
INVALID_HANDLE_VALUE = -1
ERROR_PIPE_CONNECTED = 535

MESSAGE = "Yes it is\0"
szPipename = "\\\\.\\pipe\\mynamedpipe"
#--------------------------

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

    
#------------Function declarations---------------------------    
def getdistance():
    d = 2900
    return d

#---named pipe reading thread
def ReadWrite_ClientPipe_Thread(hPipe):
    chBuf = create_unicode_buffer(BUFSIZE)
    cbRead = c_ulong(0)
    while 1:
        fSuccess = windll.kernel32.ReadFile(hPipe, chBuf, BUFSIZE,
        byref(cbRead), None)
        if ((fSuccess ==1) or (cbRead.value != 0)):
            print chBuf.value
            cbWritten = c_ulong(0)
            fSuccess = windll.kernel32.WriteFile(hPipe, c_char_p(MESSAGE), len(MESSAGE), byref(cbWritten), None)
        else:
            break
        if ( (not fSuccess) or (len(MESSAGE) != cbWritten.value)):
            print "Could not reply to the client's request from the pipe"
            break
        else:
            print "Number of bytes written:", cbWritten.value

    windll.kernel32.FlushFileBuffers(hPipe)
    windll.kernel32.DisconnectNamedPipe(hPipe)
    windll.kernel32.CloseHandle(hPipe)
    return 0

#---------------Connect to the Vehicle------------------------
#print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

#-----------------------------------------------------------------

# Get all original channel values (before override)
#print "Channel values from RC Tx:", vehicle.channels

# Access channels individually
#print "Read channels individually:"
#print " Ch1: %s" % vehicle.channels['1']
#print " Ch2: %s" % vehicle.channels['2']
#print " Ch3: %s" % vehicle.channels['3']
#print " Ch4: %s" % vehicle.channels['4']
#print " Ch5: %s" % vehicle.channels['5']
#print " Ch6: %s" % vehicle.channels['6']
#print " Ch7: %s" % vehicle.channels['7']
#print " Ch8: %s" % vehicle.channels['8']
#print "Number of channels: %s" % len(vehicle.channels)




#---------------Variable Declaration-----------------------------------
"""
check rc10 switch if braking is enabled
check distance if lower than keepdistance, initiate braking 
check rc11 knob for changes to keepdistance
initiate PID loop to maintain keepdistance
"""

"""with caps at \pm 100 from midpoint, and thrust changing to reach those limits depending on a 0.1m space shift, these numbers should hold a tight distance from object of interest"""         

ON = 2000
OFF = 1000
MP = 1500  #rc midpoint this might be inutile once pid is locked in
kdchannel = 7 #keep distance channel (knob)
bechannel = 8 #braking enable channel (switch)
kdm = 2
pitchuppercap = 1600
pitchlowercap = 1400
BT = 2000 # distance above keepdistanec at which point braking is activated

#distances in mm
distance = 0  #piped from c++ app
keepdistance = 0 #set later to knob
thrust = 0 #convention '-' is in backwards direction



#----------------Test Suite-------------------------------
#print("Beggining Test Suite")
#print(type(kdchannel))
#print(type(vehicle.channels['7']))
#print kdchannel

vehicle.channels.overrides['8'] = ON
#print " CH8 override: %s" % vehicle.channels.overrides['8']


vehicle.channels.overrides['7'] =  1500
#print " CH7 override: %s" % vehicle.channels.overrides['7']



#print("Ending Test Suite")
#wait = input("PRESS ENTER TO CONTINUE.")
#----------------------Braking----------------------------------------
def main():
    THREADFUNC = CFUNCTYPE(c_int, c_int)
    thread_func = THREADFUNC(ReadWrite_ClientPipe_Thread)

    while True:
       # print "Clear all overrides"
        vehicle.channels.overrides = {}
        #print " Channel overrides: %s" % vehicle.channels.overrides

        while ON > MP: #Braking Function Enabled #switch to channels not overrides for actual usage!

            hPipe = windll.kernel32.CreateNamedPipeA(szPipename, PIPE_ACCESS_DUPLEX,
                                                     PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,
                                                     PIPE_UNLIMITED_INSTANCES, BUFSIZE, BUFSIZE,
                                                     NMPWAIT_USE_DEFAULT_WAIT, None)
            if (hPipe == INVALID_HANDLE_VALUE):
                print "Error in creating Named Pipe"
                return 0

            fConnected = windll.kernel32.ConnectNamedPipe(hPipe, None)
            if ((fConnected == 0) and (windll.kernel32.GetLastError() == ERROR_PIPE_CONNECTED)):
                fConnected = 1
            if (fConnected == 1):
                dwThreadId = c_ulong(0)
                hThread = windll.kernel32.CreateThread(None, 0, thread_func, hPipe, 0, byref(dwThreadId))
                if (hThread == -1):
                    print "Create Thread failed"
                    return 0
                else:
                    windll.kernel32.CloseHandle(hThread)
            else:
                print "Could not connect to the Named Pipe"
                windll.kernel32.CloseHandle(hPipe)


            distance = getdistance()
            keepdistance = kdm * MP #switch to channels not overrides for actual usage!
            if distance < keepdistance+BT: # if distance is below threshold and braking is enabled, then execute braking function
                #print "Engage Braking"
                time.sleep(1)

                thrust = distance - keepdistance
                pitch = MP + thrust
                if pitch > pitchuppercap:
                    pitch = pitchuppercap
                if pitch < pitchlowercap:
                    pitch = pitchlowercap
                vehicle.channels.overrides['2'] = pitch # Override Pitch channel

               # print "READOUT"
               # print "distance: %s" % distance
               # print "keepdistance: %s" % keepdistance
               # print "thrust: %s" % thrust
               # print " Ch2 override: %s" % vehicle.channels.overrides['2']
                time.sleep(5)



    #print "Clear all overrides"
    vehicle.channels.overrides = {}
    #print " Channel overrides: %s" % vehicle.channels.overrides

    #Close vehicle object before exiting script
    #print "\nClose vehicle object"
    vehicle.close()

    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()

    #print("Completed")

    wait = input("PRESS ENTER TO CONTINUE.")
    return 0

if __name__ == "__main__":
    main()