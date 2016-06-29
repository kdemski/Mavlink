"""
To prevent the drone from hitting objects, this program takes an input for distance to an object in front of the drone, and prevents the drone from hitting object by aggressively braking, and then maintaining said distance at desired value.
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, serial

"""
#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Example showing how to set and clear vehicle channel-override information.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
"""
    
#------------Function declarations---------------------------    
def getdistance():
    d = 2900
    return d
    

#---------------Connect to the Vehicle------------------------
"""
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)
"""


connection_string = 'com4'
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, baud = 57600)



#---------------------Get State--------------------------------------------

vehicle.wait_ready('autopilot_version')
# Get all vehicle attributes (state)
print "\nGet all vehicle attribute values:"
print " Autopilot Firmware version: %s" % vehicle.version
print "   Major version number: %s" % vehicle.version.major
print "   Minor version number: %s" % vehicle.version.minor
print "   Patch version number: %s" % vehicle.version.patch
print "   Release type: %s" % vehicle.version.release_type()
print "   Release version: %s" % vehicle.version.release_version()
print "   Stable release?: %s" % vehicle.version.is_stable()
print " Autopilot capabilities"
print "   Supports MISSION_FLOAT message type: %s" % vehicle.capabilities.mission_float
print "   Supports PARAM_FLOAT message type: %s" % vehicle.capabilities.param_float
print "   Supports MISSION_INT message type: %s" % vehicle.capabilities.mission_int
print "   Supports COMMAND_INT message type: %s" % vehicle.capabilities.command_int
print "   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union
print "   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp
print "   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target
print "   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned
print "   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int
print "   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain
print "   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target
print "   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination
print "   Supports mission_float message type: %s" % vehicle.capabilities.mission_float
print "   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration
print " Global Location: %s" % vehicle.location.global_frame
print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
print " Local Location: %s" % vehicle.location.local_frame
print " Attitude: %s" % vehicle.attitude
print " Velocity: %s" % vehicle.velocity
print " GPS: %s" % vehicle.gps_0
print " Gimbal status: %s" % vehicle.gimbal
print " Battery: %s" % vehicle.battery
print " EKF OK?: %s" % vehicle.ekf_ok
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Rangefinder: %s" % vehicle.rangefinder
print " Rangefinder distance: %s" % vehicle.rangefinder.distance
print " Rangefinder voltage: %s" % vehicle.rangefinder.voltage
print " Heading: %s" % vehicle.heading
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Groundspeed: %s" % vehicle.groundspeed    # settable
print " Airspeed: %s" % vehicle.airspeed    # settable
print " Mode: %s" % vehicle.mode.name    # settable
print " Armed: %s" % vehicle.armed    # settable











#--------------------------------------------------------------
# Get all original channel values (before override)
print "Channel values from RC Tx:", vehicle.channels

# Access channels individually
print "Read channels individually:"
print " Ch1: %s" % vehicle.channels['1']
print " Ch2: %s" % vehicle.channels['2']
print " Ch3: %s" % vehicle.channels['3']
print " Ch4: %s" % vehicle.channels['4']
print " Ch5: %s" % vehicle.channels['5']
print " Ch6: %s" % vehicle.channels['6']
print " Ch7: %s" % vehicle.channels['7']
print " Ch8: %s" % vehicle.channels['8']
print "Number of channels: %s" % len(vehicle.channels)




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
print("Beggining Test Suite")
#print(type(kdchannel))
#print(type(vehicle.channels['7']))
#print kdchannel

vehicle.channels.overrides['8'] = ON
print " CH8 override: %s" % vehicle.channels.overrides['8']


vehicle.channels.overrides['7'] =  1500
print " CH7 override: %s" % vehicle.channels.overrides['7']



print("Ending Test Suite")
#wait = input("PRESS ENTER TO CONTINUE.")
#----------------------Braking----------------------------------------
while True:
    print "Clear all overrides"
    vehicle.channels.overrides = {}
    print " Channel overrides: %s" % vehicle.channels.overrides
    
    while ON > MP: #Braking Function Enabled #switch to channels not overrides for actual usage!
        distance = getdistance()
        keepdistance = kdm * MP #switch to channels not overrides for actual usage!
        if distance < keepdistance+BT: # if distance is below threshold and braking is enabled, then execute braking function
            print "Engage Braking"
            
            thrust = distance - keepdistance 
            pitch = MP + thrust
            if pitch > pitchuppercap:
                pitch = pitchuppercap
            if pitch < pitchlowercap:
                pitch = pitchlowercap
            vehicle.channels.overrides['2'] = pitch # Override Pitch channel
            
            print "READOUT"
            print "distance: %s" % distance
            print "keepdistance: %s" % keepdistance 
            print "thrust: %s" % thrust
            print " Ch2 override: %s" % vehicle.channels.overrides['2']
            time.sleep(1)



print "Clear all overrides"
vehicle.channels.overrides = {}
print " Channel overrides: %s" % vehicle.channels.overrides 

#Close vehicle object before exiting script
print "\nClose vehicle object"
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")

wait = input("PRESS ENTER TO CONTINUE.")