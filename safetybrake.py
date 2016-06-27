"""
To prevent the drone from hitting objects, this program takes an input for distance to an object in front of the drone, and prevents the drone from hitting object by aggressively braking, and then maintaining said distance at desired value.
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time


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

    
#------------Function declarations---------------------------    
def getdistance():
    d = 2900
    return d
    

#---------------Connect to the Vehicle------------------------
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

#-----------------------------------------------------------------

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
            time.sleep(1)
            
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
            time.sleep(5)



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