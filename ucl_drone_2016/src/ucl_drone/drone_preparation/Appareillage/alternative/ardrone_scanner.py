#!/usr/bin/env python

# AR.Drone Wireless Reconfiguration Script
# Written by Mike Clement, February 2014

import re
from sys import exit
from time import sleep
from subprocess import check_output, CalledProcessError

# START Configuration section
iface = "wlan1"

new_ssid = "myssid"

ssid_to_ip = { 'ardrone2_001270' : '192.168.1.6', \
               'ardrone2_008286' : '192.168.1.3', \
               'ardrone2_009941' : '192.168.1.5', \
               'ardrone2_008733' : '192.168.1.4' }

# END Configuration section

# Wrap check_output(), taking a *list* of words (command + args)
# and a boolean whether to interpret command using the shell
# (allows pipes, etc)
# Returns the result value and the text output
def callout_shell(cmdlist, use_shell):
    result = 0
    output = ''
    try:
        output = check_output(cmdlist, shell=use_shell)
    except CalledProcessError as err:
        result = err.returncode
    except Exception as ex:
        result = 1  # Catch-all for other exceptions
    return [result, output]

# Wrap callout_shell, defaulting to *not* use the shell
def callout(cmdlist):
    return callout_shell(cmdlist, False)

# Initialize some repeatedly-used variables
res = 0
out = ''

print "AR.Drone Wireless Reconfiguration Scanner"
print "  Scanner interface:\t" + iface
print "  Reconfigured SSID:\t" + new_ssid
print ""
print "Initializing interface..."

try:
    # Check if we're root
    res, out = callout('whoami')
    if (res != 0) or not re.search('root', out):
        raise Exception("Must be root")
    
    # Unblock (all) wireless devices
    res, out = callout(['rfkill', 'unblock', 'all'])
    if (res != 0):
        raise Exception("Could not unblock wireless interfaces")
    
    # Bring down wireless interface
    res, out = callout(['ifconfig', iface, 'down'])
    if (res != 0):
        raise Exception("Could not bring down interface")
    
    # Change to initial wireless settings
    res, out = callout(['iwconfig', iface, 'mode', 'ad-hoc', \
                       'essid', 'badssid', 'chan', 'auto', 'ap', 'any'])
    if (res != 0):
        raise Exception("Could not change wireless settings")
    
    # Bring up wireless interface with default IP
    res, out = callout(['ifconfig', iface, 'inet', \
                        '192.168.1.10/24', 'up'])
    if (res != 0):
        raise Exception("Could not bring up interface")
    
    # Sleep to let the interface 'settle'
    print "Letting interface settle..."
    sleep(5)
    
# On all initialization errors, just exit
except Exception as ex:
    print "Error: " + ex.args[0]
    exit(-1)

print "Configuration complete; starting scans."

# Continuously scan and reconfigure until user ends program
while True:
    # Count how many unconfigured drones were found this round
    found_drones = 0
    
    print "\nScanning for unconfigured AR.Drones..."
    
    # Perform the scan, sleep and retry if it fails
    res, out = callout(['iwlist', iface, 'scan'])
    if (res != 0):
        print "Error performing scan; will try again in 5 seconds..."
        sleep(5)
        continue
    
    # Split results by each wireless 'cell'
    cells = re.split('Cell', out)
    
    # Iterate through each cell, looking for AR.Drones
    for cell in cells:
        # Search cell report for ESSID field
        match_obj = re.search('ESSID:"(ardrone_\d+)"', cell)
        if not match_obj:
            continue
        ssid = match_obj.group(1)
        ap = 'any'
        match_obj = re.search('Address:\s(\w+:\w+:\w+:\w+:\w+:\w+)', cell)
        if match_obj:
          ap = match_obj.group(1)
        
        # Search cell report for Last beacon field
        match_obj = re.search('Last\sbeacon:\s(\d+)ms\sago', cell)
        age = -1
        if match_obj:
            age = int(match_obj.group(1))
        
        print 'Found AR.Drone "' + ssid + '" (' + ap + ')'
        
        # Check that we know about this drone and that it's
        # not a "ghost" (old) cell report
        if ssid not in ssid_to_ip:
            print '  SSID is not in map, ignoring.'
            continue
        elif age > 10000:
            print '  Report is %d ms old, ignoring.'%age
            continue
        else:
            print '  Attempting to reconfigure as %s...'%ssid_to_ip[ssid]
        
        # Reconfigure wireless device to talk to drone,
        # then connect and execute its onboard script
        try:
            # Bring down wireless interface
            res, out = callout(['ifconfig', iface, 'down'])
            if (res != 0):
                raise Exception("Could not bring down interface")
            
            # Change wireless settings
            res, out = callout(['iwconfig', iface, 'essid', ssid, \
                                'chan', 'auto', 'ap', ap])
            if (res != 0):
                raise Exception("Could not change wireless settings")
            
            # Bring up wireless interface
            res, out = callout(['ifconfig', iface, 'up'])
            if (res != 0):
                raise Exception("Could not bring up interface")
            
            # Let settle briefly
            sleep(10)
            
            # Build command to issue, using telnet protocol
            ipaddr = "192.168.0.103"
            cmd = 'echo "' \
                + 'killall udhcpd; ' \
                + 'ifconfig ath0 down; ' \
                + 'iwconfig ath0 mode managed channel auto essid ' \
                + new_ssid + '; ' \
                + 'ifconfig ath0 inet ' \
                + ssid_to_ip[ssid] \
                + ' netmask 255.255.255.0 up"' \
                + ' | nc -w 5 -q 0 192.168.1.1 23'
            
            # Issue command (TODO: better netcat error reporting)
            res, out = callout_shell(cmd, True)
            if (res != 0):
                raise Exception("Could not connect via telnet")
            
            print "  Successfully reconfigured"
            found_drones += 1
            
        except Exception as ex:
            print "  Error: " + ex.args[0]
            continue
    
    # Reset local SSID and re-settle
    res, out = callout(['iwconfig', iface, 'essid', 'badssid'])
    if (res != 0):
        print "Warning: couldn't reset local SSID"
    sleep(2)
    
    # If drones were found this time, let's rescan immediately
    # Otherwise, sleep for a bit
    if found_drones == 0:
        print "No configurable drones found this scan, " \
              "sleeping for 10 seconds..."
        sleep(10)





