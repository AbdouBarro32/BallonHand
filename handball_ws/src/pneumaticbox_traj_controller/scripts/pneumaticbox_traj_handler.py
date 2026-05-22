#!/usr/bin/env python

import rospy
import time
import signal
import sys
from collections import deque
from pneumaticbox import utils, api
from std_msgs.msg import Float64MultiArray
from pneumaticbox_traj_playback.msg import goals

class PneumaticBoxHandler:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pneumaticbox_handler', anonymous=True)

        # Connect to the pneumatic box
        self.airserver = utils.connectToDefaultPneumaticbox(receiveInBackground=True)

        # Setup controllers
        self.setup_controllers()

        # Initialize instance variables
        self.delay = 0.01
        self.current_pressures = [0.0] * 6  # Store current pressures for 6 channels
        self.pressure_targets = [0.0] * 6
        
        # Safety limits
        self.max_pressure = 150.0  # Maximum allowed pressure in KPa
        self.safety_pressure_limit = 140.0  # Safety limit to stop inflation
        
        # Add command tracking variables
        self.command_in_progress = False
        self.command_lock_time = 0
        self.command_timeout = 3.0  # 3 seconds timeout for operations (increased for bang-bang)
        
        # Set test configuration
        self.test_mode = False       # Testing with just one channel
        self.test_channel = 2       # Index for channel 3 (0-indexed)
        
        # Command queue
        self.command_queue = deque()
        self.queue_timer = rospy.Timer(rospy.Duration(0.1), self.process_command_queue)

        self.signals_topics = (
            (api.SIGNAL_PRESSURE_0, "pressure_0"),
            (api.SIGNAL_PRESSURE_1, "pressure_1"),
            (api.SIGNAL_PRESSURE_2, "pressure_2"),
            (api.SIGNAL_PRESSURE_3, "pressure_3"),
            (api.SIGNAL_PRESSURE_4, "pressure_4"),
            (api.SIGNAL_PRESSURE_5, "pressure_5")
            
       )

        self.subscribe_to_sensors()
        self.sensor_timer = rospy.Timer(rospy.Duration(0.1),self.read_pressure_sensors)
        # Subscribe to the pressure_goal topic
        self.sub = rospy.Subscriber("pressure_goal", goals, self.goal_callback)

        # Setup shutdown handling
        rospy.on_shutdown(self.shutdown_handler)

        rospy.loginfo("PneumaticBox Handler initialized")


    def setup_controllers(self):
        """Setup threshold controllers for the pneumatic box"""
        rospy.loginfo("Configuring and activating Threshold controllers in default configuration.")
        msgs_config = []
        for i in range(8):
            msgs_config.append(api.MsgConfigurationControllerThreshold(i=i))
            msgs_config.append(api.MsgControllerActivate(i))
        self.airserver.submit(msgs_config)
        time.sleep(0.05)

    def subscribe_to_sensors(self):
        msgs = [api.MsgSubscribe(signal_topic[0], 0.1) for signal_topic in self.signals_topics]
        print(msgs)
        self.airserver.submit(msgs)
        rospy.loginfo("Subscribed to all signals")

    def read_pressure_sensors(self, event=None):
        self.airserver.service_incoming_messages()

        for i in range(6):
            try:
                data = self.airserver._serversignals[self.signals_topics[i][0]]
                if data and len(data[1]) >0:
                    self.current_pressures[i]=data[1][-1]
                    #rospy.loginfo("Channel %s pressure:%s",i,self.current_pressures[i])
            except (KeyError,AttributeError) as e:
                rospy.loginfo("No pressure data available for %s ",i+1)

    def goal_callback(self, msg):
        """Handle incoming pressure goals - add to queue"""
        pressure_goals = msg.pressure_goal
        
        # When in test mode, only accept commands for the test channel
        # if self.test_mode and channel_id != self.test_channel:
        #     rospy.logwarn("In test mode: Only channel %s (ID: %s) is active. Ignoring command for channel %s", 
        #                  self.test_channel + 1, self.test_channel, channel_id)
        #     return
        for channel_id in range(6):
            pressure_goal=pressure_goals[channel_id]
            self.command_queue.append((channel_id,pressure_goal))
        
        #OLD LOGIC##############################
        # if channel_id >= 0 and channel_id < 6:
        #     # Add command to the queue
        #     self.command_queue.append((channel_id, pressure_goal))
        #     rospy.loginfo("Command queued for channel %s, target pressure: %s, queue size: %s", 
        #                  channel_id, pressure_goal, len(self.command_queue))
        # else:
        #     rospy.logerr("Invalid channel ID:%s", channel_id)

    def calculate_inflation_duration(self, target_pressure, current_pressure):
        """Calculate duration needed for inflation based on experimental data."""
        pressure_diff = target_pressure - current_pressure
        # Experimental: Safer rate based on faster balloon (0.02s for 64 KPa) -> 0.02/64 = 0.0003125 s/KPa
        calculated_duration = 0.0003125 * pressure_diff 
        return max(0.001, min(0.5, calculated_duration))

    def calculate_deflation_duration(self, current_pressure, target_pressure):
        """Calculate duration needed for deflation based on experimental data."""
        pressure_diff = current_pressure - target_pressure
        # Experimental: 0.008s for 5 KPa -> 0.008/5 = 0.0016 s/KPa
        calculated_duration = 0.0016 * pressure_diff
        return max(0.001, min(1.0, calculated_duration))

    def inflate(self, duration, inflate_all, number, delay=0.0):
        """Generate inflation messages"""
        if inflate_all:
            rospy.loginfo("Inflating all with duration: %s",duration)
            for i in range(6):
                yield api.MsgSignalEvent(api.BLOCK_SIGNALS_CLIENT + i, 1.0, delay)
                yield api.MsgSignalEvent(api.BLOCK_SIGNALS_CLIENT + i, 0.0, delay + duration[i])
        else:
            rospy.loginfo("Inflating %s with duration: %s",number,duration)
            yield api.MsgSignalEvent(api.BLOCK_SIGNALS_CLIENT + number , 1.0, delay)
            yield api.MsgSignalEvent(api.BLOCK_SIGNALS_CLIENT + number , 0.0, delay + duration)

    def deflate(self, duration, deflate_all, number, delay=0.0):
        """Generate deflation messages"""
        if deflate_all:
            rospy.loginfo("Deflating all with duration: %s",duration)
            for i in range(6):
                yield api.MsgSignalEvent(api.BLOCK_SIGNALS_CLIENT + i, -1.0, delay)
                yield api.MsgSignalEvent(api.BLOCK_SIGNALS_CLIENT + i, 0.0, delay + duration)
        else:
            rospy.loginfo("Deflating %s with duration: %s",number,duration)
            yield api.MsgSignalEvent(api.BLOCK_SIGNALS_CLIENT + number , -1.0, delay)
            yield api.MsgSignalEvent(api.BLOCK_SIGNALS_CLIENT + number , 0.0, delay + duration)


    def shutdown_handler(self):
        """Handle shutdown properly"""
        rospy.loginfo("Shutting down, deflating all channels")
        it_msgs = list(self.deflate(3.0, True, 0, self.delay))
        self.airserver.submit(it_msgs)

    def run(self):
        """Main loop"""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Regular updates if needed
            #self.publish_pressures()
            rate.sleep()

    def release_command_lock(self, event=None):
        """Release the command lock after operation completes"""
        self.command_in_progress = False
        queue_size = len(self.command_queue)
        rospy.loginfo("Command completed, %s commands remaining in queue", queue_size)
        
        # Immediately process next command if available
        if queue_size > 0:
            # Use a very short timer to avoid recursion but process quickly
            rospy.Timer(rospy.Duration(0.01), self.process_command_queue, oneshot=True)

    def process_command_queue(self, event=None):
        """Process the next command in the queue if not busy"""
        if self.command_in_progress:
            # Check if we've exceeded the timeout
            current_time = time.time()
            if current_time - self.command_lock_time > self.command_timeout:
                rospy.logwarn("Command timeout exceeded, proceeding with next command")
                self.command_in_progress = False
            else:
                # Still busy, will try again later
                return
        
        # If we have commands in the queue and we're not busy, process the next command
        if self.command_queue and not self.command_in_progress:
            channel_id, pressure_goal = self.command_queue.popleft()
            self.execute_pressure_command(channel_id, pressure_goal)

    def execute_pressure_command(self, channel_id, pressure_goal):
        """Execute a pressure command for a specific channel using bang-bang control"""
        eps = 15  # Larger epsilon for bang-bang control to avoid oscillation
        current_pressure = self.current_pressures[channel_id]
        self.pressure_targets[channel_id] = pressure_goal
        
        rospy.loginfo("Executing command - channel: %s, current pressure: %s, goal pressure: %s", 
                     channel_id, current_pressure, pressure_goal)
        
        # Set command in progress
        self.command_in_progress = True
        self.command_lock_time = time.time()

        # If we're already within epsilon of the goal, we're done
        if abs(current_pressure - pressure_goal) <= eps:
            rospy.loginfo("Channel %s pressure (%s) already at target (%s) within epsilon (%s). No action needed.",
                          channel_id + 1, current_pressure, pressure_goal, eps)
            self.release_command_lock()
            return

        # Start the bang-bang control process
        self.execute_bang_bang_control(channel_id, pressure_goal, eps)

    def execute_bang_bang_control(self, channel_id, pressure_goal, eps):
        """Execute bang-bang control for a channel"""
        # Minimum pulse duration for inflation and deflation
        min_inflate_duration = 0.02  # Minimum time to start inflation
        min_deflate_duration = 0.01  # Minimum time to start deflation
        
        # Force an update of the pressure reading
        success = self.force_update_pressure_reading(channel_id)
        if not success:
            rospy.logwarn("Bang-bang: Could not get pressure reading for channel %s, aborting for safety", 
                         channel_id )
            self.release_command_lock()
            return
            
        # Get current pressure
        current_pressure = self.current_pressures[channel_id]
        
        # Safety check - if pressure exceeds safety limit, abort
        if current_pressure > self.safety_pressure_limit:
            rospy.logerr("Bang-bang: SAFETY LIMIT EXCEEDED on channel %s (%s KPa). Aborting and deflating.", 
                       channel_id , current_pressure)
            # Emergency deflate
            it_msgs = list(self.deflate(1, False, channel_id , self.delay))
            self.airserver.submit(it_msgs)
            self.release_command_lock()
            return
            
        # Determine if we need to inflate or deflate
        if pressure_goal > current_pressure + eps and current_pressure < self.safety_pressure_limit:
            # Need to inflate
            rospy.loginfo("Bang-bang: Inflating channel %s (current: %s, target: %s)", 
                         channel_id + 1, current_pressure, pressure_goal)
            # Apply a small inflation pulse
            #--------
            it_msgs = list(self.inflate(min_inflate_duration, False, channel_id , self.delay))
            self.airserver.submit(it_msgs)
            # Schedule next check after pulse plus a small delay for pressure to settle
            rospy.Timer(rospy.Duration(min_inflate_duration + self.delay + 0.1), 
                       lambda event: self.check_bang_bang_progress(channel_id, pressure_goal, eps), 
                       oneshot=True)
        elif pressure_goal < current_pressure - eps:
            # Need to deflate
            rospy.loginfo("Bang-bang: Deflating channel %s (current: %s, target: %s)", 
                         channel_id + 1, current_pressure, pressure_goal)
            # Apply a small deflation pulse
            it_msgs = list(self.deflate(min_deflate_duration, False, channel_id , self.delay))
            self.airserver.submit(it_msgs)
            # Schedule next check after pulse plus a small delay for pressure to settle
            rospy.Timer(rospy.Duration(min_deflate_duration + self.delay + 0.1), 
                       lambda event: self.check_bang_bang_progress(channel_id, pressure_goal, eps), 
                       oneshot=True)
        else:
            # We've reached the target pressure
            rospy.loginfo("Bang-bang: Channel %s reached target pressure %s (current: %s)", 
                         channel_id + 1, pressure_goal, current_pressure)
            self.release_command_lock()

    def check_bang_bang_progress(self, channel_id, pressure_goal, eps):
        """Check progress of bang-bang control and continue if needed"""
        # Force an update of the pressure reading
        success = self.force_update_pressure_reading(channel_id)

        if not success:
            rospy.logwarn("Bang-bang: Could not get pressure reading for channel %s, aborting for safety", 
                         channel_id + 1)
            self.release_command_lock()
            return
            
        # Get latest pressure reading
        current_pressure = self.current_pressures[channel_id]
        rospy.loginfo("Current Pressure %s",current_pressure)
        # Safety check - if pressure exceeds safety limit, abort
        if current_pressure > self.safety_pressure_limit:
            rospy.logerr("Bang-bang: SAFETY LIMIT EXCEEDED on channel %s (%s KPa). Aborting and deflating.", 
                       channel_id + 1, current_pressure)
            # Emergency deflate
            it_msgs = list(self.deflate(0.5, False, channel_id , self.delay))
            self.airserver.submit(it_msgs)
            self.release_command_lock()
            return
        
        # Check if we've reached the goal
        if abs(current_pressure - pressure_goal) <= eps:
            rospy.loginfo("Bang-bang: Channel %s reached target pressure %s (current: %s)", 
                         channel_id + 1, pressure_goal, current_pressure)
            self.release_command_lock()
        else:
            # Continue the bang-bang control
            self.execute_bang_bang_control(channel_id, pressure_goal, eps)

        # Check for timeout to avoid infinite loops
        current_time = time.time()
        if current_time - self.command_lock_time > self.command_timeout:
            rospy.logwarn("Bang-bang: Timeout exceeded for channel %s, stopping at pressure %s (target: %s)", 
                         channel_id , current_pressure, pressure_goal)
            self.release_command_lock()

    def force_update_pressure_reading(self, channel_id):
        """Force an update of the pressure reading for a specific channel"""
        # Call service_incoming_messages to get the latest data
        self.airserver.service_incoming_messages()
        
        try:
            data = self.airserver._serversignals[self.signals_topics[channel_id][0]]
            if data and len(data[1]) > 0:
                self.current_pressures[channel_id] = data[1][-1]
                rospy.loginfo("Updated pressure reading for channel %s: %s KPa", 
                              channel_id , self.current_pressures[channel_id])
                return True
            else:
                rospy.logwarn("No pressure data available for channel %s after forced update", channel_id )
                return False
        except (KeyError, AttributeError) as e:
            rospy.logwarn("Error updating pressure for channel %s: %s", channel_id , e)
            return False

def signal_handler(signum, frame):
    """Signal handler for proper shutdown"""
    rospy.loginfo("Signal received, initiating shutdown")
    rospy.signal_shutdown("External signal")
    sys.exit(0)


if __name__ == '__main__':
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)

    try:
        handler = PneumaticBoxHandler()
        handler.run()
    except rospy.ROSInterruptException:
        pass
