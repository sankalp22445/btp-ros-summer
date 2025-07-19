#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from collections import defaultdict
import csv
import os
from datetime import datetime

class BeliefFusionNode:
    def __init__(self):
        rospy.init_node('belief_fusion_node')

        # Store all received beliefs per agent
        self.fused_beliefs = defaultdict(list)
        self.latest_beliefs = {}  # Current average belief per agent

        # Subscribe to belief messages
        self.sub = rospy.Subscriber("/agent/belief", String, self.belief_callback)

        # CSV logging setup
        log_filename = os.path.expanduser(
            f"~/fused_beliefs_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        self.log_file = open(log_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(["Time", "Agent", "FusedBelief"])

        rospy.loginfo("✅ BeliefFusionNode started and listening to /agent/belief")

    def belief_callback(self, msg):
        """
        Message format: 'agent_name,0.75'
        """
        try:
            agent_name, belief_str = msg.data.strip().split(',')
            belief = float(belief_str)

            # Add to belief history
            self.fused_beliefs[agent_name].append(belief)

            # Fusion strategy: average
            fused = sum(self.fused_beliefs[agent_name]) / len(self.fused_beliefs[agent_name])
            self.latest_beliefs[agent_name] = fused

            # Log to CSV
            now = rospy.get_time()
            self.csv_writer.writerow([now, agent_name, f"{fused:.2f}"])

            rospy.loginfo(f"[Fusion] Agent: {agent_name}, New: {belief:.2f}, Fused Avg: {fused:.2f}")

        except Exception as e:
            rospy.logwarn(f"⚠️ Failed to parse belief message: '{msg.data}'. Error: {e}")

    def run(self):
        try:
            rospy.spin()
        finally:
            self.log_file.close()
            rospy.loginfo("✅ Fused belief log saved and node shutting down.")

if __name__ == "__main__":
    node = BeliefFusionNode()
    node.run()

