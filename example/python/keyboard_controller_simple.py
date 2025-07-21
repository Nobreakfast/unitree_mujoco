import time
import sys
import numpy as np
import threading
from queue import Queue

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

# Initial joint positions for different states
# Order: FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf
stand_up_joint_pos = np.array([
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763
], dtype=float)

stand_down_joint_pos = np.array([
    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
    1.22187, -2.44375, -0.0473455, 1.22187, -2.44375
], dtype=float)

walk_joint_pos = np.array([
    0.0, 0.8, -1.6, 0.0, 0.8, -1.6,
    0.0, 0.8, -1.6, 0.0, 0.8, -1.6
], dtype=float)

class SimpleKeyboardController:
    def __init__(self):
        self.dt = 0.002
        self.running_time = 0.0
        self.crc = CRC()
        
        # Control states
        self.robot_state = "standing"  # standing, walking, sitting
        self.velocity_x = 0.0  # forward/backward
        self.velocity_y = 0.0  # left/right
        self.angular_z = 0.0   # turning
        
        self.running = True
        self.command_queue = Queue()
        
        # Track state transition timing for smooth gains
        self.state_transition_time = 0.0
        self.previous_state = "standing"
        
    def input_thread(self):
        """Simple input thread using standard input"""
        print("\n=== Simple Keyboard Controller ===")
        print("Commands:")
        print("  w: Forward    s: Backward")
        print("  a: Turn Left  d: Turn Right")
        print("  j: Strafe Left  l: Strafe Right")
        print("  u: Stand up   i: Sit down")
        print("  space: Stop   q: Quit")
        print("\nType commands and press Enter:")
        print("==============================\n")
        
        while self.running:
            try:
                cmd = input("> ").strip().lower()
                if cmd:
                    self.command_queue.put(cmd)
                    if cmd == 'q':
                        self.running = False
                        break
            except (EOFError, KeyboardInterrupt):
                self.running = False
                break
    
    def process_commands(self):
        """Process commands from the queue"""
        while not self.command_queue.empty():
            cmd = self.command_queue.get()
            
            if cmd == 'w':
                self.velocity_x = 0.3
                print(f"Forward: {self.velocity_x:.1f}")
            elif cmd == 's':
                self.velocity_x = -0.3
                print(f"Backward: {self.velocity_x:.1f}")
            elif cmd == 'a':
                self.angular_z = 0.5
                print(f"Turn Left: {self.angular_z:.1f}")
            elif cmd == 'd':
                self.angular_z = -0.5
                print(f"Turn Right: {self.angular_z:.1f}")
            elif cmd == 'j':
                self.velocity_y = 0.2
                print(f"Strafe Left: {self.velocity_y:.1f}")
            elif cmd == 'l':
                self.velocity_y = -0.2
                print(f"Strafe Right: {self.velocity_y:.1f}")
            elif cmd == 'u':
                if self.robot_state != "standing":
                    self.previous_state = self.robot_state
                    self.robot_state = "standing"
                    self.state_transition_time = 0.0  # Reset transition timer
                print("Standing")
            elif cmd == 'i':
                if self.robot_state != "sitting":
                    self.previous_state = self.robot_state
                    self.robot_state = "sitting"
                    self.state_transition_time = 0.0  # Reset transition timer
                print("Sitting")
            elif cmd == ' ' or cmd == 'space':
                self.velocity_x = 0.0
                self.velocity_y = 0.0
                self.angular_z = 0.0
                print("Stopped")
            elif cmd == 'q':
                self.running = False
    
    def calculate_joint_positions(self, gait_phase):
        """Calculate joint positions based on gait and velocities"""
        if self.robot_state == "sitting":
            return stand_down_joint_pos
        elif self.robot_state == "standing" and abs(self.velocity_x) < 0.01 and abs(self.velocity_y) < 0.01 and abs(self.angular_z) < 0.01:
            return stand_up_joint_pos
        else:
            # Use standing position as base for walking to reduce instability
            joint_pos = stand_up_joint_pos.copy()
            
            # Apply small, smooth modifications for movement
            # Correct joint order: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
            if abs(self.velocity_x) > 0.01:  # Forward/backward motion
                # Much smaller offsets to prevent shaking
                hip_offset = 0.05 * self.velocity_x * np.sin(gait_phase * 2 * np.pi)
                knee_offset = 0.03 * self.velocity_x * np.cos(gait_phase * 2 * np.pi)
                
                # Apply to hip joints: FR(0), FL(3), RR(6), RL(9)
                joint_pos[0] += hip_offset  # FR hip
                joint_pos[3] -= hip_offset  # FL hip
                joint_pos[6] -= hip_offset  # RR hip
                joint_pos[9] += hip_offset  # RL hip
                
                # Apply to thigh joints: FR(1), FL(4), RR(7), RL(10)
                joint_pos[1] += knee_offset  # FR thigh
                joint_pos[4] += knee_offset  # FL thigh
                joint_pos[7] += knee_offset  # RR thigh
                joint_pos[10] += knee_offset  # RL thigh
            
            if abs(self.angular_z) > 0.01:  # Turning
                # Very small turn offset to prevent instability
                turn_offset = 0.02 * self.angular_z
                joint_pos[0] += turn_offset  # FR hip
                joint_pos[3] += turn_offset  # FL hip
                joint_pos[6] += turn_offset  # RR hip
                joint_pos[9] += turn_offset  # RL hip
            
            if abs(self.velocity_y) > 0.01:  # Strafing
                # Small strafe offset
                strafe_offset = 0.03 * self.velocity_y
                joint_pos[0] += strafe_offset  # FR hip
                joint_pos[3] -= strafe_offset  # FL hip
                joint_pos[6] += strafe_offset  # RR hip
                joint_pos[9] -= strafe_offset  # RL hip
            
            return joint_pos
    
    def run(self):
        """Main control loop"""
        # Initialize SDK - Match simulation config
        ChannelFactoryInitialize(1, "lo")
        
        # Create publisher
        pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        pub.Init()
        
        # Initialize command
        cmd = unitree_go_msg_dds__LowCmd_()
        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF
        cmd.gpio = 0
        
        for i in range(20):
            cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            cmd.motor_cmd[i].q = 0.0
            cmd.motor_cmd[i].kp = 0.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 0.0
            cmd.motor_cmd[i].tau = 0.0
        
        # Start input thread
        input_thread = threading.Thread(target=self.input_thread)
        input_thread.daemon = True
        input_thread.start()
        
        print("Controller started! Robot will stand up automatically.")
        
        try:
            while self.running:
                step_start = time.perf_counter()
                self.running_time += self.dt
                
                # Process any pending commands
                self.process_commands()
                
                # Apply velocity decay for smoother control
                decay_factor = 0.95
                if abs(self.velocity_x) < 0.05:
                    self.velocity_x *= decay_factor
                if abs(self.velocity_y) < 0.05:
                    self.velocity_y *= decay_factor
                if abs(self.angular_z) < 0.05:
                    self.angular_z *= decay_factor
                
                # Calculate gait phase for walking (slower for stability)
                gait_phase = (self.running_time * 1.0) % 1.0  # 1 Hz gait frequency for stability
                
                # Get target joint positions
                target_positions = self.calculate_joint_positions(gait_phase)
                
                # Track state transitions for smooth gain changes
                if self.robot_state != self.previous_state:
                    self.state_transition_time += self.dt
                else:
                    self.state_transition_time = min(self.state_transition_time + self.dt, 2.0)  # Cap at 2 seconds
                
                # Set motor commands with smooth gain transitions
                for i in range(12):
                    cmd.motor_cmd[i].q = target_positions[i]
                    
                    # Use smooth gain transitions to prevent oscillations
                    if self.robot_state == "sitting":
                        # Gradual transition to sitting gains
                        transition_phase = min(self.state_transition_time / 1.2, 1.0)
                        cmd.motor_cmd[i].kp = 50.0 * (1 - transition_phase) + 20.0 * transition_phase
                    elif abs(self.velocity_x) > 0.01 or abs(self.velocity_y) > 0.01 or abs(self.angular_z) > 0.01:
                        cmd.motor_cmd[i].kp = 25.0  # Lower stiffness for walking to prevent shaking
                    else:
                        # Gradual transition to standing gains
                        transition_phase = min(self.state_transition_time / 1.2, 1.0)
                        cmd.motor_cmd[i].kp = 20.0 * (1 - transition_phase) + 35.0 * transition_phase  # Lower max gain
                    
                    cmd.motor_cmd[i].dq = 0.0
                    cmd.motor_cmd[i].kd = 3.5  # Keep same damping as working version
                    cmd.motor_cmd[i].tau = 0.0
                
                # Send command
                cmd.crc = self.crc.Crc(cmd)
                pub.Write(cmd)
                
                # Maintain loop timing
                time_until_next_step = self.dt - (time.perf_counter() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                    
        except KeyboardInterrupt:
            self.running = False
        finally:
            print("\nController stopped.")

if __name__ == '__main__':
    controller = SimpleKeyboardController()
    controller.run()