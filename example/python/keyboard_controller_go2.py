import time
import sys
import numpy as np
import threading
try:
    import getch
except ImportError:
    # Fallback for systems without getch
    import select
    import termios
    import tty

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
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

class KeyboardController:
    def __init__(self):
        self.dt = 0.002
        self.running_time = 0.0
        self.crc = CRC()
        
        # Control states
        self.robot_state = "standing"  # standing, walking, sitting
        self.velocity_x = 0.0  # forward/backward
        self.velocity_y = 0.0  # left/right
        self.angular_z = 0.0   # turning
        
        # Track state transition timing for smooth gains
        self.state_transition_time = 0.0
        self.previous_state = "standing"
        
        # Keyboard input
        self.keys_pressed = set()
        self.running = True
        
        # Terminal settings
        self.old_settings = None
        
    def setup_terminal(self):
        """Setup terminal for keyboard input"""
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        except:
            self.old_settings = None
        
    def restore_terminal(self):
        """Restore terminal settings"""
        if self.old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except:
                pass
    
    def get_key(self):
        """Non-blocking keyboard input"""
        try:
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                return sys.stdin.read(1)
        except:
            pass
        return None
    
    def keyboard_thread(self):
        """Thread for handling keyboard input"""
        print("\nKeyboard input active. Use the following keys:")
        print("W/S: Forward/Backward, A/D: Turn, J/L: Strafe, U: Stand, I: Sit, SPACE: Stop, Q: Quit")
        
        while self.running:
            try:
                key = self.get_key()
                if key:
                    if key.lower() == 'q' or ord(key) == 3:  # q or Ctrl+C
                        self.running = False
                        break
                    elif key.lower() == 'w':
                        self.velocity_x = 0.3  # forward
                        print(f"\rForward: {self.velocity_x:.1f}", end="", flush=True)
                    elif key.lower() == 's':
                        self.velocity_x = -0.3  # backward
                        print(f"\rBackward: {self.velocity_x:.1f}", end="", flush=True)
                    elif key.lower() == 'a':
                        self.angular_z = 0.5  # turn left
                        print(f"\rTurn Left: {self.angular_z:.1f}", end="", flush=True)
                    elif key.lower() == 'd':
                        self.angular_z = -0.5  # turn right
                        print(f"\rTurn Right: {self.angular_z:.1f}", end="", flush=True)
                    elif key.lower() == 'j':
                        self.velocity_y = 0.2  # strafe left
                        print(f"\rStrafe Left: {self.velocity_y:.1f}", end="", flush=True)
                    elif key.lower() == 'l':
                        self.velocity_y = -0.2  # strafe right
                        print(f"\rStrafe Right: {self.velocity_y:.1f}", end="", flush=True)
                    elif key.lower() == 'u':
                        if self.robot_state != "standing":
                            self.previous_state = self.robot_state
                            self.robot_state = "standing"
                            self.state_transition_time = 0.0  # Reset transition timer
                        print(f"\rStanding", end="", flush=True)
                    elif key.lower() == 'i':
                        if self.robot_state != "sitting":
                            self.previous_state = self.robot_state
                            self.robot_state = "sitting"
                            self.state_transition_time = 0.0  # Reset transition timer
                        print(f"\rSitting", end="", flush=True)
                    elif key == ' ':  # spacebar to stop
                        self.velocity_x = 0.0
                        self.velocity_y = 0.0
                        self.angular_z = 0.0
                        print(f"\rStopped", end="", flush=True)
            except KeyboardInterrupt:
                self.running = False
                break
            except:
                pass
            time.sleep(0.01)
    
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
    
    def print_controls(self):
        """Print control instructions"""
        print("\n=== Unitree Go2 Keyboard Controller ===")
        print("Movement Controls:")
        print("  W/S: Forward/Backward")
        print("  A/D: Turn Left/Right")
        print("  J/L: Strafe Left/Right")
        print("  SPACE: Stop movement")
        print("\nPose Controls:")
        print("  U: Stand up")
        print("  I: Sit down")
        print("\nOther:")
        print("  Q: Quit")
        print("\nPress any key to start...")
        print("======================================\n")
    
    def run(self):
        """Main control loop"""
        self.print_controls()
        input()  # Wait for user to press enter
        
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
        
        # Setup terminal and start keyboard thread
        self.setup_terminal()
        keyboard_thread = threading.Thread(target=self.keyboard_thread)
        keyboard_thread.daemon = True
        keyboard_thread.start()
        
        print("Controller started! Use WASD for movement, U/I for pose, Q to quit.")
        
        try:
            while self.running:
                step_start = time.perf_counter()
                self.running_time += self.dt
                
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
                
                # Set motor commands with smooth gain transitions like stand_go2.py
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
                
                # Print status every second
                if int(self.running_time * 10) % 10 == 0:
                    print(f"\rState: {self.robot_state}, Vel: ({self.velocity_x:.1f}, {self.velocity_y:.1f}), Turn: {self.angular_z:.1f}", end="")
                
                # Maintain loop timing
                time_until_next_step = self.dt - (time.perf_counter() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                    
        except KeyboardInterrupt:
            self.running = False
        finally:
            self.restore_terminal()
            print("\nController stopped.")

if __name__ == '__main__':
    controller = KeyboardController()
    controller.run()