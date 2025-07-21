import time
import sys
import numpy as np
try:
    import pygame
except ImportError:
    print("pygame not found. Install with: pip install pygame")
    sys.exit(1)

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

class KeyboardControllerPygame:
    def __init__(self):
        self.dt = 0.002
        self.running_time = 0.0
        self.crc = CRC()
        
        # Control states
        self.robot_state = "standing"  # standing, walking, sitting
        self.velocity_x = 0.0  # forward/backward
        self.velocity_y = 0.0  # left/right
        self.angular_z = 0.0   # turning
        
        # Control parameters
        self.max_velocity = 0.5
        self.max_angular = 0.8
        self.velocity_step = 0.1
        self.angular_step = 0.2
        
        self.running = True
        
        # Track state transition timing for smooth gains
        self.state_transition_time = 0.0
        self.previous_state = "standing"
        
        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Unitree Go2 Keyboard Controller")
        self.clock = pygame.time.Clock()
        
    def handle_keyboard_input(self):
        """Handle pygame keyboard events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_u:
                    if self.robot_state != "standing":
                        self.previous_state = self.robot_state
                        self.robot_state = "standing"
                        self.state_transition_time = 0.0  # Reset transition timer
                    print("Robot state: Standing")
                elif event.key == pygame.K_i:
                    if self.robot_state != "sitting":
                        self.previous_state = self.robot_state
                        self.robot_state = "sitting"
                        self.state_transition_time = 0.0  # Reset transition timer
                    print("Robot state: Sitting")
                elif event.key == pygame.K_SPACE:
                    self.velocity_x = 0.0
                    self.velocity_y = 0.0
                    self.angular_z = 0.0
                    print("Movement stopped")
        
        # Handle continuous key presses
        keys = pygame.key.get_pressed()
        
        # Forward/Backward
        if keys[pygame.K_w]:
            self.velocity_x = min(self.max_velocity, self.velocity_x + self.velocity_step)
        elif keys[pygame.K_s]:
            self.velocity_x = max(-self.max_velocity, self.velocity_x - self.velocity_step)
        else:
            self.velocity_x *= 0.9  # Gradual deceleration
            if abs(self.velocity_x) < 0.01:
                self.velocity_x = 0.0
        
        # Left/Right strafe
        if keys[pygame.K_j]:
            self.velocity_y = min(self.max_velocity, self.velocity_y + self.velocity_step)
        elif keys[pygame.K_l]:
            self.velocity_y = max(-self.max_velocity, self.velocity_y - self.velocity_step)
        else:
            self.velocity_y *= 0.9  # Gradual deceleration
            if abs(self.velocity_y) < 0.01:
                self.velocity_y = 0.0
        
        # Turning
        if keys[pygame.K_a]:
            self.angular_z = min(self.max_angular, self.angular_z + self.angular_step)
        elif keys[pygame.K_d]:
            self.angular_z = max(-self.max_angular, self.angular_z - self.angular_step)
        else:
            self.angular_z *= 0.9  # Gradual deceleration
            if abs(self.angular_z) < 0.01:
                self.angular_z = 0.0
    
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
    
    def update_display(self):
        """Update the pygame display with current status"""
        self.screen.fill((0, 0, 0))  # Black background
        
        font = pygame.font.Font(None, 24)
        y_offset = 20
        
        # Title
        title = font.render("Unitree Go2 Controller", True, (255, 255, 255))
        self.screen.blit(title, (10, y_offset))
        y_offset += 40
        
        # Controls
        controls = [
            "Controls:",
            "W/S: Forward/Backward",
            "A/D: Turn Left/Right",
            "J/L: Strafe Left/Right",
            "U: Stand up, I: Sit down",
            "SPACE: Stop, Q/ESC: Quit",
            "",
            f"State: {self.robot_state}",
            f"Velocity X: {self.velocity_x:.2f}",
            f"Velocity Y: {self.velocity_y:.2f}",
            f"Angular Z: {self.angular_z:.2f}"
        ]
        
        for line in controls:
            color = (255, 255, 0) if line.startswith(("State:", "Velocity", "Angular")) else (255, 255, 255)
            text = font.render(line, True, color)
            self.screen.blit(text, (10, y_offset))
            y_offset += 25
        
        pygame.display.flip()
    
    def print_controls(self):
        """Print control instructions to console"""
        print("\n=== Unitree Go2 Pygame Keyboard Controller ===")
        print("Movement Controls:")
        print("  W/S: Forward/Backward")
        print("  A/D: Turn Left/Right")
        print("  J/L: Strafe Left/Right")
        print("  SPACE: Stop movement")
        print("\nPose Controls:")
        print("  U: Stand up")
        print("  I: Sit down")
        print("\nOther:")
        print("  Q/ESC: Quit")
        print("\nA pygame window will open for control.")
        print("Press Enter to start...")
        print("==========================================\n")
    
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
        
        print("Controller started! Use the pygame window for control.")
        
        try:
            while self.running:
                step_start = time.perf_counter()
                self.running_time += self.dt
                
                # Handle input
                self.handle_keyboard_input()
                
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
                
                # Update display at 30 FPS
                if int(self.running_time * 30) % 1 == 0:
                    self.update_display()
                
                # Limit pygame event processing to avoid blocking
                self.clock.tick(60)  # 60 FPS for smooth input
                
                # Maintain loop timing
                time_until_next_step = self.dt - (time.perf_counter() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                    
        except KeyboardInterrupt:
            self.running = False
        finally:
            pygame.quit()
            print("\nController stopped.")

if __name__ == '__main__':
    controller = KeyboardControllerPygame()
    controller.run()