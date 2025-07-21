# Unitree Go2 Keyboard Controllers

This directory contains keyboard-controlled implementations for the Unitree Go2 robot based on the original `stand_go2.py` example.

## Available Controllers

### 1. keyboard_controller_go2.py
A terminal-based keyboard controller using raw terminal input.

**Features:**
- Real-time keyboard input without pressing Enter
- Basic walking gait implementation
- Standing and sitting poses
- Forward/backward, turning, and strafing movements

**Requirements:**
- Unix-like system (Linux/macOS) with termios support
- No additional dependencies beyond unitree_sdk2py

### 2. keyboard_controller_pygame.py (Recommended)
A pygame-based keyboard controller with visual feedback.

**Features:**
- Cross-platform compatibility
- Visual status display in pygame window
- Smooth velocity control with gradual acceleration/deceleration
- Enhanced gait implementation
- Real-time status monitoring

**Requirements:**
- pygame library: `pip install pygame`
- unitree_sdk2py

## Controls

Both controllers use the same key mappings:

### Movement Controls
- **W/S**: Forward/Backward movement
- **A/D**: Turn left/right
- **J/L**: Strafe left/right
- **SPACE**: Stop all movement

### Pose Controls
- **U**: Stand up
- **I**: Sit down

### System Controls
- **Q** (or ESC for pygame version): Quit the controller

## Usage

### Basic Usage
```bash
# Terminal-based controller
python keyboard_controller_go2.py

# Pygame-based controller (recommended)
python keyboard_controller_pygame.py
```

### With Network Interface
```bash
# Specify network interface (e.g., eth0)
python keyboard_controller_pygame.py eth0
```

## Robot States

1. **Standing**: Robot maintains upright posture
2. **Walking**: Robot performs walking gait based on velocity commands
3. **Sitting**: Robot sits down in a low position

## Implementation Details

### Joint Control
- Uses position control mode (PMSM mode 0x01)
- Adaptive stiffness based on robot state:
  - Standing: High stiffness (60.0)
  - Walking: Medium stiffness (40.0)
  - Sitting: Low stiffness (20.0)
- Fixed damping coefficient (3.5)

### Gait Implementation
- Simple sinusoidal gait pattern
- 1.5 Hz gait frequency for pygame version
- 2.0 Hz gait frequency for terminal version
- Phase-shifted leg movements for natural walking

### Safety Features
- Gradual velocity changes to prevent sudden movements
- Bounded velocity limits
- Proper motor initialization
- CRC checksum for command integrity

## Troubleshooting

### Common Issues

1. **"pygame not found" error**
   ```bash
   pip install pygame
   ```

2. **Terminal input not working (terminal version)**
   - Ensure you're running on a Unix-like system
   - Try the pygame version instead

3. **Robot not responding**
   - Check network connection
   - Verify the correct network interface is specified
   - Ensure the robot is powered on and in the correct mode

4. **Jerky movements**
   - Adjust the velocity step sizes in the code
   - Check the loop timing (dt = 0.002)

### Performance Tips

- Use the pygame version for better cross-platform compatibility
- Adjust `max_velocity` and `max_angular` parameters for different movement speeds
- Modify `velocity_step` and `angular_step` for different acceleration rates

## Customization

### Modifying Movement Parameters
```python
# In the controller class __init__ method
self.max_velocity = 0.5      # Maximum linear velocity
self.max_angular = 0.8       # Maximum angular velocity
self.velocity_step = 0.1     # Acceleration step
self.angular_step = 0.2      # Angular acceleration step
```

### Adding New Poses
```python
# Define new joint positions
new_pose_joint_pos = np.array([
    # 12 joint angles for the new pose
], dtype=float)

# Add to calculate_joint_positions method
if self.robot_state == "new_pose":
    return new_pose_joint_pos
```

### Modifying Gait Parameters
```python
# In the main loop
gait_phase = (self.running_time * frequency) % 1.0  # Adjust frequency

# In calculate_joint_positions
hip_offset = amplitude * self.velocity_x * np.sin(gait_phase * 2 * np.pi)
```

## Based On

These controllers are based on the original `stand_go2.py` example, extending it with:
- Interactive keyboard control
- Multiple robot states
- Walking gait implementation
- Real-time parameter adjustment
- Enhanced user interface