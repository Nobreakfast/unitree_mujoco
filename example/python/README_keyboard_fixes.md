# Keyboard Controller Fixes for Unitree Go2

## Issues Identified and Fixed

### 1. SDK Configuration Mismatch
**Problem**: The keyboard controllers were using different domain IDs and network interfaces than the simulation.
- `unitree_mujoco.py` uses: `ChannelFactoryInitialize(1, "lo")`
- Original keyboard controllers used: `ChannelFactoryInitialize(0, "eth0")` or command-line arguments

**Solution**: Updated all keyboard controllers to use `ChannelFactoryInitialize(1, "lo")` to match the simulation configuration.

### 2. Terminal Input Issues on macOS
**Problem**: The terminal-based keyboard controller (`keyboard_controller_go2.py`) had issues with raw terminal mode on macOS, causing unresponsive or erratic behavior.

**Solution**: 
- Added fallback import for `getch` function
- Improved terminal setup/restore with error handling
- Added Ctrl+C as quit option
- Enhanced user feedback with print statements

### 3. Robot Shaking and Control Issues
**Problem**: The robot was shaking excessively and barely responding to keyboard input due to:
- Overly aggressive walking gait with large joint offsets
- Mismatched control gains compared to the working `stand_go2.py`
- Using unstable walking positions as base instead of stable standing positions
- Too fast gait frequency causing instability

**Solution**:
- **Reduced joint offsets**: Changed from 0.2-0.3 to 0.02-0.05 for much smaller, smoother movements
- **Fixed control gains**: Used stable gains matching `stand_go2.py` (kp: 20.0 for sitting, 30.0 for walking, 50.0 for standing)
- **Changed base position**: Use stable standing position as base for walking instead of separate walking positions
- **Slower gait frequency**: Reduced from 2.0 Hz to 1.0 Hz for better stability
- **Added velocity decay**: Gradual deceleration when keys are not pressed to prevent sudden stops
- **Improved state transitions**: Smoother transitions between standing, sitting, and walking states

### 4. Pygame Controller Issues
**Problem**: The Pygame controller had timing and display update issues causing the GUI to quit unexpectedly.

**Solution**:
- Fixed display update frequency to 30 FPS
- Added proper frame rate limiting with `clock.tick(60)`
- Improved event handling loop
- Applied same stability fixes as other controllers

## Available Controllers

### 1. keyboard_controller_go2.py (Fixed)
- Uses terminal-based input with raw mode
- Real-time key detection
- Better error handling for macOS

### 2. keyboard_controller_pygame.py (Fixed)
- Uses pygame for GUI-based control
- Visual feedback window
- Smooth input handling

### 3. keyboard_controller_simple.py (New)
- Simple command-line interface
- Type commands and press Enter
- Most reliable on all systems
- No special terminal modes required

## Usage Instructions

### Step 1: Start the Simulation
```bash
cd /path/to/unitree_mujoco/simulate_python
python unitree_mujoco.py
```

### Step 2: Choose a Controller

#### Option A: Simple Controller (Recommended)
```bash
cd /path/to/unitree_mujoco/example/python
python keyboard_controller_simple.py
```
Then type commands like `w`, `s`, `a`, `d`, etc., and press Enter.

#### Option B: Real-time Terminal Controller
```bash
cd /path/to/unitree_mujoco/example/python
python keyboard_controller_go2.py
```
Press keys directly (no Enter needed).

#### Option C: Pygame GUI Controller
```bash
cd /path/to/unitree_mujoco/example/python
python keyboard_controller_pygame.py
```
Use the pygame window for control.

## Controls

| Key | Action |
|-----|--------|
| W/S | Forward/Backward |
| A/D | Turn Left/Right |
| J/L | Strafe Left/Right |
| U | Stand up |
| I | Sit down |
| SPACE | Stop movement |
| Q | Quit |

## Troubleshooting

1. **"No response from robot"**: Make sure the simulation is running first
2. **"Connection error"**: Ensure both simulation and controller use the same domain ID (1) and interface ("lo")
3. **"Terminal issues"**: Try the simple controller instead
4. **"Pygame not found"**: Install with `pip install pygame`

## Technical Details

The main issue was that the keyboard controllers were trying to connect to a different communication channel than the simulation. The simulation uses:
- Domain ID: 1
- Interface: "lo" (loopback)

The original controllers were using domain ID 0 or taking it from command line arguments, causing a communication mismatch.