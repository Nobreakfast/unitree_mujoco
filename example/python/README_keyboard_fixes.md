# Keyboard Controller Fixes for Unitree Go2

## Issues Identified and Fixed

### 1. SDK Configuration Mismatch
**Problem**: The keyboard controllers were using different domain IDs and interfaces than the simulation.
- Original controllers used `ChannelFactoryInitialize(0, sys.argv[1])` or `ChannelFactoryInitialize(1, "lo")`
- Simulation uses `ChannelFactoryInitialize(1, "lo")` (from config.py)

**Fix**: Updated both controllers to use `ChannelFactoryInitialize(1, "lo")` to match the simulation.

### 2. Terminal Input Issues on macOS
**Problem**: The original `keyboard_controller_go2.py` used raw terminal mode which can be problematic on macOS.

**Fix**: 
- Added better error handling for terminal operations
- Improved keyboard input feedback
- Added Ctrl+C handling

### 3. Pygame Controller Issues
**Problem**: The pygame controller had timing and display update issues.

**Fix**:
- Fixed display update frequency (30 FPS instead of erratic updates)
- Added proper pygame clock management (60 FPS)
- Ensured SDK configuration matches simulation

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