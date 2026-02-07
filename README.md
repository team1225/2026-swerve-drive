# FRC 2025 ReefScape Robot

This repository contains the code for our 2025 FRC competition robot, designed for the FIRST ReefScape challenge.

## Hardware Specifications

### Drivetrain
- Swerve Drive Specialties MK4i modules
- NEO 500 motors for both drive and turning
- Gear ratio: L2 6.75:1
- Maximum speed: TBD

To set the zeros of the drivetrain align the wheels so the bolt head faces the port side (left if standing behind the robot).  Use the Rev Hardware Client to set the zero on of the absolute encoders remembering to persist parameters.

### Sensors
- CTRE Pigeon 2.0 IMU for robot orientation and field-relative driving

## Software

### Development Environment
- Written in Java
- Built with Gradle
- WPILib 2025 (required)
- REV Hardware Client (for NEO motor configuration)
- Phoenix Tuner X (for Pigeon 2.0 configuration)

### Project Structure
```
src/main/java/frc/robot/
├── commands/        # Command classes for robot actions
├── subsystems/      # Subsystem classes for major robot components
├── constants/       # Configuration constants and ports
└── Robot.java      # Main robot class
```

### Key Features
- Field-oriented swerve drive control
- Autonomous Commands:
  - Simple Drive: Drives forward at 40% power for 3 seconds
  - Do Nothing: Robot remains stationary
- [Add other major features]
- [Add autonomous routines if developed]

### Controls

#### Driver Controls (Xbox Controller)
- Left Stick: Drive robot (Y axis for forward/backward, X axis for strafing)
- Right Stick: Rotate robot (X axis)
- A Button: Set pivot arm to 0 degrees
- B Button: Set pivot arm to 30 degrees

#### Co-Driver Controls (Xbox Controller)
- Right Stick: Manually adjust telescoping arm
- Left Stick: Manually adjust pivot arm
- A Button: Set telescoping arm to position 0
- B Button: Set telescoping arm to position 30

#### Characterization Controls (Xbox Controller)
- D-Pad Up: Pivot arm quasistatic forward
- D-Pad Down: Pivot arm quasistatic backward  
- D-Pad Right: Pivot arm dynamic forward
- D-Pad Left: Pivot arm dynamic backward

## Getting Started

### Prerequisites
1. Install WPILib 2025
2. Install REV Hardware Client
3. Install Phoenix Tuner X
4. [Any other required software]

### Installation
1. Clone the repository:
```bash
git clone [repository URL]
```

2. Open the project in WPILib VS Code
3. Build the project:
```bash
./gradlew build
```

### Development Workflow
1. Create a new branch for your feature
2. Make your changes
3. Test on the robot
   - For autonomous testing:
     - Use SmartDashboard to select desired autonomous routine
     - Ensure ample clear space in front of robot when testing Simple Drive
     - Always be ready to disable the robot if needed
4. Create a pull request
5. Get code review
6. Merge after approval

## Contributing
[Add contribution guidelines]

## Team
Team 1225 - The Gorillas
Henderson County Public Schools High School Robotics

Our team represents Henderson County Public Schools in North Carolina, competing in the FIRST Robotics Competition. The Gorillas have been inspiring students and promoting STEM education through competitive robotics since our founding.

## License
[Add license information]