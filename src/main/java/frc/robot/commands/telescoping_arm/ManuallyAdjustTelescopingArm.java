// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescoping_arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.TelescopingArm;

public class ManuallyAdjustTelescopingArm extends Command {
  /** Creates a new JogArm. */
  private TelescopingArm m_arm;
  private CommandXboxController m_controller;

  public ManuallyAdjustTelescopingArm(TelescopingArm arm, CommandXboxController controller) {
    m_arm = arm;
    m_controller = controller;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yval = m_controller.getRightY() / 2;
    if (Math.abs(m_controller.getRightY()) < 0.1) {
      yval = 0;
    }
    m_arm.setSpeed(-yval);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}