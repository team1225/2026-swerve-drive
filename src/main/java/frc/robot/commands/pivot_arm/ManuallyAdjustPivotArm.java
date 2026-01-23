// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot_arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.PivotArm;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ManuallyAdjustPivotArm extends Command {
  /** Creates a new JogArm. */
  private PivotArm m_arm;
  private CommandXboxController m_controller;
  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(0.75);

  public ManuallyAdjustPivotArm(PivotArm arm, CommandXboxController controller) {
    m_arm = arm;
    m_controller = controller;
    addRequirements(m_arm);
    //pidTab = Shuffleboard.getTab("Pivot Arm Controls");
    //pidTab.addNumber("Y controller", () -> m_controller.getLeftY());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yval = m_controller.getLeftY(); // Left stick Y axis
    m_arm.driveManuallyVoltage(m_magLimiter.calculate(-yval*6));
    //if (yval < 0.1 && yval > -0.1)
    // m_arm.driveManually(m_magLimiter.calculate(-yval)); this can work the problem is in manual controle i think
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setDesiredState(Rotation2d.fromRadians(m_arm.getPosition()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
