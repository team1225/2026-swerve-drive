// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopingArmConstants;
import frc.robot.commands.telescoping_arm.ManuallyAdjustTelescopingArm;
import frc.robot.Ports;

public class TelescopingArm extends SubsystemBase {
  private final SparkMax motor;
	private final SparkMaxConfig motorConfig;

  /** Creates a new Arm. */
  public TelescopingArm() {
    motor = new SparkMax(Ports.CAN.TELESCOPING_ARM, MotorType.kBrushless);
		motorConfig = new SparkMaxConfig();

    motorConfig
			.inverted(true)
      .idleMode(TelescopingArmConstants.IDLE_MODE)
      .smartCurrentLimit(TelescopingArmConstants.CURRENT_LIMIT_AMPS);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  // ******************************************************************************************
  // Setters
  // ******************************************************************************************

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  // ******************************************************************************************
  // Commands
  // ******************************************************************************************

  public Command setSpeedCommand(double speed) {
    return Commands.runOnce(() -> setSpeed(speed));
  }

  // ******************************************************************************************
  // Getters
  // ******************************************************************************************
}
