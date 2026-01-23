// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.PivotArmConstants;

public class Climber extends SubsystemBase {
  private final SparkMax motor;
	private final SparkMaxConfig motorConfig;

  /** Creates a new Climber. */
  public Climber() {
    motor = new SparkMax(Ports.CAN.CLIMBER, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    motorConfig
			.inverted(false)
      .idleMode(ClimberConstants.MOTOR_IDLE_MODE)
      .smartCurrentLimit(ClimberConstants.CURRENT_LIMIT_AMPS);
      motorConfig.softLimit
                .forwardSoftLimit(ClimberConstants.SOFT_LIMIT_FORWARD)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(ClimberConstants.SOFT_LIMIT_REVERSE)
                .reverseSoftLimitEnabled(true);
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Climb() {
    motor.set(ClimberConstants.CLIMB_SPEED);
  }

  public void Out() {
    motor.set(ClimberConstants.OUT_SPEED);
  }

  public void Stop() {
    motor.set(0);
  }
}
