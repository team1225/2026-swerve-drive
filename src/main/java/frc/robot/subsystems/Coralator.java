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
import frc.robot.Constants.CoralatorConstants;

public class Coralator extends SubsystemBase {
  private final SparkMax motor;
	private final SparkMaxConfig motorConfig;

  /** Creates a new Coralator. */
  public Coralator() {
    motor = new SparkMax(Ports.CAN.CORALATOR, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    motorConfig
			.inverted(false)
      .idleMode(CoralatorConstants.MOTOR_IDLE_MODE)
      .smartCurrentLimit(CoralatorConstants.CURRENT_LIMIT_AMPS);
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Intake() {
    motor.set(CoralatorConstants.INTAKE_SPEED);
  }

  public void Eject() {
    motor.set(CoralatorConstants.EJECT_SPEED);
  }

  public void Stop() {
    motor.set(0);
  }
}
