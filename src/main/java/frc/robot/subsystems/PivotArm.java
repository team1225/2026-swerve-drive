// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotArmConstants;
import frc.robot.commands.pivot_arm.ManuallyAdjustPivotArm;
import frc.robot.Ports;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PivotArm extends SubsystemBase {
    private final SparkMax pivotMotor;
    private final SparkMaxConfig pivotMotorConfig;
    private final SparkAbsoluteEncoder pivotAbsoluteEncoder;
    private Rotation2d pivotDesiredState;
    private final SparkClosedLoopController pivotClosedLoopController;
    private final SysIdRoutine sysIdRoutine;
    
    private boolean atUpperLimit;
    private boolean atLowerLimit;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
    
    private final ShuffleboardTab pidTab;
    private double kP = PivotArmConstants.TURNING_P;
    private double kI = PivotArmConstants.TURNING_I;
    private double kD = PivotArmConstants.TURNING_D;
    private double kFF = PivotArmConstants.TURNING_FF;
    private double speedFactor = 50;

    /** Creates a new Arm. */
    public PivotArm() {
        pivotMotor = new SparkMax(Ports.CAN.PIVOT_ARM, MotorType.kBrushless);
        pivotMotorConfig = new SparkMaxConfig();
        pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();
        pivotClosedLoopController = pivotMotor.getClosedLoopController();
        
        pivotMotorConfig
                .inverted(false)
                .idleMode(PivotArmConstants.TURNING_MOTOR_IDLE_MODE)
                .smartCurrentLimit(PivotArmConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS);
        pivotMotorConfig.absoluteEncoder
                .inverted(true)
                .positionConversionFactor(PivotArmConstants.TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION) // radians
                .velocityConversionFactor(PivotArmConstants.TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM); // radians per second
        pivotMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                // These are example gains you may need to them for your own robot!
                .pid(PivotArmConstants.TURNING_P, PivotArmConstants.TURNING_I, PivotArmConstants.TURNING_D)
                .outputRange(PivotArmConstants.TURNING_MIN_OUTPUT_NORMALIZED,
                        PivotArmConstants.TURNING_MAX_OUTPUT_NORMALIZED)
                // .velocityFF(PivotArmConstants.TURNING_FF)
                // Disable PID wrap around for the ARM. We do not want a Ferris Wheel.
                .positionWrappingEnabled(false);
        pivotMotorConfig.closedLoop.maxMotion
                // Set MAXMotion parameters for position control. We don't need to pass
                // a closed loop slot, as it will default to slot 0.
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .maxVelocity(PivotArmConstants.MAX_VELOCITY)
                .maxAcceleration(PivotArmConstants.MAX_VELOCITY * PivotArmConstants.MAX_ACCELERATION_FACTOR)
                .allowedClosedLoopError(PivotArmConstants.ALLOWED_ERROR);
        pivotMotorConfig.softLimit
                .forwardSoftLimit(PivotArmConstants.SOFT_LIMIT_FORWARD)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(PivotArmConstants.SOFT_LIMIT_REVERSE)
                .reverseSoftLimitEnabled(false);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotDesiredState = new Rotation2d(pivotAbsoluteEncoder.getPosition());

        // sets the desired state to the current state so the arm does not move during
        // robot startup
        setDesiredState(pivotDesiredState);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (volts) -> {
                        pivotMotor.setVoltage(volts.in(Volts));
                    },
                    log -> {
                        log.motor("PivotArm")
                        .voltage(m_appliedVoltage.mut_replace(
                            pivotMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(this.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(pivotAbsoluteEncoder.getVelocity(), MetersPerSecond));
                    },
                    this));

        // Create Shuffleboard tab for PID tuning
        pidTab = Shuffleboard.getTab("Pivot Arm PID");
        pidTab.addNumber("Current Position (rad)", this::getPosition);
        pidTab.addNumber("Target Position (rad)", () -> getDesiredState().getRadians());
        pidTab.addNumber("Position Error (rad)", () -> getDesiredState().getRadians() - getPosition());
        
        // Add PID coefficients to Shuffleboard
        pidTab.add("P Gain", kP);
        pidTab.add("I Gain", kI);
        pidTab.add("D Gain", kD);
        pidTab.add("Feed Forward", kFF);
    }

    @Override
    public void periodic() {
        atUpperLimit = getPosition() >= (PivotArmConstants.SOFT_LIMIT_FORWARD);
        atLowerLimit = getPosition() <= (PivotArmConstants.SOFT_LIMIT_REVERSE);
        
        // // Update PID values from Shuffleboard
        // double p = pidTab.getLayout("P Gain").get.getEntry("value").getDouble(kP);
        // double i = pidTab.getLayout("I Gain").getEntry("value").getDouble(kI);
        // double d = pidTab.getLayout("D Gain").getEntry("value").getDouble(kD);
        // double ff = pidTab.getLayout("Feed Forward").getEntry("value").getDouble(kFF);

        // // Only update if values have changed
        // if (p != kP || i != kI || d != kD || ff != kFF) {
        //     kP = p;
        //     kI = i;
        //     kD = d;
        //     kFF = ff;
        //     updatePIDValues();
        // }
    }

    /**
     * Updates the PID values on the motor controller
     */
    private void updatePIDValues() {
        pivotMotorConfig.closedLoop
            .pid(kP, kI, kD)
            .velocityFF(kFF);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ******************************************************************************************
    // Setters
    // ******************************************************************************************
    
    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(Rotation2d desiredState) {
        if (desiredState.getRadians() > PivotArmConstants.SOFT_LIMIT_FORWARD) {
            desiredState = Rotation2d.fromRadians(PivotArmConstants.SOFT_LIMIT_FORWARD);
        }
        if (desiredState.getRadians() < PivotArmConstants.SOFT_LIMIT_REVERSE) {
            desiredState = Rotation2d.fromRadians(PivotArmConstants.SOFT_LIMIT_REVERSE);
        }
        pivotClosedLoopController.setReference(desiredState.getRadians(),
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0);
        pivotDesiredState = desiredState;
    }

    public void driveManually (double speed) {
        // if ( getPosition() > getDesiredState().getRadians()- 0.1 && getPosition() < getDesiredState().getRadians() + 0.1){
            setDesiredState(Rotation2d.fromRadians(getDesiredState().getRadians() + (speed / speedFactor)));
        // }
    }

    public void driveManuallyVoltage(double voltage) {
        pivotClosedLoopController.setReference(voltage, ControlType.kVoltage);
    }

    public void setGoalDegrees(Rotation2d targetDegrees) {
        setDesiredState(targetDegrees);
    }

    public void setVoltage(double voltage) {
        pivotMotor.set(voltage);
    }

    public void setSpeedFactor(double factor) {
        this.speedFactor = factor;
    }

    // ******************************************************************************************
    // Commands
    // ******************************************************************************************
    public Command setGoalDegreesCommand(double targetDegrees) {
        return Commands.runOnce(() -> setDesiredState(Rotation2d.fromDegrees(targetDegrees)));
    }

    public Command clearStickyFaultsCommand() {
        return Commands.runOnce(() -> pivotMotor.clearFaults());
    }

    public Command setVoltageCommand(double voltage) {
        return Commands.runOnce(() -> setVoltage(voltage));
    }

    public Command setSpeedFactorCommand(double speedFactor) {
        return Commands.runOnce(() -> setSpeedFactor(speedFactor));
    }
    
    // ******************************************************************************************
    // Getters
    // ******************************************************************************************
    public SparkAbsoluteEncoder getTurningAbsoluteEncoder() {
        return pivotAbsoluteEncoder;
    }

    public Rotation2d getDesiredState() {
        return pivotDesiredState;
    }

    public double getPosition() {
        return pivotAbsoluteEncoder.getPosition();
    }

    public boolean onPlusSoftwareLimit() {
        return pivotAbsoluteEncoder.getPosition() >= pivotMotor.configAccessor.softLimit.getForwardSoftLimit();
    }

    public boolean onMinusSoftwareLimit() {
        return pivotMotor.getEncoder().getPosition() <= pivotMotor.configAccessor.softLimit.getReverseSoftLimit();
    }

    public boolean onPlusHardwareLimit() {
        return pivotMotor.getForwardLimitSwitch().isPressed();
    }

    public boolean onMinusHardwareLimit() {
        return pivotMotor.getReverseLimitSwitch().isPressed();
    }

    public boolean onLimit() {
        return onPlusHardwareLimit() || onMinusHardwareLimit() ||
                onPlusSoftwareLimit() || onMinusSoftwareLimit();
    }

    public boolean isAtUpperLimit() {
        return atUpperLimit;
    }

    public boolean isAtLowerLimit() {
        return atLowerLimit;
    }

// ******************************************************************************************

    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(Direction.kForward);
    }

    public Command quasistaticBackward() {
        return sysIdRoutine.quasistatic(Direction.kReverse);
    }

    public Command dynamicForward() {
        return sysIdRoutine.dynamic(Direction.kForward);
    }

    public Command dynamicBackward() {
        return sysIdRoutine.dynamic(Direction.kReverse);
    }
}
