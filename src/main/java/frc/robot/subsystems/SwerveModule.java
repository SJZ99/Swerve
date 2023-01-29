// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;
  private final CANCoder m_turningEncoder;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningMotorEncoderChannel,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {

    // initialize
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel);
    
    m_turningEncoder = new CANCoder(turningMotorEncoderChannel);

    // setting
    m_driveMotor.configFactoryDefault();
    m_turningMotor.configFactoryDefault();

    // voltage compensation
    m_driveMotor.enableVoltageCompensation(true);
    m_driveMotor.configVoltageCompSaturation(9);
    m_turningMotor.enableVoltageCompensation(true);
    m_turningMotor.configVoltageCompSaturation(9);

    // sensor
    
    m_turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    m_turningEncoder.configSensorDirection(turningEncoderReversed);
    m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    m_turningEncoder.configFeedbackCoefficient(0.087890625, "deg", SensorTimeBase.PerSecond);

    m_turningMotor.configRemoteFeedbackFilter(m_turningEncoder, 0);
    m_turningMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    
    // m_turningMotor.setSelectedSensorPosition(m_turningEncoder.getAbsolutePosition());

    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_driveMotor.configSelectedFeedbackCoefficient(ModuleConstants.kDriveCoefficient);

    m_turningMotor.setSelectedSensorPosition(m_turningEncoder.getAbsolutePosition() / (360.0 / 2048.0));

    m_driveMotor.setInverted(driveEncoderReversed);
    m_driveMotor.setSensorPhase(driveEncoderReversed);

    m_driveMotor.configNeutralDeadband(0.08);
    m_turningMotor.configNeutralDeadband(0.08);

    m_turningMotor.setInverted(driveEncoderReversed);
    m_turningMotor.setSensorPhase(turningEncoderReversed);

    // current limit
    var currentLimit = new StatorCurrentLimitConfiguration();
    currentLimit.enable = true;
    currentLimit.triggerThresholdTime = 25;
    currentLimit.triggerThresholdCurrent = 30;
    currentLimit.triggerThresholdTime = 0.1;

    m_driveMotor.configStatorCurrentLimit(currentLimit);
    m_turningMotor.configStatorCurrentLimit(currentLimit);

    // PIDF
    m_driveMotor.config_kF(0, 0.1);
    m_driveMotor.config_kP(0, 0);
    m_driveMotor.config_kI(0, 0);
    m_driveMotor.config_kD(0, 0);

    m_turningMotor.config_kF(0, 0.3);
    m_turningMotor.config_kP(0, 1.5);
    m_turningMotor.config_kI(0, 0);
    m_turningMotor.config_kD(0, 0);

    m_turningMotor.configMotionAcceleration(1000);
    m_turningMotor.configMotionCruiseVelocity(1000);

    // close loop setting
    // m_driveMotor.configClosedloopRamp(0.3);
    // m_turningMotor.configClosedloopRamp(0.5);
    // m_driveMotor.configMaxIntegralAccumulator(0, 2000);
    // m_turningMotor.configMaxIntegralAccumulator(0, 500);
    m_turningMotor.configClosedLoopPeakOutput(0, 1);


  }

  // unit: meter
  public double getDriveEncoderPosition() {
    return m_driveMotor.getSelectedSensorPosition();
  }

  public double getDriveEncoderVelocity() {
    return m_driveMotor.getSelectedSensorVelocity();
  }

  public double getTurningEncoderAngle() {
    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    return m_turningMotor.getSelectedSensorPosition() * (360 / 2048);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveEncoderVelocity(), new Rotation2d(getTurningEncoderAngle()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveEncoderPosition(), new Rotation2d(getTurningEncoderAngle()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningEncoderAngle()));

    // state = new SwerveModuleState(0.3, Rotation2d.fromDegrees(0));
    SmartDashboard.putNumber("desired(raw)", state.angle.getDegrees() / (360.0 / 2048.0));
    SmartDashboard.putNumber("desired(deg)", state.angle.getDegrees());
    SmartDashboard.putNumber("raw", m_turningMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("absolute", m_turningEncoder.getAbsolutePosition());
    // SmartDashboard.putNumber("", getDriveEncoderPosition())
    m_driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond / ModuleConstants.kDriveCoefficient);
    m_turningMotor.set(ControlMode.MotionMagic, state.angle.getDegrees() / (360.0 / 2048.0));
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0);
    m_turningMotor.setSelectedSensorPosition(0);
  }
}
