// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
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
  // private PIDController mRotorPID; 
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
      boolean turningEncoderReversed,
      boolean turningMotorReversed,
      boolean driveReversed,
      double cancoderOffset) {

    // initialize
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel);
    
    m_turningEncoder = new CANCoder(turningMotorEncoderChannel);

    // setting
    m_driveMotor.configFactoryDefault();
    m_turningMotor.configFactoryDefault();
    m_turningEncoder.configFactoryDefault();

    // voltage compensation
    m_driveMotor.enableVoltageCompensation(true);
    m_driveMotor.configVoltageCompSaturation(9);
    m_turningMotor.enableVoltageCompensation(true);
    m_turningMotor.configVoltageCompSaturation(9);

    // sensor
    m_turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turningEncoder.configFeedbackCoefficient(0.087890625, "deg", SensorTimeBase.PerSecond);
    m_turningEncoder.configSensorDirection(turningEncoderReversed);
    m_turningEncoder.configMagnetOffset(cancoderOffset);
    m_turningEncoder.setPositionToAbsolute();

    // set remote sensor
    m_turningMotor.configRemoteFeedbackFilter(m_turningEncoder, 0);
    m_turningMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 10);
    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.None, 1, 10);
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // reverse
    m_driveMotor.setInverted(driveReversed);
    m_driveMotor.setSensorPhase(driveEncoderReversed);

    m_turningMotor.setInverted(turningMotorReversed);

    // deadband
    m_driveMotor.configNeutralDeadband(0.1);
    m_turningMotor.configNeutralDeadband(0.1);

    // stator current limit
    var currentLimit = new StatorCurrentLimitConfiguration();
    currentLimit.enable = true;
    currentLimit.triggerThresholdTime = 25;
    currentLimit.triggerThresholdCurrent = 30;
    currentLimit.triggerThresholdTime = 0.1;

    m_driveMotor.configStatorCurrentLimit(currentLimit);
    m_turningMotor.configStatorCurrentLimit(currentLimit);

    // PIDF
    m_driveMotor.config_kF(0, 0.2);
    m_driveMotor.config_kP(0, 0);
    m_driveMotor.config_kI(0, 0);
    m_driveMotor.config_kD(0, 0);

    m_turningMotor.config_kF(0, 0.3);
    m_turningMotor.config_kP(0, 0.75);
    m_turningMotor.config_kI(0, 0);
    m_turningMotor.config_kD(0, 0.8);

    m_turningMotor.configMotionAcceleration(1500);
    m_turningMotor.configMotionCruiseVelocity(1500);

    // close loop setting
    m_turningMotor.configClosedLoopPeakOutput(0, 1);

    // mRotorPID = new PIDController(
    //   0.75, 0, 0.8
    // );
    // mRotorPID.enableContinuousInput(-180, 180);


  }
  // CANcoder to talon
  private double deg2raw(double deg) {
    return ((deg)) / (360.0 / 4096.0);
  }

  // m
  public double getDriveEncoderPosition() {
    return m_driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveCoefficient;
  }

  // m/s
  public double getDriveEncoderVelocity() {
    return m_driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveCoefficient;
  }

  // deg
  public double getTurningEncoderAngle() {
    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    return m_turningMotor.getSelectedSensorPosition() / 4096.0 * 360.0;
  }
  public double getTurningEncoderRaw() {
    return deg2raw(getTurningEncoderAngle());
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
    return new SwerveModulePosition(getDriveEncoderPosition(), new Rotation2d(getTurningEncoderAngle() / 180 * Math.PI));
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

    // state = new SwerveModuleState(0.3, Rotation2d.fromDegrees(90));
    // SmartDashboard.putNumber("", getDriveEncoderPosition())
    SmartDashboard.putNumber("target(deg)" + m_turningEncoder.getDeviceID(), (state.angle.getDegrees()));
    SmartDashboard.putNumber("target(raw)" + m_turningEncoder.getDeviceID(), deg2raw(state.angle.getDegrees()));
    // m -> raw
    m_driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond / ModuleConstants.kDriveCoefficient);
    // deg -> raw
    m_turningMotor.set(ControlMode.MotionMagic, deg2raw(state.angle.getDegrees()));
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0);
    m_turningMotor.setSelectedSensorPosition(0);
  }
}
