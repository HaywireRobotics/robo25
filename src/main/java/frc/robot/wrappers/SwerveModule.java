package frc.robot.wrappers;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.kConstants;

public class SwerveModule {

  private static final double kModuleMaxAngularVelocity = kConstants.kMaxAngularVelocity;
  private static final double kModuleMaxAngularAcceleration = kConstants.kMaxAngularAcceleration; // radians per second squared

  private final SparkMax m_driveMotor;

  private final SparkMax m_turningMotor;

  private final CANcoder m_turningEncoder;

  private final double kTurningEncoderOffsetRotations;

  private final String m_idForDashboard;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(
    kConstants.kSwerveDriveKP, 
    kConstants.kSwerveDriveKI, 
    kConstants.kSwerveDriveKD
  );
  private double appliedDriveVoltage = 0;
  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
        kConstants.kSwerveTurningKP,
        kConstants.kSwerveTurningKI,
        kConstants.kSwerveTurningKD,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(kConstants.kSwerveDriveKS, kConstants.kSwerveDriveKV, kConstants.kSwerveDriveKA);
  // private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(kConstants.kSwerveTurningKS, kConstants.kSwerveTurningKV);
  
  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param turningEncoderChannel DIO input for the turning encoder channel
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double offset) {
    m_driveMotor = new SparkMax(driveMotorChannel,MotorType.kBrushless);
    m_driveMotor.configure(kConstants.kSwerveNominalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningMotor = new SparkMax(turningMotorChannel,MotorType.kBrushless);
    m_turningMotor.configure(kConstants.kSwerveNominalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_turningEncoder = new CANcoder(turningEncoderChannel);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    kTurningEncoderOffsetRotations = offset;

    m_idForDashboard = String.valueOf(driveMotorChannel);

    SmartDashboard.putData("SwerveDrivePID " + m_idForDashboard, m_drivePIDController);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveEncoderPosition(), new Rotation2d(getTurningEncoderPosition()));
  }

  /**
   * Returns the calculated position of the turningEncoder
   * 
   * @return the calculated position of the turningEncoder
   */
  public double getTurningEncoderPosition() {
    // -0.5 - 0.5
    // -pi - pi
    // 0 - 2pi
    // Make sure it is 0 - 2pi
    // -pi - pi
    // return Statics.trueMod((((m_turningEncoder.getPosition().getValueAsDouble() - kTurningEncoderOffsetRotations)*Math.PI*2)+Math.PI), (Math.PI*2)) - Math.PI;
    return (m_turningEncoder.getPosition().getValueAsDouble()-kTurningEncoderOffsetRotations)*2*Math.PI;
  }

  /**
   * Returns the calculated position of the driveEncoder
   * 
   * @return the calculated position of the driveEncoder
   */
  public double getDriveEncoderPosition() {
    return m_driveMotor.getEncoder().getPosition();
    
  }

  /**
   * Returns the calculated velocity of the driveEncoder in rotations per minute
   *
   * @return the calculated velocity of the driveEncoder
   */
  public double getDriveEncoderVelocityRPM() {
    return m_driveMotor.getEncoder().getVelocity();
  }

  /**
   * Returns the calculated velocity of the driveEncoder in rotations per second
   *
   * @return the calculated velocity of the driveEncoder
   */
  public double getDriveEncoderVelocityRPS() {
    return m_driveMotor.getEncoder().getVelocity() / 60.0;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      applyDriveRatio(getDriveEncoderPosition()), new Rotation2d(getTurningEncoderPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredVoltage(SwerveModuleState desiredState) {
    final double[] wrappedValues = calculatePIDValuesFromDesiredState(desiredState);
    final double turnOutput = wrappedValues[0];
    driveMotorsAtVoltage(desiredState.speedMetersPerSecond, turnOutput);
}

  public void setDesiredState(SwerveModuleState desiredState) {
    

    final double[] wrappedValues = calculatePIDValuesFromDesiredState(desiredState);
    final double turnOutput = wrappedValues[0];
    final double driveOutput = wrappedValues[1];
    driveMotorsAtVoltage(driveOutput, turnOutput);
  }
  
  private double[] calculatePIDValuesFromDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(getTurningEncoderPosition());
    desiredState.optimize(encoderRotation); // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.cosineScale(encoderRotation); // Smooths driving (look at docs for more)
    // Calculate the drive output from the drive PID controller.
    
    final double driveOutput = m_drivePIDController.calculate(getDriveVelocity(), desiredState.speedMetersPerSecond);
    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("SwervePIDOutput " + m_idForDashboard, driveOutput); // this is a debugging readout
    SmartDashboard.putNumber("SwerveFeedforwardOutput " + m_idForDashboard, driveFeedforward); // this is a debugging readout
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(
          getTurningEncoderPosition(), MathUtil.angleModulus(desiredState.angle.getRadians()));

    // Turn Feed Forward was found to be not needed for Dyson
    // final double turnFeedforward =
    //     m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    final double[] output = {turnOutput,driveOutput + driveFeedforward};
    return output;
  }

  private void driveMotorsAtVoltage(double driveOutput, double turnOutput) {
    if (kConstants.kEnableFeedforwardTuning) appliedDriveVoltage = driveOutput;
    m_driveMotor.setVoltage(driveOutput);
    SmartDashboard.putNumber("SwerveDriveMotorVoltage " + m_idForDashboard, driveOutput);
    SmartDashboard.putNumber("SwerveDriveVelocity " + m_idForDashboard, getDriveVelocity());
    m_turningMotor.setVoltage(turnOutput);
    SmartDashboard.putNumber("SwerveTurnMotorVoltage " + m_idForDashboard, turnOutput);
    
  }
  public void periodic(){
    SmartDashboard.putNumber("SwerveDriveMotorPosition " + m_idForDashboard, getDriveEncoderPosition());
  }
  public double getDriveVelocity() {
    return applyDriveRatio(getDriveEncoderVelocityRPS());
  }
  public double applyDriveRatio(double encoderValue){
    return encoderValue / kConstants.kDriveGearRatio * kConstants.kWheelDiameter * Math.PI * kConstants.kInchesToMeters;
  }
  public double getDrivePosition(){
    return applyDriveRatio(getDriveEncoderPosition());
  }
  public double getDriveVoltage(){
    return appliedDriveVoltage;
  }
}