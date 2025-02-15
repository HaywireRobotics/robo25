// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.kConstants;
import frc.robot.Robot;
import frc.robot.wrappers.SwerveModule;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import com.kauailabs.navx.frc.AHRS;

/** Represents a swerve drive style drivetrain. */
public class DorsalFin extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(kConstants.kDriveTrainLength/2, kConstants.kDriveTrainWidth/2);
  private final Translation2d m_frontRightLocation = new Translation2d(kConstants.kDriveTrainLength/2, -kConstants.kDriveTrainWidth/2);
  private final Translation2d m_backLeftLocation = new Translation2d(-kConstants.kDriveTrainLength/2, kConstants.kDriveTrainWidth/2);
  private final Translation2d m_backRightLocation = new Translation2d(-kConstants.kDriveTrainLength/2, -kConstants.kDriveTrainWidth/2);

  private final SwerveModule m_frontLeft = new SwerveModule(
    kConstants.kSwerveFrontLeftDriveMotor, 
    kConstants.kSwerveFrontLeftTurnMotor, 
    kConstants.kSwerveFrontLeftEncoder, 
    kConstants.kSwerveFrontLeftOffset);
  private final SwerveModule m_frontRight = new SwerveModule(
    kConstants.kSwerveFrontRightDriveMotor, 
    kConstants.kSwerveFrontRightTurnMotor, 
    kConstants.kSwerveFrontRightEncoder, 
    kConstants.kSwerveFrontRightOffset);
  private final SwerveModule m_backLeft = new SwerveModule(
    kConstants.kSwerveBackLeftDriveMotor, 
    kConstants.kSwerveBackLeftTurnMotor, 
    kConstants.kSwerveBackLeftEncoder, 
    kConstants.kSwerveBackLeftOffset);
  private final SwerveModule m_backRight = new SwerveModule(
    kConstants.kSwerveBackRightDriveMotor, 
    kConstants.kSwerveBackRightTurnMotor, 
    kConstants.kSwerveBackRightEncoder, 
    kConstants.kSwerveBackRightOffset);
  
  private final SwerveModule[] m_swerveModules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};

  private final String[] swerveModuleNames = {"front left", "back left", "front right", "back right"};
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
  private final AHRS m_gyro = new AHRS();

  private Rotation2d m_calibratedOffset = new Rotation2d();

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation,
          m_frontRightLocation,
          m_backLeftLocation,
          m_backRightLocation);

  private final SwerveDriveOdometry m_odometry;
  private Pose2d fieldPose = new Pose2d();

  private final Robot m_robot;

  public DorsalFin(Robot robot) {
    this.resetGyro();
    m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          getRotationAroundUpAxisInRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });
    m_robot = robot;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("Y Speed", ySpeed);
    SmartDashboard.putNumber("Rotations", rot);
    drive(fieldRelative ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rot, getRotationAroundUpAxisInRotation2d())
      :
      new ChassisSpeeds(ySpeed, xSpeed, rot)
    );
  }

  public void drive(ChassisSpeeds speed) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                speed,
                m_robot.getPeriod()));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void sysIdVoltageDrive(Voltage voltage){
    setAllToState(new SwerveModuleState(voltage.in(Volts), new Rotation2d(0)));
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    fieldPose = m_odometry.update(
        getRotationAroundUpAxisInRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public Pose2d getFieldPose(){
    return fieldPose;
  }

  public void setAllToState(SwerveModuleState state){
    m_frontLeft.setDesiredState(state);
    m_frontRight.setDesiredState(state);
    m_backLeft.setDesiredState(state);
    m_backRight.setDesiredState(state);
  }

  public void periodic(){
    for(SwerveModule module : m_swerveModules){
      module.periodic();
    }
  }

  public double[] getSwerveDriveLocations() {
    return new double[] {
      m_frontLeft.getPosition().distanceMeters,
      m_frontRight.getPosition().distanceMeters,
      m_backLeft.getPosition().distanceMeters,
      m_backRight.getPosition().distanceMeters
    };
  }

  public double getRotationAroundUpAxis() {
    return m_gyro.getYaw();
  }

  public Rotation2d getRotationAroundUpAxisInRotation2d() {
    return m_gyro.getRotation2d().minus(m_calibratedOffset);
  }

  public SysIdRoutineLog driveLogs(SysIdRoutineLog logs){
    for(int i = 0; i < m_swerveModules.length;i++){
      SwerveModule module = m_swerveModules[i];
      logs.motor("motor" + swerveModuleNames[i])
        .linearVelocity(m_velocity.mut_replace(module.getDriveVelocity(),MetersPerSecond))
        .voltage(m_appliedVoltage.mut_replace(module.getDriveVoltage(),Volts))
        .linearPosition(m_distance.mut_replace(module.getDrivePosition(),Meters));
    }
   return logs;
  }

  public Pose2d getPose2D() {
    return fieldPose;
  }

  public void resetGyro() {
    // m_gyro.reset();
    m_calibratedOffset = m_gyro.getRotation2d();
  }
  /* 
  public double getLinearVelocity() {

  }

  public double getAngularVelocity() {
    
  }
  */
}
