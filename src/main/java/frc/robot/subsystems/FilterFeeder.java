// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.kConstants;

public class FilterFeeder extends SubsystemBase {
  private final SparkMax m_intakeAssemblyMotor;
  private final DutyCycleEncoder m_encoder;
  
 
  private final ProfiledPIDController m_intakeAssemblyPIDController = new ProfiledPIDController(
    kConstants.kIntakeAssemblyKP,
    kConstants.kIntakeAssemblyKI,
    kConstants.kIntakeAssemblyKD,
    new TrapezoidProfile.Constraints(
      kConstants.kIntakeAssemblyMaxVelocity, 
      kConstants.kIntakeAssemblyMaxAcceleration
    )
  );

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Assembly Encoder Angle", getIntakeAssemblyEncoderPosition());
    SmartDashboard.putNumber("Intake Setpoint", m_intakeAssemblyPIDController.getGoal().position);
  }
  /** Creates a new FilterFeeder. */
  public FilterFeeder() {
    m_intakeAssemblyMotor = new SparkMax(kConstants.kIntakeAssemblyMotor, MotorType.kBrushless);
    m_intakeAssemblyPIDController.setTolerance(0.001, 0.001);
    m_intakeAssemblyPIDController.setGoal(kConstants.kIntakeAssemblyUpPoint);
    m_intakeAssemblyMotor.configure(kConstants.kNeoNominalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_encoder = new DutyCycleEncoder(2, 1, 0);
    m_encoder.setAssumedFrequency(975.6);
  }

  public void lowerIntakeAssembly() {
    m_intakeAssemblyPIDController.setGoal(kConstants.kIntakeAssemblyDownPoint);
  }

  public void raiseIntakeAssembly() {
    m_intakeAssemblyPIDController.setGoal(kConstants.kIntakeAssemblyUpPoint);
  }

  public void moveIntakeAssemblyToGrabAlgae() {
    m_intakeAssemblyPIDController.setGoal(kConstants.kIntakeAssemblyAlgaePoint);
  }

  public void bumpIntakeAssembly() {
    m_intakeAssemblyPIDController.setGoal(kConstants.kIntakeAssemblyBumpPoint);
  }

  public void setPIDTarget(double target) {
    m_intakeAssemblyPIDController.setGoal(target);
  }

  public void reset() {
    m_intakeAssemblyPIDController.reset(this.getIntakeAssemblyEncoderPosition());
  }

  /**
   * Runs the intake assembly motor and checks if it is at hard limit.
   */
  public void assemblyPeriodic() {
    double motorPower = m_intakeAssemblyPIDController.calculate(getIntakeAssemblyEncoderPosition());
    if (getIntakeAssemblyEncoderPosition() >= kConstants.kIntakeAssemblyDownPoint) {
      motorPower = Math.min(0, motorPower);
    }
    if (getIntakeAssemblyEncoderPosition() <= kConstants.kIntakeAssemblyUpPoint) {
      motorPower = Math.max(0, motorPower);
    }
    m_intakeAssemblyMotor.setVoltage(motorPower);
  }

  public boolean isIntakeAssemblyAtTarget() {
    return m_intakeAssemblyPIDController.atGoal();
  }

  public double getIntakeAssemblyEncoderPosition() {
    return m_encoder.get();
  }

  public void configure(SparkBaseConfig config) {
    m_intakeAssemblyMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
