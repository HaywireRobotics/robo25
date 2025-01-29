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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.kConstants;

public class FilterFeeder extends SubsystemBase {
  private final SparkMax m_intakeMotor;
  private final SparkMax m_intakeAssemblyMotor;
  private final SparkMax m_indexMotor;
 
  private final ProfiledPIDController m_intakeAssemblyPIDController = new ProfiledPIDController(
      kConstants.kIntakeAssemblyKP,
      kConstants.kIntakeAssemblyKI,
      kConstants.kIntakeAssemblyKD,
      new TrapezoidProfile.Constraints(
          kConstants.kIntakeAssemblyMaxVelocity, kConstants.kIntakeAssemblyMaxAcceleration));

  private boolean isIntakeEnabled = false;
  private boolean isIndexEnabled = false;

  /** Creates a new FilterFeeder. */
  public FilterFeeder() {
    m_intakeMotor = new SparkMax(kConstants.kIntakeMotor, MotorType.kBrushless);
    m_intakeAssemblyMotor = new SparkMax(kConstants.kIntakeAssemblyMotor, MotorType.kBrushless);
    m_indexMotor = new SparkMax(kConstants.kIndexMotor, MotorType.kBrushless);
    m_intakeAssemblyPIDController.setTolerance(3, 2);
    m_intakeAssemblyMotor.configure(kConstants.kIntakeAssemblyNominalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_intakeMotor.configure(kConstants.kDefaultNeo550NominalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_indexMotor.configure(kConstants.kDefaultNeo550NominalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void lowerIntakeAssembly() {
    m_intakeAssemblyPIDController.setGoal(kConstants.kIntakeAssemblyDownPoint);
  }

  public void raiseIntakeAssembly() {
    m_intakeAssemblyPIDController.setGoal(kConstants.kIntakeAssemblyUpPoint);
  }

  /**
   * Runs the intake assembly motor and checks if it is at hard limit.
   */
  public void assemblyPeriodic() {
    double motorPower = m_intakeAssemblyPIDController.calculate(getIntakeAssemblyEncoderPosition());
    m_intakeAssemblyMotor.setVoltage(motorPower);
    if (getIntakeAssemblyEncoderPosition() <= kConstants.kIntakeAssemblyDownPoint) {
      m_intakeAssemblyMotor.setVoltage(0);

    }
    if (getIntakeAssemblyEncoderPosition() >= kConstants.kIntakeAssemblyUpPoint) {
      m_intakeAssemblyMotor.setVoltage(0);
    }
  }

  public double getIntakeAssemblyEncoderPosition() {

    return m_intakeAssemblyMotor.getEncoder().getPosition();

  }

  public void enableIntakeMotor() {
    m_intakeMotor.setVoltage(kConstants.kEnableIntake);
    isIntakeEnabled = true;
  }

  public void disableIntakeMotor() {
    m_intakeMotor.setVoltage(0);
    isIntakeEnabled = false;
  }

  public boolean isIntakeMotorEnabled() {
    return isIntakeEnabled;
  }

  public void enableIndexMotor() {
    m_indexMotor.setVoltage(kConstants.kEnableIndex);
    isIndexEnabled = true;
  }

  public void disableIndexMotor() {
    m_indexMotor.setVoltage(0);
    isIndexEnabled = false;
  }

  public boolean isIndexMotorEnabled() {
    return isIndexEnabled;
  }
}
