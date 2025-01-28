// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.kConstants;

public class FilterFeeder extends SubsystemBase {
  private final SparkMax m_intakeMotor;
  private final SparkMax m_intakeAssemblyMotor;
  private final SparkMax m_indexMotor;

  /** Creates a new FilterFeeder. */
  public FilterFeeder() {
    m_intakeMotor = new SparkMax(kConstants.kIntakeMotor, MotorType.kBrushless);
    m_intakeAssemblyMotor = new SparkMax(kConstants.kIntakeAssemblyMotor, MotorType.kBrushless);
    m_indexMotor = new SparkMax(kConstants.kIndexMotor, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void lowerIntakeAssembly() {

  }

  public void raiseIntakeAssembly() {

  }
  /**
   * Runs the intake assembly motor and checks if it is at hard limit.
   */
  public void assemblyPeriodic() {

  }

  public void enableIntakeMotor() {
    // m_intakeMotor.set
  }

  public void disableIntakeMotor() {

  }

  public boolean isIntakeMotorEnabled() {
    return false;
  }

  public void enableIndexMotor() {

  }

  public void disableIndexMotor() {

  }

  public boolean isIndexMotorEnabled() {
    return false;
  }
}
