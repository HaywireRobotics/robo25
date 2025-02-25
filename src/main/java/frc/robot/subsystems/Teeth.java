// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.kConstants;

public class Teeth extends SubsystemBase {
  private final SparkMax m_intakeMotor;
  private boolean m_isEnabled = false;
  
  /** Creates a new Teeth. */
  public Teeth() {
    m_intakeMotor = new SparkMax(kConstants.kIntakeMotor, MotorType.kBrushless);
    m_intakeMotor.configure(kConstants.kNeoNominalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runIntake() {
    m_intakeMotor.setVoltage(kConstants.kEnableIntakeVoltage);
    m_isEnabled = true;
  }

  public void stopIntake() {
    m_intakeMotor.setVoltage(0);
    m_isEnabled = false;
  }

  public boolean isIntakeEnabled() {
    return m_isEnabled;
  }
}
