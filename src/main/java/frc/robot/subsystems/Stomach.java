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

public class Stomach extends SubsystemBase {
  private final SparkMax m_frontIndexMotor;
  // private final SparkMax m_backLeftIndexMotor;
  private final SparkMax m_backIndexMotor;

  private boolean isIndexEnabled = false;

  /** Creates a new Stomach. */
  public Stomach() {
    m_frontIndexMotor = new SparkMax(kConstants.kIndexMotor, MotorType.kBrushless);
    // m_backLeftIndexMotor = new SparkMax(kConstants.kIndexBackLeftMotor, MotorType.kBrushless);
    m_backIndexMotor = new SparkMax(kConstants.kIndexRightMotor, MotorType.kBrushless);
    m_frontIndexMotor.configure(kConstants.kNeo550NominalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // m_backLeftIndexMotor.configure(kConstants.kNeo550NominalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_backIndexMotor.configure(kConstants.kNeo550NominalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void enableIndexMotor() {
    m_frontIndexMotor.setVoltage(kConstants.kEnableFrontIndexVoltage);
    // m_backLeftIndexMotor.setVoltage(-kConstants.kEnableBackIndexVoltage);
    m_backIndexMotor.setVoltage(kConstants.kEnableBackIndexVoltage);
    isIndexEnabled = true;
  }

  public void reverseIndexMotor() {
    m_frontIndexMotor.setVoltage(-kConstants.kEnableFrontIndexVoltage);
    // m_backLeftIndexMotor.setVoltage(kConstants.kEnableBackIndexVoltage);
    m_backIndexMotor.setVoltage(-kConstants.kEnableBackIndexVoltage);
    isIndexEnabled = true;
  }

  public void disableIndexMotor() {
    m_frontIndexMotor.setVoltage(0);
    // m_backLeftIndexMotor.setVoltage(0);
    m_backIndexMotor.setVoltage(0);
    isIndexEnabled = false;
  }

  public boolean isIndexMotorEnabled() {
    return isIndexEnabled;
  }
}
