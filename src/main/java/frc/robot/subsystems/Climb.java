// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.kConstants;

public class Climb extends SubsystemBase {
  private final SparkMax m_climbMotor;
  private final PIDController m_climbController = new PIDController(
    kConstants.kClimbKP,
    kConstants.kClimbKI,
    kConstants.kClimbKD
  );

  /** Creates a new ClimbSubsystem. */
  public Climb() {
    m_climbMotor = new SparkMax(kConstants.kClimbMotor, MotorType.kBrushless);
    m_climbMotor.configure(kConstants.kNeoNominalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_climbController.setTolerance(0.1, 0.1);
  }

  public double getPosition() {
    return m_climbMotor.getEncoder().getPosition() * kConstants.kClimbRatio;
  }

  public void setPIDTarget(double target) {
    m_climbController.setSetpoint(target);
  }

  public void assemblyPeriodic() {
    m_climbMotor.setVoltage(m_climbController.calculate(getPosition()));
  }

  public boolean atGoal() {
    return m_climbController.atSetpoint();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb Position: ", getPosition());
  }
}
