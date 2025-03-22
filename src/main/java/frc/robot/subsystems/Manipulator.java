// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.kConstants;

public class Manipulator extends SubsystemBase {

   private final SparkMax m_manipulatorMotor;
   private final ProfiledPIDController m_manipulatorPIDController =
      new ProfiledPIDController(
        kConstants.kManipulatorKP,
        kConstants.kManipulatorKI,
        kConstants.kManipulatorKD,
          new TrapezoidProfile.Constraints(
              kConstants.kManipulatorMaxVelocity, kConstants.kManipulatorMaxAcceleration));

  private final DutyCycleEncoder m_encoder;


  /** Creates a new Manipulator. */
  public Manipulator() {
    m_manipulatorMotor = new SparkMax(kConstants.kManipulatorMotor, MotorType.kBrushless);
    m_manipulatorMotor.configure(kConstants.kNeo550HighStallCurrentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_manipulatorPIDController.setTolerance(0.01, 0.01);
    m_manipulatorPIDController.setGoal(kConstants.kManipulatorDownPoint - 0.375);

    m_encoder = new DutyCycleEncoder(kConstants.kManipulatorEncoderID, 1, 0);
    m_encoder.setAssumedFrequency(975.6);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Manipulator Angle", this.getManipulatorPos());
    SmartDashboard.putNumber("Manipulator Setpoint", m_manipulatorPIDController.getGoal().position);
  }

  public double getManipulatorPos(){
    return m_encoder.get();
  }

  public double getRawManipulatorPos() {
    return m_manipulatorMotor.getEncoder().getPosition();
  }

  public boolean isEncoderConnected(){
    return m_encoder.isConnected();
  }

  public void reset() {
    m_manipulatorPIDController.reset(this.getManipulatorPos());
  }

  public void setPIDTarget(double position){
    if (position > kConstants.kManipulatorDownPoint){
      position = kConstants.kManipulatorDownPoint;
    }
    if (position < kConstants.kManipulatorDownPoint - 0.5){
      position = kConstants.kManipulatorDownPoint - 0.5;
    }
    m_manipulatorPIDController.setGoal(position);
  }

  public boolean atGoal(){
    return m_manipulatorPIDController.atGoal();
  }

  public void setMotorPower(double power) {
    m_manipulatorMotor.setVoltage(power * kConstants.kManipulatorPowerMultiplier);
  }
  
  public void assemblyPeriodic() {
    double power = m_manipulatorPIDController.calculate(this.getManipulatorPos());
    double position = this.getManipulatorPos();

    if (position > kConstants.kManipulatorDownPoint) {
      power = Math.min(0, power);
    }
    if (position < kConstants.kManipulatorDownPoint - 0.4444) {
      power = Math.max(0, power);
    }

    this.setMotorPower(power);
  }

  public void configure(SparkBaseConfig config) {
    m_manipulatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
