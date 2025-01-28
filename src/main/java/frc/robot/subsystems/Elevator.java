// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.kConstants;

public class Elevator extends SubsystemBase {

   private final SparkMax m_elevatorMotor;
   private final ProfiledPIDController m_elevatorPIDController =
      new ProfiledPIDController(
        kConstants.kElevatorKP,
        kConstants.kElevatorKI,
        kConstants.kElevatorKD,
          new TrapezoidProfile.Constraints(
              kConstants.kElevatorMaxVelocity, kConstants.kElevatorMaxAcceleration));

  /** Creates a new Elevator. */
  public Elevator() {
    m_elevatorMotor = new SparkMax(kConstants.kElevatorMotor, MotorType.kBrushless);
    m_elevatorPIDController.setTolerance(3, 2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getElevatorPos(){
    return (m_elevatorMotor.getEncoder().getPosition())*kConstants.kElevatorRatio;
  }

  public void elevatorGoTo0(){
    m_elevatorMotor.getEncoder();
  }

  public void setPIDTarget(double position){
    m_elevatorPIDController.setGoal(position);
  
  }

  public boolean atGoal(){
    return m_elevatorPIDController.atGoal();
  }

  public void setMotorPower(double power){
    m_elevatorMotor.setVoltage(power);
  }
  
  public void pidLoop(){
    setMotorPower(m_elevatorPIDController.calculate(getElevatorPos()));
  }
}
