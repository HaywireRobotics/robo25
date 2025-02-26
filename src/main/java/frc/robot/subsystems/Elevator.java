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
    m_elevatorMotor.configure(kConstants.kNeoNominalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevatorPIDController.setTolerance(0.5, 0);
  }

  @Override
  public void periodic() {
    System.out.println(this.getElevatorPos());
  }

  public double getElevatorPos(){
    return (m_elevatorMotor.getEncoder().getPosition()) * kConstants.kElevatorRatio * kConstants.kElevatorRotationsToInches;
  }

  public void elevatorGoTo0() {
    setPIDTarget(0.1);
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
  
  public void assemblyPeriodic(){
    double motorPower = m_elevatorPIDController.calculate(getElevatorPos());
    double elevatorPos = this.getElevatorPos();
    
    if (elevatorPos > 60) {
      motorPower = Math.min(0, motorPower);
    }
    if (elevatorPos < 1) {
      motorPower = Math.max(0, motorPower);
    }

    setMotorPower(motorPower);
  }
}
