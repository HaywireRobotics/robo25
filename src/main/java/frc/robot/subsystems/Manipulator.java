// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  /** Creates a new Manipulator. */
  public Manipulator() {
    m_manipulatorMotor = new SparkMax(kConstants.kManipulatorMotor, MotorType.kBrushless);
    m_manipulatorPIDController.setTolerance(3, 2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getManipulatorPos(){
    return (m_manipulatorMotor.getEncoder().getPosition())*kConstants.kManipulatorRatio;
  }

  public void manipulatorGoTo0(){
    setPIDTarget(0);
  }


  public void setPIDTarget(double position){
    if(position < 0){
      position = 0;
    }
    else if(position > kConstants.kManipulatorMaxAngle){
      position = kConstants.kManipulatorMaxAngle;
    }

    m_manipulatorPIDController.setGoal(position);
  
  }

  public boolean atGoal(){
    return m_manipulatorPIDController.atGoal();
  }

  public void setMotorPower(double power){
    m_manipulatorMotor.setVoltage(power);
  }
  
  public void pidLoop(){
    setMotorPower(m_manipulatorPIDController.calculate(getManipulatorPos()));
  }
}
