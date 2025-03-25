// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.kConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.wrappers.Controller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StowCommand extends Command {
  private final Elevator m_elevator;
  private final Manipulator m_claw;
  private final Controller m_controller;

  private int m_state = 0;

  /** Creates a new StowCommand. */
  public StowCommand(Elevator elevator, Manipulator claw, Controller controller) {
    m_elevator = elevator;
    m_claw = claw;
    m_controller = controller;
    addRequirements(elevator, claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_state == 0) {
      m_claw.setPIDTarget((kConstants.kManipulatorDownPoint - kConstants.kManipulatorUpAngle)+0.1);
      if (m_claw.atGoal()) {
        m_state = 1;
      }
      m_claw.assemblyPeriodic();
    }
    if (m_state == 1) {
      m_elevator.setPIDTarget(0);
      if (m_elevator.atGoal()) {
        m_state = 2;
      }
      m_elevator.assemblyPeriodic();
    }
    if (m_state == 2) {
      m_claw.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.getByName(kConstants.kStopElevatorStowButton).getAsBoolean();
  }
}
