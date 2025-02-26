// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorCommand extends Command {
  private final Elevator m_elevator;
  private final double m_goal;

  /** Creates a new MoveElevatorCommand. */
  public MoveElevatorCommand(Elevator elevator, double goal) {
    addRequirements(elevator);
    
    m_elevator = elevator;
    m_goal = goal;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setPIDTarget(m_goal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.assemblyPeriodic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.atGoal();
  }
}
