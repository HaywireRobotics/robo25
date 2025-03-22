// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.wrappers.PositionMemory;

import frc.robot.kConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPositionAndWaitCommand extends Command {
  private final Elevator m_elevator;
  private final PositionMemory m_memory;
  private final int m_position;

  /** Creates a new SetElevatorPositionAndWaitCommand. */
  public SetElevatorPositionAndWaitCommand(Elevator elevator, PositionMemory memory, int position) {
    m_elevator = elevator;
    m_memory = memory;
    m_position = position;

    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_memory.set(m_position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int level = m_memory.get();
    if (level == 0) {
      m_elevator.setPIDTarget(kConstants.kElevatorScoreL2Position);
    }
    if (level == 1) {
      m_elevator.setPIDTarget(kConstants.kElevatorGrabCoralPosition);
    }
    if (level == 2) {
      m_elevator.setPIDTarget(kConstants.kElevatorScoreL3Position);
    }
    if (level == 3) {
      m_elevator.setPIDTarget(kConstants.kElevatorScoreL4Position);
    }

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
