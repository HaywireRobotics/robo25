// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WithoutRequirements extends Command {
  private final Command m_command;
  private boolean m_isFinished = false;

  /** Creates a new WithoutRequirements. */
  public WithoutRequirements(Command command) {
    final WithoutRequirements self = this;
    m_command = command.finallyDo(() -> {
      self.m_isFinished = true;
      System.out.println("Wrapped Command " + command.getName() + " ended.");
    });
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_command.schedule();
    m_isFinished = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_command.cancel();
      m_command.end(true);
    }
    // m_isFinished = true;
    System.out.println("Wrapped Command " + m_command.getName() + " ended. For real this time {" + m_isFinished + ", " + interrupted + "}");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
