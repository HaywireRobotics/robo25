// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.kConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveClawCommand extends Command {
  private final Manipulator m_manipulator;
  private final double m_setpoint;
  private final double m_extraTime;
  private boolean m_timerRunning = false;
  private final Timer m_extraTimeTimer = new Timer();

  /** Creates a new MoveClawCommand. */
  public MoveClawCommand(Manipulator manipulator, double setpoint, double extraTime) {
    addRequirements(manipulator);

    m_manipulator = manipulator;
    m_setpoint = (-setpoint) + kConstants.kManipulatorDownPoint;
    m_extraTime = extraTime;
  }

  public MoveClawCommand(Manipulator manipulator, double setpoint) {
    addRequirements(manipulator);

    m_manipulator = manipulator;
    m_setpoint = (-setpoint) + kConstants.kManipulatorDownPoint;
    m_extraTime = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulator.setPIDTarget(m_setpoint);
    m_timerRunning = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_manipulator.assemblyPeriodic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_manipulator.setPIDTarget(m_manipulator.getManipulatorPos());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = m_manipulator.atGoal();

    if (!finished) {
      m_timerRunning = false;
    }

    if (finished && !m_timerRunning) {
      m_extraTimeTimer.restart();
      m_timerRunning = true;
    }
    if (finished && m_timerRunning && m_extraTimeTimer.get() > m_extraTime) {
      return true;
    }
    return false;
  }
}
