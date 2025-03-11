// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.kConstants;
import frc.robot.subsystems.Climb;
import frc.robot.wrappers.Controller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultClimbCommand extends Command {
  private final Climb m_climb;
  private final Controller m_controller;
  private double m_position = 0;

  /** Creates a new DefaultClimbCommand. */
  public DefaultClimbCommand(Climb climb, Controller controller) {
    m_climb = climb;
    m_controller = controller;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getByName(kConstants.kClimbUpButton).getAsBoolean()) {
      m_position -= kConstants.kClimbChangeAmount;
    } else if (m_controller.getByName(kConstants.kClimbDownButton).getAsBoolean()) {
      m_position += kConstants.kClimbChangeAmount;
    }
    m_climb.setPIDTarget(m_position);
    m_climb.assemblyPeriodic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
