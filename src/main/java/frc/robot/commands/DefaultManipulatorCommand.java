// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.kConstants;
import frc.robot.subsystems.Manipulator;
import frc.robot.wrappers.Controller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultManipulatorCommand extends Command {
  private final Manipulator m_manipulator;
  private final Controller m_controller;
  /** Creates a new DefaultManipulatorCommand. */
  public DefaultManipulatorCommand(Manipulator manipulator, Controller controller) {
    addRequirements(manipulator);
    m_manipulator = manipulator;
    m_manipulator.reset();
    m_controller = controller;
    m_manipulator.setPIDTarget(m_manipulator.getManipulatorPos());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double stickX = m_controller.getLeftX();
    double stickY = m_controller.getLeftY();

    if (Math.abs(stickX) > 0.5) {
      m_manipulator.setPIDTarget(kConstants.kManipulatorDownPoint - 0.3);
    }
    if (stickY < -0.5) {
      m_manipulator.setPIDTarget(kConstants.kManipulatorDownPoint - 0.5);
    }
    if (stickY > 0.5) {
      m_manipulator.setPIDTarget(kConstants.kManipulatorDownPoint);
    }

    m_manipulator.assemblyPeriodic();
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
