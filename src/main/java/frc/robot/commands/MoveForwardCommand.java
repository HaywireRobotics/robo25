// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DorsalFin;

public class MoveForwardCommand extends Command {
  private final DorsalFin m_dorsalFin;
  private final double m_targetDistance;
  private double m_startPosition;

  /** Creates a new Move1MeterCommand. 
   * Implicitly uses the FrontRight swerve as the distance calculator
  */
  public MoveForwardCommand(DorsalFin dorsalFin, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dorsalFin);
    m_dorsalFin = dorsalFin;
    m_targetDistance = distance;
  }

  // Called when the command is initially scheduled. 
  @Override
  public void initialize() {
    m_startPosition = m_dorsalFin.getSwerveDriveLocations()[1];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_dorsalFin.drive(0, 1, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dorsalFin.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final double frontLeftLocation = m_dorsalFin.getSwerveDriveLocations()[1];
    return Math.abs(m_startPosition-frontLeftLocation) > m_targetDistance;
  }
}
