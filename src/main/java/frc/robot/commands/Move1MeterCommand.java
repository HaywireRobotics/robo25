// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DorsalFin;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Move1MeterCommand extends Command {
  private final DorsalFin m_dorsalFin;
  private double m_targetDistance;

  /** Creates a new Move1MeterCommand. 
   * Implicitly uses the FrontRight swerve as the distance calculator
  */
  public Move1MeterCommand(DorsalFin dorsalFin) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dorsalFin);
    m_dorsalFin = dorsalFin;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_targetDistance = m_dorsalFin.getSwerveDriveLocations()[1] + 1; // 1 Meter Forward
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
    return m_dorsalFin.getSwerveDriveLocations()[1] > this.m_targetDistance;
  }
}
