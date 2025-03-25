// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DorsalFin;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveAtSpeedCommand extends Command {
  private final DorsalFin m_dorsalFin;
  private final ChassisSpeeds m_speed;

  /** Creates a new DriveAtSpeedCommand. */
  public DriveAtSpeedCommand(DorsalFin dorsalFin, ChassisSpeeds speed) {
    m_dorsalFin = dorsalFin;
    m_speed = speed;

    addRequirements(dorsalFin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_dorsalFin.drive(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_dorsalFin.drive(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dorsalFin.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
