// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DorsalFin;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowAprilTagCommand extends Command {
  private final DorsalFin m_dorsalFin;
  private final PhotonCamera m_camera;
  /** Creates a new FollowAprilTagCommand. */
  public FollowAprilTagCommand(DorsalFin dorsalFin, PhotonCamera camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dorsalFin);
    m_dorsalFin = dorsalFin;
    m_camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
