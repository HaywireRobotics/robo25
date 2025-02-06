// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Statics;
import frc.robot.subsystems.DorsalFin;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowAprilTagCommand extends Command {
  private final DorsalFin m_dorsalFin;
  private final PhotonCamera m_camera;
  private final Robot m_robot;
  private final LTVUnicycleController m_controller;
  private int m_countNotSeen = 0;
  /** Creates a new FollowAprilTagCommand. */
  public FollowAprilTagCommand(DorsalFin dorsalFin, PhotonCamera camera, Robot robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dorsalFin);
    m_dorsalFin = dorsalFin;
    m_camera = camera;
    m_robot = robot;
    m_controller = new LTVUnicycleController(m_robot.getPeriod());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_countNotSeen = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate April Tag position
    Transform2d tagPosition = new Transform2d();
    boolean isTag = false;
    var results = m_camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          if (target.getFiducialId() == 2) {
            Transform3d tag3dpose = target.getBestCameraToTarget();
            tagPosition = new Transform2d(tag3dpose.getX(), tag3dpose.getY(), new Rotation2d(tag3dpose.getRotation().getZ()));
            isTag = true;
            m_countNotSeen = 0;
          }
        }
      }
    }
    if (!isTag) {
      if (m_countNotSeen > 25) {
        m_dorsalFin.drive(0, 0, 0, false);
      } else {
        m_countNotSeen += 1;
      }
      return;
    }
    /*
    // tagPosition.plus(kConstants.kCameraPosition); // TODO offset tagPosition by the camera position
    Pose2d worldTagPosition = m_dorsalFin.getPose2D().plus(tagPosition);
    SmartDashboard.putString("World Tag Position", worldTagPosition.toString());
    SmartDashboard.putString("Tag Position", tagPosition.toString());
    SmartDashboard.putString("Robot Position", m_dorsalFin.getPose2D().toString());
    
    // Run Controller
    double linearVelocity = m_dorsalFin.getLinearVelocity();
    double angularVelocity = m_dorsalFin.getAngularVelocity();
    ChassisSpeeds movement = m_controller.calculate(m_dorsalFin.getPose2D(), worldTagPosition, linearVelocity, angularVelocity);
    SmartDashboard.putString("Chassis Speed", movement.toString());
    // m_dorsalFin.drive(movement.vyMetersPerSecond, movement.vxMetersPerSecond, movement.omegaRadiansPerSecond, false);
    */
    m_dorsalFin.drive(tagPosition.getY()*5, (tagPosition.getX() - 1)*5, Statics.applyDeadband(-(tagPosition.getRotation().plus(Rotation2d.k180deg).getRadians()), 0.05), false);
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
