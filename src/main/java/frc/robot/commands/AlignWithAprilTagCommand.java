// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.databind.PropertyNamingStrategies.KebabCaseStrategy;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.kConstants;
import frc.robot.subsystems.DorsalFin;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.wrappers.Camera;
import frc.robot.wrappers.FieldLayout;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithAprilTagCommand extends Command {
  private final DorsalFin m_dorsalFin;
  private final Robot m_robot;
  private final Camera m_camera;
  private final HolonomicDriveController m_controller;

  private final Timer m_timer;
  private final double m_xOffset;
  private final boolean m_terminateAfterTime;


  private final FieldLayout m_fieldLayout = new FieldLayout(kConstants.kFieldAprilTagJSON);
  private Trajectory m_trajectory;
  private Field2d m_position = new Field2d();

  /** Creates a new AlignWithAprilTagCommand. */
  public AlignWithAprilTagCommand(DorsalFin dorsalFin, Robot robot, Camera camera, double xOffset, boolean terminateAfterTime) {
    addRequirements(dorsalFin);
    m_dorsalFin = dorsalFin;
    m_robot = robot;
    m_camera = camera;
    m_xOffset = xOffset;
    m_terminateAfterTime = terminateAfterTime;

    final ProfiledPIDController headingController = new ProfiledPIDController(5, 0.1, 0, new TrapezoidProfile.Constraints(6.28, 6.28));
    m_controller = new HolonomicDriveController(
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      headingController
    );
    m_timer = new Timer();
    m_timer.stop();
    m_timer.reset();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    PhotonTrackedTarget target = m_camera.getBestAprilTag();

    if (target == null) {
      this.cancel();
      return;
    }

    m_trajectory = TrajectoryGenerator.generateTrajectory(
      m_dorsalFin.getPose2D(),
      new ArrayList<Translation2d>(0),
      m_fieldLayout.getTag(target.fiducialId)
      .translate(new Transform2d(0, 0, Rotation2d.k180deg))
      .translate(new Transform2d(-(kConstants.kRobotLength/2.0), m_xOffset, Rotation2d.kZero)).toPose(),
      new TrajectoryConfig(0.5, 1));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State reference = m_trajectory.sample(m_timer.get());
    m_position.setRobotPose(reference.poseMeters);
    SmartDashboard.putData("TrajectoryOutput", m_position);
    Rotation2d targetRotation = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters.getRotation();
    ChassisSpeeds movement = m_controller.calculate(m_dorsalFin.getPose2D(), reference, targetRotation);
    movement.omegaRadiansPerSecond = -movement.omegaRadiansPerSecond;
    m_dorsalFin.drive(movement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dorsalFin.drive(0, 0, 0, false);
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_terminateAfterTime) {
      return (m_trajectory.getTotalTimeSeconds()+1) > m_timer.get();
    }
    return false;
  }
}
