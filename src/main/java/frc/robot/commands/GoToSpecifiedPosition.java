// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.DorsalFin;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToSpecifiedPosition extends Command {
  private final DorsalFin m_dorsalFin;
  private final Robot m_robot;
  private final LTVUnicycleController m_controller;
  private Trajectory m_trajectory;
  private final Timer m_timer;
  /** Creates a new GoToSpecifiedPosition. */
  public GoToSpecifiedPosition(DorsalFin dorsalFin, Robot robot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dorsalFin);
    m_dorsalFin = dorsalFin;
    m_robot = robot;
    m_controller = new LTVUnicycleController(m_robot.getPeriod());
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    final ArrayList<Translation2d> waypoints = new ArrayList<>(2);
    waypoints.add(new Translation2d(0.5, 0.5));
    m_trajectory = TrajectoryGenerator.generateTrajectory(m_dorsalFin.getPose2D(), waypoints, new Pose2d(1, 1, new Rotation2d(0)), new TrajectoryConfig(0.5, 1));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
    Trajectory.State reference = m_trajectory.sample(m_timer.get());
    ChassisSpeeds movement = m_controller.calculate(m_dorsalFin.getPose2D(), reference);
    // movement.omegaRadiansPerSecond/(Math.PI * 2)
    m_dorsalFin.drive(movement.vyMetersPerSecond, movement.vxMetersPerSecond, 0, false);
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
