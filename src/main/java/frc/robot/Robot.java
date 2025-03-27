// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_testCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    super();
    m_robotContainer = new RobotContainer(this);
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.updateOdometry();
    SmartDashboard.putData("fieldPose", m_robotContainer.updateFieldPose());
    m_robotContainer.putAllSmartDashboardData();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.startDisabled();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledPeriodic();
  }

  @Override
  public void disabledExit() {
    m_robotContainer.endDisabled();
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.reset();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.reset();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.periodic();
  }

  @Override
  public void teleopExit() {
    m_robotContainer.endTeleop();
  }

  @Override
  public void testInit() {
    m_robotContainer.reset();
    m_testCommand = m_robotContainer.getTestCommand();

    if (m_testCommand != null) {
      m_testCommand.schedule();
    }
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
