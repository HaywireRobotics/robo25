// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.TuneSwerveAutonomousCommand;
import frc.robot.subsystems.DorsalFin;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
  private final CommandXboxController m_driveController = new CommandXboxController(0);
  private final DorsalFin m_dorsalFin;

  public final DefaultDriveCommand defaultDriveCommand;
  public final TuneSwerveAutonomousCommand tuneSwerveAutonomousCommand;
  private final SysIdRoutine sysidRoutine;

  public RobotContainer(Robot robot) {
    m_dorsalFin = new DorsalFin(robot);
    defaultDriveCommand = new DefaultDriveCommand(m_dorsalFin, m_driveController);
    m_dorsalFin.setDefaultCommand(defaultDriveCommand);
    tuneSwerveAutonomousCommand = new TuneSwerveAutonomousCommand(m_dorsalFin);
    sysidRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(m_dorsalFin::sysIdVoltageDrive,m_dorsalFin::driveLogs, m_dorsalFin)
    );
    configureBindings();
  }

  private void configureBindings() {
    if (kConstants.kEnableFeedforwardTuning) {
      m_driveController.a().whileTrue(this.sysIdDynamic(SysIdRoutine.Direction.kForward));
      m_driveController.b().whileTrue(this.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    }
  }

  public Command getAutonomousCommand() {
    return tuneSwerveAutonomousCommand;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysidRoutine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysidRoutine.dynamic(direction);
  }
}
