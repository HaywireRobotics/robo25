// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.DefaultFilterFeederCommand;
import frc.robot.commands.TuneSwerveAutonomousCommand;
import frc.robot.subsystems.DorsalFin;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FilterFeeder;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
  private final CommandXboxController m_driveController = new CommandXboxController(0);
  //define SUBSYSTEMS!!!
  private final DorsalFin m_dorsalFin;
 // private final Elevator m_elevator;
  //private final FilterFeeder m_filterFeeder;

//DEFINE default COMMAND?  
  public final DefaultDriveCommand defaultDriveCommand;
  //public final DefaultElevatorCommand defaultElevatorCommand;
//  public final DefaultFilterFeederCommand defaultFilterFeederCommand;

  public final TuneSwerveAutonomousCommand tuneSwerveAutonomousCommand;
  private final SysIdRoutine sysidRoutine;

  public RobotContainer(Robot robot) {
    m_dorsalFin = new DorsalFin(robot);
   // m_filterFeeder = new FilterFeeder();
    //m_elevator = new Elevator();

    defaultDriveCommand = new DefaultDriveCommand(m_dorsalFin, m_driveController);
   // defaultElevatorCommand = new DefaultElevatorCommand(m_elevator);
    //defaultFilterFeederCommand = new DefaultFilterFeederCommand(m_filterFeeder);

    m_dorsalFin.setDefaultCommand(defaultDriveCommand);
   // m_elevator.setDefaultCommand(defaultElevatorCommand);
   //m_filterFeeder.setDefaultCommand(defaultFilterFeederCommand);

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
