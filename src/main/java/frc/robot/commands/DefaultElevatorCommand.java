// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.kConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSuperSystem;
import frc.robot.wrappers.Controller;
import frc.robot.wrappers.PositionMemory;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultElevatorCommand extends Command {
  /** Creates a new DefaultElevatorCommand. */
  private final Elevator m_elevator;
  private final PositionMemory m_position;

  private final LEDSuperSystem m_ledSuper;

  private static final LEDPattern kElevatorL2Pattern = LEDPattern.solid(Color.kRed);
  private static final LEDPattern kElevatorCoralPattern = LEDPattern.solid(Color.kOrange);
  private static final LEDPattern kElevatorL3Pattern = LEDPattern.solid(Color.kGreen);
  private static final LEDPattern kElevatorL4Pattern = LEDPattern.solid(Color.kBlue);

  private final LEDSubsystem m_led;


  public DefaultElevatorCommand(Elevator elevator, PositionMemory position, LEDSuperSystem led) {
    m_elevator = elevator;
    m_elevator.reset();
    m_position = position;
    m_ledSuper = led;
    m_led = m_ledSuper.getTopElevatorSubsystem();
    addRequirements(elevator, m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int level = m_position.get();
    if (level == 0) {
      m_elevator.setPIDTarget(kConstants.kElevatorScoreL2Position);
      m_led.setPattern(kElevatorL2Pattern);
    }
    if (level == 1) {
      m_elevator.setPIDTarget(kConstants.kElevatorGrabCoralPosition);
      m_led.setPattern(kElevatorCoralPattern);
    }
    if (level == 2) {
      m_elevator.setPIDTarget(kConstants.kElevatorScoreL3Position);
      m_led.setPattern(kElevatorL3Pattern);
    }
    if (level == 3) {
      m_elevator.setPIDTarget(kConstants.kElevatorScoreL4Position);
      m_led.setPattern(kElevatorL4Pattern);
    }

    m_elevator.assemblyPeriodic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
