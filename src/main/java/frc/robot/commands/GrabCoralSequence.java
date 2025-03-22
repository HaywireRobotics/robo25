// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.kConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSuperSystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.wrappers.PositionMemory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabCoralSequence extends SequentialCommandGroup {
  private static final LEDPattern kGrabCoralPattern = LEDPattern.solid(Color.kBurlywood);
  
  /** Grabs a coral actually safely. */
  public GrabCoralSequence(Elevator elevator, Manipulator manipulator, LEDSuperSystem led, PositionMemory elevatorPosition) {
    final LEDSubsystem m_led = led.getTopElevatorSubsystem();
    addRequirements(m_led);
    addCommands(
      new InstantCommand(() -> m_led.setPattern(kGrabCoralPattern)),
      new PrintCommand("[COMMAND] Grab Coral Sequence Initalized"),
      new MoveElevatorCommand(elevator, kConstants.kElevatorGrabCoralPosition + 10),
      new PrintCommand("Elevator Up"),
      new MoveClawCommand(manipulator, 0, 0.1),
      new PrintCommand("Claw Down"),
      new MoveElevatorCommand(elevator, kConstants.kElevatorGrabCoralPosition),
      new PrintCommand("Elevator Down"),
      new MoveElevatorCommand(elevator, kConstants.kElevatorGrabCoralPosition + 10),
      new PrintCommand("Elevator Up"),
      new ParallelDeadlineGroup(
        new WaitCommand(Seconds.of(1)),
        new MoveClawCommand(manipulator, 0.3)
      ),
      new PrintCommand("Claw Up")
    );
  }
}
