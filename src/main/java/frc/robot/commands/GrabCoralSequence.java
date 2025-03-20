// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.kConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSuperSystem;
import frc.robot.subsystems.Manipulator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabCoralSequence extends SequentialCommandGroup {
  private static final LEDPattern kGrabCoralPattern = LEDPattern.solid(Color.kBurlywood);
  
  /** Grabs a coral actually safely. */
  public GrabCoralSequence(Elevator elevator, Manipulator manipulator, LEDSuperSystem led) {
    final LEDSubsystem m_led = led.getTopElevatorSubsystem();
    addRequirements(m_led);
    addCommands(
      new InstantCommand(() -> m_led.setPattern(kGrabCoralPattern)),
      new PrintCommand("[COMMAND] Grab Coral Sequence Initalized"),
      new MoveElevatorCommand(elevator, kConstants.kElevatorGrabCoralPosition + 10),
      new MoveClawCommand(manipulator, 0, 0.1),
      new MoveElevatorCommand(elevator, kConstants.kElevatorGrabCoralPosition),
      new MoveElevatorCommand(elevator, kConstants.kElevatorGrabCoralPosition + 10),
      new MoveClawCommand(manipulator, 0.3)
    );
  }
}
