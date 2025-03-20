// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LEDCameraAction extends InstantCommand {
  private final LEDSubsystem m_led;
  private final LEDPattern m_pattern;
  public LEDCameraAction(LEDSubsystem led, LEDPattern pattern) {
    addRequirements(led);
    m_led = led;
    m_pattern = pattern;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.setPattern(m_pattern);
  }
}
