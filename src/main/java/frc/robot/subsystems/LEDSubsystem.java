// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLEDBufferView[] m_views;
  /** Creates a new LedSubsystem. */
  public LEDSubsystem(AddressableLEDBufferView ...view) {
    m_views = view;

    setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Black"));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> {
      for (int i = 0; i < m_views.length; i++) {
        pattern.applyTo(m_views[i]);
      }
    });
  }

  public void setPattern(LEDPattern pattern) {
    for (int i = 0; i < m_views.length; i++) {
      pattern.applyTo(m_views[i]);
    }
  }
}
