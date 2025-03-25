// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSuperSystem extends SubsystemBase {
  private static final int kPort = 0;
  private static final int kLength = 95;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  // private final AddressableLEDBufferView m_leftElevator;
  // private final AddressableLEDBufferView m_topElevator;
  // ?
  private final LEDSubsystem m_elevatorSides;
  private final LEDSubsystem m_topElevator;

  /** Creates a new LEDSubsystem. */
  public LEDSuperSystem() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    m_elevatorSides = new LEDSubsystem(m_buffer.createView(0, 26), m_buffer.createView(41, 62).reversed());
    m_topElevator  = new LEDSubsystem(m_buffer.createView(27, 40));
  }
 // pls d
  @Override
  public void periodic() {
    m_led.setData(m_buffer);
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> {pattern.applyTo(m_buffer);});
  }

  public void setPattern(LEDPattern pattern) {
    pattern.applyTo(m_buffer);
  }

  // Individual Sections
  public LEDSubsystem getElevatorSidesSubsystem() {
    return m_elevatorSides;
  }
  public LEDSubsystem getTopElevatorSubsystem() {
    return m_topElevator;
  }
}
