// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Stomach;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlternatingDigestionCommand extends Command {
  private final Stomach m_stomach;
  private final DigitalInput m_limitSwitch; // Normally open

  private final Timer m_timer;
  private boolean m_on = true;
  private final double m_onTime;
  private final double m_offTime;

  /** Creates a new AlternatingDigestionCommand. */
  public AlternatingDigestionCommand(Stomach stomach, DigitalInput limitSwitch, double onTime, double offTime) {
    addRequirements(stomach);

    m_stomach = stomach;
    m_limitSwitch = limitSwitch;

    m_timer = new Timer();
    m_onTime = onTime;
    m_offTime = offTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    m_on = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_on) {
      m_stomach.enableIndexMotor();
    } else {
      m_stomach.disableIndexMotor();
    }
    if (m_timer.get() > (m_on ? m_onTime : m_offTime)) {
      m_on = !m_on;
      m_timer.restart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_stomach.disableIndexMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_limitSwitch.get();
  }
}
