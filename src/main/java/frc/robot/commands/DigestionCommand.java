// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Stomach;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DigestionCommand extends Command {
  private final Stomach m_stomach;
  private final DigitalInput m_limitSwitch; // Normally open

  /** Creates a new DigestionCommand. */
  public DigestionCommand(Stomach stomach, DigitalInput limitSwitch) {
    addRequirements(stomach);

    m_stomach = stomach;
    m_limitSwitch = limitSwitch;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_stomach.enableIndexMotor();
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
