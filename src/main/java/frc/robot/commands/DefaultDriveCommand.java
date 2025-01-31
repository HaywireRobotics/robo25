package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Statics;
import frc.robot.subsystems.DorsalFin;




public class DefaultDriveCommand extends Command {
    private final DorsalFin m_subsystem;
    private final CommandXboxController controller;
    private boolean m_fieldRelative = false;

    //the buffer not only affects the "deadzone", 
    // but also prohibits small angles near the x/y axis
    private static final double JOYSTICK_DEADBAND = 0.1;
    public double teleopSpeedMultiplier = 1.6;

    public DefaultDriveCommand(DorsalFin subsystem, CommandXboxController xboxController) {
        this.m_subsystem = subsystem;
        this.controller = xboxController;

        addRequirements(subsystem);
        SmartDashboard.putBoolean("FieldRelative", m_fieldRelative);
    }

    

    @Override
    public void execute() {
        double rightX = controller.getRightX();
        double leftX = controller.getLeftX();
        double leftY = controller.getLeftY();

        m_fieldRelative = SmartDashboard.getBoolean("FieldRelative", m_fieldRelative);

        leftX = Statics.applyDeadband(leftX, JOYSTICK_DEADBAND);
        leftY = Statics.applyDeadband(leftY, JOYSTICK_DEADBAND);
        rightX = Statics.applyDeadband(rightX, JOYSTICK_DEADBAND);
        
        m_subsystem.drive(teleopSpeedMultiplier*leftX, teleopSpeedMultiplier*leftY, -teleopSpeedMultiplier*rightX,m_fieldRelative);
        
        /* Odometry */
        // m_subsystem.updateOdometry();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setAllToState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
