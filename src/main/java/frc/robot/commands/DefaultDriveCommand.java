package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Statics;
import frc.robot.wrappers.Controller;
import frc.robot.kConstants;
import frc.robot.subsystems.DorsalFin;




public class DefaultDriveCommand extends Command {
    private final DorsalFin m_subsystem;
    private final Controller m_controller;
    private boolean m_fieldRelative = false;

    //the buffer not only affects the "deadzone", 
    // but also prohibits small angles near the x/y axis
    private static final double JOYSTICK_DEADBAND = 0.1;
    public double teleopSpeedMultiplier = 1.6;

    public DefaultDriveCommand(DorsalFin subsystem, Controller xboxController) {
        this.m_subsystem = subsystem;
        this.m_controller = xboxController;

        addRequirements(subsystem);
    }

    

    @Override
    public void execute() {
        double rightX = m_controller.getRightX();
        double leftX = m_controller.getLeftX();
        double leftY = m_controller.getLeftY();

        if (m_controller.getByName(kConstants.kDisableFieldCentricButton).getAsBoolean()) {
            m_fieldRelative = false;
        }
        if (m_controller.getByName(kConstants.kEnableFieldCentricButton).getAsBoolean()) {
            m_fieldRelative = true;
        }

        leftX = Statics.applyDeadband(leftX, JOYSTICK_DEADBAND);
        leftY = Statics.applyDeadband(leftY, JOYSTICK_DEADBAND);
        rightX = Statics.applyDeadband(rightX, JOYSTICK_DEADBAND);

        double slowdownValue = (kConstants.kSlowModeDivider - m_controller.getLeftTriggerAxis()) / kConstants.kSlowModeDivider;
        
        leftX = leftX * slowdownValue;
        leftY = leftY * slowdownValue;
        rightX = rightX * slowdownValue;


        m_subsystem.drive(kConstants.kNavigationMultiplier*leftX, kConstants.kNavigationMultiplier*leftY, -kConstants.kRotationMultiplier*rightX,m_fieldRelative);
        
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
