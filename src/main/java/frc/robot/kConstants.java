package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class kConstants {
    public static final double kDriveGearRatio = 6.75;
    public static final double kWheelDiameter = 4.00000;
    public static final double kMaxAngularVelocity = 10;
    public static final double kMaxAngularAcceleration = 20;

    public static final double kSwerveDriveKP = 0.0;
    public static final double kSwerveDriveKI = 0.01;
    public static final double kSwerveDriveKD = 0.0;
    public static final double kSwerveDriveKS = 0;
    public static final double kSwerveDriveKV = 1;

    public static final double kSwerveTurningKP = 10;
    public static final double kSwerveTurningKI = 0.2;
    public static final double kSwerveTurningKD = 0.1;
    public static final double kSwerveTurningKS = 0.2;
    public static final double kSwerveTurningKV = 47.12;

    // Swerve motors
    public static final SparkBaseConfig kSwerveNominalConfig = new SparkMaxConfig().smartCurrentLimit(150).idleMode(IdleMode.kBrake).inverted(false);

    public static final int kSwerveFrontRightTurnMotor = 4;
    public static final int kSwerveFrontRightDriveMotor = 3;
    public static final int kSwerveFrontRightEncoder = 14;
    public static final double kSwerveFrontRightOffset = 0.607666 + 0.5;

    public static final int kSwerveFrontLeftTurnMotor = 7;
    public static final int kSwerveFrontLeftDriveMotor = 8;
    public static final int kSwerveFrontLeftEncoder = 13;
    public static final double kSwerveFrontLeftOffset = 0.532227 + 0.5;
    
    public static final int kSwerveBackRightTurnMotor = 17;
    public static final int kSwerveBackRightDriveMotor = 2;
    public static final int kSwerveBackRightEncoder = 12;
    public static final double kSwerveBackRightOffset = 0.101074;

    public static final int kSwerveBackLeftTurnMotor = 6;
    public static final int kSwerveBackLeftDriveMotor = 9;
    public static final int kSwerveBackLeftEncoder = 11;
    public static final double kSwerveBackLeftOffset = 0.065430;


    // Elevator
    public static final int kElevatorMotor = 19;
    public static final double kElevatorRatio = 1.0;
    
    public static final double kElevatorKP = 1;
    public static final double kElevatorKI = 0;
    public static final double kElevatorKD = 0;
    public static final double kElevatorMaxAcceleration = 1;
    public static final double kElevatorMaxVelocity = 1;


    // Intake
    // TODO Once this assembly is actully added, check the PIDs
    public static final int kIntakeMotor = 20;
    public static final int kIntakeAssemblyMotor = 21;
    public static final int kIndexMotor = 22;


    // Feature Flags
    public static final boolean kEnableFeedforwardTuning = true;
}
/* kConstants.kSwerve */