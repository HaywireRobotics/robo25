package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class kConstants {
    public static final double kDriveGearRatio = 6.75;
    public static final double kWheelDiameter = 4.00000;
    public static final double kMaxAngularVelocity = 10;
    public static final double kMaxAngularAcceleration = 20;
    public static final double kInchesToMeters = 0.0254;

    public static final double kNavigationMultiplier = 1.6; // Number to multiply the left joystick by
    public static final double kRotationMultiplier = 2.5; // Number to multiply the right joystick by

    public static final double kSlowModeDivider = 1.5; // Number to divide both joysticks 1.5 = [1,0.3] 

    public static final double kSwerveDriveKP = 0.44072;
    public static final double kSwerveDriveKI = 0.0;
    public static final double kSwerveDriveKD = 0.0;
    public static final double kSwerveDriveKS = 0.11831;
    public static final double kSwerveDriveKV = 2.4461;
    public static final double kSwerveDriveKA = 0.28161;

    public static final double kSwerveTurningKP = 8;
    public static final double kSwerveTurningKI = 0;
    public static final double kSwerveTurningKD = 0;
    public static final double kSwerveTurningKS = 0.2;
    public static final double kSwerveTurningKV = 47.12;

    public static final double kDriveTrainWidth = 24.5 * kInchesToMeters;
    public static final double kDriveTrainLength = 20 * kInchesToMeters;


    // Swerve motors
    public static final SparkBaseConfig kSwerveNominalConfig = new SparkMaxConfig().smartCurrentLimit(150).idleMode(IdleMode.kBrake).inverted(false);

    public static final int kSwerveFrontRightTurnMotor = 4;
    public static final int kSwerveFrontRightDriveMotor = 3;
    public static final int kSwerveFrontRightEncoder = 14;
    public static final double kSwerveFrontRightOffset = 0.607666;

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
    public static final double kSwerveBackLeftOffset = 0.065430 + 0.5;


    // Elevator
    public static final int kElevatorMotor = 19;
    public static final double kElevatorRatio = 1.0/20.0;
    
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
    public static final double kIntakeAssemblyKP = 1;
    public static final double kIntakeAssemblyKI = 0;
    public static final double kIntakeAssemblyKD = 0;
    public static final double kIntakeAssemblyMaxVelocity = 1;
    public static final double kIntakeAssemblyMaxAcceleration = 1;
    public static final double kIntakeAssemblyDownPoint = -1;
    public static final double kIntakeAssemblyUpPoint = 0;
    public static final double kEnableIndex = 4;
    public static final double kEnableIntake = 7;
    public static final SparkBaseConfig kIntakeAssemblyNominalConfig = new SparkMaxConfig().smartCurrentLimit(95).idleMode(IdleMode.kBrake).inverted(false);
    


    // Manipulator
    public static final double kManipulatorRatio = 1.0/35.0;
    public static final int kManipulatorMotor = 23;
    public static final double kManipulatorKP = 1;
    public static final double kManipulatorKI = 0;
    public static final double kManipulatorKD = 0;
    public static final double kManipulatorMaxAcceleration = 1;
    public static final double kManipulatorMaxVelocity = 1;
    public static final double kManipulatorMaxAngle = 180;



    public static final SparkBaseConfig kDefaultNeo550NominalConfig = new SparkMaxConfig().smartCurrentLimit(95).idleMode(IdleMode.kBrake).inverted(false);

    
    // Feature Flags
    public static final boolean kEnableFeedforwardTuning = false; // A, B, X, Y run feedforward tuning code for the Sysid tool
    public static final boolean kEnable1MeterTuning = false; // A moves the robot forward 1 meter
    public static final boolean kEnableFollowApriltag = true; // A follows april tag 2 at a distance of 1 meter
    public static final boolean kEnableGoToSpecifiedPosition = false; // A goes to (currently) 0,0 the place where the robot was restarted
}
/* kConstants.kSwerve */