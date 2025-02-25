package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Filesystem;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class kConstants {
    public static final double kDriveGearRatio = 6.75;
    public static final double kWheelDiameter = 4.00000;
    public static final double kMaxAngularVelocity = 10;
    public static final double kMaxAngularAcceleration = 20;
    public static final double kInchesToMeters = 0.0254;

    public static final double kNavigationMultiplier = 1.6; // Number to multiply the left joystick by
    public static final double kRotationMultiplier = 5; // Number to multiply the right joystick by

    public static final double kSlowModeDivider = 1.5; // Number to divide both joysticks 1.5 = [1,0.3]

    public static final double kMaxPoseAmbiguity = 0.1; // Ambiguity Ratio
    public static final double kMaxTagDistance = 1.5; // Meters

    public static final double kSwerveDriveKP = 0.44072;
    public static final double kSwerveDriveKI = 0.0;
    public static final double kSwerveDriveKD = 0.0;
    public static final double kSwerveDriveKS = 0.098939; 
    public static final double kSwerveDriveKV = 2.1788;
    public static final double kSwerveDriveKA = 0.39833;

    public static final double kSwerveTurningKP = 5;
    public static final double kSwerveTurningKI = 0;
    public static final double kSwerveTurningKD = 0;
    public static final double kSwerveTurningKS = 0.2;
    public static final double kSwerveTurningKV = 47.12;

    public static final double kDriveTrainWidth = 24.5 * kInchesToMeters;
    public static final double kDriveTrainLength = 20 * kInchesToMeters;


    // Swerve motors
    public static final SparkBaseConfig kNeoNominalConfig = new SparkMaxConfig().smartCurrentLimit(150).idleMode(IdleMode.kBrake).inverted(false);

    public static final int kSwerveFrontRightTurnMotor = 4;
    public static final int kSwerveFrontRightDriveMotor = 3;
    public static final int kSwerveFrontRightEncoder = 5;
    public static final double kSwerveFrontRightOffset = -0.321777;

    public static final int kSwerveFrontLeftTurnMotor = 1;
    public static final int kSwerveFrontLeftDriveMotor = 20;
    public static final int kSwerveFrontLeftEncoder = 2;
    public static final double kSwerveFrontLeftOffset = 0.272705;
    
    public static final int kSwerveBackRightTurnMotor = 10;
    public static final int kSwerveBackRightDriveMotor = 9;
    public static final int kSwerveBackRightEncoder = 11;
    public static final double kSwerveBackRightOffset = -0.345459;

    public static final int kSwerveBackLeftTurnMotor = 7;
    public static final int kSwerveBackLeftDriveMotor = 6;
    public static final int kSwerveBackLeftEncoder = 8;
    public static final double kSwerveBackLeftOffset = -0.807861;


    // Elevator
    public static final int kElevatorMotor = 17;
    public static final double kElevatorRatio = 1.0/20.0;
    public static final double kElevatorRotationsToInches = 4 * Math.PI;
    
    public static final double kElevatorKP = 1;
    public static final double kElevatorKI = 0;
    public static final double kElevatorKD = 0;
    public static final double kElevatorMaxAcceleration = 10;
    public static final double kElevatorMaxVelocity = 5;


    // Intake
    public static final int kIntakeMotor = 13;
    public static final double kEnableIntakeVoltage = -7;

    public static final int kIntakeAssemblyMotor = 12;
    public static final double kIntakeAssemblyKP = 1;
    public static final double kIntakeAssemblyKI = 0;
    public static final double kIntakeAssemblyKD = 0;
    public static final double kIntakeAssemblyMaxVelocity = 5;
    public static final double kIntakeAssemblyMaxAcceleration = 10;

    public static final int kIndexMotor = 14;
    public static final int kIndexBackLeftMotor = 16;
    public static final int kIndexBackRightMotor = 15;
    public static final double kIntakeAssemblyDownPoint = 17.7;
    public static final double kIntakeAssemblyBumpPoint = 16;
    public static final double kIntakeAssemblyUpPoint = 0.1;

    public static final double kEnableBackIndexVoltage = 2;
    public static final double kEnableFrontIndexVoltage = 4;
    
    
    

    // Manipulator
    public static final double kManipulatorRatio = 1.0/35.0;
    public static final int kManipulatorMotor = 18;
    public static final double kManipulatorKP = 12;
    public static final double kManipulatorKI = 0.1;
    public static final double kManipulatorKD = 0;
    public static final double kManipulatorMaxAcceleration = 10;
    public static final double kManipulatorMaxVelocity = 5;
    public static final double kManipulatorMinAngle = 0.74082186;
    public static final double kManipulatorDownPoint = 0.74082186;
    public static final int kManipulatorEncoderID = 0;

    public static final double kManipulatorPowerMultiplier = -1;
    
    // Feature Flags
    public static final boolean kEnableFeedforwardTuning = true; // A, B, X, Y run feedforward tuning code for the Sysid tool
    public static final boolean kEnable1MeterTuning = false; // A moves the robot forward 1 meter
    public static final boolean kEnableFollowApriltag = false; // A follows april tag 2 at a distance of 1 meter
    public static final boolean kEnableGoToSpecifiedPosition = false; // A goes to (currently) 0,0 the place where the robot was restarted

    // Buttons
    public static final String kDisableFieldCentricButton = "back";
    public static final String kEnableFieldCentricButton = "start";
    public static final String kLowerIntakeAssemblyButton = "down";
    public static final String kRunIntakeButton = "a";
    public static final String kRunIndexesCommand = "b";

    public static final String kMoveManipulatorToDownButton = "lb";
    public static final String kMoveManipulatorToMiddleButton = "rb";

    // Files
    public static final String kFieldAprilTagJSON = Filesystem.getDeployDirectory() + "/fields/2025-reefscape-andymark.json";
}
/* kConstants.kSwerve */