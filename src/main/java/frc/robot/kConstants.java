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
    public static final double kDriveTrainLength = 19.5 * kInchesToMeters;


    // Swerve motors
    public static final SparkBaseConfig kNeoNominalConfig = new SparkMaxConfig().smartCurrentLimit(150).idleMode(IdleMode.kBrake).inverted(false);
    public static final SparkBaseConfig kNeo550NominalConfig = new SparkMaxConfig().smartCurrentLimit(90).idleMode(IdleMode.kBrake).inverted(false);

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

    public static final double kElevatorGrabCoralPosition = 17.015689;
    public static final double kElevatorScoreL2Position = 14.690601;
    public static final double kElevatorScoreL3Position = 35.694858;
    public static final double kElevatorScoreL4Position = 59.168884;
    public static final double kElevatorRemoveAlgaeLowPosition = 21.721669;
    public static final double kManipulatorRemoveAlgaePosition = 0.426240;
    public static final double kElevatorRemoveAlgaeHighPosition = 45.464669;
    
    public static final double kElevatorKP = 2;
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
    public static final double kIntakeAssemblyMaxVelocity = 10;
    public static final double kIntakeAssemblyMaxAcceleration = 40;

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
    public static final double kManipulatorKP = 13;
    public static final double kManipulatorKI = 2;
    public static final double kManipulatorKD = 1;
    public static final double kManipulatorMaxAcceleration = 10;
    public static final double kManipulatorMaxVelocity = 2;
    public static final double kManipulatorMaxAngle = 0.74082186;
    public static final double kManipulatorDownPoint = 0.74082186;
    public static final int kManipulatorEncoderID = 0;

    public static final double kManipulatorPowerMultiplier = -1;
    
    // Feature Flags
    public static final boolean kEnableFeedforwardTuning = false; // A, B, X, Y run feedforward tuning code for the Sysid tool
    public static final boolean kEnable1MeterTuning = false; // A moves the robot forward 1 meter
    public static final boolean kEnableFollowApriltag = false; // A follows april tag 2 at a distance of 1 meter
    public static final boolean kEnableGoToSpecifiedPosition = false; // A goes to (currently) 0,0 the place where the robot was restarted

    // Buttons
    // Driver
    public static final String kDisableFieldCentricButton = "back";
    public static final String kEnableFieldCentricButton = "start";

    // Manipulator
    public static final String kLowerIntakeAssemblyButton = "a";
    public static final String kRunIntakeButton = "rt";

    // Algae
    public static final String kRaiseIntakeAssemblyButton = "b";
    public static final String kReverseIntakeButton = "rb";

    // public static final String kGrabCoralButton = "x";

    public static final String kElevatorUpButton = "lb";
    public static final String kElevatorDownButton = "lt";


    // Joystick is part of a different system
    // public static final String kManipulatorAngleJoystick = "left_stick";


    /* 
    public static final String kElevatorPosition0Button = "back";
    public static final String kElevatorPosition1Button = "up";
    public static final String kElevatorPosition2Button = "right";
    public static final String kElevatorPosition3Button = "down";
    public static final String kElevatorPosition4Button = "left";
    */

    /* 
    public static final String kMoveManipulatorToMiddleButton = "rt";
    public static final String kMoveManipulatorToTopButton = "rb";
    public static final String kMoveManipulatorToDownButton = "lt";
    public static final String kMoveManipulatorToUpButton = "start";
    */

    // Files
    public static final String kFieldAprilTagJSON = Filesystem.getDeployDirectory() + "/fields/2025-reefscape-andymark.json";
}
/* kConstants.kSwerve */