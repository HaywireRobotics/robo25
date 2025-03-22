// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minutes;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;
import java.util.Optional;
import java.util.ResourceBundle.Control;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BreatheCommand;
import frc.robot.commands.ChewCommand;
import frc.robot.commands.ChompCommand;
import frc.robot.commands.DecreasePositionCommand;
import frc.robot.commands.DefaultClimbCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.DefaultFilterFeederCommand;
import frc.robot.commands.DefaultManipulatorCommand;
import frc.robot.commands.DigestionCommand;
import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.commands.GoToSpecifiedPosition;
import frc.robot.commands.GrabCoralSequence;
import frc.robot.commands.IncreasePositionCommand;
import frc.robot.commands.MoveForwardCommand;
import frc.robot.commands.MoveClawCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.commands.OpenWideCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.SetElevatorPositionAndWaitCommand;
import frc.robot.commands.SetPositionCommand;
import frc.robot.commands.SpitOutCommand;
import frc.robot.commands.StopDrivingCommand;
import frc.robot.commands.StowCommand;
import frc.robot.commands.TagIDReporterCommand;
import frc.robot.commands.TuneSwerveAutonomousCommand;
import frc.robot.commands.WaitForElevatorCommand;
import frc.robot.commands.YawnCommand;
import frc.robot.commands.AlignWithAprilTagCommand;
import frc.robot.commands.AlternatingDigestionCommand;
import frc.robot.commands.AntacidCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DorsalFin;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FilterFeeder;
import frc.robot.subsystems.LEDSuperSystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Stomach;
import frc.robot.subsystems.Teeth;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.wrappers.Camera;
import frc.robot.wrappers.Controller;
import frc.robot.wrappers.PositionMemory;
import frc.robot.wrappers.WithoutRequirements;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
  private final Controller m_driveController = new Controller(0);
  private final Controller m_manipulatorController = new Controller(1);

  private final DigitalInput m_coralLimitSwitch = new DigitalInput(1);

  private final SendableChooser<LEDPattern> m_sendableChooserForLEDs = new SendableChooser<>();

  private final Robot m_robot;
  private final PowerDistribution m_pdp;

  // define SUBSYSTEMS!!!
  private final DorsalFin m_dorsalFin;
  private final Elevator m_elevator;
  private final FilterFeeder m_filterFeeder;
  private final Manipulator m_manipulator;
  private final Teeth m_teeth;
  private final Stomach m_stomach;
  private final Climb m_climb;
  private final LEDSuperSystem m_led;

  private final PositionMemory m_elevatorPositionMemory = new PositionMemory(0, 3,1);

  // DEFINE default COMMAND?
  public final DefaultDriveCommand defaultDriveCommand;
  public final DefaultElevatorCommand defaultElevatorCommand;
  public final DefaultFilterFeederCommand defaultFilterFeederCommand;
  public final DefaultManipulatorCommand defaultManipulatorCommand;
  public final DefaultClimbCommand defaultClimbCommand;

  public final TuneSwerveAutonomousCommand tuneSwerveAutonomousCommand;
  public final Command exampleAutoCommand;
  private final SysIdRoutine sysidRoutine;

  private final Camera m_camera1 = new Camera("Camera_Module_v1", new Transform3d(
    new Translation3d(0.17, -0.19, 0.35),
    new Rotation3d(0, 0, 0))
  );

  private final Camera m_camera2 = new Camera("Logitech_Webcam_C930e", new Transform3d(
    new Translation3d(0.17, -0.19, 0.35),
    new Rotation3d(0, 0, 0))
  );

  private Field2d fieldPose = new Field2d();
  private final SendableChooser<Command> autoChooser;

  public RobotContainer(Robot robot) {
    m_dorsalFin = new DorsalFin(robot);
    m_filterFeeder = new FilterFeeder();
    m_elevator = new Elevator();
    m_manipulator = new Manipulator();
    m_teeth = new Teeth();
    m_stomach = new Stomach();
    m_climb = new Climb();
    m_led = new LEDSuperSystem();
    m_robot = robot;
    
    m_pdp = new PowerDistribution(50, ModuleType.kRev);

    defaultDriveCommand = new DefaultDriveCommand(m_dorsalFin, m_driveController, m_led);
    defaultElevatorCommand = new DefaultElevatorCommand(m_elevator, m_elevatorPositionMemory, m_led);
    defaultFilterFeederCommand = new DefaultFilterFeederCommand(m_filterFeeder);
    defaultManipulatorCommand = new DefaultManipulatorCommand(m_manipulator, m_manipulatorController, m_elevator);
    defaultClimbCommand = new DefaultClimbCommand(m_climb, m_driveController);

    m_dorsalFin.setDefaultCommand(defaultDriveCommand);
    m_elevator.setDefaultCommand(defaultElevatorCommand);
    m_filterFeeder.setDefaultCommand(defaultFilterFeederCommand);
    m_manipulator.setDefaultCommand(defaultManipulatorCommand);
    m_climb.setDefaultCommand(defaultClimbCommand);

    tuneSwerveAutonomousCommand = new TuneSwerveAutonomousCommand(m_dorsalFin);
    sysidRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(BaseUnits.VoltageUnit.of(0.1).per(BaseUnits.TimeUnit), BaseUnits.VoltageUnit.of(1.6),BaseUnits.TimeUnit.of(10)),
        new SysIdRoutine.Mechanism(m_dorsalFin::sysIdVoltageDrive, m_dorsalFin::driveLogs, m_dorsalFin)
      );
    configureBindings();
    configureNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);


    exampleAutoCommand = new PathPlannerAuto("Test Auto");

    m_sendableChooserForLEDs.setDefaultOption("Haywire Yellow", LEDPattern.solid(Color.kOrange));
    m_sendableChooserForLEDs.addOption("Rainbow", LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(0.025)).atBrightness(Percent.of(25)));
    m_sendableChooserForLEDs.addOption("Red", LEDPattern.solid(Color.kRed));
    m_sendableChooserForLEDs.addOption("Blue", LEDPattern.solid(Color.kBlue));
    m_sendableChooserForLEDs.addOption("Red and Blue Scroll", 
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kBlue)
      .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(0.025))
      .atBrightness(Percent.of(25))
    );
    m_sendableChooserForLEDs.addOption("Red and Blue Solid", 
      LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlue))
    );


    
    SmartDashboard.putData("LED", m_sendableChooserForLEDs);
  }

  private void configureBindings() {
    if (kConstants.kEnableFeedforwardTuning) {
      m_driveController.a().whileTrue(this.sysIdDynamic(SysIdRoutine.Direction.kForward));
      m_driveController.b().whileTrue(this.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      m_driveController.x().whileTrue(this.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      m_driveController.y().whileTrue(this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    }
    if (kConstants.kEnable1MeterTuning) {
      m_driveController.a().whileTrue(new MoveForwardCommand(m_dorsalFin, 1));
    }
    if (kConstants.kEnableFollowApriltag) {
      m_driveController.a().whileTrue(new FollowAprilTagCommand(m_dorsalFin, m_camera1, m_robot));
    }
    if (kConstants.kEnableGoToSpecifiedPosition) {
      m_driveController.a().whileTrue(new GoToSpecifiedPosition(m_dorsalFin, m_robot, 8));
      m_driveController.b().whileTrue(new GoToSpecifiedPosition(m_dorsalFin, m_robot, 2));
    }

    m_driveController.getByName(kConstants.kAlignReefLeftButton).whileTrue(
      new AlignWithAprilTagCommand(m_dorsalFin, m_robot, m_camera1, 0.165, false)
    );
    m_driveController.getByName(kConstants.kAlignReefCenterButton).whileTrue(
      new AlignWithAprilTagCommand(m_dorsalFin, m_robot, m_camera1, 0, false)
    );
    m_driveController.getByName(kConstants.kAlignReefRightButton).whileTrue(
      new AlignWithAprilTagCommand(m_dorsalFin, m_robot, m_camera1, -0.165, false)
    );
    m_driveController.getByName(kConstants.kResetGyroButton).onTrue(
      new ResetGyroCommand(m_dorsalFin)
    );

    // Manipulator Controller Stuff
    m_manipulatorController.getByName(kConstants.kLowerIntakeAssemblyButton).whileTrue(
      new ChompCommand(m_filterFeeder)
    ).onFalse(
      new BreatheCommand(m_filterFeeder)
    );
    m_manipulatorController.getByName(kConstants.kMoveIntakeAssemblyToGrabAlgaeButton).whileTrue(
      new YawnCommand(m_filterFeeder)
    );
    m_manipulatorController.getByName(kConstants.kRaiseIntakeAssemblyButton).whileTrue(
      new OpenWideCommand(m_filterFeeder)
    );
    m_manipulatorController.getByName(kConstants.kRunIntakeButton).whileTrue(
      new SequentialCommandGroup(
        new ParallelRaceGroup(
          new AlternatingDigestionCommand(m_stomach, m_coralLimitSwitch, 0.2, 0.05),
          // new DigestionCommand(m_stomach, m_coralLimitSwitch),
          new ChewCommand(m_teeth)
        ),
        new GrabCoralSequence(m_elevator, m_manipulator, m_led, m_elevatorPositionMemory).asProxy()
      )
    );
    m_manipulatorController.getByName(kConstants.kReverseIntakeButton).whileTrue(
      new SpitOutCommand(m_teeth)
    ).whileTrue(
      new AntacidCommand(m_stomach)
    );

    m_manipulatorController.getByName(kConstants.kElevatorUpButton).onTrue(
      new IncreasePositionCommand(m_elevatorPositionMemory)
    );
    m_manipulatorController.getByName(kConstants.kElevatorDownButton).onTrue(
      new DecreasePositionCommand(m_elevatorPositionMemory)
    );

    m_manipulatorController.getByName(kConstants.kGrabCoralButton).whileTrue(
      new GrabCoralSequence(m_elevator, m_manipulator, m_led, m_elevatorPositionMemory)
    );
    m_manipulatorController.getByName(kConstants.kStowManipulatorButton).whileTrue(
      new SequentialCommandGroup(
        new SetPositionCommand(m_elevatorPositionMemory, 2),
        new WaitForElevatorCommand(m_elevator),
        new MoveClawCommand(m_manipulator, 0)
      )
    );
    m_manipulatorController.getByName(kConstants.kElevatorStowButton).onTrue(
      new StowCommand(m_elevator, m_manipulator, m_manipulatorController)
    );
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Stop Driving",
      new SequentialCommandGroup(
        new StopDrivingCommand(m_dorsalFin),
        new PrintCommand("Stopped Driving")
      )
    );
    NamedCommands.registerCommand("Prepare To Score Top",
      new SequentialCommandGroup(
        new PrintCommand("A"),
        new MoveClawCommand(m_manipulator, 0.3).raceWith(
          new WaitCommand(2)
        ),
        new PrintCommand("B"),
        new MoveElevatorCommand(m_elevator, kConstants.kElevatorScoreL4Position),
        new PrintCommand("C"),
        new MoveClawCommand(m_manipulator, kConstants.kManipulatorUpAngle).raceWith(
          new WaitCommand(2)
        ),
        new PrintCommand("D")
      )
    );
    NamedCommands.registerCommand("Align Left Bar",
      new AlignWithAprilTagCommand(m_dorsalFin, m_robot, m_camera1, 0.165, true)
    );
    NamedCommands.registerCommand("Align Right Bar",
      new AlignWithAprilTagCommand(m_dorsalFin, m_robot, m_camera1, -0.165, true)
    );
    NamedCommands.registerCommand("Score",
      new MoveClawCommand(m_manipulator, 0)
    );
    NamedCommands.registerCommand("Prepare To Grab",
      new SequentialCommandGroup(
        new SetPositionCommand(m_elevatorPositionMemory, 2),
        new WaitForElevatorCommand(m_elevator),
        new MoveClawCommand(m_manipulator, 0)
      )
    );
    NamedCommands.registerCommand("Align Coral",
      new SequentialCommandGroup(
        new PrintCommand("Starting to Align Coral"),
        new SetElevatorPositionAndWaitCommand(m_elevator, m_elevatorPositionMemory, 2),
        new PrintCommand("Moved Elevator"),
        new AlternatingDigestionCommand(m_stomach, m_coralLimitSwitch, 0.2, 0.05),
        new PrintCommand("Indexed Coral"),
        new GrabCoralSequence(m_elevator, m_manipulator, m_led, m_elevatorPositionMemory),
        new PrintCommand("Aligned Coral")
      )
    );
    NamedCommands.registerCommand("Back Up A Little",
      new ParallelDeadlineGroup(
        new WaitCommand(Seconds.of(1)),
        Commands.run(() -> {
          m_dorsalFin.drive(
            new ChassisSpeeds(-1.0, 0.0, 0.0)
          );
        }, m_dorsalFin)
      ).andThen(
        Commands.run(() -> {
          m_dorsalFin.drive(
            new ChassisSpeeds(0.0, 0.0, 0.0)
          );
        }, m_dorsalFin)
      )
      
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return tuneSwerveAutonomousCommand;
  }

  public Command getTestCommand() {
    return new TagIDReporterCommand(m_camera1);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysidRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysidRoutine.dynamic(direction);
  }

  public void updateOdometry() {
    m_dorsalFin.updateOdometry();
    Optional<Pose2d> estimated_pose1 = m_camera1.estimatePose(m_dorsalFin.getPose2D());
    Optional<Pose2d> estimated_pose2 = m_camera2.estimatePose(m_dorsalFin.getPose2D());
    if (estimated_pose1.isPresent() && estimated_pose2.isEmpty() ) {
      m_dorsalFin.setOdometry(estimated_pose1.get());
    }
    if (estimated_pose1.isEmpty() && estimated_pose2.isPresent() ) {
      m_dorsalFin.setOdometry(estimated_pose2.get());
    }
    if (estimated_pose1.isPresent() && estimated_pose2.isPresent()) {
      m_dorsalFin.setOdometry(estimated_pose2.get().interpolate(estimated_pose1.get(), 0.5));
    }
  }

  public Pose2d getFieldPose(){
    return m_dorsalFin.getFieldPose();
  }

  public Field2d updateFieldPose() {
    fieldPose.setRobotPose(getFieldPose());
    return fieldPose;
  }

  public void putAllSmartDashboardData(){
    SmartDashboard.putNumber("Intake Current", m_pdp.getCurrent(13));
    SmartDashboard.putNumber("Front Index Current", m_pdp.getCurrent(14));
    SmartDashboard.putBoolean("Limit Switch", m_coralLimitSwitch.get());
  }

  public void reset() {
    m_filterFeeder.reset();
    m_elevator.reset();
    m_manipulator.reset();
  }

  public void disabledLED() {
    m_led.setPattern(
      m_sendableChooserForLEDs.getSelected()
    );
  }
}
