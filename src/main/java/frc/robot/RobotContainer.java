// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.ResourceBundle.Control;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
import frc.robot.commands.SpitOutCommand;
import frc.robot.commands.StopDrivingCommand;
import frc.robot.commands.TagIDReporterCommand;
import frc.robot.commands.TuneSwerveAutonomousCommand;
import frc.robot.commands.YawnCommand;
import frc.robot.commands.AlignWithAprilTagCommand;
import frc.robot.commands.AntacidCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DorsalFin;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FilterFeeder;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Stomach;
import frc.robot.subsystems.Teeth;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.wrappers.Camera;
import frc.robot.wrappers.Controller;
import frc.robot.wrappers.PositionMemory;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
  private final Controller m_driveController = new Controller(0);
  private final Controller m_manipulatorController = new Controller(1);

  private final DigitalInput m_coralLimitSwitch = new DigitalInput(1);

  private final Robot m_robot;

  // define SUBSYSTEMS!!!
  private final DorsalFin m_dorsalFin;
  private final Elevator m_elevator;
  private final FilterFeeder m_filterFeeder;
  private final Manipulator m_manipulator;
  private final Teeth m_teeth;
  private final Stomach m_stomach;
  private final Climb m_climb;

  private final PositionMemory m_elevatorPositionMemory = new PositionMemory(0, 3);

  // DEFINE default COMMAND?
  public final DefaultDriveCommand defaultDriveCommand;
  public final DefaultElevatorCommand defaultElevatorCommand;
  public final DefaultFilterFeederCommand defaultFilterFeederCommand;
  public final DefaultManipulatorCommand defaultManipulatorCommand;
  public final DefaultClimbCommand defaultClimbCommand;

  public final TuneSwerveAutonomousCommand tuneSwerveAutonomousCommand;
  public final Command exampleAutoCommand;
  private final SysIdRoutine sysidRoutine;

  private final Camera m_camera = new Camera("Camera_Module_v1", new Transform3d(
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
    m_robot = robot;

    defaultDriveCommand = new DefaultDriveCommand(m_dorsalFin, m_driveController);
    defaultElevatorCommand = new DefaultElevatorCommand(m_elevator, m_elevatorPositionMemory);
    defaultFilterFeederCommand = new DefaultFilterFeederCommand(m_filterFeeder);
    defaultManipulatorCommand = new DefaultManipulatorCommand(m_manipulator, m_manipulatorController, m_elevator);
    defaultClimbCommand = new DefaultClimbCommand(m_climb, m_manipulatorController);

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
      m_driveController.a().whileTrue(new FollowAprilTagCommand(m_dorsalFin, m_camera, m_robot));
    }
    if (kConstants.kEnableGoToSpecifiedPosition) {
      m_driveController.a().whileTrue(new GoToSpecifiedPosition(m_dorsalFin, m_robot, 8));
      m_driveController.b().whileTrue(new GoToSpecifiedPosition(m_dorsalFin, m_robot, 2));
    }

    m_driveController.getByName(kConstants.kAlignReefLeftButton).whileTrue(
      new AlignWithAprilTagCommand(m_dorsalFin, m_robot, m_camera, 0.165, false)
    );
    m_driveController.getByName(kConstants.kAlignReefCenterButton).whileTrue(
      new AlignWithAprilTagCommand(m_dorsalFin, m_robot, m_camera, 0, false)
    );
    m_driveController.getByName(kConstants.kAlignReefRightButton).whileTrue(
      new AlignWithAprilTagCommand(m_dorsalFin, m_robot, m_camera, -0.165, false)
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
      new DigestionCommand(m_stomach, m_coralLimitSwitch)
    ).whileTrue(
      new ChewCommand(m_teeth)
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
      new GrabCoralSequence(m_elevator, m_manipulator)
    );
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Stop Driving",
      new StopDrivingCommand(m_dorsalFin)
    );
    NamedCommands.registerCommand("Prepare To Score Top",
      new MoveClawCommand(m_manipulator, 0.3).andThen(
        new MoveElevatorCommand(m_elevator, kConstants.kElevatorScoreL4Position),
        new MoveClawCommand(m_manipulator, kConstants.kManipulatorUpAngle).raceWith(
          new WaitCommand(3)
        ),
        new PrintCommand("[COMMAND] Preparing to score on the top!")
      )
    );
    NamedCommands.registerCommand("Align Left Bar",
      new AlignWithAprilTagCommand(m_dorsalFin, m_robot, m_camera, 0.165, true)
    );
    NamedCommands.registerCommand("Align Right Bar",
      new AlignWithAprilTagCommand(m_dorsalFin, m_robot, m_camera, -0.165, true)
    );
    NamedCommands.registerCommand("Score",
      new MoveClawCommand(m_manipulator, 0)
    );
    NamedCommands.registerCommand("Align Coral",
      new MoveElevatorCommand(m_elevator, kConstants.kElevatorGrabCoralPosition + 10).andThen(
        new DigestionCommand(m_stomach, m_coralLimitSwitch),
        new GrabCoralSequence(m_elevator, m_manipulator)
      )
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return tuneSwerveAutonomousCommand;
  }

  public Command getTestCommand() {
    return new TagIDReporterCommand(m_camera);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysidRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysidRoutine.dynamic(direction);
  }

  public void updateOdometry() {
    m_dorsalFin.updateOdometry();
    Optional<Pose2d> estimated_pose = m_camera.estimatePose(m_dorsalFin.getPose2D());
    if (estimated_pose.isPresent()) {
      m_dorsalFin.setOdometry(estimated_pose.get());
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
    //TODO
  }

  public void reset() {
    m_filterFeeder.reset();
    m_elevator.reset();
    m_manipulator.reset();
  }
}
