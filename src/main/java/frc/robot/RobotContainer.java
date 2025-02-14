package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Creates our objects from our methods for our classes
  DriveTrain m_DriveTrain = new DriveTrain(Constants.defaultRobotVersion);
  Limelight m_LimelightFront = new Limelight("limelight-front");
  Limelight m_LimelightBack = new Limelight("limelight-back");
  LedStrand mLedStrand = new LedStrand();
  CoralMechanism mCoralMechanism = new CoralMechanism();
  ClimbMechanism mClimbMechanism = new ClimbMechanism();
  ElevatorMechanism mElevatorMechanism = new ElevatorMechanism();
  ElevatorPID elevatorPID = new ElevatorPID(mElevatorMechanism, ()-> mElevatorMechanism.desiredPosGet());
  AprilAlignCommand LimelightCodeFront = new AprilAlignCommand(() -> m_LimelightFront.getCurrentAprilTag(), () ->  m_LimelightFront.getAprilRotation2d(), m_DriveTrain, new Transform2d(0,1, new Rotation2d()), false, mLedStrand);
  AprilAlignCommand LimelightCodeBack = new AprilAlignCommand(() -> m_LimelightBack.getCurrentAprilTag(), () ->  m_LimelightBack.getAprilRotation2d(), m_DriveTrain, new Transform2d(0,1, new Rotation2d()), true,mLedStrand);

final Command runCoral = mCoralMechanism.CoralForwardCmd().onlyWhile(()->!mCoralMechanism.IsCoralLoaded()).withName("RunCoral");
//final Command MoveElevatorUp = Commands.sequence(mElevatorMechanism.SetPosUp().alongWith(elevatorPID));
//final Command MoveElevatorDown = Commands.sequence(mElevatorMechanism.SetPosDown().andThen(this::elevatorPID, mElevatorMechanism));
  /** 
   * Creates buttons and controller for: - the driver controller (port 0) - the manipulator
   * controller (port 1) - the debug controller (port 2)
   */
  CommandXboxController driverController =
      new CommandXboxController(Constants.Controller.DriverControllerChannel);

  CommandXboxController manipController =
      new CommandXboxController(Constants.Controller.ManipControllerChannel);
  CommandXboxController debugController =
      new CommandXboxController(Constants.Controller.DebugControllerChannel);
  final Command DriveForward =
      new SwerveDriveCommand(() -> 1, () -> 0, () -> 0, () -> .3, m_DriveTrain);
  final Command DriveSide =
      new SwerveDriveCommand(() -> 0, () -> 1, () -> 0, () -> .3, m_DriveTrain);
  final Command Rotate = new SwerveDriveCommand(() -> 0, () -> 0, () -> .3, () -> 0, m_DriveTrain);
  // commands
  

  // A chooser for autonomous commands
  private final SendableChooser<Command> autoChooser;
  // Creating 2d field in Sim/ShuffleBoard
  private final Field2d field;
  // Trying to get feedback from auto
  List<Pose2d> currentPath = new ArrayList<Pose2d>();

  public RobotContainer() {
   mElevatorMechanism.setName("ElevatorMechanism");
    m_DriveTrain.setName("DriveTrain");
    mCoralMechanism.setName("CoralMechnaism");
    elevatorPID.setName("ElevatorPIDCommand");
    configureShuffleboard();
    configureBindings();
    // Build an auto chooser. This will use Commands.none() as the default option.

    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name:
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Creates a field to be put to the shuffleboard
    field = new Field2d();

    SmartDashboard.putData("Field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.setRobotPose(pose);
        });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.getObject("target pose").setPose(pose);
        });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          // Do whatever you want with the poses here
          field.getObject("path").setPoses(poses);
        });
  }

  public void periodic() {
    // us trying to set pose for field2d
    field.setRobotPose(m_DriveTrain.getPose2d());
  }

  /**
   * Creates Command Bindings. Read description down below:
   *
   * <p>Use this method to define your trigger->comand mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /**
     * Swerve Drive Controller Command
     *
     * <p>Controls: - Left Stick: Steering - Right Stick: Rotate the robot - Right Trigger: provide
     * gas - Left Trigger: reduce maximum driving speed by 50% RECOMMENDED TO USE
     */
    m_DriveTrain.setDefaultCommand(
        new SwerveDriveCommand(
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> driverController.getRightX(),
            () -> driverController.getRightTriggerAxis(),
            m_DriveTrain,
            () -> driverController.getLeftTriggerAxis() >= 0.5));
    
    // Driver Controller commands
    // - DriveTrain commands (outside of actual driving)
    driverController.a().onTrue(m_DriveTrain.toggleFieldRelativeEnable());
    driverController.b().onTrue(m_DriveTrain.ZeroGyro());
    driverController.start().onTrue(m_DriveTrain.resetPose2d()); // RESETING OUR POSE 2d/ odometry
    driverController.rightStick().onTrue(m_DriveTrain.WheelLockCommand()); // lock wheels
    driverController.x().whileTrue(LimelightCodeFront); 
    driverController.y().whileTrue(LimelightCodeBack);
    driverController.povUp().whileTrue(mCoralMechanism.CoralForwardCmd());//mCoralMechanism.CoralForwardCmd());
    driverController.povDown().whileTrue(mCoralMechanism.CoralBackwardCmd());
    // Manipulator Controller commands
    manipController.y().onTrue(mLedStrand.changeLedCommand()); 
    manipController.povUp().onTrue(mElevatorMechanism.MovePosUp());
    manipController.povDown().onTrue(mElevatorMechanism.MovePosDown());
    manipController.a().whileTrue(elevatorPID);
  }

  private void configureShuffleboard() {
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData(mElevatorMechanism);
    SmartDashboard.putData(elevatorPID);
    SmartDashboard.putData(elevatorPID);
    // Add subsystems
    SmartDashboard.putData(m_DriveTrain);
    SmartDashboard.putData(m_DriveTrain.getName() + "/Reset Pose 2D", m_DriveTrain.resetPose2d());
    SmartDashboard.putData(mCoralMechanism);  
    SmartDashboard.putData(m_LimelightFront);
  }

  public Command getAutonomousCommand() {
    // loads New Auto auto file

    Command autoCommand = autoChooser.getSelected();

    return autoCommand.beforeStarting(
        () -> m_DriveTrain.resetPose(new Pose2d(1.27, 5.55, new Rotation2d())));
  }
}
