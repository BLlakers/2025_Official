package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
  Limelight m_LimelightFrl = new Limelight("limelight-frl");
  Limelight m_LimelightFrr = new Limelight("limelight-frr");
 
  Limelight m_LimelightBack = new Limelight("limelight-back");
  LedStrand mLedStrand = new LedStrand();
  CoralMechanism mCoralMechanism = new CoralMechanism();
  ClimbMechanism mClimbMechanism = new ClimbMechanism();
  AlgaeMechanism mAlgaeMechanism = new AlgaeMechanism();
  AlgaeIntake m_AlgaeIntake = new AlgaeIntake();
  ElevatorMechanism mElevatorMechanism = new ElevatorMechanism();
  ElevatorPID elevatorPIDDown = new ElevatorPID(mElevatorMechanism,  ElevatorMechanism.Down);
  ElevatorPID elevatorPIDDown2 = new ElevatorPID(mElevatorMechanism,  ElevatorMechanism.Down);

  ElevatorPID elevatorPIDL2 = new ElevatorPID(mElevatorMechanism,  ElevatorMechanism.L2);
  ElevatorPID elevatorPIDL3 = new ElevatorPID(mElevatorMechanism,  ElevatorMechanism.L3);
  ElevatorPID elevatorPIDL4 = new ElevatorPID(mElevatorMechanism,  ElevatorMechanism.L4);
  ElevatorPID elevatorPIDAlgae3 = new ElevatorPID(mElevatorMechanism,  ElevatorMechanism.AlgaeL3);
  ElevatorPID elevatorPIDAlgae3v2 = new ElevatorPID(mElevatorMechanism,  ElevatorMechanism.AlgaeL3);
  ElevatorPID elevatorPIDAlgae4 = new ElevatorPID(mElevatorMechanism,  ElevatorMechanism.AlgaeL4);
  Servo mServo = new Servo();
  AprilAlignCommand LimelightCodeFrontLeft = new AprilAlignCommand(() -> m_LimelightFrl.getCurrentAprilTag(), () ->  m_LimelightFrl.getAprilRotation2d(), m_DriveTrain, new Transform2d(.05,0.00, new Rotation2d(.15)), false, true, mLedStrand);
  AprilAlignCommand LimelightCodeFrontRight = new AprilAlignCommand(() -> m_LimelightFrr.getCurrentAprilTag(), () ->  m_LimelightFrr.getAprilRotation2d(), m_DriveTrain, new Transform2d(.05,0.00, new Rotation2d(-0.15)), false, false, mLedStrand);
  AprilAlignCommand LimelightCodeBack = new AprilAlignCommand(() -> m_LimelightBack.getCurrentAprilTag(), () ->  m_LimelightBack.getAprilRotation2d(), m_DriveTrain, new Transform2d(-.52,-0.05, new Rotation2d()), true, false, mLedStrand);
  AlgaePID algaePIDDown = new AlgaePID(mAlgaeMechanism, AlgaeMechanism.PosDown);
  AlgaePID algaePIDDown2 = new AlgaePID(mAlgaeMechanism, AlgaeMechanism.PosDown);
  AlgaePID algaePIDMiddle = new AlgaePID(mAlgaeMechanism, AlgaeMechanism.PosMiddle);
  AlgaePID algaePIDUp = new AlgaePID(mAlgaeMechanism, AlgaeMechanism.PosUp);
  AlgaePID algaePIDUp2 = new AlgaePID(mAlgaeMechanism, AlgaeMechanism.PosUp);
  
  AlgaePID algaePIDGround = new AlgaePID(mAlgaeMechanism, AlgaeMechanism.PosGround);

  


//  final Command TEST = elevatorPIDDown.finallyDo(elevatorPIDL2::schedule);

Command runCoralFoward = mCoralMechanism.CoralForwardCmd().onlyWhile(()->!mCoralMechanism.IsCoralLoaded()).withName("RunCoral");
//  Command algaeIntakeForward = m_AlgaeIntake.IntakeForwardCmd();
//  Command algaeIntakeBackward = m_AlgaeIntake.IntakeBackwardCmd();

// Compose the commands correctly, ensuring that each use is a new composition
Command algaeDownAndRunA3 = Commands.race(new AlgaePID(mAlgaeMechanism, AlgaeMechanism.PosDown), m_AlgaeIntake.IntakeForwardOnceCmd(), new ElevatorPID(mElevatorMechanism, ElevatorMechanism.AlgaeL3)); 
Command algaeDownAndRunA4 = Commands.race(new AlgaePID(mAlgaeMechanism, AlgaeMechanism.PosDown), m_AlgaeIntake.IntakeForwardOnceCmd(), new ElevatorPID(mElevatorMechanism, ElevatorMechanism.AlgaeL4)); 
Command algaeUpAndStop = Commands.race(new AlgaePID(mAlgaeMechanism, AlgaeMechanism.PosUp), m_AlgaeIntake.IntakeStopCmd()); 
Command algaeMiddleAndStop = Commands.race(new AlgaePID(mAlgaeMechanism, AlgaeMechanism.PosMiddle), m_AlgaeIntake.IntakeStopCmd());
Command algaeGroundCommand = Commands.race(new AlgaePID(mAlgaeMechanism, AlgaeMechanism.PosGround), m_AlgaeIntake.IntakeBackwardOnceCmd(), new ElevatorPID(mElevatorMechanism, ElevatorMechanism.L2));

// Command algaeDownAndRun = Commands.deadline(algaePIDDown2, m_AlgaeIntake.IntakeForwardOnceCmd()); 
// Command algaeUpAndStopADown = Commands.parallel(algaePIDUp2, m_AlgaeIntake.IntakeStopCmd(), elevatorPIDDown); 


// Command algaeL3Down = Commands.parallel(algaePIDDown, elevatorPIDAlgae3);
//  Command algaeL4Down = Commands.parallel(algaePIDDown2, elevatorPIDAlgae4);
//  Command algaeUp = Commands.parallel(algaePIDUp, elevatorPIDDown);
//  Command algaeUp2 = Commands.parallel(algaePIDUp2, elevatorPIDDown2);

// Command to retrieve algae from L3
//  Command algaeL3 = Commands.sequence(algaeL3Down, algaeUp);

// Command to retrieve algae from L4
//  Command algaeL4 = Commands.sequence(algaeL4Down, algaeUp2);

// Command to retrieve algae from the ground
// Command algaeGround = Commands.parallel(algaePIDGroud, elevatorPIDAlgae3v2);
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
      new SwerveDriveCommand(() -> .1, () -> 0, () -> 0, () -> .3, m_DriveTrain);
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
    
    m_AlgaeIntake.setName("AlgaeIntake");

    mAlgaeMechanism.setName("AlgaeMechanism");
    configureShuffleboard();
    configureBindings();
   NamedCommands.registerCommand("Limelight",LimelightCodeFrontLeft);
    NamedCommands.registerCommand("ElevatorL2",elevatorPIDL2);
    NamedCommands.registerCommand("IntakeCoral", mCoralMechanism.CoralIntakeAutoCmd());
    NamedCommands.registerCommand("ResetOdom", m_DriveTrain.resetPose2d());
    NamedCommands.registerCommand("ElevatorL4",new ElevatorPID(mElevatorMechanism,ElevatorMechanism.L4).withTimeout(.5));
    NamedCommands.registerCommand("ElevatorBottom",new ElevatorPID(mElevatorMechanism,ElevatorMechanism.Down));
    NamedCommands.registerCommand("ElevatorUp",mElevatorMechanism.ElevatorUpCmd());
    NamedCommands.registerCommand("ShootCoral",mCoralMechanism.CoralForwardCmd().withTimeout(1));
    NamedCommands.registerCommand("ToggleFieldRelativel", m_DriveTrain.toggleFieldRelativeEnable());
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
            () -> mElevatorMechanism.getElevatorDecelerateRatio(),
            m_DriveTrain,
            () -> driverController.getLeftTriggerAxis() >= 0.5, "Ben"));

    
    
    // Driver Controller commands
    // - DriveTrain commands (outside of actual driving)
    // driverController.a().onTrue(m_DriveTrain.toggleFieldRelativeEnable());
    driverController.b().onTrue(m_DriveTrain.ZeroGyro());
    driverController.start().onTrue(m_DriveTrain.resetPose2d()); // RESETING OUR POSE 2d/ odometry
    driverController.rightStick().onTrue(m_DriveTrain.WheelLockCommand()); // lock wheels
    driverController.x().whileTrue(LimelightCodeFrontLeft); 
    driverController.y().whileTrue(LimelightCodeFrontRight);
    // driverController.povUp().whileTrue(runCoralFoward);//mCoralMechanism.CoralForwardCmd());
    // driverController.povDown().whileTrue(mCoralMechanism.CoralBackwardCmd());
    // Manipulator Controller commands
    // manipController.y().onTrue(mLedStrand.changeLedCommand()); 
    // manipController.povUp().onTrue(mElevatorMechanism.MovePosUp());
    // manipController.povDown().onTrue(mElevatorMechanism.MovePosDown());
    
    //Elevator Commands
    manipController.a().onTrue(elevatorPIDDown);
    manipController.b().onTrue(elevatorPIDL2);
    manipController.y().onTrue(elevatorPIDL3);
    manipController.x().onTrue(elevatorPIDL4);
    manipController.rightStick().onTrue(mElevatorMechanism.ResetPositionCMD());
    //Algae Commands
    manipController.leftBumper().onTrue(algaeDownAndRunA3);
    manipController.rightBumper().onTrue(algaeDownAndRunA4);
    manipController.back().onTrue(algaeUpAndStop);
    manipController.start().whileTrue(m_AlgaeIntake.IntakeBackwardCmd());
    manipController.povLeft().onTrue(algaeGroundCommand);
    manipController.povRight().whileTrue(m_AlgaeIntake.IntakeForwardCmd());
    manipController.leftStick().onTrue(algaeMiddleAndStop);
    //Coral Commands
    manipController.leftTrigger(.5).whileTrue(runCoralFoward);
    manipController.rightTrigger(.5).whileTrue(mCoralMechanism.CoralForwardCmd());
    manipController.povUp().whileTrue(mClimbMechanism.WindForwardCmd());
    manipController.povDown().whileTrue(mClimbMechanism.WindDownCmd());
    mServo.setDefaultCommand(mServo.ServoForwardCommand());

  //  //debugController.rightBumper().whileTrue(mElevatorMechanism.ElevatorDownLimitCmd());
  //  //debugController.leftBumper().whileTrue(mElevatorMechanism.ElevatorUpLimitCmd());
    // debugController.povLeft().whileTrue(m_AlgaeIntake.IntakeForwardCmd());
    // debugController.povRight().whileTrue(mAlgaeMechanism.AlgaeBackwardCmd());
  //   debugController.povUp().whileTrue(mAlgaeMechanism.AlgaeIntakeGet().IntakeBackwardCmd());
  //   //debugController.a().whileTrue(IntakeAndRaise);
  //   debugController.x().whileTrue(mAlgaeMechanism.AlgaePIDDownThroughBore());
  //   debugController.a().whileTrue(mAlgaeMechanism.AlgaePIDUpThroughBore());
    debugController.povUp().onTrue(mAlgaeMechanism.resetAlgae());
    debugController.a().onTrue(algaePIDUp);
    debugController.b().onTrue(algaePIDMiddle);
    debugController.x().onTrue(algaePIDDown);
    debugController.y().onTrue(algaePIDGround);
    debugController.rightBumper().whileTrue(mClimbMechanism.WindForwardCmd());
    debugController.leftBumper().whileTrue(mClimbMechanism.WindDownCmd());
    debugController.rightStick().onTrue(algaeDownAndRunA4);
    // debugController.leftBumper().onTrue(algaeUpAndStop);
    debugController.povDown().onTrue(elevatorPIDAlgae3);
    debugController.leftStick().whileTrue(m_AlgaeIntake.IntakeBackwardCmd());
    debugController.start().onTrue(algaeMiddleAndStop);
    debugController.leftTrigger(.5).onTrue(algaeGroundCommand);
    
    // debugController.povDown().onTrue(algaeL3Down);
  //   debugController.leftBumper().whileTrue(mClimbMechanism.WindBackwardCmd());
  //   debugController.rightBumper().whileTrue(mClimbMechanism.WindForwardCmd());
  
  }

  private void configureShuffleboard() {
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData(mElevatorMechanism);
   
    
    // Add subsystems
    SmartDashboard.putData(m_DriveTrain);
    SmartDashboard.putData(m_DriveTrain.getName() + "/Reset Pose 2D", m_DriveTrain.resetPose2d());
    SmartDashboard.putData(mCoralMechanism);  
    //SmartDashboard.putData(m_LimelightFrl);
    SmartDashboard.putData(mAlgaeMechanism);
    SmartDashboard.putData(m_AlgaeIntake);
  }

  public Command getAutonomousCommand() {
    // loads New Auto auto file

    Command autoCommand = autoChooser.getSelected();
    return autoCommand;
  }
}
