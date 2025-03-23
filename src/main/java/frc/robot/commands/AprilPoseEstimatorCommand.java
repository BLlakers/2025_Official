package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LedStrand;
import frc.robot.support.limelight.LimelightHelpers;

public class AprilPoseEstimatorCommand extends Command{
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(.5, .5);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(.5, .5);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(Units.degreesToRadians(400), Units.degreesToRadians(360));

  private final ProfiledPIDController m_xController =
      new ProfiledPIDController(1.25, 0, 0.0, X_CONSTRAINTS);
  private final ProfiledPIDController m_yController =
      new ProfiledPIDController(1.45, 0, 0.0, Y_CONSTRAINTS);
  private final ProfiledPIDController m_omegaController =
      new ProfiledPIDController(7, 0, 0.0, OMEGA_CONSTRAINTS);
   
  private DriveTrain m_drivetrain;
  private Supplier<SwerveDrivePoseEstimator> m_currentEstimatedPose;
  private Supplier<AprilTag> m_currentAprilTag;
  
  private Boolean m_isLeft;

  private double m_goalX;
  private double m_goalY;
  private double m_goalRot;

  public AprilPoseEstimatorCommand(Supplier<SwerveDrivePoseEstimator> currentEstimatedPose, Supplier<AprilTag> currentAprilTag, boolean isLeft, DriveTrain drivetrainSubsystem){
    m_currentEstimatedPose = currentEstimatedPose;
    m_currentAprilTag = currentAprilTag;
    m_isLeft = isLeft;
    m_drivetrain = drivetrainSubsystem;

    m_xController.setTolerance(0.1);
    m_yController.setTolerance(0.1);
    m_omegaController.setTolerance(Units.degreesToRadians(1));
    m_omegaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivetrainSubsystem);
}

@Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_drivetrain.m_FieldRelativeEnable = false;
    AprilTag aprilTag = m_currentAprilTag.get();

    if(getGoalPose(aprilTag.ID).getX() == -99999){
      m_drivetrain.stopModules();
      return;
    } 

    if (aprilTag.ID
        <= 0) { // is valid if > 0: we update our current estimate of where the april tag is
      // relative to the robot
      m_drivetrain.stopModules();
      return;
    }

    double xSpeed = 0; 
    double ySpeed = 0;
    double rotSpeed = 0;
    m_drivetrain.drive(xSpeed, ySpeed, rotSpeed);

  }



  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopModules();
    m_drivetrain.m_FieldRelativeEnable = true;
  }

  @Override
  public boolean isFinished() {
     return m_omegaController.atGoal() && m_xController.atGoal() && m_yController.atGoal();
  }

  public Pose2d getGoalPose(int currentAprilTagID){
    Pose2d goalPose2d;
    switch (currentAprilTagID) {
      case 1:
        goalPose2d = new Pose2d(0,0, new Rotation2d());
        break;
      case 2:
        goalPose2d = new Pose2d(0,0, new Rotation2d());
        break;
      case 3:
        goalPose2d = new Pose2d(0,0, new Rotation2d());
        break;
      case 4:
        goalPose2d = new Pose2d(0,0, new Rotation2d());
        break;
      case 5:
        goalPose2d = new Pose2d(0,0, new Rotation2d());
        break;
      case 6:
        goalPose2d = new Pose2d(0,0, new Rotation2d());
        break;
      default:
        goalPose2d = new Pose2d(-99999,0, new Rotation2d());
        break;
    }
    return goalPose2d;
  }
}
