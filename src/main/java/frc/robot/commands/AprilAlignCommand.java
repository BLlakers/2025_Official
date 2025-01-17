package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AprilAlignCommand extends Command{
private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(1, 2); // TODO DO 1 PID AT A TIME !!!!!
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(1, 2); // TODO DO 1 PID AT A TIME !!!!!
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          Units.degreesToRadians(60), 8); // TODO DO 1 PID AT A TIME !!!!!

  private final ProfiledPIDController xController =
      new ProfiledPIDController(1.0, 0, 0.0, X_CONSTRAINTS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(1.0, 0, 0.0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController =
      new ProfiledPIDController(0.8, 0, 0.0, OMEGA_CONSTRAINTS); 
      
  private DriveTrain m_drivetrain;
  private Supplier<AprilTag> m_aprilTagProvider;

  // fields for tracking
  private Pose2d m_goalPose;
  private Transform2d m_tagToGoal;

  public AprilAlignCommand(Supplier<AprilTag> aprilTagSupplier, DriveTrain drivetrainSubsystem, Transform2d
  goalTransformRelativeToAprilTag){
    m_aprilTagProvider = aprilTagSupplier;
    m_drivetrain = drivetrainSubsystem;
    m_tagToGoal = goalTransformRelativeToAprilTag;
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
}
@Override
  public void initialize() {
    m_goalPose = null;
    var robotPose = m_drivetrain.getPose2d();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }
  @Override
  public void execute() {
    Pose2d robotPose = m_drivetrain.getPose2d();
    AprilTag aprilTag = m_aprilTagProvider.get();
    if (aprilTag.ID
        <= 0) { // is valid if > 0: we update our current estimate of where the april tag is
      // relative to the robot
      m_drivetrain.stopModules();
      return;
    }
    
  }




  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopModules();
  }

  @Override
  public boolean isFinished() {
    return omegaController.atGoal() && xController.atGoal() && yController.atGoal();
  }
}
