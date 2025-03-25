package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LedStrand;
import frc.robot.support.limelight.LimelightHelpers;

public class AprilAlignHoloCommand extends Command{
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(Units.degreesToRadians(720), Units.degreesToRadians(360));

  private final PIDController m_xController = new PIDController(1, 0, 0);
  private final PIDController m_yController = new PIDController(1.2, 0, 0);
  private final ProfiledPIDController m_omegaController = new ProfiledPIDController(1, 0, 0.0, OMEGA_CONSTRAINTS);
  
  private final HolonomicDriveController drive_controller = new HolonomicDriveController(m_xController,m_yController, m_omegaController); 

  private Pose2d m_goalPose;
  private String m_limelightname;
  private DriveTrain m_drivetrain;
  

  public AprilAlignHoloCommand(Pose2d goalPose, String limelightname, DriveTrain drivetrainSubsystem){
    m_goalPose = goalPose;
    m_limelightname = limelightname;
    m_drivetrain = drivetrainSubsystem;
  
    addRequirements(drivetrainSubsystem);
}

@Override
  public void initialize() {
    drive_controller.setTolerance(new Pose2d(.25,.25,new Rotation2d(Math.toRadians(10))));
  }

  @Override
  public void execute() {
    m_drivetrain.m_FieldRelativeEnable = false;


    if( LimelightHelpers.getTV(m_limelightname)){
      double[] postions = LimelightHelpers.getBotPose_TargetSpace(m_limelightname);

      SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignHoloCommand/x", postions[2]);
      SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignHoloCommand/y", postions[0]);
      SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignHoloCommand/omega", postions[4]);


      ChassisSpeeds chassisSpeeds = drive_controller.calculate(new Pose2d(postions[2], -postions[0], new Rotation2d(Math.toDegrees(-postions[4]))),m_goalPose, .5, new Rotation2d(Math.toRadians(0)) );
      SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignHoloCommand/xSpeed", chassisSpeeds.vxMetersPerSecond);
      SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignHoloCommand/ySped", chassisSpeeds.vyMetersPerSecond);
      SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignHoloCommand/omegaSpeed", chassisSpeeds.omegaRadiansPerSecond);


      // m_drivetrain.driveChassisSpeeds(chassisSpeeds);
    }

    m_drivetrain.driveChassisSpeeds(new ChassisSpeeds());
  }




  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopModules();
    m_drivetrain.m_FieldRelativeEnable = true;
  }

  @Override
  public boolean isFinished() {
    return drive_controller.atReference();
  }
}

