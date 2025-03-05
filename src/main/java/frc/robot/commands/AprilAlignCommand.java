package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

public class AprilAlignCommand extends Command{
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(.5, 1);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(.5, 1);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(360));

  private final ProfiledPIDController m_xController =
      new ProfiledPIDController(1, 0, 0.0, X_CONSTRAINTS);
  private final ProfiledPIDController m_yController =
      new ProfiledPIDController(1, 0, 0.0, Y_CONSTRAINTS);
  private final ProfiledPIDController m_omegaController =
      new ProfiledPIDController(5, 0, 0.0, OMEGA_CONSTRAINTS);
      
  private DriveTrain m_drivetrain;
  private Supplier<AprilTag> m_aprilTagProvider;

  //Rotation2d of AprilTag
  private Supplier<Rotation2d> m_aprilRotation;
  
  private Boolean m_isBackwards;
  private Boolean m_isLeft;
  private LedStrand mLedStrand;

  private double m_goalX;
  private double m_goalY;
  private double m_goalRot;

  public AprilAlignCommand(Supplier<AprilTag> aprilTagSupplier, Supplier<Rotation2d> aprilTagRotation2d, DriveTrain drivetrainSubsystem, Transform2d
  goalTransformRelativeToAprilTag, boolean isBackwards, Boolean isLeft, LedStrand leds){
    m_aprilTagProvider = aprilTagSupplier;
    m_drivetrain = drivetrainSubsystem;
    m_aprilRotation = aprilTagRotation2d;
    m_isBackwards = isBackwards;
    m_isLeft = isLeft;
    mLedStrand = leds;
    m_goalX = goalTransformRelativeToAprilTag.getX();
    m_goalY = goalTransformRelativeToAprilTag.getY();
    m_goalRot =goalTransformRelativeToAprilTag.getRotation().getRadians();


    m_xController.setTolerance(0.09);
    m_yController.setTolerance(0.015);
    m_omegaController.setTolerance(Units.degreesToRadians(2.5));
    m_omegaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivetrainSubsystem);
    addRequirements(leds);
}

@Override
  public void initialize() {
    mLedStrand.changeLed(0, 255, 0);
  }

  @Override
  public void execute() {
    
    AprilTag aprilTag = m_aprilTagProvider.get();
    double aprilSkew;
    if (aprilTag.ID
        <= 0) { // is valid if > 0: we update our current estimate of where the april tag is
      // relative to the robot
      m_drivetrain.stopModules();
      return;
    }

    m_xController.setGoal(m_goalX); 
    m_yController.setGoal(m_goalY);
    m_omegaController.setGoal(m_goalRot);


    double xSpeed = m_xController.calculate(aprilTag.pose.getY());
    if (m_xController.atGoal()) {
      xSpeed = 0;
    }
    double ySpeed = m_yController.calculate(m_aprilRotation.get().getRadians());
    if (m_yController.atGoal()) {
      ySpeed = 0;
    }

    if(m_isBackwards){
      aprilSkew = Math.toRadians(LimelightHelpers.getTX("limelight-back"));
    }else if (m_isLeft){
      aprilSkew = Math.toRadians(LimelightHelpers.getTX("limelight-frl"));
    } else{
      aprilSkew = Math.toRadians(LimelightHelpers.getTX("limelight-frr"));
    }
    double rotSpeed = m_omegaController.calculate(aprilSkew);
    if (m_omegaController.atGoal()) {
      rotSpeed = 0;
    }
    m_drivetrain.m_FieldRelativeEnable = false; 
    if(m_isBackwards){
      m_drivetrain.drive(xSpeed,-1* ySpeed, rotSpeed);
    }else {
      m_drivetrain.drive(-xSpeed, ySpeed, rotSpeed);
      // m_drivetrain.drive(0, 0, rotSpeed);
      // m_drivetrain.drive(0, ySpeed, rotSpeed);
    }

    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/CalcVelX", xSpeed);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/CalcVelY", ySpeed);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/CalcVelRot", rotSpeed);

    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/MeasurementX", aprilTag.pose.getX());
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/MeasurementY", aprilTag.pose.getY());
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/MeasurementRot", Math.toRadians(LimelightHelpers.getTX("limelight-frr")));
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/MeasurementSkew", m_aprilRotation.get().getRadians());

    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/NEWGoalXController", m_goalX);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/NEWGoalOmegaController", m_goalRot);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/NEWGoalYController",m_goalY);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/NEWMeasurementXController", aprilTag.pose.getY());
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/NEWMeasurementOmegaControllerFRR", Math.toRadians(LimelightHelpers.getTX("limelight-frr")));
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/NEWMeasurementOmegaControllerFrl", Math.toRadians(LimelightHelpers.getTX("limelight-frl")));
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/NEWMeasurementYController", m_aprilRotation.get().getRadians());
    SmartDashboard.putBoolean(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/NEWAtGoalXController", m_xController.atGoal());
    SmartDashboard.putBoolean(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/NEWAtGoalOmegaController", m_omegaController.atGoal());
    SmartDashboard.putBoolean(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/NEWAtGoalYController",  m_yController.atGoal());
    SmartDashboard.putBoolean(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/NEWFullyAtGoal",  isFinished());

    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/GoalX", m_xController.getGoal().position);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/GoalY", m_yController.getGoal().position);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommandV2/Command/GoalSkew", m_omegaController.getGoal().position);

    

  }



  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopModules();
    m_drivetrain.m_FieldRelativeEnable = true;
    mLedStrand.changeLed(128,0,0);
  }

  @Override
  public boolean isFinished() {
    return m_omegaController.atGoal() && m_xController.atGoal() && m_yController.atGoal();
  }
}
