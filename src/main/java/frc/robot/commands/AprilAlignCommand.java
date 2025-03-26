package frc.robot.commands;

import java.util.function.Supplier;

import org.w3c.dom.traversal.DocumentTraversal;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LedStrand;
import frc.robot.support.limelight.LimelightHelpers;

public class AprilAlignCommand extends Command{
double XkP = 1;
double XkD = 1;
double XkI = 1;
double YkP = 1;
double YkD = 1;
double YkI = 1;
double RotkP = 1;
double RotkD = 1;
double RotkI = 1;
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(.5, .5);
      private static final TrapezoidProfile.Constraints BACK_XCONSTRAINTS =
      new TrapezoidProfile.Constraints(3, 10);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(.5, .5);
      private static final TrapezoidProfile.Constraints BACK_YCONSTRAINTS =
      new TrapezoidProfile.Constraints(1.5, 5);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(Units.degreesToRadians(720), Units.degreesToRadians(360));

  private final ProfiledPIDController m_xController =
      new ProfiledPIDController(1.25, 0, 0.0, X_CONSTRAINTS);
  private final ProfiledPIDController m_yController =
      new ProfiledPIDController(1.45, 0, 0.0, Y_CONSTRAINTS);
  private final ProfiledPIDController m_omegaController =
      new ProfiledPIDController(10, 0, 0.0, OMEGA_CONSTRAINTS);
  private final ProfiledPIDController Back_xController = new ProfiledPIDController(1.1, 0, 0.0, BACK_XCONSTRAINTS);
  private final ProfiledPIDController Back_yController = new ProfiledPIDController(.8, 0, 0.0, BACK_YCONSTRAINTS);
  private final ProfiledPIDController Back_omegaController = new ProfiledPIDController(4, 0, 0.0, OMEGA_CONSTRAINTS);
  
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
  private HolonomicDriveController driveController = new HolonomicDriveController(new PIDController(1.7, 0, 0),new PIDController(.65, 0, 0), m_omegaController);


public void mXkP(double d){
  XkP =d;
}
public void mXkD(double d){
  XkD =d;
}public void mXkI(double d){
  XkI =d;
}public void mYkP(double d){
  YkP =d;
}public void mYkD(double d){
  YkD =d;
}public void mYkI(double d){
  YkI =d;
}public void mRotkP(double d){
  RotkP =d;
}public void mRotkD(double d){
  RotkD =d;
}public void mRotkI(double d){
  RotkI =d;
}




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


  
    driveController.setTolerance(new Pose2d(.05,.05, new Rotation2d(Math.toRadians(5))));
    addRequirements(drivetrainSubsystem);
    addRequirements(leds);
}

@Override
  public void initialize() {
    mLedStrand.changeLed(0, 255, 0);
  }

  @Override
  public void execute() {
    ChassisSpeeds speed = new ChassisSpeeds();
    m_drivetrain.m_FieldRelativeEnable = false;
    AprilTag aprilTag = m_aprilTagProvider.get();
    double aprilSkew;
    if (aprilTag.ID
        <= 0) { // is valid if > 0: we update our current estimate of where the april tag is
      // relative to the robot
      m_drivetrain.stopModules();
      return;
    }
    if (m_isBackwards){
    aprilSkew = Math.toRadians(LimelightHelpers.getTX("limelight-back"));
  }    else if (m_isLeft){
      aprilSkew = Math.toRadians(LimelightHelpers.getTX("limelight-frl"));
    } else{
      aprilSkew = Math.toRadians(LimelightHelpers.getTX("limelight-frr"));
    }

    speed = driveController.calculate(new Pose2d(aprilTag.pose.getY(), m_aprilRotation.get().getRadians(), new Rotation2d(aprilSkew)),
    new Pose2d(m_goalX, m_goalY, new Rotation2d(m_goalRot)), 1, new Rotation2d());
    m_drivetrain.driveChassisSpeeds(new ChassisSpeeds( -speed.vxMetersPerSecond, speed.vyMetersPerSecond, speed.omegaRadiansPerSecond));
    // rotSpeed = m_omegaController.calculate(aprilSkew);
    // if (m_omegaController.atGoal()) {
      // rotSpeed = 0;
    // }
    // m_drivetrain.drive(-xSpeed, ySpeed, rotSpeed);
  

   
   
     
        
      // m_drivetrain.drive(0, 0, rotSpeed);
      // m_drivetrain.drive(0, ySpeed, rotSpeed);
    

    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/GoalX", m_goalX);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/ChassisGoalErrorX", driveController.getXController().getError());
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/ChassisSetpointX", driveController.getXController().getSetpoint());
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/ChassisToleranceX", driveController.getXController().getErrorTolerance());
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/GoalY", m_goalY);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/GoalRot", m_goalRot);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/x", aprilTag.pose.getY());
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/y", m_aprilRotation.get().getRadians());
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/omega", aprilSkew);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/CalcVelX", -speed.vxMetersPerSecond);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/CalcVelY", speed.vyMetersPerSecond);
    SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignCommand/Command/CalcVelRot", speed.omegaRadiansPerSecond);
  }



  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopModules();
    m_drivetrain.m_FieldRelativeEnable = true;
    mLedStrand.changeLed(128,0,0);
  }

  @Override
  public boolean isFinished() {
    return driveController.atReference();
}
@Override
public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(getName() + "driveController/X/P" , ()->driveController.getXController().getP(), (s)-> mXkP(s));
    builder.addDoubleProperty(getName() + "driveController/X/D" , ()->driveController.getXController().getD(), (s)-> mXkD(s));
    builder.addDoubleProperty(getName() + "driveController/X/I" , ()->driveController.getXController().getI(), (s)-> mXkI(s));
    builder.addDoubleProperty(getName() + "driveController/Y/P" , ()->driveController.getYController().getP(), (s)-> mYkP(s));
    builder.addDoubleProperty(getName() + "driveController/Y/D" , ()->driveController.getYController().getD(), (s)-> mYkD(s));
    builder.addDoubleProperty(getName() + "driveController/Y/I" , ()-> driveController.getYController().getI(),(s)-> mYkI(s));
    builder.addDoubleProperty(getName() + "driveController/Rot/P" ,()->driveController.getThetaController().getP(), (s)-> mRotkP(s));
    builder.addDoubleProperty(getName() + "driveController/Rot/D" ,()->driveController.getThetaController().getD(), (s)-> mRotkD(s));
    builder.addDoubleProperty(getName() + "driveController/Rot/I" ,()->driveController.getThetaController().getI(), (s)-> mRotkI(s));
}
}
