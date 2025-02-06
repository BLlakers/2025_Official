// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.support.SparkMaxEncoderStrategy;

/**
 * This is the code to run a single swerve module <br>
 * <br>
 * It is called by the Drivetrain subsysem
 */
public class SwerveModule extends SubsystemBase {

  private static final double FULL_RANGE = 2*Math.PI;

  private static final double kPositionConversionFactor =
      (Constants.Conversion.kWheelDiameterM * Math.PI) / Constants.Conversion.DriveGearRatio;
  private static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

  // kWheelCircumference used to be
  public static final double kDriveMaxSpeed = Units.feetToMeters(12.5);
  public static final double kModuleMaxAngularVelocity = DriveTrain.kMaxAngularSpeed;
  public static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  public final SparkMax m_driveMotor;
  public final SparkMax m_turningMotor;
  private final RelativeEncoder driverMotorEncoder;

  private SparkMaxConfig config;

  private final SparkMaxEncoderStrategy sparkMaxEncoderStrategy;

  public final DutyCycleEncoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel CAN ID for the drive motor.
   * @param turningMotorChannel CAN ID for the turning motor
   * @param turnEncoderPWMChannel DIO input for the drive encoder channel B
   * @param turnOffset offset from 0 to 1 for the home position of the encoder
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turnEncoderPWMChannel,
      double turnOffset) {
    // can spark max motor controller objects
    m_driveMotor =
        new SparkMax(driveMotorChannel, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    //m_driveMotor.restoreFactoryDefaults();

    this.sparkMaxEncoderStrategy = new SparkMaxEncoderStrategy(m_driveMotor);

    this.driverMotorEncoder = m_driveMotor.getEncoder();

    m_turningMotor =
        new SparkMax(turningMotorChannel, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    //m_turningMotor.restoreFactoryDefaults();

    //m_driveMotor.setOpenLoopRampRate(0.1);


    //m_drivePID = m_driveMotor.getPIDController();
    // m_drivePID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    // m_drivePID.setSmartMotionMaxAccel(0.2, 0);

    // spark max built-in encoder
    //m_driveEncoder.setPositionConversionFactor(kPositionConversionFactor); // meters
    //w.setVelocityConversionFactor(kVelocityConversionFactor); // meters per second
    //m_driveEncoder.setPosition(0);

    // PWM encoder from CTRE mag encoders
    m_turningEncoder = new DutyCycleEncoder(turnEncoderPWMChannel, FULL_RANGE, turnOffset);
    m_turningEncoder.setAssumedFrequency(242);

    //**To do: What do we do with these in new api? **/
    // m_turningEncoder.reset();
    // m_turningEncoder.setPositionOffset(turnOffset);
    // m_turningEncoder.setDistancePerRotation(2 * Math.PI); // radians ?

    this.config = new SparkMaxConfig();

    config
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    config.encoder
      .positionConversionFactor(kPositionConversionFactor)
      .velocityConversionFactor(kVelocityConversionFactor);
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(1.0,0,0);

    m_driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module. <pi> This takes a current velocity for each diffrent
   * drive encoder and a current angle.
   *
   * @return The current state of each Swerve Module. --> The speed and angle of a Module
   */
  public SwerveModuleState getModuleState() {
    // the getVelocity() function normally returns RPM but is scaled in the
    // SwerveModule constructor to return actual wheel speed
    return new SwerveModuleState(
      m_driveMotor.getEncoder().getVelocity(), Rotation2d.fromRadians(m_turningEncoder.get()));
  }

  /**
   * This gets a current Position (Distance per rotation in meters) for each diffrent drive encoder
   * and a current angle from the Duty Cycle encoder.
   *
   * @return The current Position of each Swerve Module
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
            this.sparkMaxEncoderStrategy.getPosition(), Rotation2d.fromRadians(m_turningEncoder.get()));
  }

  /**
   * Sets the desired state for the module.
   *
   * <p>This means the speed it should be going and the angle it should be going.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(getModulePosition().angle);
    //SwerveModuleState optimizedState = desiredState.optimize(getModulePosition().angle);

    final double signedAngleDifference =
        closestAngleCalculator(
            getModulePosition().angle.getRadians(), desiredState.angle.getRadians());
    double rotateMotorPercentPower =
        signedAngleDifference / (2 * Math.PI); // proportion error control

    double driveMotorPercentPower = desiredState.speedMetersPerSecond / kDriveMaxSpeed;
    double turnMotorPercentPower = 1.6 * rotateMotorPercentPower;

    SmartDashboard.putNumber(
        "DriveTrain/"
            + getName()
            + "/Drive Encoder/ID: "
            + m_driveMotor.getDeviceId()
            + "/DrivePercent",
        driveMotorPercentPower);
    SmartDashboard.putNumber(
        "DriveTrain/"
            + getName()
            + "/Turn Encoder/ID: "
            + m_turningMotor.getDeviceId()
            + "/DrivePercent",
        turnMotorPercentPower);
    SmartDashboard.putNumber(
        "DriveTrain/"
            + getName()
            + "/Turn Encoder/SignedAngleBLAHBLAHBLAH:"
            + m_turningMotor.getDeviceId()
            + "/Angle",
        signedAngleDifference);
    SmartDashboard.putNumber(
        "DriveTrain/"
            + getName()
            + "/Turn Encoder/DesiredState:"
            + m_turningMotor.getDeviceId()
            + "/DesiredAngle",
          desiredState.angle.getRadians());
          SmartDashboard.putNumber(
            "DriveTrain/"
                + getName()
                + "/Turn Encoder/CurrentState:"
                + m_turningMotor.getDeviceId()
                + "/DesiredAngle",
              getModulePosition().angle.getRadians());
    m_driveMotor.set(driveMotorPercentPower);
    m_turningMotor.set(turnMotorPercentPower);
  }

  /**
   * Calculates the closest angle and direction between two points on a circle.
   *
   * @param currentAngle
   *     <ul>
   *       <li>where you currently are
   *     </ul>
   *
   * @param desiredAngle
   *     <ul>
   *       <li>where you want to end up
   *     </ul>
   *
   * @return
   *     <ul>
   *       <li>signed double of the angle (rad) between the two points
   *     </ul>
   */
  public double closestAngleCalculator(double currentAngle, double desiredAngle) {
    double signedDiff = 0.0;
    double rawDiff =
        currentAngle > desiredAngle
            ? currentAngle - desiredAngle
            : desiredAngle - currentAngle; // find the positive raw distance between the angles
    double modDiff = rawDiff % (2 * Math.PI); // constrain the difference to a full circle
    if (modDiff > Math.PI) { // if the angle is greater than half a rotation, go backwards
      signedDiff = ((2 * Math.PI) - modDiff); // full circle minus the angle
      if (desiredAngle > currentAngle)
        signedDiff = signedDiff * -1; // get the direction that was lost calculating raw diff
    } else {
      signedDiff = modDiff;
      if (currentAngle > desiredAngle) signedDiff = signedDiff * -1;
    }
    return signedDiff;
  }

  /** Tells the drive and turning motor to stop */
  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.publishConstInteger("TurnMotor/ID", m_turningMotor.getDeviceId());
    builder.publishConstInteger("DriveMotor/ID", m_driveMotor.getDeviceId());

    builder.addDoubleProperty(
        "TurnMotor/Angle", () -> Units.radiansToDegrees(m_turningEncoder.get()), null);
    builder.addDoubleProperty("DriveMotor/Pos", this.sparkMaxEncoderStrategy::getPosition, null);
    builder.addDoubleProperty("DriveMotor/Vel", this.driverMotorEncoder::getVelocity, null);

    builder.addDoubleProperty(
        "TurnMotor/Encoder/AbsolutePosition", this.m_turningEncoder::get, null);

    builder.setSafeState(this::stop);
  }
}
