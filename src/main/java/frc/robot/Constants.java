package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.DriveTrain;
import frc.robot.support.RobotVersion;

public final class Constants {
  public static class Drive {

    public static final Translation2d SMFrontRightLocation = new Translation2d(0.285, -0.285);
    public static final Translation2d SMFrontLeftLocation = new Translation2d(0.285, 0.285);
    public static final Translation2d SMBackLeftLocation = new Translation2d(-0.285, 0.285);
    public static final Translation2d SMBackRightLocation = new Translation2d(-0.285, -0.285);
    //public static final PPHolonomicDriveController pathFollowerConfig =
    //    new PPHolonomicDriveController(
    //        new PIDConstants(5, 0, 0), // Translation constants
    //        new PIDConstants(3, 0, 0), // Rotation constants
    //        3.68, // what should be our robots fastest chassis speeds in m/s
    //        0.3875, // The radius of the robot in meters
    //        new ReplanningConfig());
    public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(
            0,
            0,
            0,
            new Rotation3d(
                0, 0,
                0)); // we do conversion in limelight. would normally tell robot where the camera
    // is relative to the center of the bot.
    public static final TrapezoidProfile.Constraints kthetaController =
        new TrapezoidProfile.Constraints(
            DriveTrain.kMaxAngularSpeed, DriveTrain.kModuleMaxAngularAcceleration);
  }

  public static class Conversion {
    public static final double driveEncoderCtsperRev = 6.8;
    public static final double kWheelDiameterM = Inches.of(4).in(Meters);
    public static final double kWheelCircumference = Math.PI * kWheelDiameterM;
    public static final double NeoEncoderCountsPerRev = 42;
    public static final double NeoRevPerEncoderCounts = 1 / NeoEncoderCountsPerRev;
    public static final double NeoMaxSpeedRPM = 5820;
    public static final double MagEncoderCountsPerRev = 4096;
    public static final double MagRevPerEncoderCounts = 1 / MagEncoderCountsPerRev;
    public static final double DriveGearRatio = 8.14;
    public static final double TurnGearRatio = 12.8;
    public static final double driveEncoderConversion = DriveGearRatio * kWheelCircumference;
  }

  public static class Controller {
    public static final int DriverControllerChannel = 0;
    public static final int ManipControllerChannel = 1;
    public static final int DebugControllerChannel = 2;
    public static final int buttonA = 1;
    public static final int buttonB = 2;
    public static final int buttonX = 3;
    public static final int buttonY = 4;
    public static final int buttonLeft = 5;
    public static final int buttonRight = 6;
    public static final int buttonOptions = 7;
    public static final int buttonStart = 8;
    public static final int buttonLS = 9;
    public static final int buttonRS = 10;
    public static final double deadzone = 0.17;
    public static final double RTdeadzone = .01;
  }

  public static class AprilTagID {
    public static final int PracticeSpeakerCenter = 1;
    public static final int BlueSpeakerCenter = 7;
    public static final int RedSpeakerCenter = 4;

    public static final int BlueStageCenter = 14;
    public static final int RedStageCenter = 13;

    public static final int BlueStageLeft = 15;
    public static final int RedStageRight = 12;

    public static final int RedStageLeft = 11;
    public static final int BlueStageRight = 16;

    public static final Pose2d BlueSpeakerCenterPose = new Pose2d(); // TODO
    public static final Pose2d RedSpeakerCenterPose = new Pose2d(); // TODO

    public static final PathConstraints pathConstraints = new PathConstraints(2, 3, 360, 540);
  }

  public static class Port {
    public static final int blSteerMtrC = 1;
    public static final int blDriveMtrC = 2;
    public static final int flDriveMtrC = 3;
    public static final int flSteerMtrC = 4;
    public static final int frSteerMtrC = 5;
    public static final int frDriveMtrC = 6;
    public static final int brDriveMtrC = 7;
    public static final int brSteerMtrC = 8;
    public static final int m_ElevatorMtrC = 11;
    public static final int m_ClimbMtrC = 12;
    public static final int m_CoralMtrRC = 13;
    public static final int m_CoralMtrLC = 14;
    public static final int m_Follower = 15;
    public static final int blTurnEncoderDIOC = 0;
    public static final int flTurnEncoderDIOC = 1;
    public static final int frTurnEncoderDIOC = 2;
    public static final int brTurnEncoderDIOC = 3;
    public static final int hangerLeftMagSwitchDIOC = 7;
    public static final int hangerRightMagSwitchDIOC = 8;
    public static final int climbMagSwitchDIOC = 4;
    public static final int PHChannel = 30; // REV Pneumatic Hub
    public static final int PDHChannel = 20; // REV Power Distribution Hub
  }
  public static class Algae {
    public static final int m_IntakeMtrC = 9;
    public static final int m_AlgaeMtrC = 10;
  }

  // SHOOTER
  public static class Shooter {
    public static final int LeftMtrC = 11;
    public static final int RightMtrC = 12;
    public static final int AngleMtrC = 13;
    public static final int LimitSwitchTopDIO = 4;
    public static final int LimitSwitchBottomDIO =
        -1; // TODO: add the digital input channel for this limit
  }

  // HANGER
  public static class Hanger {
    public static final int LeftMtrC = 9;
    public static final int RightMtrC = 10;
  }

  public abstract class RobotVersionConstants {
    public static final double flTurnEncoderOffset = 0;
    public static final double frTurnEncoderOffset = 0;
    public static final double blTurnEncoderOffset = 0;
    public static final double brTurnEncoderOffset = 0;
  }

  public class RobotVersion2025 extends RobotVersionConstants {
    public static final double flTurnEncoderOffset = 3.84-.04 + Math.PI;
    public static final double frTurnEncoderOffset = 1.7 + Math.PI - .03 + Math.PI;
    public static final double blTurnEncoderOffset = 3.284 + Math.PI;
    public static final double brTurnEncoderOffset = 4.49 + Math.PI;
  }

  public class RobotVersion2023 extends RobotVersionConstants {
    public static final double flTurnEncoderOffset = 5.3038;
    public static final double frTurnEncoderOffset = Math.PI/2 - 0.1242 - .05759;
    public static final double blTurnEncoderOffset = 4.2+0.0385;
    public static final double brTurnEncoderOffset = 2.736-.06098;
  }
  public static final class Poses {
  

 public static final Pose2d SixLeft = new Pose2d(13.714,2.868, new Rotation2d(Math.toRadians(120)));
 public static final Pose2d SixRight = new Pose2d(13.930,3.012, new Rotation2d(Math.toRadians(120)));
 public static final Pose2d SevenLeft = new Pose2d(14.373,4.019, new Rotation2d(Math.toRadians(180)));
 public static final Pose2d SevenRight = new Pose2d(14.373,4.199, new Rotation2d(Math.toRadians(180)));
 public static final Pose2d EightLeft = new Pose2d(13.738,5.158, new Rotation2d(Math.toRadians(-120)));
 public static final Pose2d EightRight = new Pose2d(13.570,5.266, new Rotation2d(Math.toRadians(-120)));
 public static final Pose2d NineLeft = new Pose2d(14.373,5.182, new Rotation2d(Math.toRadians(-60)));
 public static final Pose2d NineRight = new Pose2d(12.263,5.098, new Rotation2d(Math.toRadians(-60)));
 public static final Pose2d TenLeft = new Pose2d(11.736,4.019, new Rotation2d(Math.toRadians(0)));
 public static final Pose2d TenRight = new Pose2d(11.736,3.839, new Rotation2d(Math.toRadians(0)));
 public static final Pose2d ElevenLeft = new Pose2d(12.407,2.904, new Rotation2d(Math.toRadians(60)));
 public static final Pose2d ElevenRight = new Pose2d(12.563,2.796, new Rotation2d(Math.toRadians(60)));
 public static final Pose2d SeventeenLeft = new Pose2d(3.824,2.904, new Rotation2d(Math.toRadians(60)));
 public static final Pose2d SeventeenRight = new Pose2d(3.992,2.820, new Rotation2d(Math.toRadians(60)));
 public static final Pose2d EighteenLeft = new Pose2d(3.165,4.031, new Rotation2d(Math.toRadians(0)));
 public static final Pose2d EighteenRight = new Pose2d(3.177,3.815, new Rotation2d(Math.toRadians(0)));
 public static final Pose2d NineteenLeft = new Pose2d(3.824,5.170, new Rotation2d(Math.toRadians(-60)));
 public static final Pose2d NineteenRight = new Pose2d(3.668,5.086, new Rotation2d(Math.toRadians(-60)));
 public static final Pose2d TwentyLeft = new Pose2d(5.155,5.170, new Rotation2d(Math.toRadians(-60)));
 public static final Pose2d TwentyRight = new Pose2d(5.023,5.242, new Rotation2d(Math.toRadians(-60)));
 public static final Pose2d TwentyoneLeft = new Pose2d(5.814,4.019, new Rotation2d(Math.toRadians(180)));
 public static final Pose2d TwentyoneRight = new Pose2d(5.814,4.307, new Rotation2d(Math.toRadians(180)));
 public static final Pose2d TwentytwoLeft = new Pose2d(5.119,2.880, new Rotation2d(Math.toRadians(120)));
 public static final Pose2d TwentytwoRight = new Pose2d(5.394,3.024, new Rotation2d(Math.toRadians(120)));
 public static final Pose2d defaultGoal = new Pose2d(-99999,0, new Rotation2d());
    
  public static List<Pose2d> Positions = Arrays.asList(SixLeft,SixRight,SevenLeft,SevenRight, EightLeft,EightRight, NineLeft, NineRight, TenLeft, TenRight, ElevenLeft, ElevenRight, SeventeenLeft,SeventeenRight, EighteenLeft, EighteenRight, NineteenLeft, NineteenRight, TwentyLeft, TwentyRight, TwentyoneLeft, TwentyoneRight, TwentytwoLeft, TwentytwoRight);
  public static List<Pose2d> PositionsLeft = Arrays.asList(SixLeft,SevenLeft, EightLeft, NineLeft, TenLeft, ElevenLeft, SeventeenLeft, EighteenLeft, NineteenLeft, TwentyLeft, TwentyoneLeft, TwentytwoLeft);
  public static List<Pose2d> PositionsRight = Arrays.asList(SixRight,SevenRight,EightRight, NineRight, TenRight, ElevenRight, SeventeenRight, EighteenRight, NineteenRight, TwentyRight, TwentyoneRight, TwentytwoRight);
  
  }

  public static final RobotVersion defaultRobotVersion = RobotVersion.v2023;


}
