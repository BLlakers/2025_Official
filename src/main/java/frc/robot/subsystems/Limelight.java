package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final DoubleArraySubscriber aprilTagPoseTopic;
  private final IntegerPublisher priorityTagIdPub;
  private final String limelightName;
  private AprilTag currentAprilTag = new AprilTag(-1, new Pose3d());

  public Limelight() {
    this("limelight");
  }

  public Limelight(String cameraName) {
    limelightName = cameraName;
    NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
    aprilTagPoseTopic = table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[]{0, 0, 0, 0, 0, 0});
    priorityTagIdPub = table.getIntegerTopic("priorityid").publish();
  }

  public void SetTagIDToTrack(int tagID) {
    priorityTagIdPub.accept(tagID);
  }

  @Override
  public void periodic() {
    currentAprilTag = getCurrentAprilTag();
    SmartDashboard.putNumber("AprilTag/tagID", currentAprilTag.ID);
    SmartDashboard.putNumber("AprilTag/pose/X", currentAprilTag.pose.getX());
    SmartDashboard.putNumber("AprilTag/pose/Y", currentAprilTag.pose.getY());
    SmartDashboard.putNumber("AprilTag/pose/Z", currentAprilTag.pose.getZ());
    SmartDashboard.putNumber("AprilTag/pose/rotX", Math.toDegrees(currentAprilTag.pose.getRotation().getX()));
    SmartDashboard.putNumber("AprilTag/pose/rotY", Math.toDegrees(currentAprilTag.pose.getRotation().getY()));
    SmartDashboard.putNumber("AprilTag/pose/rotZ", Math.toDegrees(currentAprilTag.pose.getRotation().getZ()));
  }

  /**
   * This function gets the april tag the camera is viewing.
   *
   * @return The AprilTag the camera is looking at plus a Pose3d - x, y, z, ROTx, ROTy, ROTz.
   */
  public AprilTag getCurrentAprilTag() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
    NetworkTableEntry tid = table.getEntry("tid");
    int aprilTagId = (int) tid.getInteger(-1);
    TimestampedDoubleArray poseArray = aprilTagPoseTopic.getAtomic(); // (x, y, z, rotx, roty, rotz)

    if (poseArray.value.length < 6) return new AprilTag(-1, new Pose3d());

    Translation3d poseTranslation = new Translation3d(poseArray.value[0], // x
            poseArray.value[1], // y
            poseArray.value[2] // z
    );

    Rotation3d poseOrientation = new Rotation3d(poseArray.value[3], // roll = rotx
            poseArray.value[4], // pitch = roty
            poseArray.value[5] // yaw = rotz
    );

    Pose3d aprilTagPose = new Pose3d(poseTranslation, poseOrientation); // creating pose3d based off of our
    // translation3d and rot3d and tid
    return new AprilTag(aprilTagId, aprilTagPose);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("AprilTag/tagID", () -> currentAprilTag.ID, null);
    builder.addDoubleProperty("AprilTag/pose/X", currentAprilTag.pose::getX, null);
    builder.addDoubleProperty("AprilTag/pose/Y", currentAprilTag.pose::getY, null);
    builder.addDoubleProperty("AprilTag/pose/Z", currentAprilTag.pose::getZ, null);
    builder.addDoubleProperty("AprilTag/pose/rotX", currentAprilTag.pose.getRotation()::getX, null);
    builder.addDoubleProperty("AprilTag/pose/rotY", currentAprilTag.pose.getRotation()::getY, null);
    builder.addDoubleProperty("AprilTag/pose/rotZ", currentAprilTag.pose.getRotation()::getZ, null);
  }
}
