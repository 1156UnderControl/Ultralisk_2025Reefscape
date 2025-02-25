package frc.Java_Is_UnderControl.Logging.EnhancedLoggers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.Java_Is_UnderControl.Logging.Pose2dLogEntry;

public class CustomPose2dLogger extends Pose2dLogEntry {

  private static boolean isFmsMatch;

  private String name;

  private Pose2d loggedValue;

  private StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic(name, Pose2d.struct).publish();

  public CustomPose2dLogger(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomPose2dLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
    this.loggedValue = new Pose2d(new Translation2d(100, 100), new Rotation2d()); // Set to something different than
                                                                                // default for initial logging
    this.appendRadians(new Pose2d());
  }

  @Override
  public void appendRadians(Pose2d pose) {
    if (!pose.equals(this.loggedValue)) {
      this.loggedValue = pose;
      super.appendRadians(pose);
      if (!CustomPose2dLogger.isFmsMatch) {
        publisher.set(pose);
      }
    }
  }

  @Override
  public void appendDegrees(Pose2d pose) {
    super.appendDegrees(pose);
    if (CustomPose2dLogger.isFmsMatch) {
      publisher.set(pose);
    }
  }

}
