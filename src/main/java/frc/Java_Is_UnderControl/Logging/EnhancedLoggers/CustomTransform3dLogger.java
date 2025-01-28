package frc.Java_Is_UnderControl.Logging.EnhancedLoggers;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Java_Is_UnderControl.Logging.Transform3dLogEntry;

public class CustomTransform3dLogger extends Transform3dLogEntry {
  private static boolean isFmsMatch;

  private String name;

  private Transform3d loggedValue;

  public CustomTransform3dLogger(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomTransform3dLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
    this.appendRadians(new Transform3d());
  }

  @Override
  public void appendRadians(Transform3d pose) {
    if (!pose.equals(this.loggedValue)) {
      this.loggedValue = pose;
      super.appendRadians(pose);
      if (!CustomTransform3dLogger.isFmsMatch) {
        double[] data = new double[4];
        data[0] = pose.getTranslation().getX();
        data[1] = pose.getTranslation().getY();
        data[2] = pose.getZ();
        data[3] = pose.getRotation().getAngle();
        SmartDashboard.putNumberArray(this.name, data);
      }
    }
  }

  @Override
  public void appendDegrees(Transform3d pose) {
    super.appendDegrees(pose);
    if (CustomTransform3dLogger.isFmsMatch) {
      double[] data = new double[3];
      data[0] = pose.getTranslation().getX();
      data[1] = pose.getTranslation().getY();
      data[2] = Units.radiansToDegrees(pose.getRotation().getAngle());
      SmartDashboard.putNumberArray(this.name, data);
    }
  }

}
