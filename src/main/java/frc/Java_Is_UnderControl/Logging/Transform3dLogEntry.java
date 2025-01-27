package frc.Java_Is_UnderControl.Logging;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;

public class Transform3dLogEntry {
      private DoubleArrayLogEntry baseLogger;

  public Transform3dLogEntry(DataLog log, String name) {
    this.baseLogger = new DoubleArrayLogEntry(log, name);
  }

  public void appendRadians(Transform3d transform3d) {
    double[] data = new double[4];
    data[0] = transform3d.getTranslation().getX();
    data[1] = transform3d.getTranslation().getY();
    data[2] = transform3d.getZ();
    data[3] = transform3d.getRotation().getAngle();
    this.baseLogger.append(data);
  }

  public void appendDegrees(Transform3d transform3d) {
    double[] data = new double[4];
    data[0] = transform3d.getTranslation().getX();
    data[1] = transform3d.getTranslation().getY();
    data[2] = transform3d.getZ();
    data[3] = Units.radiansToDegrees(transform3d.getRotation().getAngle());
    this.baseLogger.append(data);
  }
}
