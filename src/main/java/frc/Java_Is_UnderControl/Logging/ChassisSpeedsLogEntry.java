package frc.Java_Is_UnderControl.Logging;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;

public class ChassisSpeedsLogEntry {
    private DoubleArrayLogEntry baseLogger;

    public ChassisSpeedsLogEntry(DataLog log, String name) {
        this.baseLogger = new DoubleArrayLogEntry(log, name);
    }

    public void append(ChassisSpeeds chassisSpeeds){
        double[] data = new double[3];
        data[0] = chassisSpeeds.vxMetersPerSecond;
        data[1] = chassisSpeeds.vyMetersPerSecond;
        data[2] = chassisSpeeds.omegaRadiansPerSecond;
        this.baseLogger.append(data);
    }
}
