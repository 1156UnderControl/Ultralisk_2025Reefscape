package frc.Java_Is_UnderControl.Sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;

public class InfraRed implements frc.Java_Is_UnderControl.Sensors.DigitalInput {
  private DigitalInput IR;
  private boolean inverted;
  private CustomBooleanLogger measurementLog;

  public InfraRed(int port, boolean invert) {
    IR = new DigitalInput(port);
    inverted = invert;
    this.measurementLog = new CustomBooleanLogger("/sensors/InfraRed/" + port + "/measurement");
  }

  @Override
  public boolean getBoolean() {
    boolean measurement = inverted ? !IR.get() : IR.get();
    this.measurementLog.append(measurement);
    return measurement;
  }

  @Override
  public int getBinary() {
    if (getBoolean()) {
      return 1;
    } else {
      return 0;
    }
  }

  @Override
  public boolean getAsBoolean() {
    return this.getBoolean();
  }
}
