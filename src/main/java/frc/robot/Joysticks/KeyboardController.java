package frc.robot.joysticks;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class KeyboardController {
  NetworkTable table;

  public KeyboardController() {
    this.table = NetworkTableInstance.getDefault().getTable("SmartDashBoard/keyboard");
  }

  public Trigger qTrigger() {
    Trigger keyTrigger;

    if (this.table.containsKey("q")) {
      keyTrigger = new Trigger(() -> true);
    } else {
      return new Trigger(() -> false);
    }
    return keyTrigger;
  }
}
