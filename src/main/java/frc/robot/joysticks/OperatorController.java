package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OperatorController implements IOperatorController {
  private static OperatorController mInstance = null;

  public static OperatorController getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorController();
    }

    return mInstance;
  }

  CommandXboxController controller;

  private OperatorController() {
    controller = new CommandXboxController(1);
  }

}
