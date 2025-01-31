package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorController implements IOperatorController {
  private static OperatorController mInstance = null;

  public static OperatorController getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorController();
    }

    return mInstance;
  }

  KeyboardController keyboard;

  private OperatorController() {
    keyboard = new KeyboardController();
  }

  public Trigger getQTrigger() {
    return keyboard.qTrigger();
  }
}
