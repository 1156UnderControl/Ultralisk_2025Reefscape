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

  public Trigger goToReefA() {
    return keyboard.getGTrigger();
  }

  public Trigger goToReefB() {
    return keyboard.getHTrigger();
  }

  public Trigger goToReefC() {
    return keyboard.getJTrigger();
  }

  public Trigger goToReefD() {
    return keyboard.getKTrigger();
  }

  public Trigger goToReefE() {
    return keyboard.getITrigger();
  }

  public Trigger goToReefF() {
    return keyboard.getUTrigger();
  }

  public Trigger goToReefG() {
    return keyboard.getYTrigger();
  }

  public Trigger goToReefH() {
    return keyboard.getTTrigger();
  }

  public Trigger goToReefI() {
    return keyboard.getRTrigger();
  }

  public Trigger goToReefJ() {
    return keyboard.getETrigger();
  }

  public Trigger goToReeK() {
    return keyboard.getFTrigger();
  }

  public Trigger goToReeL() {
    return keyboard.getDTrigger();
  }

  public Trigger reefL1() {
    return keyboard.get1Trigger();
  }

  public Trigger reefL2() {
    return keyboard.get2Trigger();
  }

  public Trigger reefL3() {
    return keyboard.get3Trigger();
  }

  public Trigger reefL4() {
    return keyboard.get4Trigger();
  }
}
