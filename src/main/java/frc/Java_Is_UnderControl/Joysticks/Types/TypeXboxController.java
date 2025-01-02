package frc.Java_Is_UnderControl.Joysticks.Types;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TypeXboxController implements IController {
  CommandXboxController controller;

  public TypeXboxController(int controllerID) {
    controller = new CommandXboxController(controllerID);
  }

  @Override
  public Trigger buttomUp() {
    return controller.x();
  }

  public Trigger buttomDown() {
    return controller.a();
  }

  public Trigger buttomRight() {
    return controller.b();
  }

  public Trigger buttomLeft() {
    return controller.y();
  }

  @Override
  public double leftY() {
    return controller.getLeftY();
  }

  @Override
  public double leftX() {
    return controller.getLeftX();
  }

  @Override
  public double rightY() {
    return controller.getRightY();
  }

  @Override
  public double rightX() {
    return controller.getRightX();
  }

  // Can be pressed with the back right up buttom
  @Override
  public Trigger dPadUp() {
    return controller.povUp();
  }

  // Can be pressed with the back right down buttom
  @Override
  public Trigger dPadRight() {
    return controller.povRight();
  }

  // Can be pressed with the back left down buttom
  @Override
  public Trigger dPadLeft() {
    return controller.povLeft();
  }

  // Can be pressed with the back left up buttom
  @Override
  public Trigger dPadDown() {
    return controller.povDown();
  }

  @Override
  public boolean R1() {
    return controller.rightBumper().getAsBoolean();
  }

  @Override
  public Trigger R2() {
    return controller.rightTrigger();
  }

  @Override
  public Trigger R3() {
    return controller.rightStick();
  }

  @Override
  public boolean L1() {
    return controller.leftBumper().getAsBoolean();
  }

  @Override
  public Trigger L2() {
    return controller.leftTrigger();
  }

  @Override
  public Trigger L3() {
    return controller.leftStick();
  }

  @Override
  public Trigger options() {
    return controller.start();
  }

  @Override
  public Trigger share() {
    return controller.back();
  }
}