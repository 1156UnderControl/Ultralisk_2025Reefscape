package frc.Java_Is_UnderControl.Joysticks.Types;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TypePS5Controller implements IController {
  CommandPS5Controller controller;

  public TypePS5Controller(int controllerID) {
    controller = new CommandPS5Controller(controllerID);
  }

  public Trigger buttomUp() {
    return controller.button(3);
  }

  public Trigger buttomDown() {
    return controller.button(0);
  }

  public Trigger buttomRight() {
    return controller.button(1);
  }

  public Trigger buttomLeft() {
    return controller.button(2);
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

  @Override
  public Trigger dPadUp() {
    return controller.povUp();
  }

  @Override
  public Trigger dPadRight() {
    return controller.povRight();
  }

  @Override
  public Trigger dPadLeft() {
    return controller.povLeft();
  }

  @Override
  public Trigger dPadDown() {
    return controller.povDown();
  }

  @Override
  public Trigger R1() {
    return controller.R1();
  }

  @Override
  public Trigger R2() {
    return controller.R2();
  }

  @Override
  public Trigger R3() {
    return controller.R3();
  }

  @Override
  public Trigger L1() {
    return controller.L1();
  }

  @Override
  public Trigger L2() {
    return controller.L2();
  }

  @Override
  public Trigger L3() {
    return controller.L3();
  }

  @Override
  public Trigger options() {
    return controller.button(7);
  }

  @Override
  public Trigger share() {
    return controller.button(6);
  }
}
