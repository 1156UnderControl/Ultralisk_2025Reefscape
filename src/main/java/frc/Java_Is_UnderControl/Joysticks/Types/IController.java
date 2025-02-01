package frc.Java_Is_UnderControl.joysticks.Types;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IController {

  public Trigger buttomUp();

  public Trigger buttomDown();

  public Trigger buttomRight();

  public Trigger buttomLeft();

  public double leftY();

  public double leftX();

  public double rightY();

  public double rightX();

  public Trigger dPadUp();

  public Trigger dPadRight();

  public Trigger dPadLeft();

  public Trigger dPadDown();

  public Trigger R1();

  public Trigger R2();

  public Trigger R3();

  public Trigger L1();

  public Trigger L2();

  public Trigger L3();

  public Trigger options();

  public Trigger share();
}
