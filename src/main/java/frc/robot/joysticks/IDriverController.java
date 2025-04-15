package frc.robot.joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IDriverController {
  double getXtranslation();

  double getYtranslation();

  double getCOS_Joystick();

  double getSIN_Joystick();

  Trigger turboActivate();

  boolean notUsingJoystick();

  Trigger y();

  Trigger b();

  Trigger a();

  Trigger x();

  Trigger leftBumper();

  Trigger rightBumper();

  Trigger resetGyro();

  Trigger isForcingDriverControl();

  Trigger cancelAction();

  Trigger setHasAlgae();

  Trigger setHasCoral();

  Trigger setNoObject();

}
