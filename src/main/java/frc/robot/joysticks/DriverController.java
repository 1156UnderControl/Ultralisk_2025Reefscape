package frc.robot.joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.Java_Is_UnderControl.Util.Util;
import frc.robot.constants.JoystickConstants;

public class DriverController implements IDriverController {

  private static DriverController mInstance = null;

  public static DriverController getInstance() {
    if (mInstance == null) {
      mInstance = new DriverController();
    }

    return mInstance;
  }

  final CommandXboxController driverController;

  private DriverController() {
    driverController = new CommandXboxController(0);
  }

  @Override
  public double getXtranslation() {
    if (turboActivate().getAsBoolean()) {
      return -MathUtil.applyDeadband(driverController.getLeftX(),
          JoystickConstants.DEADBAND);
    }
    return -MathUtil.applyDeadband(driverController.getLeftX(),
        JoystickConstants.DEADBAND) * 0.6;
  }

  @Override
  public double getYtranslation() {
    if (turboActivate().getAsBoolean()) {
      return -MathUtil.applyDeadband(driverController.getLeftY(),
          JoystickConstants.DEADBAND);
    }
    return -MathUtil.applyDeadband(driverController.getLeftY(),
        JoystickConstants.DEADBAND) * 0.6;
  }

  @Override
  public double getCOS_Joystick() {
    return -driverController.getRightX();
  }

  @Override
  public double getSIN_Joystick() {
    return -driverController.getRightY();
  }

  @Override
  public Trigger turboActivate() {
    return driverController.rightTrigger(0.2);
  }

  @Override
  public boolean notUsingJoystick() {
    return Util.inRange(getCOS_Joystick(),
        -JoystickConstants.DEADBAND,
        0.2)
        && Util.inRange(getSIN_Joystick(), -JoystickConstants.DEADBAND,
            JoystickConstants.DEADBAND);
  }

  @Override
  public Trigger a() {
    return driverController.a();
  }

  @Override
  public Trigger y() {
    return driverController.y();
  }

  @Override
  public Trigger x() {
    return driverController.x();
  }

  @Override
  public Trigger b() {
    return driverController.b();
  }

  @Override
  public Trigger leftBumper() {
    return driverController.leftBumper();
  }

  @Override
  public Trigger rightBumper() {
    return driverController.rightBumper();
  }

  @Override
  public Trigger isForcingDriverControl() {
    return driverController.leftTrigger(0.2);
  }

  @Override
  public Trigger resetGyro() {
    return driverController.back();
  }

}
