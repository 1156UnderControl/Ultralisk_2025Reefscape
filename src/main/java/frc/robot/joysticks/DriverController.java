package frc.robot.joystick;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
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
  final XboxController driverControllerBoolean;

  private DriverController() {
    driverController = new CommandXboxController(0);
    driverControllerBoolean = new XboxController(0);
  }

  @Override
  public double getXtranslation() {
    if (turboActivate()) {
      return -MathUtil.applyDeadband(performAllianceSpeedDirectionCorrection(driverController.getLeftX()),
          JoystickConstants.DEADBAND);
    }
    return -MathUtil.applyDeadband(performAllianceSpeedDirectionCorrection(driverController.getLeftX()),
        JoystickConstants.DEADBAND) * 0.6;
  }

  @Override
  public double getYtranslation() {
    if (turboActivate()) {
      return -MathUtil.applyDeadband(performAllianceSpeedDirectionCorrection(driverController.getLeftY()),
          JoystickConstants.DEADBAND);
    }
    return -MathUtil.applyDeadband(performAllianceSpeedDirectionCorrection(driverController.getLeftY()),
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
  public boolean turboActivate() {
    return 0.2 < driverController.getRightTriggerAxis();
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
  public boolean rotateLeft() {
    return driverControllerBoolean.getLeftBumperButton();
  }

  @Override
  public boolean rotateRight() {
    return driverControllerBoolean.getRightBumperButton();
  }

  @Override
  public boolean isForcingDriverControl() {
    return 0.2 < driverController.getLeftTriggerAxis();
  }

  @Override
  public Trigger resetGyro() {
    return driverController.back();
  }

  private double performAllianceSpeedDirectionCorrection(Double value) {
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get()
        : DriverStation.Alliance.Red;
    if (alliance == Alliance.Red) {
      return -value;
    }
    return value;
  }

}
