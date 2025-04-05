package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.LEDs.ILed;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToBranch extends Command {
  ISwerve swerve;
  ILed led;
  TargetBranch targetBranch;
  boolean isSpacedToBranch;

  public SwerveGoToBranch(SwerveSubsystem swerve, ILed led, TargetBranch branch, boolean isSpacedToBranch) {
    this.swerve = swerve;
    this.led = led;
    this.targetBranch = branch;
    this.isSpacedToBranch = isSpacedToBranch;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    led.setSolidColor(LedColor.YELLOW);
    this.swerve.driveToBranch(this.targetBranch, this.isSpacedToBranch);
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPosition();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      led.setSolidColor(LedColor.RED);
    } else {
      led.setBlink(LedColor.GREEN);
    }
    this.swerve.stopSwerve();
  }
}
