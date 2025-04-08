package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.SuperStructure;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToBranch extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  SuperStructure superStructure;
  boolean isBackupNecessary;
  boolean reachedBackupPosition = false;
  boolean isGoingToNonBackupPosition;
  boolean goDirect;

  public SwerveGoToBranch(SwerveSubsystem swerve, SuperStructure superStructure, TargetBranch branch) {
    this.swerve = swerve;
    this.targetBranch = branch;
    this.superStructure = superStructure;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.swerve.setTargetBranch(targetBranch);
    isBackupNecessary = this.swerve.checkBackupNecessary();
    isGoingToNonBackupPosition = false;
  }

  @Override
  public void execute() {
    if (isBackupNecessary && !reachedBackupPosition) {
      this.superStructure.led.setSolidColor(LedColor.YELLOW);
      this.swerve.driveToBranch(targetBranch, true, true);
      if (this.swerve.isAtTargetPositionWithoutHeading()) {
        reachedBackupPosition = true;
      }
    } else {
      this.superStructure.led.setSolidColor(LedColor.RED);
      this.swerve.driveToBranch(targetBranch, false, false);
      isGoingToNonBackupPosition = true;
    }
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPositionWithoutHeading() && isGoingToNonBackupPosition;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      this.superStructure.led.setSolidColor(LedColor.RED);
    } else {
      this.superStructure.led.setBlink(LedColor.PURPLE);
    }
    this.swerve.stopSwerve();
  }
}
