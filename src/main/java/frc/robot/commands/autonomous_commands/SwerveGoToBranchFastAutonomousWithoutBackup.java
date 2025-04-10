package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.SuperStructure;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToBranchFastAutonomousWithoutBackup extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  SuperStructure superStructure;
  boolean reachedBackupPosition = false;
  boolean isGoingToNonBackupPosition;
  boolean goDirect;

  public SwerveGoToBranchFastAutonomousWithoutBackup(SwerveSubsystem swerve, TargetBranch branch,
      SuperStructure superStructure,
      boolean goDirect) {
    this.swerve = swerve;
    this.targetBranch = branch;
    this.goDirect = goDirect;
    this.superStructure = superStructure;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    this.superStructure.led.setSolidColor(LedColor.RED);
    if (this.goDirect) {
      this.swerve.driveToBranch(targetBranch, false, true);
    } else {
      this.swerve.driveToBranch(targetBranch, false, false);
    }
    isGoingToNonBackupPosition = true;
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtTargetPositionWithoutHeading();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      this.superStructure.led.setSolidColor(LedColor.RED);
    } else {
      this.superStructure.led.setBlink(LedColor.GREEN);
    }
    this.swerve.stopSwerve();
  }
}
