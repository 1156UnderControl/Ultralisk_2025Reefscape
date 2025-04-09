package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.SuperStructure;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveGoToFace extends Command {
  ISwerve swerve;
  TargetBranch targetBranch;
  SuperStructure superStructure;

  public SwerveGoToFace(SwerveSubsystem swerve, SuperStructure superStructure, TargetBranch branch) {
    this.swerve = swerve;
    this.targetBranch = branch;
    this.superStructure = superStructure;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.swerve.setTargetBranch(targetBranch);
  }

  @Override
  public void execute() {
    this.superStructure.led.setSolidColor(LedColor.RED);
    this.swerve.goToFaceTeleoperated();
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
      this.superStructure.led.setBlink(LedColor.PURPLE);
    }
    this.swerve.stopSwerve();
  }
}
