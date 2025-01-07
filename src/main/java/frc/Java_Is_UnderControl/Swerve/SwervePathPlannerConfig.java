package frc.Java_Is_UnderControl.Swerve;

import com.pathplanner.lib.config.PIDConstants;

public class SwervePathPlannerConfig {
  public PIDConstants translationPid;

  public PIDConstants anglePid;

  public double maxModuleSpeed;

  public SwervePathPlannerConfig(PIDConstants translationPid, PIDConstants anglePid) {
    this.translationPid = translationPid;
    this.anglePid = anglePid;
  }
}
