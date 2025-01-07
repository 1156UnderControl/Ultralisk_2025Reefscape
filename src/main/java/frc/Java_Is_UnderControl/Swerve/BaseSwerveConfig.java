package frc.Java_Is_UnderControl.Swerve;

public class BaseSwerveConfig {

  public final double maxRotationRate;// Use fractions, example: 0.75 = 3/4 of a rotation

  public final double dtDiscretization;

  public final SwervePathPlannerConfig pathPlannerConfig;

  public BaseSwerveConfig(double maxRotationSpeed, double dtDiscretization,
      SwervePathPlannerConfig pathPlannerConfig) {
    this.dtDiscretization = dtDiscretization;
    this.maxRotationRate = maxRotationSpeed;
    this.pathPlannerConfig = pathPlannerConfig;
  }

  public BaseSwerveConfig(double maxRotationSpeed,
      SwervePathPlannerConfig pathPlannerConfig) {
    this(maxRotationSpeed, 0.02, pathPlannerConfig);
  }
}
