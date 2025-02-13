package frc.robot.constants;

public class ClimberConstants {
  public static final int ID_cageIntakeMotor = 20;
  public static final int ID_climberArmMotor = 19;

  public class tunning_values_climber {
    public static final double VELOCITY_FALL_FOR_CAGE_INTAKE_DETECTION = 100;

    public static final double KS = 0;
    public static final double KG = 0;
    public static final double KV = 0;
    public static final double KA = 0;

    public static final double MAX_VELOCITY = 0;
    public static final double MAX_ACCELERATION = 0;
    public static final double JERK = 0;

    public class PID {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double F = 0;
      public static final double IZone = 0;
    }
  }

  public class setpoints {
    public static final double DUTY_CYCLE_INTAKE = 1.0;
    public static final double DUTY_CYCLE_EXPELL = -1.0;
  }
}
