package frc.robot.constants;

public class ScorerConstants {
  public static final int ID_pivotMotor = 0;
  public static final int ID_endEffectorMotor = 0;

  public static final double POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_DEGREES = 0;
  public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_RPM = 0;

  public class tunning_values_scorer {
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double MAX_VELOCITY = 0;
    public static final double MAX_ACCELERATION = 0;
    public static final double POSITION_ERROR_ALLOWED = 0;

    public class PID {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double F = 0;
      public static final double IZone = 0;
    }

    public class setpoints {
      public static final double MAX_ANGLE_PIVOT = 2.5;
      public static final double MIN_ANGLE_PIVOT = 0;

      public static final double DUTY_CYCLE_INTAKE = 1.0;
      public static final double DUTY_CYCLE_EXPELL = -1.0;
    }
  }
}
