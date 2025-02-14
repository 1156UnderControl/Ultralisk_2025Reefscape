package frc.robot.constants;

public class PivotConstants {
  public static final int ID_pivotMotor = 0;
  public static final double POSITION_FACTOR_MOTOR_ROTATION_TO_MECHANISM_DEGREES = 0;
  public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_DEG_PER_SECOND = 0;

  public class tunning_values_pivot {
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double MAX_VELOCITY = 0;
    public static final double MAX_ACCELERATION = 0;
    public static final double POSITION_ERROR_ALLOWED = 0;

    public class PID {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
      public static final double arbFF = 0;
      public static final double IZone = 0;
    }

    public class setpoints {
      public static final double MAX_ANGLE = 2.5;
      public static final double MIN_ANGLE = 0;
      public static final double DEFAULT_ANGLE = 0;
      public static final double L1_ANGLE = 10;
      public static final double L2_ANGLE = 30;
      public static final double L3_ANGLE = 50;
      public static final double L4_ANGLE = 70;
      public static final double FACE0_ALGAE_REMOVAL = 15;
      public static final double FACE1_ALGAE_REMOVAL = 25;
      public static final double FACE2_ALGAE_REMOVAL = 35;
      public static final double FACE3_ALGAE_REMOVAL = 45;
      public static final double FACE4_ALGAE_REMOVAL = 55;
      public static final double FACE5_ALGAE_REMOVAL = 65;
      public static final double SECURE_FOR_ELEVATOR_UP = 45;

      public static final double UNSECURE_POSITON_FOR_ROTATION_WITH_ELEVATOR_UP = 45;
    }
  }
}
