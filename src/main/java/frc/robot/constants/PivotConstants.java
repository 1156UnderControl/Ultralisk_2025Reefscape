package frc.robot.constants;

public class PivotConstants {
  public static final int ID_pivotMotor = 3;
  public static final double CONVERSION_FOR_MATCH_EXTERNAL_ENCODER = 1;// 0.724500; // feito com regra de tres
  public static final double POSITION_FACTOR_MECHANISM_ROTATION_TO_MECHANISM_DEGREES = 360;
  public static final double POSITION_FACTOR_ROTOR_ROTATION_TO_MECHANISM_DEGREES = 4.1638
      * CONVERSION_FOR_MATCH_EXTERNAL_ENCODER;
  public static final double VELOCITY_FACTOR_MOTOR_RPM_TO_MECHANISM_DEG_PER_SECOND = 6;

  public class tunning_values_pivot {
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double MAX_VELOCITY = 900;
    public static final double MAX_ACCELERATION = 10000;
    public static final double POSITION_ERROR_ALLOWED = 0;
    public static final double MIN_DEAD_BAND_FOR_MOTOR_STOP = -5;
    public static final double MAX_DEAD_BAND_FOR_MOTOR_STOP = 5;

    public class PID {
      public static final double P = 0.02;
      public static final double I = 0.001;
      public static final double D = 0.1;
      public static final double arbFF = 0;
      public static final double IZone = 20;
    }

    public class setpoints {
      public static final double MAX_ANGLE = 220;
      public static final double MIN_ANGLE = 0;
      public static final double DEFAULT_ANGLE = 100;
      public static final double L1_ANGLE = 129;
      public static final double L2_ANGLE = 170.5;
      public static final double L3_ANGLE = 170.5;
      public static final double L4_ANGLE = 207;
      public static final double ALGAE_MID_REMOVAL = 173;
      public static final double ALGAE_LOW_REMOVAL = 173;
      public static final double SECURE_FOR_ELEVATOR_UP = 120;
      public static final double UNSECURE_POSITON_FOR_ROTATION_WITH_ELEVATOR_UP = 160;
      public static final double COLLECT_ANGLE = 12;
    }
  }
}
