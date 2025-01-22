package frc.Java_Is_UnderControl.Motors;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomIntegerLogger;

import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SparkMAXMotor implements IMotor{

    private SparkMaxConfig config;

    private boolean factoryDefaultOcurred = false;

    private final TrapezoidProfile m_profile;

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private double maxVelocity = 0;

    private double rampRate = Double.NaN;

    private double maxAcceleration = 0;

    private String motorName;

    // I don't how it works, and if it's truly necessary to use
    private Supplier<Double> velocity;
    private Supplier<Double> position;

    private CustomDoubleLogger appliedOutputLog;
    private CustomDoubleLogger targetOutputLog;
    private CustomDoubleLogger currentLog;
    private CustomDoubleLogger positionLog;
    private CustomDoubleLogger velocityLog;
    private CustomDoubleLogger temperatureLog;
    private CustomIntegerLogger faultsLog;
    private CustomDoubleLogger targetPositionLog;
    private CustomDoubleLogger targetSpeedLog;

    boolean isInverted = false;

    private double targetPercentage;
    private double targetPosition;
    private double targetVelocity;

    private SimpleMotorFeedforward feedforward;

    private double velocityFF;
    
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    private SysIdRoutine sysIdRoutine;

    private SparkMax motor;

    public SparkMAXMotor(int motorID, String motorName){
        this(motorID, false, motorName);
    }

    public SparkMAXMotor(int motorID, boolean usingAlternateEncoder, String motorName){
        this.motor = new SparkMax(motorID, MotorType.kBrushless);
        this.config = new SparkMaxConfig();
        this.m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(this.maxVelocity, this.maxAcceleration));
        this.motorName = motorName;

        this.setAlternateEncoder(usingAlternateEncoder);
        this.setupLogs(motorID, usingAlternateEncoder);
        this.updateLogs();
    }

    private void setAlternateEncoder(boolean usingAlternateEncoder) {
        if (usingAlternateEncoder) {
          this.config.alternateEncoder.countsPerRevolution(8192);
        } else {
          this.motor.getEncoder();
        }
    }

    private void setupLogs(int motorId, boolean isAlternateEncoder) {
    this.appliedOutputLog = new CustomDoubleLogger("/motors/" + motorId + "/appliedOutput");
    this.targetOutputLog = new CustomDoubleLogger("/motors/" + motorId + "/targetOutput");
    this.currentLog = new CustomDoubleLogger("/motors/" + motorId + "/current");
    this.positionLog = new CustomDoubleLogger("/motors/" + motorId + "/position");
    this.velocityLog = new CustomDoubleLogger("/motors/" + motorId + "/velocity");
    this.temperatureLog = new CustomDoubleLogger("/motors/" + motorId + "/temperature");
    this.faultsLog = new CustomIntegerLogger("/motors/" + motorId + "/faults");
    this.targetPositionLog = new CustomDoubleLogger("/motors/" + motorId + "/targetPosition");
    this.targetSpeedLog = new CustomDoubleLogger("/motors/" + motorId + "/targetSpeed");
    StringLogEntry firmwareVersionLog = new StringLogEntry(DataLogManager.getLog(),
        "/motors/" + motorId + "/firmwareVersion");
    firmwareVersionLog.append(this.motor.getFirmwareString());
    BooleanLogEntry isAlternateEncoderLog = new BooleanLogEntry(DataLogManager.getLog(),
        "/motors/" + motorId + "/isAlternateEncoder");
    isAlternateEncoderLog.append(isAlternateEncoder);
    BooleanLogEntry isFollowerLog = new BooleanLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/isFollower");
    isFollowerLog.append(this.motor.isFollower());
    BooleanLogEntry isInvertedLog = new BooleanLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/isInverted");
    isInvertedLog.append(isInverted);
  }

  @Override
  public void updateLogs() {
    this.appliedOutputLog.append(this.getAppliedOutput());
    this.targetOutputLog.append(this.targetPercentage);
    this.currentLog.append(this.getVoltage());
    this.positionLog.append(this.getPosition());
    this.velocityLog.append(this.getVelocity());
    this.temperatureLog.append(this.motor.getMotorTemperature());
    //this.faultsLog.append(this.motor.getFaults());
    this.targetPositionLog.append(this.targetPosition);
    this.targetSpeedLog.append(this.targetVelocity);
  }


    private void configureSparkMax(Supplier<REVLibError> config) {
        for (int i = 0; i < maximumRetries; i++) {
        if (config.get() == REVLibError.kOk) {
            return;
        }
        Timer.delay(Milliseconds.of(5).in(Seconds));
    }
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
  }

    @Override
    public String getMotorName(){
        return this.motorName;
    }

// To restore factory default its necessary to wait until revlib 2025 is officially launched
    @Override
    public void factoryDefault(){
        if(!factoryDefaultOcurred){
        }
        factoryDefaultOcurred = true;
    }

    @Override
    public void clearStickyFaults(){
        configureSparkMax(motor::clearFaults);
    }

    public void updateConfig(SparkMaxConfig cfgGiven)
  {
    if (!DriverStation.isDisabled())
    {
      throw new RuntimeException("Configuration changes cannot be applied while the robot is enabled.");
    }
    config.apply(cfgGiven);
    configureSparkMax(() -> motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

// To implement the Kg, Ks and Kv its necessary to wait until revlib 2025 is officially launched
    @Override
    public void configureFeedForward(double Kg, double Ks, double Kv){
        config.closedLoop.velocityFF(Kg);
    }

    @Override
    public void configurePIDF(double P, double I, double D, double F, double Izone){
        this.configurePIDF(P, I, D, F);
        config.closedLoop.iZone(Izone);
    }

    @Override
    public void configurePIDF(double P, double I, double D, double F) {
        config.closedLoop.pidf(P, I, D, F);
    }

    @Override
    public void configurePIDWrapping(double minInput, double maxInput){
        config.closedLoop
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(minInput, maxInput);
    }

    @Override
    public void setMotorBrake(boolean isBrakeMode){
        config.idleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setInverted(boolean inverted){
        config.inverted(inverted);
    }

    @Override
    public void setInvertedEncoder(boolean inverted){
        config.encoder.inverted(inverted);
    }

    @Override
    public void burnFlash(){
        if (!DriverStation.isDisabled()){
            throw new RuntimeException("Config updates cannot be applied while the robot is Enabled!");
        }
        configureSparkMax(() -> {
            return motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        });
    }

    @Override
    public void set(double percentOutput){
        if (percentOutput != this.targetPercentage) {
            this.motor.set(percentOutput);
          }
          this.targetPercentage = percentOutput;
          this.targetPosition = Double.NaN;
          this.targetVelocity = Double.NaN;
          this.updateLogs();
    }

    @Override
    public void set(Voltage percentOutput){
        if (percentOutput.in(Volts) != this.targetPercentage) {
            this.motor.set(percentOutput.in(Volts));
          }
          this.targetPercentage = percentOutput.in(Volts);
          this.targetPosition = Double.NaN;
          this.targetVelocity = Double.NaN;
          this.updateLogs();
    }

    @Override
    public void setPositionReference(double position){
        if(this.getPosition() != position){
            motor.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition);
        }
        this.targetPercentage = Double.NaN;
        this.targetVelocity = Double.NaN;
        this.targetPosition = position;
        this.updateLogs();
    }

    public void setPositionReferenceArbFF(double position, ClosedLoopSlot feedforward){
        if(this.getPosition() != position){
            motor.getClosedLoopController().setReference(position, SparkBase.ControlType.kPosition, feedforward);
        }
        this.targetPercentage = Double.NaN;
        this.targetVelocity = Double.NaN;
        this.targetPosition = position;
        this.updateLogs();
    }

    @Override
    public void setPositionReferenceMotionProfiling(double position, double maxVelocity, double maxAcceleration){
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        if(this.getPosition() != position){
            // not implemented
        }
        this.targetPercentage = Double.NaN;
        this.targetVelocity = Double.NaN;
        this.targetPosition = position;
        this.updateLogs();
    }

    public void configureMaxMotion(double P, double I, double D, double ff, double maxVelocity, double maxAcceleration, double positionErrorAllowed){
        config.closedLoop.maxMotion
            .maxVelocity(maxVelocity, ClosedLoopSlot.kSlot0)
            .maxAcceleration(maxAcceleration, ClosedLoopSlot.kSlot0)
            .allowedClosedLoopError(positionErrorAllowed, ClosedLoopSlot.kSlot0);
        config.closedLoop
            .p(P, ClosedLoopSlot.kSlot0)
            .i(I, ClosedLoopSlot.kSlot0)
            .d(D, ClosedLoopSlot.kSlot0)
            .velocityFF(ff, ClosedLoopSlot.kSlot0)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0);
    }

    public void setPositionMaxMotion(double position){
        motor.getClosedLoopController().setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
    }

    public void setAngleMaxMagic(Angle angle){

    }

    public void setVelocityReference(double velocity, ClosedLoopSlot feedforward){
        if(this.getVelocity() != velocity){
            motor.getClosedLoopController().setReference(velocity, ControlType.kVelocity, feedforward);
        }
        this.targetPercentage = Double.NaN;
        this.targetVelocity = velocity;
        this.targetPosition = Double.NaN;
        this.updateLogs();
    }

    @Override
    public double getVoltage(){
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public double getDutyCycleSetpoint(){
        return motor.get();
    }

    @Override
    public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    @Override
    public double getAppliedOutput(){
        return motor.getAppliedOutput();
    }

    @Override
    public double getPosition(){
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getVelocity(){
        return motor.getEncoder().getVelocity();
    }

    @Override
    public void setPositionFactor(double factor){
        config.encoder.positionConversionFactor(factor);
    }

    @Override
    public void setVelocityFactor(double factor){
        config.encoder.velocityConversionFactor(factor);
    }

    @Override
    public void setPosition(double position){
        this.motor.getEncoder().setPosition(position);
    }

    @Override
    public void setVoltageCompensation(double nominalVoltage){
        config.voltageCompensation(nominalVoltage);
    }

    @Override
    public void setCurrentLimit(int currentLimit){
        config.smartCurrentLimit(currentLimit);
    }

    @Override
    public void setLoopRampRate(double rampRate) {
        config.closedLoopRampRate(rampRate)
       .openLoopRampRate(rampRate);
    }

    @Override
    public Object getMotor(){
        return motor;
    }

    @Override
    public void setSysID(Subsystem currentSubsystem){
        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(5), Seconds.of(10)),
            new SysIdRoutine.Mechanism(
            voltage -> {
                this.set(voltage);
            },
            log -> {
                log.motor(this.motorName)
                .voltage(m_appliedVoltage.mut_replace(this.getAppliedOutput()* RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(this.getPosition(), Meters))
                .linearVelocity(m_velocity.mut_replace(this.getVelocity(), MetersPerSecond));
            },
            currentSubsystem
            )    
        );  
    }
    @Override
    public void setTwoSysIDMotors(Subsystem currentSubsystem, IMotor otherMotor){
        this.sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(5), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            voltage -> {
                this.set(voltage);
                otherMotor.set(voltage);
            },
            log -> {
                log.motor(this.motorName)
                .voltage(m_appliedVoltage.mut_replace(this.getAppliedOutput()* RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(this.getPosition(), Meters))
                .linearVelocity(m_velocity.mut_replace(this.getVelocity(), MetersPerSecond));

                log.motor(otherMotor.getMotorName())
                .voltage(m_appliedVoltage.mut_replace(otherMotor.getAppliedOutput()* RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(otherMotor.getPosition(), Meters))
                .linearVelocity(m_velocity.mut_replace(otherMotor.getVelocity(), MetersPerSecond));
            },
            currentSubsystem
            )    
        );  
    }

    @Override
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.quasistatic(direction);
    }

    @Override
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.dynamic(direction);
    }
}
    
    
