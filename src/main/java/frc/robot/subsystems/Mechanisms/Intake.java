package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/*
* INTAKE
*   brushed CIM x1 for the rollers
*   brushed non-CIM x1 for lifting
*/ 

public class Intake extends SubsystemBase {

    private static final int INTAKE_FEED_ID = 4; 
    private static final int INTAKE_LIFT_ID = 6;

    // PID constants
    private static final double kP = 0.05;
    private static final double kI = 0;
    private static final double kD = 0;

    // private static final int TOP_LIMIT_SWITCH_ID = 0;
    // private static final int BOTTOM_LIMIT_SWITCH_ID = 1;

    // private final DigitalInput topLimitSwitch = new DigitalInput(TOP_LIMIT_SWITCH_ID);
    // private final DigitalInput bottomLimitSwitch = new DigitalInput(BOTTOM_LIMIT_SWITCH_ID);

    private final SparkMax intakeFeed;
    private final SparkMax intakeLift;

    SparkMaxConfig config = new SparkMaxConfig();

    private final RelativeEncoder liftEncoder;

    private static final double MIN_POS = -100.0;
    private static final double MAX_POS = 50.0;

    double targetPos = 0.0;
    
    private final SparkClosedLoopController liftLoop;

    public Intake(){
      // intake motors
      intakeFeed = new SparkMax(INTAKE_FEED_ID, SparkLowLevel.MotorType.kBrushless);
      intakeLift = new SparkMax(INTAKE_LIFT_ID, SparkLowLevel.MotorType.kBrushless);

      liftLoop = intakeLift.getClosedLoopController();
      liftEncoder = intakeLift.getEncoder();

      targetPos = liftEncoder.getPosition();

      configureIntakeMotor(intakeFeed, true);
      configureIntakeLiftMotor(intakeLift, true);
    }

    public void setLiftMotorSpeed(double power){

      if (Math.abs(power) < 0.05) power = 0;

      // scale kinda
      targetPos += power * 0.5; // Establishes setpoint to current position to avoid moving

      if (targetPos > MAX_POS)targetPos = MAX_POS;
      if (targetPos < MIN_POS)targetPos = MIN_POS;

      liftLoop.setSetpoint(targetPos, ControlType.kPosition);
      // if (power > 0 && pos >= MAX_POS){intakeLift.set(0); return;}
      // if (power < 0 && pos <= MIN_POS){intakeLift.set(0); return;}
      // if (power > 0 && pos >= MAX_POS - SLOWZONE_THRESH_M) {pos_set = 0.3 ;} // constant for top
      // if (power < 0 && pos <= MIN_POS + SLOWZONE_THRESH_M) {pos_set = 0.3;} // constant for bottom
      // if (pos >= MAX_POS ) {zeroLift();}
      // liftLoop.setSetpoint(pos_set, ControlType.kPosition);
      // intakeLift.set(power*0.50);
    }

    public void zeroLift(){
      liftEncoder.setPosition(0.0);
    }
  
    public void setFeedMotorSpeed(double speed){
      intakeFeed.set(speed*0.5);
    }
    @Override
    public void periodic(){
      SmartDashboard.putNumber("LiftPos", liftEncoder.getPosition());
    }

    public void configureIntakeLiftMotor(SparkMax motor, boolean isInverted){
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(isInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(12)
            .encoder.velocityConversionFactor(1.0)
                    .positionConversionFactor(1.0);
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD)
        .outputRange(-0.5, 0.5);
            // .openLoopRampRate(0.2);

      motor.configureAsync(
        config, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kPersistParameters);
    }

    public void configureIntakeMotor(SparkMax motor, boolean isInverted){
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(isInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(12);
            // .openLoopRampRate(0.1);
      motor.configureAsync(
        config, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kPersistParameters);
    }

    public void stop() {
        intakeLift.stopMotor();
        intakeFeed.stopMotor();
    }
}