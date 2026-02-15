package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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


public class Shooter extends SubsystemBase {
    
    private static final int SHOOTER_NEO_LOW_ID = 0; // TODO: type an actual ID
    private static final int SHOOTER_NEO_TOP_ID = 0; // TODO: type an actual ID
    /*  
    *   brushed - CIS for the rollers
    *   brushed - non CIS lifting
    *   brushed - CIS for the kicker
    *   brushless - NEO x2
    */  

    private final SparkMax shooterNeoLeft;
    private final SparkMax shooterNeoRight;

    private final RelativeEncoder topEncoder;
    private final RelativeEncoder botttomEncoder;

    private final SparkClosedLoopController topLoop;
    private final SparkClosedLoopController bottomLoop;

    private double targetTopRPM = 0.0;
    private double targetBottomRPM = 0.0;

    // PID constants
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // NEOs free speed = 5676 rpm
    private static final double kV = 12.0 / 5676.0;

    private static final double RPM_TOL = 50.0;

    public Shooter(){

    // shooter motors
    shooterNeoRight = new SparkMax(SHOOTER_NEO_TOP_ID, SparkLowLevel.MotorType.kBrushless);
    shooterNeoLeft = new SparkMax(SHOOTER_NEO_LOW_ID, SparkLowLevel.MotorType.kBrushless);
    
    topEncoder = shooterNeoRight.getEncoder();
    botttomEncoder = shooterNeoLeft.getEncoder();

    topLoop = shooterNeoRight.getClosedLoopController();
    bottomLoop = shooterNeoLeft.getClosedLoopController();
    
        
    configureShooterMotor(shooterNeoLeft, false); 
    configureShooterMotor(shooterNeoRight, false);
    
    stop();
    }

    public void setTargetRPM(double topRPM, double bottomRPM){
        targetTopRPM = topRPM;
        targetBottomRPM = bottomRPM;

        topLoop.setSetpoint(targetTopRPM, ControlType.kVelocity);
        bottomLoop.setSetpoint(targetBottomRPM, ControlType.kVelocity);
    }

    public double getTopRPM(){return topEncoder.getVelocity();}
    public double getBottomRPM(){return botttomEncoder.getVelocity();}

    public boolean atSpeed(){
        return Math.abs(getTopRPM() - targetTopRPM) < RPM_TOL && Math.abs(getBottomRPM() - targetBottomRPM) < RPM_TOL;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter/TopRPM", getTopRPM());
        SmartDashboard.putNumber("Shooter/BottomRPM", getBottomRPM());
        SmartDashboard.putNumber("Shooter/TopTarget", targetTopRPM);
        SmartDashboard.putNumber("Shooter/BottomTarget", targetBottomRPM);
        SmartDashboard.putBoolean("Shooter/AtSpeed", atSpeed());
    }

    public void configureShooterMotor(SparkMax motor, boolean isInverted){
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(isInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(50)
            .voltageCompensation(12)
            .openLoopRampRate(0.25);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .outputRange(-1.0, 1.0);
        config.closedLoop.feedForward
            .sv(0.0, kV);
    
        motor.configureAsync(
            config, 
            ResetMode.kNoResetSafeParameters, 
            PersistMode.kPersistParameters);
    }

    public void stop() {
        targetTopRPM = 0.0;
        targetBottomRPM = 0.0;
        shooterNeoRight.stopMotor();
        shooterNeoLeft.stopMotor();
    }
}
