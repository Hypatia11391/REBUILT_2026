package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* 
 * SHOOTER
 *      brushless NEO x2
 * 
*/

public class Shooter extends SubsystemBase {
    
    private static final int SHOOTER_NEO_LEFT_ID = 0; // TODO: type an actual ID
    private static final int SHOOTER_NEO_RIGHT_ID = 0; // TODO: type an actual ID 

    private final SparkMax shooterNeoLeft;
    private final SparkMax shooterNeoRight;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SparkClosedLoopController leftLoop;
    private final SparkClosedLoopController rightLoop;

    private double targetLeftRPM = 0.0;
    private double targetRightRPM = 0.0;

    // PID constants
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // NEOs free speed = 5676 rpm
    private static final double kV = 12.0 / 5676.0;

    private static final double RPM_TOL = 50.0;

    public Shooter(){

    // shooter motors
    shooterNeoRight = new SparkMax(SHOOTER_NEO_RIGHT_ID, SparkLowLevel.MotorType.kBrushless);
    shooterNeoLeft = new SparkMax(SHOOTER_NEO_LEFT_ID, SparkLowLevel.MotorType.kBrushless);
    
    rightEncoder = shooterNeoRight.getEncoder();
    leftEncoder = shooterNeoLeft.getEncoder();

    rightLoop = shooterNeoRight.getClosedLoopController();
    leftLoop = shooterNeoLeft.getClosedLoopController();
    
        
    configureShooterMotor(shooterNeoLeft, false); 
    configureShooterMotor(shooterNeoRight, false);
    
    stop();
    }

    public void setTargetRPM(double topRPM, double bottomRPM){
        targetRightRPM = topRPM;
        targetLeftRPM = bottomRPM;

        rightLoop.setSetpoint(targetRightRPM, ControlType.kVelocity);
        leftLoop.setSetpoint(targetLeftRPM, ControlType.kVelocity);
    }

    public double getTopRPM(){return rightEncoder.getVelocity();}
    public double getBottomRPM(){return leftEncoder.getVelocity();}

    public boolean atSpeed(){
        return Math.abs(getTopRPM() - targetRightRPM) < RPM_TOL && Math.abs(getBottomRPM() - targetLeftRPM) < RPM_TOL;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter/TopRPM", getTopRPM());
        SmartDashboard.putNumber("Shooter/BottomRPM", getBottomRPM());
        SmartDashboard.putNumber("Shooter/TopTarget", targetRightRPM);
        SmartDashboard.putNumber("Shooter/BottomTarget", targetLeftRPM);
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
        targetRightRPM = 0.0;
        targetLeftRPM = 0.0;
        shooterNeoRight.stopMotor();
        shooterNeoLeft.stopMotor();
    }
}
