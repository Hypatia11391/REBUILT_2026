package frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* 
 * SHOOTER
 *      brushless NEO x2
 * 
*/

public class Shooter extends SubsystemBase {
    
    private static final int SHOOTER_NEO_LEFT_ID = 3;
    private static final int SHOOTER_NEO_RIGHT_ID = 5;

    private final SparkMax shooterNeoLeft;
    private final SparkMax shooterNeoRight;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SparkClosedLoopController leftLoop;
    private final SparkClosedLoopController rightLoop;

    private double targetLeftRPM = 0.0;
    private double targetRightRPM = 0.0;

    // PID constants
    private static final double kP = 0.00006;
    private static final double kI = 0.0;
    private static final double kD = 0.1; //0.00000001

    // for the pid things
    private double prevRightError = 0.0;
    private double prevLeftError = 0.0;
    private double prevTime = 0.0;

    // NEOs free speed = 5676 rpm
    private static final double kV = 1.0 / 5676.0; // 0.00005; // feed-forward constant or 12.0 / 5676.0

    private static final double RPM_TOL = 400.0;

    public Shooter(){ 

    // shooter motors
    shooterNeoRight = new SparkMax(SHOOTER_NEO_RIGHT_ID, SparkLowLevel.MotorType.kBrushless);
    shooterNeoLeft = new SparkMax(SHOOTER_NEO_LEFT_ID, SparkLowLevel.MotorType.kBrushless);
    
    // must be the opposites
    configureShooterMotor(shooterNeoLeft, true); 
    configureShooterMotor(shooterNeoRight, false);

    rightEncoder = shooterNeoRight.getEncoder();
    leftEncoder = shooterNeoLeft.getEncoder();

    rightLoop = shooterNeoRight.getClosedLoopController();
    leftLoop = shooterNeoLeft.getClosedLoopController();
    
    stop();
    }

    public void setTargetRPM(double targetRightRPM, double targetLeftRPM){

        this.targetRightRPM = targetRightRPM;
        this.targetLeftRPM = targetLeftRPM;
        
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - prevTime;
        prevTime = currentTime;

        double rightError = targetRightRPM - getRightRPM();
        double leftError = targetLeftRPM - getLeftRPM();

        double rightDerivative = 0.0;
        double leftDerivative = 0.0;

        prevRightError = rightError;
        prevLeftError = leftError;

        if (dt > 0){
            rightDerivative = (rightError - prevRightError) / dt;
            leftDerivative = (leftError - prevLeftError) / dt;
        }

        double ffRight = kV * targetRightRPM;
        double ffLeft = kV * targetLeftRPM;

        double rightOutput = ffRight + kP * rightError + kD * rightDerivative;
        double leftOutput = ffLeft +  kP * leftError + kD * leftDerivative;

        shooterNeoRight.set(rightOutput); // - kP(delta(f)) - kD(delta(f'))) () look at previous velocity and current
        shooterNeoLeft.set(leftOutput);

        // rightLoop.setSetpoint(targetRightRPM, ControlType.kVelocity);
        // leftLoop.setSetpoint(targetLeftRPM, ControlType.kVelocity);
    
    }

    public double getRightRPM(){return rightEncoder.getVelocity();}
    public double getLeftRPM(){return leftEncoder.getVelocity();}

    public boolean atSpeed(){
        return Math.abs(getRightRPM() - targetRightRPM) < RPM_TOL && Math.abs(getLeftRPM() - targetLeftRPM) < RPM_TOL;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter/TopRPM", getRightRPM());
        SmartDashboard.putNumber("Shooter/BottomRPM", getLeftRPM());
        SmartDashboard.putNumber("Shooter/TopTarget", targetRightRPM);
        SmartDashboard.putNumber("Shooter/BottomTarget", targetLeftRPM);
        SmartDashboard.putBoolean("Shooter/AtSpeed", atSpeed());
        double actualVelRight = shooterNeoRight.get();
        double actualVelLeft = shooterNeoLeft.get();
        SmartDashboard.putNumber("NeoRight/actualvelocity", actualVelRight);
        SmartDashboard.putNumber("NeoLeft/actualvelocity", actualVelLeft);
    }

    public void configureShooterMotor(SparkMax motor, boolean isInverted){
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(isInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .voltageCompensation(12)
            .openLoopRampRate(0.25);
        config.absoluteEncoder
            .velocityConversionFactor(1);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .outputRange(-1.0, 1.0);

        // config.closedLoop.feedForward
        //     .sv(0.0, kV);
    
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
