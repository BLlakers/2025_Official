package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbMechanism extends SubsystemBase{
    double climbPositionConversionFactor = 10.0;
    double climbVelocityConversionFactor = 10.0;

    //A motor to rotate up and down
    SparkMax m_windMotor = new SparkMax(0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    SparkMaxConfig config = new SparkMaxConfig();

    RelativeEncoder m_climbMotorEncoder = m_windMotor.getEncoder();

    public ClimbMechanism(){
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(climbPositionConversionFactor)
            .velocityConversionFactor(climbVelocityConversionFactor);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1.0,0,0);
    }

    public void windStringForward() {
        m_windMotor.set(.85);
    }

    public void windStringBackward() {
        m_windMotor.set(-.85);
    }

    public void windStop() {
        m_windMotor.set(0);
    }

    public Command windForwardCmd() {
        return this.runOnce(this::windStringForward);
    }

    public Command windBackwardCmd() {
        return this.runOnce(this::windStringBackward);
    }

    public Command windStopCmd() {
        return this.runOnce(this::windStop);
    }
}
