package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeMechanism extends SubsystemBase{
    double algaePositionConversionFactor = 10.0;
    double algaeVelocityConversionFactor = 10.0;

    //A motor to rotate up and down
    SparkMax m_armMotor = new SparkMax(0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    SparkMaxConfig config = new SparkMaxConfig();

    RelativeEncoder m_algaeMotorEncoder = m_armMotor.getEncoder();


    public AlgaeMechanism(){
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(algaePositionConversionFactor)
            .velocityConversionFactor(algaeVelocityConversionFactor);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.0,0,0);
    }

    public void moveAlgaeForward() {
        m_armMotor.set(.85);
    }

    public void moveAlgaeBackward() {
        m_armMotor.set(-.85);
    }

    public void algaeStop() {
        m_armMotor.set(0);
    }

    public Command algaeForwardCmd() {
        return this.runOnce(this::moveAlgaeForward);
    }

    public Command algaeBackwardCmd() {
        return this.runOnce(this::moveAlgaeBackward);
    }
}
