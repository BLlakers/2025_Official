package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorMechanism extends SubsystemBase{
    double elevatorPositionConversionFactor = 10.0;
    double elevatorVelocityConversionFactor = 10.0;

    //A motor to rotate up and down
    SparkMax m_elevatorMotor = new SparkMax(0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    DigitalInput bottomLimitSwitch = new DigitalInput(0);
    
    SparkMaxConfig config = new SparkMaxConfig();

    public ElevatorMechanism() {
         config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(elevatorPositionConversionFactor)
            .velocityConversionFactor(elevatorVelocityConversionFactor);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1.0,0,0);
    }

    public void elevatorMotorUp() {
        m_elevatorMotor.set(.85);
    }

    public void elevatorMotorDown() {
        if (bottomLimitSwitch.get()) {
            m_elevatorMotor.set(0);
        } else {
            m_elevatorMotor.set(-.85);
        }
    }

    public void elevatorMotorStop() {
        m_elevatorMotor.set(0);
    }

    public Command elevatorUpCmd() {
        return this.runOnce(this::elevatorMotorUp);
    }

    public Command elevatorDownCmd() {
        return this.runOnce(this::elevatorMotorDown);
    }

    public Command elevatorStopCmd() {
        return this.runOnce(this::elevatorMotorStop);
    }
}
