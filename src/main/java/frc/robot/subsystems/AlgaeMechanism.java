package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeMechanism extends SubsystemBase{
   private double algaePositionConversionFactor = 1;
   private double algaeVelocityConversionFactor = 1;

    //A motor to rotate up and down
   private SparkMax m_AlgaeMotor = new SparkMax(Constants.Port.m_ArmMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
   private SparkMax m_IntakeMotor = new SparkMax(Constants.Port.m_IntakeMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
   private SparkMaxConfig m_AlgaeConfig = new SparkMaxConfig();
   private SparkMaxConfig m_IntakeConfig = new SparkMaxConfig();

    public AlgaeMechanism(){
        m_AlgaeConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        m_AlgaeConfig.encoder
            .positionConversionFactor(algaePositionConversionFactor)
            .velocityConversionFactor(algaeVelocityConversionFactor);
        m_AlgaeConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pid(1.0,0,0);
        
        m_IntakeConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        m_IntakeConfig.encoder
            .positionConversionFactor(algaePositionConversionFactor)
            .velocityConversionFactor(algaeVelocityConversionFactor);
        m_IntakeConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.0,0,0);

            m_AlgaeMotor.configure(m_AlgaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_IntakeMotor.configure(m_IntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void AlgaeForward() {
        m_AlgaeMotor.set(.85);
    }

    public void AlgaeBackward() {
        m_AlgaeMotor.set(-.85);
    }

    public void AlgaeStop() {
        m_AlgaeMotor.set(0);
    }

    public void IntakeForward() {
        m_IntakeMotor.set(.85);
    }

    public void IntakeBackward() {
        m_IntakeMotor.set(-.85);
    }

    public void IntakeStop() {
        m_IntakeMotor.set(0);
    }

    public double getIntakeEncoderPos(){
        return m_IntakeMotor.getAlternateEncoder().getPosition();
    }
    
    public double getAlgaeEncoderPos(){
        return m_AlgaeMotor.getAlternateEncoder().getPosition();
    }

    public Command AlgaeForwardCmd() {
        return this.runOnce(this::AlgaeForward);
    }

    public Command AlgaeBackwardCmd() {
        return this.runOnce(this::AlgaeBackward);
    }

    public Command AlgaeStopCmd() {
        return this.runOnce(this::AlgaeStop);
    }

    public Command IntakeForwardCmd() {
        return this.runOnce(this::IntakeForward);
    }

    public Command IntakeBackwardCmd() {
        return this.runOnce(this::IntakeBackward);
    }

    public Command IntakeStopCmd() {
        return this.runOnce(this::IntakeStop);
    }

    
 /*@Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Algae/Intake/Position", () -> getIntakeEncoderPos(), null);
    builder.addDoubleProperty("Algae/Algae/Position", () -> getAlgaeEncoderPos(), null);
  }*/
}
