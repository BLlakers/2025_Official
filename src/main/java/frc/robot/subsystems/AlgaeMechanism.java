package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeMechanism extends SubsystemBase{
    public static double GEAR_RATIO = 1;
   double algaePositionConversionFactor =
   2 * Math.PI / AlgaeMechanism.GEAR_RATIO; // revolutions -> radians
private double algaeVelocityConversionFactor = 1;
private double aDesiredPos;
    //A motor to rotate up and down
   private SparkMax m_AlgaeMotor = new SparkMax(Constants.Algae.m_AlgaeMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
   private TalonSRX m_IntakeMotor = new TalonSRX(Constants.Algae.m_IntakeMtrC);
   private SparkMaxConfig m_AlgaeConfig = new SparkMaxConfig();

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
        
       

            m_AlgaeMotor.configure(m_AlgaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            aDesiredPos =0;
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
        m_IntakeMotor.set(ControlMode.PercentOutput,.85);
    }

    public void IntakeBackward() {
        m_IntakeMotor.set(ControlMode.PercentOutput,-.85);
    }

    public void IntakeStop() {
        m_IntakeMotor.set(ControlMode.PercentOutput, 0);
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
  public Rotation2d getAlgaePos() {
        return Rotation2d.fromRadians(m_AlgaeMotor.getAlternateEncoder().getPosition());
    }



    public Rotation2d desiredPosGet() {
        return Rotation2d.fromDegrees(aDesiredPos);
    }
    public void AlgaeMove(double m) {
        m_AlgaeMotor.set(m);
    }
 @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Algae/Algae/Position", () -> getAlgaeEncoderPos(), null);
  }
}
