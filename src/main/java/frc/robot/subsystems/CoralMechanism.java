package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class CoralMechanism extends SubsystemBase{
   private double CoralPositionConversionFactor = 1;
   private double CoralVelocityConversionFactor = 1;

    //A motor to rotate up and down
    SparkMax m_CoralMotor = new SparkMax(Constants.Port.m_CoralMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig m_CoralConfig = new SparkMaxConfig();
    AnalogInput IR = new AnalogInput(0);
    RelativeEncoder m_CoralMotorEncoder = m_CoralMotor.getAlternateEncoder();

    public CoralMechanism(){
        m_CoralConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        m_CoralConfig.alternateEncoder
            .positionConversionFactor(CoralPositionConversionFactor)
            .velocityConversionFactor(CoralVelocityConversionFactor);
        m_CoralConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pid(1.0,0,0);
        
            m_CoralMotor.configure(m_CoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void CoralForward() {
        m_CoralMotor.set(.85);
    }

    public void CoralBackward() {
        m_CoralMotor.set(-.85);
    }

    public void CoralStop() {
        m_CoralMotor.set(0);
    }
    public Command Test(){
        return this.runOnce(() -> System.out.println("jared"));
    }

    public Command CoralForwardCmd() {
        return this.run(this::CoralForward);
    }

    public Command CoralBackwardCmd() {
        return this.runEnd(this::CoralBackward, this::CoralStop);
    }

    public Command CoralStopCmd() {
        return this.runOnce(this::CoralStop);
    }
    public double getCoralEncoderPos(){
        return m_CoralMotorEncoder.getPosition();
    }

    public int IrReading(){
       return IR.getValue();
    }
    public boolean IsCoralLoaded(){
        return IrReading() > 200;
    }

    public void periodic(){
        
    }
@Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Coral/Position", () -> getCoralEncoderPos(), null);
    builder.addDoubleProperty("Coral/IR", ()-> IrReading(), null);
  }
}
