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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class CoralMechanism extends SubsystemBase{
   private double CoralPositionConversionFactor = 2;
   private double CoralVelocityConversionFactor = 1;

    //A motor to rotate up and down
    TalonSRX m_coralMotor1 = new TalonSRX(11);
    
    TalonSRX m_coralMotor2 = new TalonSRX(12);
    // SparkMax m_coralMotor1 = new SparkMax(Constants.Port.m_CoralMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig m_CoralConfig = new SparkMaxConfig();
    AnalogInput IR = new AnalogInput(0);
    // RelativeEncoder m_CoralMotorEncoder = m_coralMotor1.getAlternateEncoder();

    public CoralMechanism(){
        // m_CoralConfig
        //     .inverted(true)
        //     .idleMode(IdleMode.kBrake);
        // m_CoralConfig.alternateEncoder
        //     .positionConversionFactor(CoralPositionConversionFactor)
        //     .velocityConversionFactor(CoralVelocityConversionFactor);
        // m_CoralConfig.closedLoop
        //     .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        //     .pid(1.0,0,0);
        
        //     m_coralMotor1.configure(m_CoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void CoralForward() {
        m_coralMotor1.set(ControlMode.PercentOutput, .8);
        m_coralMotor2.set(ControlMode.PercentOutput, -.8);
    }


    public void CoralBackward() {
        m_coralMotor1.set(ControlMode.PercentOutput, -.95);
        m_coralMotor2.set(ControlMode.PercentOutput, .95);
    }
    
    public void CoralStop() {
        m_coralMotor1.set(ControlMode.PercentOutput, 0);
        m_coralMotor2.set(ControlMode.PercentOutput, 0);
    }
    public Command CoralForwardCmd() {
        return this.runEnd(this::CoralForward, this::CoralStop);
    }

    public Command CoralBackwardCmd() {
        return this.runEnd(this::CoralBackward, this::CoralStop);
    }

    public Command CoralStopCmd() {
        return this.runOnce(this::CoralStop);
    }

    public int IrReading(){
       return IR.getValue();
    }

    public boolean IsCoralLoaded(){
        return IrReading() > 2000;
    }

    public void periodic(){
    
    }
@Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  //  builder.addDoubleProperty("Coral/Position", () -> getCoralEncoderPos(), null);
    builder.addDoubleProperty("Coral/IR", ()-> IrReading(), null);
  }
}
