package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class CoralMechanism extends SubsystemBase{

    TalonSRX m_coralMotor1 = new TalonSRX(Constants.Port.m_CoralMtrRC);
    TalonSRX m_coralMotor2 = new TalonSRX(Constants.Port.m_CoralMtrLC);
    AnalogInput IRb = new AnalogInput(0);
    AnalogInput IRf = new AnalogInput(1);
  
    public CoralMechanism(){
        
    }

    public void CoralForward() {
        m_coralMotor1.set(ControlMode.PercentOutput, .55);
        m_coralMotor2.set(ControlMode.PercentOutput, -.55);
    }


    public void CoralBackward() {
        m_coralMotor1.set(ControlMode.PercentOutput, -.55);
        m_coralMotor2.set(ControlMode.PercentOutput, .55);
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

    public Command CoralIntakeAutoCmd(){
        return this.runEnd(this::CoralForward, this::CoralStop).onlyWhile(()->!IsCoralLoaded());
    }

    public Command CoralStopCmd() {
        return this.runOnce(this::CoralStop);
    }

    public int IrReadingf(){
       return IRf.getValue();
    }

    public int IrReadingb(){
        return IRb.getValue();
    }

    public boolean IsCoralLoaded(){
        return (IrReadingf() > 1400 && IrReadingb() > 1400);
    }

    public void periodic(){
    
    }
@Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Coral/IRf", ()-> IrReadingf(), null);
    builder.addDoubleProperty("Coral/IRb", ()-> IrReadingb(), null);
    builder.addBooleanProperty("Coral/CoralLoaded", ()-> IsCoralLoaded(), null);
  }
}
