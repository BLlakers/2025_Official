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
    AnalogInput IR = new AnalogInput(0);

    PWM servo = new PWM(9);
  
    public CoralMechanism(){
        
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

    public void ServoForward() {
        servo.setPosition(1);
    }

    public void ServoBackward() {
        servo.setPosition(0);
    }

    public void ServoMiddle() {
        servo.setPosition(.5);
    }

    public Command ServoForwardCommand() {
        return this.runEnd(this::ServoForward, this::ServoMiddle);
    }

    public Command ServoBackwardCommand() {
        return this.runEnd(this::ServoBackward, this::ServoMiddle);
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
    builder.addDoubleProperty("Coral/IR", ()-> IrReading(), null);
  }
}
