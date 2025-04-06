package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
/**
   * <b> DETAILED EXPLANATION </b>
   *
   * <p>In this class, we use void functions to run actions, and have commands run those void
   * functions.
   *
   * <p>Void functions don't occupy the command scheduler, but commands do.
   *
   * <p>If we wanted to indivdually run the Coral while a button is held:
   *
   * <p>We would run the Command.
   *
   * <p>In the Example, we run a set of void functions and boolean functions on diffrent modules in
   * parellel.
   *
   * <p>This command runs the void function {@link #MoveHangDown} to run the motors down on each
   * individual module.
   *
   * <p>This command also runs {@link #HangIsAtPosition}, which checks when both hangers are down
   *
   * <p>When both hangers, are down, it will run the {@link #HangStop} void function.
   *
   * <p>If we were to run commands instead of void functions within the {@link Hanger#LowerHangAuto}
   * command, it would not work.
   *
   * <p>The command scheduler can only run 1 command at the same time within in a subsystem.
   *
   * <p>That means if there are multiple commands being called at the same time, one of the commands
   * would not run.
   *
   * <p>Since we are calling the {@link Hanger#LowerHangAuto} command, all the commands within it
   * would not be able to run if they were commands.
   *
   * <p>But, since there are void functions, they are not required to be called by the Command
   * scheduler, and can run smoothly.
   */
public class CoralMechanism extends SubsystemBase{

    TalonSRX m_coralMotor1 = new TalonSRX(Constants.Port.m_CoralMtrRC);
    TalonSRX m_coralMotor2 = new TalonSRX(Constants.Port.m_CoralMtrLC);
    AnalogInput IRb = new AnalogInput(0);
    AnalogInput IRf = new AnalogInput(1);
  
    public CoralMechanism(){
        
    }

    public void CoralForward() {
        m_coralMotor1.set(ControlMode.PercentOutput, .60);
        m_coralMotor2.set(ControlMode.PercentOutput, -.60);
    }


    public void CoralBackward() {
        m_coralMotor1.set(ControlMode.PercentOutput, -.60);
        m_coralMotor2.set(ControlMode.PercentOutput, .60);
    }
    public void CoralTroph(){
        m_coralMotor1.set(ControlMode.PercentOutput, .95);
        m_coralMotor2.set(ControlMode.PercentOutput,-.25);
    }
    public void CoralStop() {
        m_coralMotor1.set(ControlMode.PercentOutput, 0);
        m_coralMotor2.set(ControlMode.PercentOutput, 0);
    }

   

    public Command CoralForwardCmd() {
        return this.runEnd(this::CoralForward, this::CoralStop);
    }
    public Command CoralTrophCmd() {
        return this.runEnd(this::CoralTroph, this::CoralStop);
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
        return (IrReadingf() > 2200 && IrReadingb() > 1600);
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
