package frc.robot.subsystems;

import java.lang.module.ModuleDescriptor.Builder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase{
 private TalonSRX m_IntakeMotor = new TalonSRX(Constants.Algae.m_IntakeMtrC);
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensorV3 = new ColorSensorV3(i2cPort);
public static double IRVALUE = 50;
private DigitalInput TEST = new DigitalInput(9);
 public AlgaeIntake(){}


 public void IntakeForward() {
        m_IntakeMotor.set(ControlMode.PercentOutput,.85);
    }
public boolean IntakeFowardIR(){
    if (m_colorSensorV3.getIR() > IRVALUE){
        return true;
    }
    else {
        return false;
    }
}

    public void IntakeBackward() {
        m_IntakeMotor.set(ControlMode.PercentOutput,-.85);
    }

    public void IntakeStop() {
        m_IntakeMotor.set(ControlMode.PercentOutput, 0);
    }
 public Command IntakeForwardCmd() {
        return this.runOnce(this::IntakeForward);
    }

    public Command IntakeBackwardCmd() {
        return this.runEnd(this::IntakeBackward, this::IntakeStop);
    }
    public Command IntakeStopCmd() {
        return this.runOnce(this::IntakeStop);
    }

    public Command RunIntake(){
        return this.runEnd(this::IntakeForward, this::IntakeStop).onlyWhile(()-> IntakeFowardIR() == false);
    }
@Override
public void initSendable(SendableBuilder builder) {
    
    super.initSendable(builder);
    builder.addBooleanProperty(this.getName() + "/Intake/IRGOOD", ()->IntakeFowardIR(), null);
    builder.addDoubleProperty(this.getName() + "/Intake/IR/Value", ()-> m_colorSensorV3.getIR(), null);

}
}
