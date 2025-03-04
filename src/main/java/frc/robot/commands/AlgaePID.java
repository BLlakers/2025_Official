package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.Constants.Algae;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeMechanism;
import frc.robot.subsystems.DriveTrain; 
import frc.robot.subsystems.ElevatorMechanism;

public class AlgaePID extends Command {
  private AlgaeMechanism m_algaeMech;
  private double position;

  public AlgaePID(AlgaeMechanism algaeMech, Double pos) {
    position = pos;
    m_algaeMech = algaeMech;
    addRequirements(m_algaeMech);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_algaeMech.initAlgaePID(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_algaeMech.AlgaePID(position);
  }
  
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algaeMech.AlgaeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_algaeMech.CheckAlgaePID();
  }
}
