package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeMechanism;
import frc.robot.subsystems.DriveTrain; 
import frc.robot.subsystems.ElevatorMechanism;

public class AlgaePID extends Command {
 private DoubleSupplier position;
  private AlgaeMechanism algae;
  private ProfiledPIDController pid = new ProfiledPIDController(.1 , 0, 0, ELEVATOR_CONSTRAINTS);
  private static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(Units.feetToMeters(5),Units.feetToMeters(2.5));


  public AlgaePID(AlgaeMechanism e, DoubleSupplier d) {

    algae = e;
    position = d;
    addRequirements(algae);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   pid.reset(algae.getAlgaeEncoderPos()); 
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    pid.setGoal(position.getAsDouble());
    double m_AlgaeSpeed = pid.calculate(algae.getAlgaePos().getRadians());
    if (pid.atGoal()) {
      m_AlgaeSpeed = 0;
    } 
      SmartDashboard.putNumber(this.getName() + "AlgaeCommand/Command/AlgaeSpeed", m_AlgaeSpeed);
      SmartDashboard.putNumber(this.getName() + "AlgaeCommand/Command/AlgaePos", algae.getAlgaePos().getRadians());
      algae.AlgaeMove(m_AlgaeSpeed); // gear ratio is 4


  }
public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    pid.initSendable(builder);
    builder.addDoubleProperty(this.getName() + "/position", ()->  position.getAsDouble(), null);
 }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  algae.AlgaeStop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}