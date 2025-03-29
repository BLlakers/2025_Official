package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class PathFindToPose extends Command {
    Supplier<Pose2d> currentPose2d;
    Boolean side;
    DriveTrain driveTrain;
    PIDController xController = new PIDController(0, 0, 0);
    PIDController yController = new PIDController(0, 0, 0);
    PIDController rotController = new PIDController(0, 0, 0);
public PathFindToPose(Supplier<Pose2d> CurrentPose, Boolean Left, DriveTrain d){
    currentPose2d = CurrentPose;
    side = Left;
    driveTrain = d;
}

    @Override
    public void execute() {
        Pose2d RoboPose = currentPose2d.get();
        Pose2d goalposition;
        double Xspeed =0;
double Yspeed =0;
double Rotspeed =0;

        if (side == true){
        goalposition = currentPose2d.get().nearest(Constants.Poses.PositionsLeft);
        } else {
        goalposition = currentPose2d.get().nearest(Constants.Poses.PositionsRight); 
        }
        xController.setSetpoint(goalposition.getX());
        yController.setSetpoint(goalposition.getY());
        rotController.setSetpoint(goalposition.getRotation().getRadians());
       
        Xspeed = xController.calculate(RoboPose.getX());
        Yspeed = yController.calculate(RoboPose.getY());
        Rotspeed = rotController.calculate(RoboPose.getRotation().getRadians());
        
        // AutoBuilder.pathfindToPose(goalposition, RobotContainer.SPEED_CONSTRAINTS,0.0);
        System.out.println(Xspeed);

        driveTrain.drive(Xspeed, Yspeed, Rotspeed);

    }
}
