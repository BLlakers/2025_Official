package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class PathFindToPose extends Command {
    Supplier<Pose2d> currentPose2d;
    Boolean side;
    DriveTrain driveTrain;

public void init(){
}
public PathFindToPose(Supplier<Pose2d> CurrentPose, Boolean Left, DriveTrain d){
    currentPose2d = CurrentPose;
    side = Left;
    driveTrain = d;
}

    @Override
    public void execute() {
        Pose2d RoboPose = currentPose2d.get();
        Pose2d goalposition;

        if (side == true){
        goalposition = RoboPose.nearest(Constants.Poses.PositionsLeft);
        } else {
        goalposition = RoboPose.nearest(Constants.Poses.PositionsRight); 
        }
       System.out.println(goalposition);
        
        AutoBuilder.pathfindToPose(goalposition, RobotContainer.SPEED_CONSTRAINTS,0.0);

    }
}
