package frc.robot.support;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.Objects;

public class SparkMaxEncoderStrategy {

    private final SparkMax motor;

    private SparkRelativeEncoderSim encoderSim;

    public SparkMaxEncoderStrategy(SparkMax motor) {
        this.motor = motor;
    }

    public double getPosition(){
        if(RobotBase.isSimulation()){
            if(Objects.isNull(encoderSim)){
                this.encoderSim = new SparkRelativeEncoderSim(motor);
            }
            return encoderSim.getPosition();
        }else{
            return motor.getEncoder().getPosition();
        }
    }
}
