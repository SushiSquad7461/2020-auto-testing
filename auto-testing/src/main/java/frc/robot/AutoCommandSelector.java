package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.databind.SequenceWriter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.trajectory.Trajectory;


public class AutoCommandSelector {
    // necessary dependencies
    private final Drivetrain drive;
    private final Ramsete ramsete;

    // sequential command groups
    public final SequentialCommandGroup test;
    public final SequentialCommandGroup circle;
    public final SequentialCommandGroup forward;
    public final SequentialCommandGroup curve;

    public final Map<SequentialCommandGroup, Trajectory> firstTrajectoryMap;

    public AutoCommandSelector(Drivetrain drive, Ramsete ramsete) {
        // instantiate dependencies
        this.drive = drive;
        this.ramsete = ramsete;

        this.firstTrajectoryMap = new HashMap<SequentialCommandGroup, Trajectory>();
        
        // create command groups
        test = new SequentialCommandGroup(
            ramsete.createRamseteCommand(Ramsete.Paths.FORWARD),
            ramsete.createRamseteCommand(Ramsete.Paths.CURVE));
        circle = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.Paths.CIRCLE));
        forward = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.Paths.FORWARD));
        curve = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.Paths.CURVE));
        

        // trajectory map
        firstTrajectoryMap.put(test, Ramsete.Paths.FORWARD.getTrajectory());
        firstTrajectoryMap.put(circle, Ramsete.Paths.CIRCLE.getTrajectory());
        firstTrajectoryMap.put(forward, Ramsete.Paths.FORWARD.getTrajectory());
        firstTrajectoryMap.put(curve, Ramsete.Paths.CURVE.getTrajectory());
    }

    public void setInitialDrivePose(SequentialCommandGroup auto) {
        if(firstTrajectoryMap.containsKey(auto)) {
            drive.setOdometry(firstTrajectoryMap.get(auto));
        }  
    }
}