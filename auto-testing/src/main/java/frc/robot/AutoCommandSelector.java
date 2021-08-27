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

    // test sequential command groups
    public final SequentialCommandGroup test;
    public final SequentialCommandGroup circle;
    public final SequentialCommandGroup forward;
    public final SequentialCommandGroup curve;

    // full auto command groups
    public final SequentialCommandGroup b8_m2_t3_startG;
    public final SequentialCommandGroup b8_m2_t3_startC;
    public final SequentialCommandGroup b8_m2_t3_startD;

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
        b8_m2_t3_startG = new SequentialCommandGroup(
            ramsete.createRamseteCommand(Ramsete.Paths.GOAL_INIT_TO_MID),
            ramsete.createRamseteCommand(Ramsete.Paths.MID_TO_SCORING),
            ramsete.createRamseteCommand(Ramsete.Paths.SCORING_TO_TRENCH),
            ramsete.createRamseteCommand(Ramsete.Paths.TRENCH_TO_SCORING)
        );
        b8_m2_t3_startC = new SequentialCommandGroup(
            ramsete.createRamseteCommand(Ramsete.Paths.CENTER_INIT_TO_MID),
            ramsete.createRamseteCommand(Ramsete.Paths.MID_TO_SCORING),
            ramsete.createRamseteCommand(Ramsete.Paths.SCORING_TO_TRENCH),
            ramsete.createRamseteCommand(Ramsete.Paths.TRENCH_TO_SCORING)
        );
        b8_m2_t3_startD = new SequentialCommandGroup(
            ramsete.createRamseteCommand(Ramsete.Paths.DEPOT_INIT_TO_MID),
            ramsete.createRamseteCommand(Ramsete.Paths.MID_TO_SCORING),
            ramsete.createRamseteCommand(Ramsete.Paths.SCORING_TO_TRENCH),
            ramsete.createRamseteCommand(Ramsete.Paths.TRENCH_TO_SCORING)
        );
        

        // trajectory map
        firstTrajectoryMap.put(test, Ramsete.Paths.FORWARD.getTrajectory());
        firstTrajectoryMap.put(circle, Ramsete.Paths.CIRCLE.getTrajectory());
        firstTrajectoryMap.put(forward, Ramsete.Paths.FORWARD.getTrajectory());
        firstTrajectoryMap.put(curve, Ramsete.Paths.CURVE.getTrajectory());
        firstTrajectoryMap.put(b8_m2_t3_startG, Ramsete.Paths.GOAL_INIT_TO_MID.getTrajectory());
        firstTrajectoryMap.put(b8_m2_t3_startC, Ramsete.Paths.CENTER_INIT_TO_MID.getTrajectory());
        firstTrajectoryMap.put(b8_m2_t3_startD, Ramsete.Paths.DEPOT_INIT_TO_MID.getTrajectory());
    }

    public void setInitialDrivePose(SequentialCommandGroup auto) {
        if(firstTrajectoryMap.containsKey(auto)) {
            drive.setOdometry(firstTrajectoryMap.get(auto));
            drive.putTrajetoryOnField(firstTrajectoryMap.get(auto));
        }  
    }
}