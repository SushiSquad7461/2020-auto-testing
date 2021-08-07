package frc.robot;

import com.fasterxml.jackson.databind.SequenceWriter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutoCommandSelector {
    // necessary dependencies
    private final Drivetrain drive;
    private final Ramsete ramsete;

    // sequential command groups
    public final SequentialCommandGroup test;
    public final SequentialCommandGroup circle;
    public final SequentialCommandGroup forward;

    public AutoCommandSelector(Drivetrain drive, Ramsete ramsete) {
        // instantiate dependencies
        this.drive = drive;
        this.ramsete = ramsete;
        
        // create command groups
        test = new SequentialCommandGroup(
            ramsete.createRamseteCommand(Ramsete.Paths.FORWARD),
            ramsete.createRamseteCommand(Ramsete.Paths.CIRCLE));
        circle = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.Paths.CIRCLE));
        forward = new SequentialCommandGroup(ramsete.createRamseteCommand(Ramsete.Paths.FORWARD));
    }
}