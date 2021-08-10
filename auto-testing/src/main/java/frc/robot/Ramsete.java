package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.List;

public class Ramsete {
    private TrajectoryConfig config;
    private DifferentialDriveVoltageConstraint voltageConstraint;
    private Drivetrain drive;
    private SimpleMotorFeedforward ramseteFF; 
    
    // path enums
    public enum Paths {
        FORWARD("Pathweaver/output/forward.wpilib.json"),
        CIRCLE("Pathweaver/output/circle.wpilib.json");
        private String json;
        Paths(String json) {
            this.json = json;
        }

        public Trajectory getTrajectory() {
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(json);
                return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) { 
                return null; 
            }
        }
    }

    public Ramsete(Drivetrain drive) {
        this.drive = drive;
        ramseteFF = new SimpleMotorFeedforward(
            Constants.kRamsete.kS,
            Constants.kRamsete.kV,
            Constants.kRamsete.kA);
        config = new TrajectoryConfig(
            Constants.kRamsete.MAX_METERS_PER_SECOND,
            Constants.kRamsete.MAX_ACCEL_METERS_PER_SECOND_SQUARED)
            .setKinematics(this.drive.getKinematics())
            .addConstraint(getVoltageConstraint());

        voltageConstraint = new DifferentialDriveVoltageConstraint(
            ramseteFF,
            this.drive.getKinematics(),
            Constants.kRamsete.MAX_VOLTAGE);
    }

    public RamseteCommand createRamseteCommand(Paths path) {
        return new RamseteCommand(
            path.getTrajectory(),
            drive::getPose,
            new RamseteController(
                Constants.kRamsete.B,
                Constants.kRamsete.ZETA),
            ramseteFF,
            drive.getKinematics(),
            drive::getWheelSpeeds,
            new PIDController(
                Constants.kRamsete.LEFT_kP,
                Constants.kRamsete.LEFT_kI,
                Constants.kRamsete.LEFT_kD),
            new PIDController(
                Constants.kRamsete.RIGHT_kP,
                Constants.kRamsete.RIGHT_kI,
                Constants.kRamsete.RIGHT_kD),
            drive::tankDriveVolts,
            drive);
    }

    public DifferentialDriveVoltageConstraint getVoltageConstraint() {
        return voltageConstraint;
    }

}
