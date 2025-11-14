// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Led;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public final PhotonCamera limelight = new PhotonCamera("OV9281");
    private PIDController alignTagPid = new PIDController(0.2,0.0,0.0);
    private PIDController strafePosePid = new PIDController(5.0, 0.003, 0.0);
    private PIDController forwPosePid = new PIDController(4.0, 0.003, 0.0);
    private PIDController rotPosePid = new PIDController(6, 0.003, 0.0);
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    public PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandJoystick joystick = new CommandJoystick(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Led ledIO = new Led();

    // final DoublePublisher rot1;
    // final DoublePublisher rot2;
    // final DoublePublisher rot3;
    // final DoublePublisher rot4;
    // final DoublePublisher rot5;
    // final DoublePublisher rot6;
    // final DoubleTopic rotTopic;

    private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
    private final NetworkTable camTable = networkTable.getTable("Cam");
    DoublePublisher rot1;
    DoublePublisher rot2;
    DoublePublisher rot3;
    DoublePublisher rot4;
    DoublePublisher rot5;
    DoublePublisher rot6;

    

    public RobotContainer() {
        configureBindings();
        rot1 = camTable.getDoubleTopic("rot1").publish();
        rot2 = camTable.getDoubleTopic("rot2").publish();
        rot3 = camTable.getDoubleTopic("rot3").publish();
        rot4 = camTable.getDoubleTopic("rot4").publish();
        rot5 = camTable.getDoubleTopic("rot5").publish();
        rot6 = camTable.getDoubleTopic("rot6").publish();
    }
    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                ledIO.setAndApplyStatus(Constants.Led.StatusList.IDLE);
                
                return drive.withVelocityX((-joystick.getY() * MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY((-joystick.getX() * MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate((-joystick.getZ() * MaxAngularRate)); // Drive counterclockwise with negative X (left)
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.button(1).whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.button(3).whileTrue(
            drivetrain.applyRequest(() -> {
                var result = limelight.getLatestResult();
            
                if (result.hasTargets()) {
                    ledIO.setAndApplyStatus(Constants.Led.StatusList.READY);
                    // Use the best target’s yaw as the measurement
                    Transform3d bestCameraToTarget = result.getBestTarget().getBestCameraToTarget();
                    var best =  result.getBestTarget();
                    
                    // forwPosePid
                    // rotPosePid
                    // strafePosePid
                    double forwardTagOffset = bestCameraToTarget.getX() - (joystick.getThrottle() + 1);
                    double strafeTagOffset = bestCameraToTarget.getY();
                    Rotation3d rotationTag = bestCameraToTarget.getRotation();
                    double rotationTagOffset = rotationTag.getQuaternion().getX();

                    rot1.set(bestCameraToTarget.getRotation().getX());
                    rot2.set(bestCameraToTarget.getRotation().getY());
                    rot3.set(bestCameraToTarget.getRotation().getZ());
                    rot4.set(bestCameraToTarget.getRotation().getAngle());
                    rot5.set(bestCameraToTarget.getRotation().getQuaternion().getW());
                    rot6.set(bestCameraToTarget.getRotation().getQuaternion().getX());


                    // System.out.println("poseAmbiguity=" + best.getPoseAmbiguity());
                    Transform3d t = best.getBestCameraToTarget(); // try this before alternate
                    // System.out.println("Transform: " + t);

                    //// System.out.print("Forward: " + forwardTagOffset + " Strafe: " + strafeTagOffset + " Rotation: " + rotationTagOffset + "\n");
                    
                    return drive.withVelocityX(-MathUtil.clamp(forwPosePid.calculate(forwardTagOffset, 0), -0.9, 0.9))
                        .withVelocityY(-MathUtil.clamp(strafePosePid.calculate(strafeTagOffset, 0), -0.9, 0.9))
                        // .withRotationalRate(MathUtil.clamp(rotPosePid.calculate(rotationTagOffset, 0), -0.4, 0.4)
                        // .withRotationalRate(-joystick.getZ() * MaxAngularRate
                        .withRotationalRate(MathUtil.clamp(rotPosePid.calculate(rotationTagOffset, 0), -1.0, 1.0));
                } else {
                    ledIO.setAndApplyStatus(Constants.Led.StatusList.UNSAFE);
                    // no target detected, stop
                    return drive.withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(0);
                }
            })
        );

        joystick.button(11).whileTrue(
            drivetrain.applyRequest(() -> {
                var result = limelight.getLatestResult();
                
                if (result.hasTargets()) {
                    ledIO.setAndApplyStatus(Constants.Led.StatusList.READY);
                    // Use the best target’s yaw as the measurement
                    double yaw = result.getBestTarget().getYaw();
                    // System.out.print("Yaw: " + yaw + "\n");
                    return drive.withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(
                                    MathUtil.clamp(alignTagPid.calculate(yaw, 0), -1.5, 1.5)
                                );
                } else {
                    ledIO.setAndApplyStatus(Constants.Led.StatusList.UNSAFE);
                    // no target detected, stop
                    return drive.withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(0);
                }
            })
        );

        joystick.button(12).whileTrue(
            drivetrain.applyRequest(() -> {
                var result = limelight.getLatestResult();
                if (result.hasTargets()) {
                    ledIO.setAndApplyStatus(Constants.Led.StatusList.READY);
                    // Use the best target’s yaw as the rotation force, but allow joystick translation at the same time
                    double yaw = result.getBestTarget().getYaw();
                    return drive.withVelocityX(-joystick.getY() * MaxSpeed)
                                .withVelocityY(-joystick.getX() * MaxSpeed)
                                .withRotationalRate(
                                    MathUtil.clamp(alignTagPid.calculate(yaw, 0), -1.5, 1.5)
                                );
                } else {
                    ledIO.setAndApplyStatus(Constants.Led.StatusList.UNSAFE);
                    // no target detected, stop
                    return drive.withVelocityX(-joystick.getY() * MaxSpeed)
                                .withVelocityY(-joystick.getX() * MaxSpeed)
                                .withRotationalRate(0);
                }
            })
        );



        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.button(2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public Command pushLimelightValuesCommand() {
        
        return Commands.none();
    }
}
