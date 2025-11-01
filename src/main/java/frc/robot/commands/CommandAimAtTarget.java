package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightIO;

import java.util.function.Supplier;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

/**
 * A command that aims the robot toward the Limelight target by commanding
 * zero translation and a rotational rate computed from a PID on tx.
 *
 * This command schedules an internal supplier-based command produced by
 * drivetrain.applyRequest(...) so it works cleanly with the drivetrain's
 * supplier-style API while keeping PID lifecycle in this class.
 */
public class CommandAimAtTarget extends Command {
    private final LimelightIO limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController pid;
    private final double maxDegPerSec;

    // The supplier-based command returned by drivetrain.applyRequest(...)
    // We build it in initialize so it captures the live PID each cycle.
    private Command internalApplyRequestCommand;

    public CommandAimAtTarget(LimelightIO limelight, CommandSwerveDrivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        // PID uses degrees as input; constants come from your Constants file
        pid = new PIDController(
            Constants.Limelight.Aim.kP,
            Constants.Limelight.Aim.kI,
            Constants.Limelight.Aim.kD
        );

        maxDegPerSec = Constants.Limelight.Aim.kMaxDegPerSec;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setTolerance(Constants.Limelight.Aim.kToleranceDeg);

        Supplier<SwerveRequest> supplier = () -> {
            // If no target visible, stop rotating
            if (limelight.getTC() < 1.0) {
                return new SwerveRequest.FieldCentric()
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
            }

            double txDeg = limelight.getTX();
            double outputDegPerSec = pid.calculate(txDeg, 0.0);

            if (outputDegPerSec > maxDegPerSec) outputDegPerSec = maxDegPerSec;
            if (outputDegPerSec < -maxDegPerSec) outputDegPerSec = -maxDegPerSec;

            double omegaRadPerSec = Math.toRadians(outputDegPerSec);

            return new SwerveRequest.FieldCentric()
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(omegaRadPerSec)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        };

        internalApplyRequestCommand = drivetrain.applyRequest(supplier);
        internalApplyRequestCommand.schedule();
    }

    @Override
    public void execute() {
        // Nothing else needed here; the internalApplyRequestCommand's supplier is doing the work.
        // We keep execute empty so all control remains inside the supplier / drivetrain API.
    }

    @Override
    public void end(boolean interrupted) {
        // Cancel the internal supplier command
        if (internalApplyRequestCommand != null) {
            internalApplyRequestCommand.cancel();
        }

        // Build a typed Supplier that returns a stopping SwerveRequest
        Supplier<SwerveRequest> stopSupplier = () -> new SwerveRequest.FieldCentric()
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Schedule the zero-motion request so drivetrain receives it for at least one cycle
        drivetrain.applyRequest(stopSupplier).schedule();
    }

    @Override
    public boolean isFinished() {
        // Finish when a target is visible and PID is at setpoint.
        // If the target is lost, keep running (or you can choose to end).
        return limelight.getTC() >= 1.0 && pid.atSetpoint();
    }
}
