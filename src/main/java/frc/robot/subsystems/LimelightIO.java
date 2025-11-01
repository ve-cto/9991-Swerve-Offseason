package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LimelightIO extends SubsystemBase {
    private final String cam = "limelight";
    
    // Instantiate
    public LimelightIO() {}

    public double getTX() {
        double tx = LimelightHelpers.getTX(cam);
        return tx;
    }
    public double getTY() {
        double ty = LimelightHelpers.getTY(cam);
        return ty;
    }
    public double getTC() {
        double tc = LimelightHelpers.getTargetCount(cam);
        return tc;
    }
}
