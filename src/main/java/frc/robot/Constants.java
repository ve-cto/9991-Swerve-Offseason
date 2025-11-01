package frc.robot;

public class Constants {
    public static final class Limelight {
        public static final double MaxAimSpeed = 0.3;
        public static final double MaxRangeSpeed = 0.3;
    
        public static final class Aim {
            public static final double kP = 0.03;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kMaxDegPerSec = 120.0;
            public static final double kToleranceDeg = 1.0;
        }
          
        public static class Range {}
    }

    public static class Led {
        public static final int l_ledID = 0;
        
        public static enum StatusList {
            DISCONNECT,
            DISABLED,
            IDLE,
            AUTONOMOUS,
            BLANK
        }
    }
    
}
