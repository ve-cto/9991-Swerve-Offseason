package frc.robot;

public class Constants {
    public static class Led {
        public static final int l_ledID = 0;
        
        public static enum StatusList {
            DISCONNECT,
            DISABLED,
            IDLE,
            IDLERED,
            IDLEBLUE,
            AUTONOMOUS,
            LOADED,
            READY,
            RELEASE,
            UNSAFE,
            BLANK
        }
    }
}
