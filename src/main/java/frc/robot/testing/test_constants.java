package frc.robot.testing;

import javax.swing.filechooser.FileNameExtensionFilter;

public final class test_constants {
    public final static class Motor{
        public static final int left_frontFalcon = 4;
        public static final int right_frontFalcon = 2;
        public static final int left_rearFalcon =0;
        public static final int right_rearFalcon = 0;
        public static final int left_front775 = 24;
        public static final int right_front775 = 22;
        public static final int right_rear775 = 0;
        public static final int left_rear775 = 0;
        /*
        These are the constants for the CAN iD of motors on the robot. When adding new motors, please use the specific
        position of the motor. 
        */
    } 
    public final static class CANCoder{
        public static final int left_frontCoder = 0;
        public static final int right_frontCoder = 0;
        public static final int left_rearCoder = 0;
        public static final int right_rearCoder = 0; 
        /*
        Same as above, the CAN ID of CANCoder, if there are new encoders with encoder ports, remember to add constants for ports. 
        Use specific position of the CANCoder when naming. 
        */
    }
    public final static class electronics{
        public static final int pdp = 0;
        public static final int pcm = 0;
        /* 
        CAN ID for other electronic devices. Pneumatics devices is also included here.  
        */
    }
    public final static class driveConstants{
        public static final double driveOutputMax = 0.4;
        public static final double turnOutputMax = 0.4;
    }
    public final static class Joystick{
        public static final int stickX = 0;
        public static final int stickY = 4;
        /**
        Ports of the Joystick. 
         */
    }
    public final static class PIDconstant{
        public static final double drivingKp = 0.2;
        public static final double drivingKd = 0;
        public static final double drivingKi = 0;
    }
}
