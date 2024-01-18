package frc.robot.subsystems.arm;

public class ArmConstants {
    private state = 0;
    /**
    @author Yellow
    @param displacement distance the arm has extended
    @param maxDisplace
    */
   //
   public static enum ArmState
   {
        ZERO(0.0),
        SPEAKER(1.0),
        AMP(2.0);

        public double position;
        private ArmState(double position)
        {
            this.position = position;
        }

   }

}
