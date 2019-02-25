/* I really just took this file and started
   hacking away because I don't know if the
   Limelight stuff should be in another file */

   package frc.robot.lib;

   //import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
   import edu.wpi.first.networktables.NetworkTable;
   import edu.wpi.first.networktables.NetworkTableEntry;
   import edu.wpi.first.networktables.NetworkTableInstance;
   
   public class Limelight {
       private NetworkTable table;
       private NetworkTableEntry tv;
       private NetworkTableEntry tx;
       private NetworkTableEntry ty;
       private NetworkTableEntry ta;
       private NetworkTableEntry ts;
       private NetworkTableEntry tl;
       private NetworkTableEntry tshort;
       private NetworkTableEntry tlong;
       private NetworkTableEntry thoriz;
       private NetworkTableEntry tvert;
       private NetworkTableEntry getpipe;
       //public byte pipeline;
       /* public enum pipe {
       PIPE1, PIPE2, PIPE3, PIPE4, PIPE5,
       PIPE6, PIPE7, PIPE8, PIPE9, PIPE10
       } 
       We only need ints for this (I'm using chars for the 
   byte's memory savings)*/
        public Limelight() {
            this("");
        }

        public Limelight(String name) {
        if(name.isBlank()){
            table = NetworkTableInstance.getDefault().getTable("limelight");
        } else {
            table = NetworkTableInstance.getDefault().getTable("limelight-"+name);

        }
       tv     = table.getEntry("tv");
       tx     = table.getEntry("tx");
       ty     = table.getEntry("ty");
       ta     = table.getEntry("ta");
       ts     = table.getEntry("ts");
       tl     = table.getEntry("tl");
       tshort = table.getEntry("tshort");
       tlong  = table.getEntry("tlong");
       thoriz = table.getEntry("thoriz");
       tvert  = table.getEntry("tvert");
       getpipe= table.getEntry("getpipe");
       }


       public void setPipeline(byte id) {
       //pipeline = id;
       table.getEntry("pipeline").setNumber(id);
       }
       public void setLED(byte mode) {
       /* 0: Pipeline default (probably should just use this)
          1: Off
          2: Blink
          3: On */
       table.getEntry("ledMode").setNumber(mode);
       }
       public void setStream(byte mode) {
       /* 0: Standard - Side-by-side streams if a webcam is attached to Limelight
          1: The secondary camera stream is placed in the lower-right corner of the primary camera stream
          2: The primary camera stream is placed in the lower-right corner of the secondary camera stream */
       table.getEntry("stream").setNumber(mode);
       } /* At the time of writing this every other site
        I visit had either had its domain expire,
        had some sort of DB error or just said OOPS
        AN ERROR OCCURED 
            Including FIRST's docs.
            REEEEEEEEEEEEEEEEEEEEEE */
       public double getPipeline() {
       /* This would have been a byte but, eh
          I don't feel like casting */
       return (getpipe.getDouble(0));
       }
       public boolean hasTarget() {
       return (tv.getDouble(0.0) == 1);
       }
       public double getTX() {
       return tx.getDouble(0.0);
       }
       public double getTY() {
       return ty.getDouble(0.0);
       }
       public double getTA() {
           return ta.getDouble(0.0);
       }
       public double getTS() {
           return ts.getDouble(0.0);
       }
       public double getTL() {
           return tl.getDouble(0.0); // At the time of writing
       }
       public double getTSHORT() {
           return tshort.getDouble(0.0);
       }
       public double getTLONG() {
           return tlong.getDouble(0.0);
       } /* Monks used to write their thoughts and 
        feelings in the margins of books they copied
        As you can see I've adopted that practise. */
       public double getTHORIZ() {
           return thoriz.getDouble(0.0);
       }
       public double getTVERT() {
           return tvert.getDouble(0.0);
       } 
       /* public double fixedAngleDist(double h1, double h2,
                    double a1) {
       
        TODO: finish this
          If the camera never changes its angle, 
          we know its height and the
          target's height, we can very accurately 
          calculate the X distance
          to said object
          
          double h1: Our height
          double h2: Target height
          double a1: Our mounting angle
          We can figure out the angle to the target using 
          the camera 	   
          } */
   }