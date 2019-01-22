/* I really just took this file and started
   hacking away because I don't know if the
   Limelight stuff should be in another file */

package frc.robot;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight {
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
    public enum pipe {
	PIPE1, PIPE2, PIPE3, PIPE4, PIPE5,
	PIPE6, PIPE7, PIPE8, PIPE9, PIPE10
    }
    public Limelight() {
        NetworkTable table;
    table = NetworkTableInstance.getDefault().getTable("limelight");

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
    }
    public double getTHORIZ() {
        return thoriz.getDouble(0.0);
    }
    public double getTVERT() {
        return tvert.getDouble(0.0);
    } /* Thank goodness for ELISP, I'd go nuts writing all that boilerplate */
    //public double fixedAngleDist(double h1, double h2, double a1) {
	/* TODO: finish this
	   If the camera never changes its angle, we know its height and the
	   target's height, we can very accurately calculate the X distance
	   to said object
	   
	   double h1: Our height
	   double h2: Target height
	   double a1: Our mounting angle
	   We can figure out the angle to the target using the camera... somehow */
	   
    //}
}
