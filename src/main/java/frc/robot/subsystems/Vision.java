package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Vision extends SubsystemBase {
    private final NetworkTable m_limelightTable;
    private double tx, ty, ta;
    private ArrayList<Double> m_targetList;
    private final int MAX_ENTRIES = 50;

    /**
    * Creates a new Vision.
    */
    public Vision() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_targetList = new ArrayList<Double>(MAX_ENTRIES);
    }

    @Override
    public void periodic() {
        m_limelightTable.getEntry("tv").getDouble(0);
        tx = m_limelightTable.getEntry("tx").getDouble(0);
        ty = m_limelightTable.getEntry("ty").getDouble(0);
        ta = m_limelightTable.getEntry("ta").getDouble(0);
                
        if (m_targetList.size() >= MAX_ENTRIES) {
            m_targetList.remove(0);
        }
        
        m_targetList.add(ta);

        if (Constants.Display.showHorizontalVisionError) {
            SmartDashboard.putNumber("Vision Data", getTX());
        }

        if (Constants.Display.showVerticalVisionError) {
            SmartDashboard.putNumber("Vision Data", getTY());
        }

        if (Constants.Display.showDistanceVisionError) {
            SmartDashboard.putNumber("Vision Data", getTA()); // Might need to be changed
        }
    }

    public double getTX() {
        return tx;
    }

    public double getTY() {
        return ty;
    }

    public double getTA() {
        double sum = 0;

        for (Double num : m_targetList) { 		      
            sum += num.doubleValue();
        }
        return sum/m_targetList.size();
    }

    public boolean hasTarget() {
        NetworkTableEntry tv = m_limelightTable.getEntry("tv");
        return tv.getDouble(0) >= 1.0; //tv represents whether it has a vision target
    }
}
