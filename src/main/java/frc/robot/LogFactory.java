package frc.robot;

import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public final class LogFactory {

    public static final DoubleLogEntry getDouble(String name) {
        return new DoubleLogEntry(DataLogManager.getLog(), name);
    }

    public static final DoubleArrayLogEntry getDoubleArray(String name) {
        return new DoubleArrayLogEntry(DataLogManager.getLog(), name);
    }

    public static final IntegerLogEntry getInteger(String name) {
        return new IntegerLogEntry(DataLogManager.getLog(), name);
    }

    public static final IntegerArrayLogEntry getIntegerArray(String name) {
        return new IntegerArrayLogEntry(DataLogManager.getLog(), name);
    }

    public static final BooleanLogEntry getBoolean(String name) {
        return new BooleanLogEntry(DataLogManager.getLog(), name);
    }

    public static final BooleanArrayLogEntry getBooleanArray(String name) {
        return new BooleanArrayLogEntry(DataLogManager.getLog(), name);
    }

    public static final StringLogEntry getString(String name) {
        return new StringLogEntry(DataLogManager.getLog(), name);
    }

    public static final StringArrayLogEntry getStringArray(String name) {
        return new StringArrayLogEntry(DataLogManager.getLog(), name);
    }


    private LogFactory() {}
}
