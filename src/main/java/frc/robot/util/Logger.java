package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * A homebrew logging class designed to be as barebones as possible.
 */
public class Logger {
    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static NetworkTable table = instance.getTable("logs");
    private static HashMap<String, NetworkTableEntry> publishedEntries = new HashMap<>(0);
    private static HashMap<String, StructPublisher<?>> structPublishers = new HashMap<>(0);

    private static Boolean enabled = true;

    /**
     * Why are you calling this?
     */
    private Logger() {}

    /**
     * <p> Logs any standard Java variable (String, int, boolean, etc) to NetworkTables. Use logStruct() for anything else.
     * <p> If WPILib fails to parse the passed object, the thrown IllegalArgumentException is caught 
     *     and an error message is printed to the terminal rather than crashing code.
     *     If the type of object being logged is different than the object previously 
     *     logged under the same name, it will fail to log and print an error message.
     * @param name A name to be assigned to the logged object.
     * @param object The object to be logged.
     */
    public static void logBasic(String name, Object object) {
        if (enabled) {
            if (publishedEntries.containsKey(name)) {
                try {
                    boolean good = publishedEntries.get(name).setValue(object);
                    if (!good) {
                        System.err.println("ERROR 409: Type of object to be logged conflicts with type of previously logged object with name \"" + name + "\"");
                    }
                } 
                catch (IllegalArgumentException e) {
                    System.err.println("ERROR 400: Caught IllegalArgumentException while trying to log object with type " + object.getClass().getCanonicalName());
                    return;
                }
            }
            else {
                NetworkTableEntry entry = table.getEntry(name);
                try {
                    boolean good = entry.setValue(object);
                    if (!good) {
                        System.err.println("ERROR 409: Type of object to be logged conflicts with type of previously logged object with same name \"" + name + "\"");
                    }
                } 
                catch (IllegalArgumentException e) {
                    System.err.println("ERROR 400: Caught IllegalArgumentException while trying to log object with type " + object.getClass().getCanonicalName());
                    return;
                }
                publishedEntries.put(name, entry);
            }
        }
    }

    /**
     * <p> Logs any standard Java variable (String, int, boolean, etc) to NetworkTables. Use logStruct() for anything else.
     * <p> If WPILib fails to parse the passed object, the thrown IllegalArgumentException is caught 
     *     and an error message is printed to the terminal rather than crashing code.
     *     If the type of object being logged is different than the object previously 
     *     logged under the same name, it will fail to log and print an error message.
     * @param name A name to be assigned to the logged object.
     * @param object The object to be logged.
     * @param tableName The name of the table to log to. Default table is {@code"logs"}.
     */
    public static void logBasic(String name, Object object, String tableName) {
        if (enabled) {
            if (publishedEntries.containsKey(name)) {
                try {
                    boolean good = publishedEntries.get(name).setValue(object);
                    if (!good) {
                        System.err.println("ERROR 409: Type of object to be logged conflicts with type of previously logged object with name \"" + name + "\"");
                    }
                } 
                catch (IllegalArgumentException e) {
                    System.err.println("ERROR 400: Caught IllegalArgumentException while trying to log object with type " + object.getClass().getCanonicalName());
                    return;
                }
            }
            else {
                NetworkTableEntry entry = instance.getTable(tableName).getEntry(name);
                try {
                    boolean good = entry.setValue(object);
                    if (!good) {
                        System.err.println("ERROR 409: Type of object to be logged conflicts with type of previously logged object with same name \"" + name + "\"");
                    }
                } 
                catch (IllegalArgumentException e) {
                    System.err.println("ERROR 400: Caught IllegalArgumentException while trying to log object with type " + object.getClass().getCanonicalName());
                    return;
                }
                publishedEntries.put(name, entry);
            }
        }
    }

    /**
     * <p> Logs any variable that can be expressed as a Struct to NetworkTables.
     *     If the code fails to parse the passed object, an error message is printed to the terminal.
     *     If the type of object being logged is different than the object previously 
     *     logged under the same name, it will fail to log and print an error message.
     * <p> You can add a new type of object to the code as long as the class has a static struct variable
     *     expressing its type as a Struct. (ex. SwerveSample.struct, Translation2d.struct)
     * @param name A name to be assigned to the logged object.
     * @param object The object to be logged.
     */
    public static void logStruct(String name, Object object) {
        if (enabled) {
            if (object == null) {
                System.err.println("ERROR 404: Tried to log null object as \"" + name + "\"");
                return;
            }

            StructPublisher<?> existing = structPublishers.get(name);

            if (existing != null) {
                // Check to make sure publishToExisting() will not return an error.
                if (!existing.getTopic().getStruct().getTypeClass().isInstance(object)) {
                    System.err.println("ERROR 409: Type of object to be logged conflicts with type of previously logged object with name \"" + name + "\"");
                    return;
                }

                // Safe cast because of check above.
                publishToExisting(existing, object);
                return;
            }

            // First time for this name, so we need to create the appropriate StructPublisher.
            StructPublisher<?> newPub = createPublisherForObject(name, object, table);

            if (newPub == null) {
                System.err.println("ERROR 400: Unsupported struct type " + object.getClass().getCanonicalName() + " for \"" + name + "\"");
                return;
            }

            structPublishers.put(name, newPub);
            publishToExisting(newPub, object);
        }
    }

    /**
     * <p> Logs any variable that can be expressed as a Struct to NetworkTables.
     *     If the code fails to parse the passed object, an error message is printed to the terminal.
     *     If the type of object being logged is different than the object previously 
     *     logged under the same name, it will fail to log and print an error message.
     * <p> You can add a new type of object to the code as long as the class has a static struct variable
     *     expressing its type as a Struct. (ex. SwerveSample.struct, Translation2d.struct)
     * @param name A name to be assigned to the logged object.
     * @param object The object to be logged.
     * @param tableName The name of the table to log to. Default table is {@code"logs"}.
     */
    public static void logStruct(String name, Object object, String tableName) {
        if (enabled) {
            if (object == null) {
                System.err.println("ERROR 404: Tried to log null object as \"" + name + "\"");
                return;
            }

            StructPublisher<?> existing = structPublishers.get(name);

            if (existing != null) {
                // Check to make sure publishToExisting() will not return an error.
                if (!existing.getTopic().getStruct().getTypeClass().isInstance(object)) {
                    System.err.println("ERROR 409: Type of object to be logged conflicts with type of previously logged object with name \"" + name + "\"");
                    return;
                }

                // Safe cast because of check above.
                publishToExisting(existing, object);
                return;
            }

            // First time for this name, so we need to create the appropriate StructPublisher.
            StructPublisher<?> newPub = createPublisherForObject(name, object, instance.getTable(tableName));

            if (newPub == null) {
                System.err.println("ERROR 400: Unsupported struct type " + object.getClass().getCanonicalName() + " for \"" + name + "\"");
                return;
            }

            structPublishers.put(name, newPub);
            publishToExisting(newPub, object);
        }
    }

    /**
     * Publishes an object with an existing StructPublisher.
     * @param publisher The publisher to use.
     * @param object The object to publish.
     */
    @SuppressWarnings({"rawtypes", "unchecked"})
    private static void publishToExisting(StructPublisher<?> publisher, Object object) {
        StructPublisher typed = publisher;
        typed.set(object);
    }

    /**
     * Returns a StructPublisher for an object under the name passed in. If the type of object isn't recognised,
     * you can add a segment to the if statement chain for that type as long as its class contains a static struct variable
     * representing its type as a Struct.
     * @param name A name to create the publisher under.
     * @param object The object to create a publisher for.
     * @return The StructPublisher created for the object passed in or null if the object type isn't recognised.
     */
    @SuppressWarnings({"unused"})
    private static StructPublisher<?> createPublisherForObject(String name, Object object, NetworkTable table) {
        if (object instanceof Pose2d pose) { // Is the object of type Pose2d?
            return table.getStructTopic(name, Pose2d.struct).publish(); // Return the StructPublisher for said type.
        } 
        else if (object instanceof Rotation2d rotation2d) {
            return table.getStructTopic(name, Rotation2d.struct).publish();
        } 
        else if (object instanceof Translation2d translation2d) {
            return table.getStructTopic(name, Translation2d.struct).publish();
        }
        else if (object instanceof Pose2d pose2d) {
            return table.getStructTopic(name, Pose2d.struct).publish();
        }
        else if (object instanceof Pose3d pose3d) {
            return table.getStructTopic(name, Pose3d.struct).publish();
        } 
        else if (object instanceof Transform2d transform2d) {
            return table.getStructTopic(name, Transform2d.struct).publish();
        } 
        else if (object instanceof SwerveModuleState state) {
            return table.getStructTopic(name, SwerveModuleState.struct).publish();
        }
        // Add other types here using the example code below.

        // else if (object instanceof StructableObjectType structableObjectType) {
        //     return table.getStructTopic(name, StructableObjectType.struct).publish();
        // }

        // Requires the object to be expressable as a Struct.
        // Most WPILib and Choreo object types contain a static variable
        // of the class as a struct. (ex. SwerveSample.struct, Transform3d.struct)

        return null;  // unsupported type
    }

    public static void setEnabled(Boolean enable) {
        enabled = enable;
    }

    public static Boolean getEnabled() {
        return enabled;
    }
}