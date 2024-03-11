package frc.robot.utilities;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.ShooterConstants;

public class JSONManager {
    // Singleton Design Pattern
    private static JSONManager instance;
    public static JSONManager getInstance() {
        if (instance == null) {
            instance = new JSONManager();
        }
        return instance;
    }

    private final File DIRECTORY = Filesystem.getDeployDirectory();
    private final File SHOOTER_DATAFILE;
    private final File INTAKE_DATAFILE;
    private final String LEFT_PIVOT_INDEX = "LEFT_PIVOT_MOTOR_POSITION";
    private final String RIGHT_PIVOT_INDEX = "RIGHT_PIVOT_MOTOR_POSITION";
    private final String INTAKE_ENCODER_INDEX = "INTAKE_MOTOR_POSITION";

    /** Creates a new JSON Manager object */
    public JSONManager() {
        SHOOTER_DATAFILE = new File(DIRECTORY, "shooter.json");
        INTAKE_DATAFILE = new File(DIRECTORY, "intake.json");

        // Creates file in case it does not exist
        try {
            if (SHOOTER_DATAFILE.createNewFile()) {
                saveShooterPivotPositions(Double.valueOf(ShooterConstants.PIVOT_ANGLE_LIMITS[1]));
            }
            if (INTAKE_DATAFILE.createNewFile()) {
                saveIntakePivotPosition(Double.valueOf(0));
            }
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Writes the shooter pivot's positions to the JSON file
     * 
     * @param leftMotorPosition
     * @param rightMotorPosition
     */ 
    public void saveShooterPivotPositions(double leftMotorPosition, double rightMotorPosition) {
        JSONObject json = getJSONObject(SHOOTER_DATAFILE);

        if (json.replace(LEFT_PIVOT_INDEX, leftMotorPosition) == null) {
            json.put(LEFT_PIVOT_INDEX, leftMotorPosition);
        }
        if (json.replace(RIGHT_PIVOT_INDEX, rightMotorPosition) == null) {
            json.put(RIGHT_PIVOT_INDEX, rightMotorPosition);
        }

        setJSONObject(SHOOTER_DATAFILE, json);
    }

    /**
     * Writes the shooter pivot's position to the JSON file (overloaded)
     * 
     * @param motorPosition
     */ 
    public void saveShooterPivotPositions(double motorPosition) {
        saveShooterPivotPositions(motorPosition, motorPosition);
    }

    /**
     * Writes the intake pivot's position to the JSON file
     * 
     * @param encoderPosition
     */ 
    public void saveIntakePivotPosition(double encoderPosition) {
        JSONObject json = getJSONObject(INTAKE_DATAFILE);
        // json.remove(LEFT_PIVOT_INDEX, RIGHT_PIVOT_INDEX);
        if (json.replace(INTAKE_ENCODER_INDEX, encoderPosition) == null) {
            json.put(INTAKE_ENCODER_INDEX, encoderPosition);
        }
        setJSONObject(INTAKE_DATAFILE, json);
    }

    /**
     * Get the pivot positions for the shooter from the JSON file
     * @return an array with the left [0] and right [1] positions
     */
    public double[] getShooterPivotPositions() {
        double[] positions = new double[]{ShooterConstants.PIVOT_ANGLE_LIMITS[1], ShooterConstants.PIVOT_ANGLE_LIMITS[1]};
        try {
            JSONObject json = getJSONObject(SHOOTER_DATAFILE);
            positions[0] = (Double) json.get(LEFT_PIVOT_INDEX);
            positions[1] = (Double) json.get(RIGHT_PIVOT_INDEX);    
        }
        /* ClassCastException only happens when casting from Long to Double during json.get()
        because JSONObject turns 0 (default value when creating file) into a Long.
        {@code positions} has default values of 0, so it will return [0,0] anyways */ 
        catch (ClassCastException | NullPointerException e) {
            e.printStackTrace();
        }
        return positions;
    }

    /**
     * Get the intake pivot position from the JSON file
     * @return a double with the intake position
     */
    public double getIntakePivotPosition() {
        double position = 0;
        try {
            JSONObject json = getJSONObject(INTAKE_DATAFILE);
            position = (Double) json.get(INTAKE_ENCODER_INDEX);    
        }
        /* ClassCastException only happens when casting from Long to Double during json.get()
        because JSONObject turns 0 (default value when creating file) into a Long.
        {@code positions} has default values of 0, so it will return [0,0] anyways */ 
        catch (ClassCastException | NullPointerException e) {
            e.printStackTrace();
        }
        return position;
    }

    /**
     * A method used to read the data from the JSON file storing positions
     * 
     * @param file to read from
     * @return the read JSON data
     */
    private JSONObject getJSONObject(File file) {
        try {
            FileReader fileReader = new FileReader(file);
            JSONObject json = (JSONObject) new JSONParser().parse(fileReader);
            fileReader.close();
            return json;
        }
        catch (IOException | ParseException e) {
            e.printStackTrace();
        }
        // File is empty
        return new JSONObject();
    }

    /**
     * A method used to write to the file containing the JSON data
     * 
     * @param file to write to
     * @param json the JSON data to write
     */
    private void setJSONObject(File file, JSONObject json) {
        try {
            FileWriter fileWriter = new FileWriter(file);
            fileWriter.write(json.toJSONString());
            fileWriter.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }
}
