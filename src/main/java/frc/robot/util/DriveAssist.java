package frc.robot.util;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.io.File;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

/** Add your docs here. */
public class DriveAssist {
    private FieldBounds fieldBounds;
    private RobotOutline robotOutline;
    private Supplier<Pose2d> poseSupplier;
    private double[] inputConversionFactor;

    public DriveAssist(RobotOutline robotOutline, FieldBounds fieldBounds, Supplier<Pose2d> poseSupplier, double[] inputConversionFactor) {
        this.fieldBounds = fieldBounds;
        this.robotOutline = robotOutline;
        this.poseSupplier = poseSupplier;
        this.inputConversionFactor = inputConversionFactor;
    }

    public Transform2d adjustInput(Transform2d input, boolean fieldRelative) {
        Pose2d pose = poseSupplier.get();

        double dx = input.getX() * inputConversionFactor[0];
        double dy = input.getY() * inputConversionFactor[0];
        double dtheta = input.getRotation().getRadians() * inputConversionFactor[1];

        // Convert to field-relative if needed
        if (!fieldRelative) {
            double heading = pose.getRotation().getRadians();
            double fdx = dx * Math.cos(heading) - dy * Math.sin(heading);
            double fdy = dx * Math.sin(heading) + dy * Math.cos(heading);
            dx = fdx;
            dy = fdy;
        }

        BoundingBox collision = fieldBounds.robotIntersects(robotOutline, pose);
        if (collision == null) return input;

        // Step 1: clamp XY, keep rotation
        Vector<N2> normal = getNearestInwardNormal();
        double dot = dx * normal.get(0, 0) + dy * normal.get(1, 0);
        if (dot < 0) {
            dx -= normal.get(0, 0) * dot;
            dy -= normal.get(1, 0) * dot;
        }

        // Check if XY fix was enough
        Pose2d nextPose = new Pose2d(
            pose.getX() + dx * 0.02,
            pose.getY() + dy * 0.02,
            pose.getRotation().plus(new Rotation2d(dtheta * 0.02))
        );
        if (fieldBounds.robotIntersects(robotOutline, nextPose) == null) {
            return toOutput(dx, dy, dtheta, pose, fieldRelative);
        }

        // Step 2: also clamp rotation
        dtheta = 0;
        nextPose = new Pose2d(
            pose.getX() + dx * 0.02,
            pose.getY() + dy * 0.02,
            pose.getRotation()
        );
        if (fieldBounds.robotIntersects(robotOutline, nextPose) == null) {
            return toOutput(dx, dy, dtheta, pose, fieldRelative);
        }

        // Step 3: completely stuck
        return new Transform2d(0, 0, new Rotation2d(0));
    }

    private Transform2d toOutput(double fdx, double fdy, double dtheta, Pose2d pose, boolean fieldRelative) {
        if (!fieldRelative) {
            double heading = pose.getRotation().getRadians();
            // rotate back to robot-relative
            double rdx = fdx * Math.cos(-heading) - fdy * Math.sin(-heading);
            double rdy = fdx * Math.sin(-heading) + fdy * Math.cos(-heading);
            return new Transform2d(rdx, rdy, new Rotation2d(dtheta));
        }
        return new Transform2d(
            fdx / inputConversionFactor[0], 
            fdy / inputConversionFactor[0], 
            new Rotation2d(dtheta / inputConversionFactor[1]));
    }

    private Vector<N2> getNearestInwardNormal() {
        Pose2d pose = poseSupplier.get();
        BoundingBox collidingBox = fieldBounds.robotIntersects(robotOutline, pose);

        if (collidingBox == null) return null;

        double robotX = pose.getX();
        double robotY = pose.getY();

        double[] coords = new double[6];
        double closestDist = Double.MAX_VALUE;
        double normalX = 0, normalY = 0;

        double prevX = 0, prevY = 0, firstX = 0, firstY = 0;

        boolean robotCenterInside = collidingBox.getArea().contains(robotX, robotY);

        Area robotCopy = new Area(robotOutline.getArea());
        robotCopy.subtract(collidingBox.getArea());
        boolean anyPartOutside = !robotCopy.isEmpty();

        Area intersection = new Area(robotOutline.getArea());
        intersection.intersect(collidingBox.getArea());
        boolean anyPartInside = !intersection.isEmpty();

        boolean wrongSide = collidingBox.getKeepIn()
            ? anyPartOutside && !robotCenterInside
            : anyPartInside && robotCenterInside;

        PathIterator iterator = collidingBox.getArea().getPathIterator(null);
        while (!iterator.isDone()) {
            int type = iterator.currentSegment(coords);

            if (type == PathIterator.SEG_MOVETO) {
                prevX = coords[0];
                prevY = coords[1];
                firstX = coords[0];
                firstY = coords[1];
            } else {
                double segEndX, segEndY;

                if (type == PathIterator.SEG_CLOSE) {
                    segEndX = firstX;
                    segEndY = firstY;
                } else { // SEG_LINETO
                    segEndX = coords[0];
                    segEndY = coords[1];
                }

                // Find closest point on this segment to robot center
                double edgeDX = segEndX - prevX;
                double edgeDY = segEndY - prevY;
                double lenSq = edgeDX * edgeDX + edgeDY * edgeDY;
                double t = ((robotX - prevX) * edgeDX + (robotY - prevY) * edgeDY) / lenSq;
                t = MathUtil.clamp(t, 0, 1);

                double closestX = prevX + t * edgeDX;
                double closestY = prevY + t * edgeDY;
                double dist = Math.hypot(robotX - closestX, robotY - closestY);

                if (dist < closestDist) {
                    closestDist = dist;

                    // Edge normal (perpendicular to edge)
                    double nx = -edgeDY;
                    double ny = edgeDX;
                    double len = Math.hypot(nx, ny);
                    nx /= len;
                    ny /= len;

                    // Flip if pointing away from robot center
                    double toCenterX = robotX - closestX;
                    double toCenterY = robotY - closestY;
                    if (nx * toCenterX + ny * toCenterY < 0) {
                        nx = -nx;
                        ny = -ny;
                    }

                    boolean pointingTowardRobot = (nx * toCenterX + ny * toCenterY) > 0;
                    if (wrongSide ? pointingTowardRobot : !pointingTowardRobot) {
                        nx = -nx; ny = -ny;
                    }

                    normalX = nx;
                    normalY = ny;
                }

                prevX = segEndX;
                prevY = segEndY;
            }

            iterator.next();
        }

        return new Vector<N2>(
            MatBuilder.fill(Nat.N2(), Nat.N1(), normalX, normalY)
        );
    }

    public static class FieldBounds {
        private ArrayList<BoundingBox> indexedBoundingBoxes;

        public FieldBounds() {
            indexedBoundingBoxes = new ArrayList<>();
        }

        public FieldBounds(String jsonName) {
            indexedBoundingBoxes = new ArrayList<>();

            File file = new File("/home/lvuser/deploy/" + jsonName + ((jsonName.endsWith(".json")) ? "" : ".json"));
            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = null;

            try {
                root = mapper.readTree(file);
            } catch (Exception e) {
                System.err.println("ERROR 404: Couldn't find a FieldBounds JSON file with specified name");
            }

            for (JsonNode boxNode : root.get("boundingBoxes")) {
                boolean keepIn = boxNode.get("keepIn").asBoolean();
                JsonNode cornersNode = boxNode.get("corners");

                Translation2d[] corners = new Translation2d[cornersNode.size()];
                for (int i = 0; i < cornersNode.size(); i++) {
                    corners[i] = new Translation2d(
                        cornersNode.get(i).get("x").asDouble(),
                        cornersNode.get(i).get("y").asDouble()
                    );
                }

                indexedBoundingBoxes.add(new BoundingBox(corners, keepIn));
            }
        }

        public FieldBounds addBoundingBox(BoundingBox box) {
            indexedBoundingBoxes.add(box);
            return this;
        }

        public BoundingBox robotIntersects(RobotOutline robotOutline, Pose2d robotPose) {
            double overlappedArea = 0;
            BoundingBox mostOverlappedBox = null;

            RobotOutline posedRobot = robotOutline.withPose(robotPose);
            for (int i = 0; i < indexedBoundingBoxes.size(); i++) {
                double amount = posedRobot.overlapAmount(indexedBoundingBoxes.get(i));
                if (amount > overlappedArea) {
                    overlappedArea = amount;
                    mostOverlappedBox = indexedBoundingBoxes.get(i);
                }
            }

            return mostOverlappedBox;
        }
    }

    public static class BoundingBox {
        private Area boxArea;
        private boolean keepIn;

        /**
         * Constructs a BoundingBox with an empty area and keepIn set to false.
         */
        public BoundingBox() {
            this.boxArea = new Area();
            this.keepIn = false;
        }

        /**
         * Constructs a BoundingBox with an area created using the passed corners and keepIn set to false.
         * @param corners The corners to construct a shape from. The order matters, 
         * perimeter lines are constructed from point 0 to point 1, then from point 1 to point 2, etc
         */
        public BoundingBox(Translation2d[] corners) {
            this.boxArea = createAreaFromCorners(corners);
            this.keepIn = false;
        }

        /**
         * Constructs a BoundingBox with an area created using the passed corners and keepIn set to the passed boolean.
         * @param corners The corners to construct a shape from. The order matters, 
         * perimeter lines are constructed from point 0 to point 1, then from point 1 to point 2, etc
         * @param keepIn Whether this BoundingBox should keep in or keep out.
         */
        public BoundingBox(Translation2d[] corners, boolean keepIn) {
            this.boxArea = createAreaFromCorners(corners);
            this.keepIn = keepIn;
        }

        public Area getArea() {
            return boxArea;
        }

        public boolean getKeepIn() {
            return keepIn;
        }

        public BoundingBox setArea(Translation2d[] corners) {
            boxArea = createAreaFromCorners(corners);
            return this;
        }

        public BoundingBox setKeepIn(boolean keepIn) {
            this.keepIn = keepIn;
            return this;
        }

        private Area createAreaFromCorners(Translation2d[] corners) {
            // construct the outside perimeter of the area
            Path2D.Double path = new Path2D.Double();

            // move to the first corner to start the perimeter
            path.moveTo(corners[0].getX(), corners[0].getY());

            // create lines between each corner
            for (int i = 1; i < corners.length; i++) {
                path.lineTo(corners[i].getX(), corners[i].getY());
            }

            // close the perimeter
            path.closePath();

            // return the created area using the perimeter
            return new Area(path);
        }
    }

    public static class RobotOutline {
        private Area robotArea;

        public RobotOutline(Translation2d[] corners) {
            robotArea = createAreaFromCorners(corners);
        }

        public RobotOutline(Area robotArea) {
            this.robotArea = robotArea;
        }

        public Area getArea() {
            return robotArea;
        }

        private Area createAreaFromCorners(Translation2d[] corners) {
            // construct the outside perimeter of the area
            Path2D.Double path = new Path2D.Double();

            // move to the first corner to start the perimeter
            path.moveTo(corners[0].getX(), corners[0].getY());

            // create lines between each corner
            for (int i = 1; i < corners.length; i++) {
                path.lineTo(corners[i].getX(), corners[i].getY());
            }

            // close the perimeter
            path.closePath();

            // return the created area using the perimeter
            return new Area(path);
        }

        public double overlapAmount(BoundingBox box2) {
            // create a copy of this BoundingBox's area 
            // the Area.intersect() function sets the Area object to the intersection instead of returning the intersected area
            // so we need to copy the area to keep the original intact
            Area intersectArea = new Area(robotArea);
            Area box2Area = new Area(box2.getArea());

            if (box2.getKeepIn()) {
                intersectArea.subtract(box2Area);
            } else {
                intersectArea.intersect(box2Area);
            }

            if (intersectArea.isEmpty()) {
                return 0;
            }

            double total = 0;
            double[] coords = new double[6];
            double prevX = 0, prevY = 0, firstX = 0, firstY = 0;

            PathIterator it = intersectArea.getPathIterator(null);
            while (!it.isDone()) {
                int type = it.currentSegment(coords);
                switch (type) {
                    case PathIterator.SEG_MOVETO:
                        firstX = prevX = coords[0];
                        firstY = prevY = coords[1];
                        break;
                    case PathIterator.SEG_LINETO:
                        total += (prevX * coords[1]) - (coords[0] * prevY);
                        prevX = coords[0];
                        prevY = coords[1];
                        break;
                    case PathIterator.SEG_CLOSE:
                        total += (prevX * firstY) - (firstX * prevY);
                        break;
                }
                it.next();
            }

            return Math.abs(total) / 2.0;
        }

        public RobotOutline withPose(Pose2d pose) {
            return new RobotOutline(robotArea.createTransformedArea(
                new AffineTransform(
                    pose.getRotation().getCos(), pose.getRotation().getSin(), 
                    -pose.getRotation().getSin(), pose.getRotation().getCos(),
                    pose.getX(), pose.getY()
                )
            ));
        }
    }
}
