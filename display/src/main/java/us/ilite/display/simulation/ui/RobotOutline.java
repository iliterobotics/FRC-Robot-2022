package us.ilite.display.simulation.ui;


import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;

public class RobotOutline extends ADrawable {

    private Translation2d[] outlinePoints;
    private DrawablePath leftPath = new DrawablePath(Color.GREEN);
    private DrawablePath rightPath = new DrawablePath(Color.PURPLE);

    public RobotOutline(Translation2d...outlinePoints) {
        this.outlinePoints = outlinePoints;
    }

    @Override
    public void draw(GraphicsContext gc, Pose2d pose, Translation2d aspectRatio) {
        Translation2d[] pointsToDraw = new Translation2d[outlinePoints.length];
        for(int pointIndex = 0; pointIndex < outlinePoints.length; pointIndex++) {
            pointsToDraw[pointIndex] = outlinePoints[pointIndex].rotateBy(pose.getRotation());
            pointsToDraw[pointIndex] = pointsToDraw[pointIndex].plus(pose.getTranslation());
            pointsToDraw[pointIndex] = new Translation2d(pointsToDraw[pointIndex].getX() * aspectRatio.getX(), pointsToDraw[pointIndex].getY() * aspectRatio.getY());
        }

        gc.setStroke(Color.BLACK);
        gc.beginPath();
        gc.moveTo(pointsToDraw[0].getX(), pointsToDraw[0].getY());
        for(int pointIndex = 1; pointIndex <= pointsToDraw.length; pointIndex++) {
            gc.lineTo(pointsToDraw[pointIndex % pointsToDraw.length].getX(), pointsToDraw[pointIndex % pointsToDraw.length].getY());
        }
        gc.stroke();
        gc.closePath();

        Translation2d leftSide = outlinePoints[0].plus(new Translation2d(33.91 / 2.0, 0)).rotateBy(pose.getRotation()).plus(pose.getTranslation());
        Translation2d rightSide = outlinePoints[1].plus(new Translation2d(33.91 / 2.0, 0)).rotateBy(pose.getRotation()).plus(pose.getTranslation());
        leftSide = new Translation2d(leftSide.getX() * aspectRatio.getX(), leftSide.getY() * aspectRatio.getY());
        rightSide = new Translation2d(rightSide.getX() * aspectRatio.getX(), rightSide.getY() * aspectRatio.getY());
        leftPath.draw(gc, new Pose2d(leftSide, Rotation2d.fromDegrees(0.0)));
        rightPath.draw(gc, new Pose2d(rightSide, Rotation2d.fromDegrees(0.0)));
    }

    public void clear() {
        leftPath.clear();
        rightPath.clear();
    }

}
