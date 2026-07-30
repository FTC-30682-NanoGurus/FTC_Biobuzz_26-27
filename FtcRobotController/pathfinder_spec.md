# Project Specification: "HiveMind" Dynamic Obstacle Avoidance for Road Runner

## 1. System Architecture Overview
The goal of this library is to intercept an active Road Runner trajectory when a sensor detects an obstacle, calculate a valid detour path around that obstacle using a discrete 2D grid, and instantly inject a new, dynamically generated spline path back into Road Runner.

## 2. Component Specifications

### Component A: `FieldGrid.java`
A thread-safe mathematical model representing the 144" x 144" FTC field workspace.
* **Internal Representation:** A discrete 2D matrix (`boolean[][]` or `byte[][]`). Grid cell resolution must be configurable via constructor (default: 2-inch or 4-inch square cells).
* **Coordinate Mapping:** Must provide bidirectional transformations:
    * `GridPoint fieldToGrid(Pose2d/Vector2d pose)` (Converts continuous field inches to matrix indices `(row, col)` where center `(0,0)` maps to the center of the grid matrix).
    * `Vector2d gridToField(GridPoint point)` (Converts matrix indices back to continuous field inches).
* **Obstacle Mutation:** Thread-safe methods to clear, set, or expand (inflate) obstacles. Inflation is required to account for the robot's physical radius so the pathfinder treats the robot as a single point.
    * `public synchronized void setObstacle(double x, double y, double radiusInches)`

### Component B: `AStarPathfinder.java`
An asynchronous implementation of the $A^*$ search algorithm optimized for a 2D grid matrix.
* **Heuristic:** Must use Diagonal Distance (Chebyshev or Octile distance) to support 8-way movement (orthogonal + diagonal steps).
* **Performance Constraints:**
    * If a path takes longer than 20 milliseconds to compute, it must abort and return a fallback path to prevent robot stuttering.
    * Must run on a separate worker thread (`CompletableFuture` or standard Java `Thread`).
* **Path Smoothing:** The raw output of $A^*$ is a jagged line of grid cells. This component must feature a path-smoothing pass (e.g., Line-of-Sight smoothing/String pulling) to reduce the node array down to key turning waypoints.

### Component C: `DynamicPathEngine.java`
The interface wrapper that plugs into the Road Runner drivetrain class (`MecanumDrive` or `TankDrive`).
* **State Machine:** Tracks if the robot is currently following a static path, evaluating a detour, or recovering from a blocked state.
* **Trajectory Interception:**
    * Monitors sensor inputs (e.g., reading a shared `ObstacleTracker` state).
    * If a running path encounters a dynamically updated blocked node in `FieldGrid`, it immediately calls `drive.breakFollowing()`.
    * Feeds the smoothed output coordinates from `AStarPathfinder` straight into a loop that constructs a chain of `.splineTo()` or `.lineTo()` trajectories on the fly.
    * Calls `drive.followTrajectoryAsync()` to restore fluid motion.

## 3. Class Skeleton & API Requirements

The library must strictly expose or follow these interfaces to allow integration with existing team code:

```java
package team.hivemind.pathfinder;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import java.util.List;

public interface IHiveMindEngine {
    /** Updates the map with a dynamic obstacle found mid-match */
    void reportDynamicObstacle(Vector2d position, double radius);
    
    /** Clears temporary obstacles (e.g., passing robots) */
    void clearDynamicObstacles();
    
    /** 
     * Computes a detour path from current position to goal.
     * Returns a list of Field coordinates (Inches) to pass to Road Runner.
     */
    List<Vector2d> calculateDetour(Vector2d currentPos, Vector2d goalPos);
}
```

## 4. Expected Deliverables (Milestones)
Your implementation workflow should progress across these milestones:
1. **Milestone 1:** `FieldGrid` data structure with coordinate translation functions and unit tests verifying coordinate accuracy across all four quadrants.
2. **Milestone 2:** Functional `AStarPathfinder` capable of navigating around a hardcoded central barrier block on a virtual field.
3. **Milestone 3:** Path-smoothing routine that converts dense grid nodes into sparse geometric waypoints.
4. **Milestone 4:** Multi-threaded wrapper handling asynchronous computation requests safely without deadlocks.