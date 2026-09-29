package org.firstinspires.ftc.teamcode.architecture.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.PriorityQueue;

/**
 * Shortest obstacle-avoiding polyline between two points inside a keep-in region: a visibility graph over points
 * sampled around each obstacle's clearance-inflated circle, searched with Dijkstra. Close to exact bitangents with
 * enough samples, and well under a millisecond for a handful of obstacles. The region is convex, so a hop between
 * two points inside it stays inside.
 */
@Config
public final class VisibilityGraphPlanner {
    public static int pointsPerObstacle = 16;
    // Neighbouring samples are joined by chords, which cut inside the circle the samples sit on.
    public static double boundaryMarginIn = 1.5;

    private VisibilityGraphPlanner() {}

    public static final class Leg {
        public final List<Pose> waypoints;
        public final double length;

        Leg(List<Pose> waypoints, double length) {
            this.waypoints = waypoints;
            this.length = length;
        }
    }

    /**
     * The waypoints start at {@code start}, keeping its heading, and end at {@code goal}'s position; every
     * later waypoint faces along the hop arriving at it. Null if either end is outside {@code keepIn} or no
     * route inside it exists.
     */
    public static Leg planPath(Pose start, Pose goal, Region keepIn, List<Obstacle> obstacles, double clearance) {
        if (!keepIn.contains(start) || !keepIn.contains(goal)) return null;
        final int startNode = 0;
        final int goalNode = 1;
        List<double[]> nodes = new ArrayList<>();
        nodes.add(new double[]{start.x(), start.y()});
        nodes.add(new double[]{goal.x(), goal.y()});

        for (Obstacle o : obstacles) {
            double placementRadius = o.radius + clearance + boundaryMarginIn;
            for (int i = 0; i < pointsPerObstacle; i++) {
                double angle = 2 * Math.PI * i / pointsPerObstacle;
                // Clamped onto the region's edge rather than dropped, so a route can squeeze past an obstacle near a wall.
                double px = Math.max(keepIn.minX, Math.min(keepIn.maxX, o.x + placementRadius * Math.cos(angle)));
                double py = Math.max(keepIn.minY, Math.min(keepIn.maxY, o.y + placementRadius * Math.sin(angle)));
                if (insideAny(px, py, obstacles, clearance)) continue;
                nodes.add(new double[]{px, py});
            }
        }

        int n = nodes.size();
        double[] dist = new double[n];
        int[] prev = new int[n];
        boolean[] visited = new boolean[n];
        Arrays.fill(dist, Double.MAX_VALUE);
        Arrays.fill(prev, -1);
        dist[startNode] = 0;

        PriorityQueue<Integer> queue = new PriorityQueue<>((a, b) -> Double.compare(dist[a], dist[b]));
        queue.add(startNode);
        while (!queue.isEmpty()) {
            int u = queue.poll();
            if (visited[u]) continue;
            visited[u] = true;
            if (u == goalNode) break;

            double[] from = nodes.get(u);
            for (int v = 0; v < n; v++) {
                if (v == u || visited[v]) continue;
                double[] to = nodes.get(v);
                if (Obstacle.anyBlocks(obstacles, from[0], from[1], to[0], to[1], clearance)) continue;
                double alt = dist[u] + Math.hypot(to[0] - from[0], to[1] - from[1]);
                if (alt < dist[v]) {
                    dist[v] = alt;
                    prev[v] = u;
                    queue.add(v);
                }
            }
        }

        if (dist[goalNode] == Double.MAX_VALUE) return null;

        List<Integer> indexPath = new ArrayList<>();
        for (int cur = goalNode; cur != -1; cur = prev[cur]) indexPath.add(0, cur);

        List<Pose> waypoints = new ArrayList<>();
        for (int i = 0; i < indexPath.size(); i++) {
            double[] p = nodes.get(indexPath.get(i));
            double heading = start.heading();
            if (i > 0) {
                double[] before = nodes.get(indexPath.get(i - 1));
                heading = Math.atan2(p[1] - before[1], p[0] - before[0]);
            }
            waypoints.add(new Pose(p[0], p[1], heading));
        }
        return new Leg(waypoints, dist[goalNode]);
    }

    private static boolean insideAny(double px, double py, List<Obstacle> obstacles, double clearance) {
        for (Obstacle o : obstacles) {
            if (Math.hypot(px - o.x, py - o.y) <= o.radius + clearance) return true;
        }
        return false;
    }
}
