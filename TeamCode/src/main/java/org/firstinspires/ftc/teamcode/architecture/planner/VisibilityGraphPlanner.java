package org.firstinspires.ftc.teamcode.architecture.planner;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.PriorityQueue;

/**
 * Shortest obstacle-avoiding polyline between two points inside a keep-in region: a visibility graph over points
 * sampled around the corners of each obstacle's clearance-inflated outline, searched with Dijkstra. Close to exact
 * bitangents with enough samples, and well under a millisecond for a handful of obstacles. The region is convex, so
 * a hop between two points inside it stays inside.
 */
@Config
public final class VisibilityGraphPlanner {
    public static int pointsPerCorner = 4;
    // Neighbouring samples are joined by chords, which cut inside the corner's arc the samples sit on.
    public static double boundaryMarginIn = 1.5;

    private VisibilityGraphPlanner() {}

    /**
     * The positions from {@code start} to {@code goal}, both included, with heading 0. Null if either end is
     * outside {@code keepIn} or no route inside it exists.
     */
    public static List<Pose> planPath(Pose start, Pose goal, Region keepIn, List<Obstacle> obstacles, double clearance) {
        if (!keepIn.contains(start) || !keepIn.contains(goal)) return null;
        if (!Obstacle.anyBlocks(obstacles, start.x(), start.y(), goal.x(), goal.y(), clearance)) {
            List<Pose> direct = new ArrayList<>();
            direct.add(new Pose(start.x(), start.y(), 0));
            direct.add(new Pose(goal.x(), goal.y(), 0));
            return direct;
        }
        final int startNode = 0;
        final int goalNode = 1;
        List<double[]> nodes = new ArrayList<>();
        nodes.add(new double[]{start.x(), start.y()});
        nodes.add(new double[]{goal.x(), goal.y()});

        double placementRadius = clearance + boundaryMarginIn;
        int perCorner = Math.max(2, pointsPerCorner);
        for (Obstacle o : obstacles) {
            Region b = o.bounds;
            double[][] corners = {{b.maxX, b.maxY}, {b.minX, b.maxY}, {b.minX, b.minY}, {b.maxX, b.minY}};
            for (int corner = 0; corner < corners.length; corner++) {
                for (int i = 0; i < perCorner; i++) {
                    double angle = Math.PI / 2 * (corner + (double) i / (perCorner - 1));
                    // Clamped onto the region's edge rather than dropped, so a route can squeeze past an obstacle near a wall.
                    double px = Math.max(keepIn.minX, Math.min(keepIn.maxX, corners[corner][0] + placementRadius * Math.cos(angle)));
                    double py = Math.max(keepIn.minY, Math.min(keepIn.maxY, corners[corner][1] + placementRadius * Math.sin(angle)));
                    if (insideAny(px, py, obstacles, clearance)) continue;
                    nodes.add(new double[]{px, py});
                }
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
                double ex = to[0] - from[0];
                double ey = to[1] - from[1];
                double alt = dist[u] + Math.sqrt(ex * ex + ey * ey);
                if (alt < dist[v]) {
                    dist[v] = alt;
                    prev[v] = u;
                    queue.add(v);
                }
            }
        }

        if (dist[goalNode] == Double.MAX_VALUE) return null;

        List<Pose> waypoints = new ArrayList<>();
        for (int cur = goalNode; cur != -1; cur = prev[cur]) {
            double[] p = nodes.get(cur);
            waypoints.add(0, new Pose(p[0], p[1], 0));
        }
        return waypoints;
    }

    private static boolean insideAny(double px, double py, List<Obstacle> obstacles, double clearance) {
        for (Obstacle o : obstacles) {
            if (o.blocks(px, py, clearance)) return true;
        }
        return false;
    }
}
