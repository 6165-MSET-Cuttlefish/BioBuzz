package android.os;

public class Process {
    public static int myTid() { return (int) Thread.currentThread().getId(); }
    public static int myPid() { return 1; }
    public static void setThreadPriority(int priority) {}
    public static void setThreadPriority(int tid, int priority) {}
    public static int getThreadPriority(int tid) { return 0; }
}
