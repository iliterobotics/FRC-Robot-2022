package us.ilite.common;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

public class CSVLoggerQueue {
    public static Queue<Object> kCSVLoggingQueue = new ConcurrentLinkedQueue<>();
}
