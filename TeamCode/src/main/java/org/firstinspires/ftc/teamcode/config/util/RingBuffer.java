package org.firstinspires.ftc.teamcode.config.util;

import java.util.ArrayList;
import java.util.List;

/** Fixed-size circular buffer. add() returns the value it overwrote (the oldest one). */
public class RingBuffer<T> {
    private final List<T> list = new ArrayList<>();
    private int index = 0;

    public RingBuffer(int length, T startingValue) {
        for (int i = 0; i < length; i++)
            list.add(startingValue);
    }

    public T add(T value) {
        T oldest = list.get(index);
        list.set(index, value);
        index = (index + 1) % list.size();
        return oldest;
    }

    public void fill(T value) {
        for (int i = 0; i < list.size(); i++)
            list.set(i, value);
        index = 0;
    }
}
