package android.util;

import java.util.HashMap;

public class LongSparseArray<E> {
    private final HashMap<Long, E> map = new HashMap<>();

    public E get(long key) { return map.get(key); }
    public E get(long key, E fallback) { E v = map.get(key); return v == null ? fallback : v; }
    public void put(long key, E value) { map.put(key, value); }
    public void remove(long key) { map.remove(key); }
    public void delete(long key) { map.remove(key); }
    public int size() { return map.size(); }
    public void clear() { map.clear(); }
}
