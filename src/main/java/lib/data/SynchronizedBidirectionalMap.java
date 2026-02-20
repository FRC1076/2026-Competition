// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.data;

import java.util.HashMap;

/** Bidirection map made using HashMaps. Allows the user to get by either key or value.
 *  Made with help from Gemini, but typed out by a user.
 * 
 * @param key The type of the keys
 * @param value The type of the values
 */
public class SynchronizedBidirectionalMap<K, V> {
    private final HashMap<K, V> forwardMap = new HashMap<K, V>();
    private final HashMap<V, K> inverseMap = new HashMap<V, K>();

    /** Associates the specified value with the specified key. */
    public synchronized void put(K key, V value) {
        if (forwardMap.containsKey(key)) {
            inverseMap.remove(forwardMap.remove(key));
        }
        if (inverseMap.containsKey(value)) {
            forwardMap.remove(inverseMap.remove(value));
        }
        
        forwardMap.put(key, value);
        inverseMap.put(value, key);
    }

    /** Returns the value with which the key is mapped, or null if none. */
    public synchronized V getByForwardKey(K key) {
        return forwardMap.get(key);
    }

    /** Returns the key with which the value is mapped, or null if none. */
    public synchronized K getByInverseKey(V value) {
        return inverseMap.get(value);
    }

    /** Returns if the specified key has a mapping. */
    public synchronized boolean containsKey(K key) {
        return forwardMap.containsKey(key);
    }

    /** Returns if the specified value has a mapping. */
    public synchronized boolean containsValue(V value) {
        return inverseMap.containsKey(value);
    }

    /** Removes the mapping for the specified key. */
    public synchronized void removeByForwardKey(K key) {
        boolean remove = forwardMap.containsKey(key);
        
        if (remove) {
            inverseMap.remove(forwardMap.remove(key));
        }
    }

    /** Removes mapping for the specified value. */
    public synchronized void removeByInverseKey(V value) {
        boolean remove = inverseMap.containsKey(value);
        
        if (remove) {
            forwardMap.remove(inverseMap.remove(value));
        }
    }
}
