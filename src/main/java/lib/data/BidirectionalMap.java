// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.data;

import java.util.HashMap;
import java.util.function.BiConsumer;

/** Bidirection map made using HashMaps. Allows the user to get by either key or value.
 *  Made with help from Gemini, but typed out by a user.
 * 
 * @param key The type of the keys
 * @param value The type of the values
 */
public class BidirectionalMap<K, V> {
    private final HashMap<K, V> forwardMap = new HashMap<K, V>();
    private final HashMap<V, K> inverseMap = new HashMap<V, K>();

    /** Associates the specified value with the specified key. */
    public void put(K key, V value) {
        if (key == null || value == null) {
            throw new IllegalArgumentException("BidirectionalMap does not support null values");
        }

        V oldValue = forwardMap.remove(key);
        if (oldValue != null) {
            inverseMap.remove(oldValue);
        }
        
        forwardMap.put(key, value);
        inverseMap.put(value, key);
    }

    /** Returns the value with which the key is mapped, or null if none. */
    public V get(K key) {
        return forwardMap.get(key);
    }

    /** Returns the key with which the value is mapped, or null if none. */
    public K getKey(V value) {
        return inverseMap.get(value);
    }

    /** Returns if the specified key has a mapping. */
    public boolean containsKey(K key) {
        return forwardMap.containsKey(key);
    }

    /** Returns if the specified value has a mapping. */
    public boolean containsValue(V value) {
        return inverseMap.containsKey(value);
    }

    /** Performs the passed in action for each key-value pair. */
    public void forEach(BiConsumer<? super K, ? super V> action) {
        forwardMap.forEach(action);
    }

    /** Removes the mapping for the specified key. */
    public void removeByForwardKey(K key) {
        boolean remove = forwardMap.containsKey(key);
        
        if (remove) {
            inverseMap.remove(forwardMap.remove(key));
        }
    }

    /** Removes mapping for the specified value. */
    public void removeByInverseKey(V value) {
        boolean remove = inverseMap.containsKey(value);
        
        if (remove) {
            forwardMap.remove(inverseMap.remove(value));
        }
    }
}
