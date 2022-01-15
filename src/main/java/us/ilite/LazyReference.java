package us.ilite;

import java.util.function.Supplier;

import static java.util.Objects.requireNonNull;

/**
 * This class came from: https://dzone.com/articles/be-lazy-with-java-8
 * @param <T> The underlying type for the reference
 */
public final class LazyReference<T> {
    private final Supplier<T> mSupplier;
    volatile T value;

    public LazyReference(Supplier<T>supplier){
        requireNonNull(supplier);
        mSupplier = supplier;

    }
    /**
     * Method to get the instance or create it first if it hasn't been created.
     * This method is "thread safe" in that the construction is wrapped in a synchronized
     * block.
     * @return
     *  An instance of the underlaying value. If this is the first time this method is called,
     *  the supplier will be called first to construct the reference. 
     */
    public T getOrCompute() {
        final T result = value; // Just one volatile read
        return result == null ? maybeCompute() : result;
    }
    private synchronized T maybeCompute() {
        if (value == null) {
            value = requireNonNull(mSupplier.get());
        }
        return value;
    }
}
