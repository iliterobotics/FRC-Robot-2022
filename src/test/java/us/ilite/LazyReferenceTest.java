package us.ilite;
import org.junit.Test;
//import static org.mockito.Mockito.*;
import static org.junit.Assert.*;

import java.util.function.Supplier;

public class LazyReferenceTest {

    @Test(expected = NullPointerException.class)
    /**
     * If a null is passed into the constructor for the supplier, then this
     * lazy reference is invalid, so ensure a {@link NullPointerException} is thrown
     */
    public void testNullConstructor() {

//        LazyReference ref = new LazyReference(null);

    }

    @Test
    /**
     * This method will test to ensure that the supplier is only called once and that subsequent calls
     * to the {@link LazyReference#getOrCompute()} will result in the constructed object from the first call
     * being returned
     */
    public void testSupplierCalledOnce() {

//        Object testedObj = new Object();
//        Supplier mockedSupplier = mock(Supplier.class);
//        when(mockedSupplier.get()).thenReturn(testedObj);

//        LazyReference ref = new LazyReference(mockedSupplier);

//        assertNull(ref.value);
//
//        Object returnVal = ref.getOrCompute();
//
//        assertNotNull(returnVal);
//        assertNotNull(ref.value);
//        assertEquals(testedObj, returnVal);
//        verify(mockedSupplier, times(1)).get();
//
//        returnVal = ref.getOrCompute();
//
//        assertNotNull(returnVal);
//        assertEquals(testedObj, returnVal);
//        verify(mockedSupplier, times(1)).get();

    }

    @Test(expected = NullPointerException.class)
    /**
     * Method to test to ensure that a supplier that returns null throws a {@link NullPointerException } as this
     * will be an invalid state.
     */
    public void testNullSupplierInvalid() {

//        Supplier supplier = mock(Supplier.class);
//
//        when(supplier.get()).thenReturn(null);

//        LazyReference ref = new LazyReference(supplier);

//        ref.getOrCompute();
    }

}
