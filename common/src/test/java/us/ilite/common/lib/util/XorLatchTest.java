package us.ilite.common.lib.util;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;
import org.junit.experimental.categories.Category;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;
import us.ilite.CriticalTest;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

@RunWith(Parameterized.class)
@Category(CriticalTest.class)
public class XorLatchTest {

    private final boolean initEntryValue;
    private final boolean initExitValue;
    private final boolean expectedEntryValue;
    private final boolean expectedExitValue;
    private final boolean input;
    private final int index;
    private XorLatch aLatch;

    @Parameterized.Parameters
    public static Collection<Object[]> data() {
        List<Object[]>testData = new ArrayList<>();
        int index  = 1;
        testData.add(new Object[]{index++, false, false, false, false,false});
        testData.add(new Object[]{index++, false, false, true, true,false});
        testData.add(new Object[]{index++, false, true, false, false,false});
        testData.add(new Object[]{index++, false, true, true, true,false});
        testData.add(new Object[]{index++, true, false, false, true,true});
        testData.add(new Object[]{index++, true, false, true, true,false});
        testData.add(new Object[]{index++, true, true, false, true,true});
        testData.add(new Object[]{index++, true, true, true, true,false});

        return testData;
    }

    public XorLatchTest(int index, boolean initEntryValue, boolean initExitValue, boolean input, boolean expectedEntryValue, boolean expectedExitValue) {
        this.initEntryValue = initEntryValue;
        this.initExitValue = initExitValue;
        this.input = input;
        this.index = index;
        this.expectedEntryValue = expectedEntryValue;
        this.expectedExitValue = expectedExitValue;
    }

    @Before
    public void init() {
        aLatch = new XorLatch();
        aLatch.reset();
    }

    @Test
    public void testThis() {

        aLatch.mEntry.update(initEntryValue);
        aLatch.mExit.update(initExitValue);
        aLatch.update(input);

        assertEquals(expectedEntryValue, aLatch.mEntry.get());
        assertEquals(expectedExitValue, aLatch.mExit.get());

    }
}
