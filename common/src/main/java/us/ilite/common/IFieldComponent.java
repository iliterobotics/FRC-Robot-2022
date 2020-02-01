package us.ilite.common;

public interface IFieldComponent {

    public int id();
    public double height();
    public boolean led();
    public int pipeline();

    public default double powerScalar() {
        return 1d;
    }

}
