package us.ilite.common;

public interface IFieldComponent {

    public int id();
    public double height();
    public int pipeline();

    default double width(){ return 0.0; }
    default double powerScalar() {
        return 1d;
    }

}
