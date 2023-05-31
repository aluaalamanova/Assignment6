import java.util.HashMap;
import java.util.Map;

public class Vertex<V> {
    private V data;
    private Map<Vertex<V>, Double> adjacentVertices;
    private double distance;

    public Vertex(V data) {
        this.data = data;
        this.adjacentVertices = new HashMap<>();
        this.distance = Double.POSITIVE_INFINITY;
    }

}
