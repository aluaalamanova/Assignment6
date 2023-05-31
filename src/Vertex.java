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
    public V getData() {
        return data;
    }
    public void addAdjacentVertex(Vertex<V> destination, double weight) {
        adjacentVertices.put(destination, weight);
    }
    public Map<Vertex<V>, Double> getAdjacentVertices() {
        return adjacentVertices;
    }
    public double getDistance() {
        return distance;
    }
    public void setDistance(double distance) {
        this.distance = distance;
    }
}
