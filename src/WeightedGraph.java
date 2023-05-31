import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class WeightedGraph<V> {
    private Map<Vertex<V>, List<Vertex<V>>> adjacencyMap;

    public WeightedGraph() {
        this.adjacencyMap = new HashMap<>();
    }
    public void addVertex(Vertex<V> vertex) {
        adjacencyMap.put(vertex, new ArrayList<>());
    }
    public void addEdge(Vertex<V> source, Vertex<V> destination, double weight) {
        source.addAdjacentVertex(destination, weight);
        destination.addAdjacentVertex(source, weight);

        adjacencyMap.get(source).add(destination);
        adjacencyMap.get(destination).add(source);
    }

}
