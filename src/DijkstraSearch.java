import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.PriorityQueue;

public class DijkstraSearch<V> extends Search<V> {
    public DijkstraSearch(WeightedGraph<V> graph) {
        super(graph);
    }
    public void dijSearch(Vertex<V> startVertex) {
        PriorityQueue<Vertex<V>> queue = new PriorityQueue<>(Comparator.comparingDouble(this::getDistance));
        Map<Vertex<V>, Double> distances = new HashMap<>();
        Map<Vertex<V>, Vertex<V>> previousVertices = new HashMap<>();

        for (Vertex<V> vertex : graph.getAdjacencyMap().keySet()) {
            if (vertex.equals(startVertex)) {
                vertex.setDistance(0.0); // Set distance of the start vertex to 0
            } else {
                vertex.setDistance(Double.POSITIVE_INFINITY); // Set distance of other vertices to infinity
            }
            distances.put(vertex, vertex.getDistance());
            queue.add(vertex);
        }

        while (!queue.isEmpty()) {
            Vertex<V> currentVertex = queue.poll();
            double currentDistance = currentVertex.getDistance();

            if (currentDistance == Double.POSITIVE_INFINITY) {
                break; // All remaining vertices are unreachable
            }

            for (Vertex<V> neighbor : graph.getNeighbors(currentVertex)) {
                double edgeWeight = currentVertex.getAdjacentVertices().get(neighbor);
                double newDistance = currentDistance + edgeWeight;

                if (newDistance < neighbor.getDistance()) {
                    queue.remove(neighbor); // Remove and re-add to update priority queue ordering
                    neighbor.setDistance(newDistance);
                    distances.put(neighbor, newDistance);
                    previousVertices.put(neighbor, currentVertex);
                    queue.add(neighbor);
                }
            }
        }

        System.out.println("Shortest paths from " + startVertex.getData() + ":");
        for (Vertex<V> vertex : distances.keySet()) {
            System.out.println("Vertex: " + vertex.getData() + ", Distance: " + distances.get(vertex));
        }
    }

    private double getDistance(Vertex<V> vertex) {
        return vertex.getDistance();
    }
}
