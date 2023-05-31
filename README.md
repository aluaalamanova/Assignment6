# Assignment6
### Edge-Weighted Graph with Vertex Implementation
### Introduction:
Edge-weighted graph with vertices is a data structure where each vertex connects to others through edges with associated weights, representing distance or cost. It uses a vertex-based approach with objects storing data and maintaining adjacent vertices and edge weights.
### Classes:
### - Vertex:
The Vertex class stores vertex data and maintains adjacent vertices with their edge weights. It provides methods to add and retrieve adjacent vertices and their weights.
```
public class Vertex<V> {
    private V data;
    private Map<Vertex<V>, Double> adjacentVertices;

    public Vertex(V data) {
        this.data = data;
        this.adjacentVertices = new HashMap<>();
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
}
```
### - WeightedGraph:
WeightedGraph class represents an edge-weighted graph, storing vertices and their adjacency lists. It supports adding vertices, edges with weights, and retrieving adjacency map and neighbors.
```
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

    public List<Vertex<V>> getNeighbors(Vertex<V> vertex) {
        return adjacencyMap.get(vertex);
    }

    public Map<Vertex<V>, List<Vertex<V>>> getAdjacencyMap() {
        return adjacencyMap;
    }
}
```
### - Search (Abstract Class):
The Search abstract class serves as the base class for search algorithms on the weighted graph. It provides a reference to the graph and can be extended by specific search algorithm implementations.
```
public abstract class Search<V> {
    protected WeightedGraph<V> graph;

    public Search(WeightedGraph<V> graph) {
        this.graph = graph;
    }
}
```
### - BreadthFirstSearch:
The BreadthFirstSearch class implements the breadth-first search algorithm on the weighted graph. It extends the Search class and provides a method to perform breadth-first search starting from a given vertex.
```
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;

public class BreadthFirstSearch<V> extends Search<V> {
    public BreadthFirstSearch(WeightedGraph<V> graph) {
        super(graph);
    }

    public void bfs(Vertex<V> startVertex) {
        Set<Vertex<V>> visited = new HashSet<>();
        Queue<Vertex<V>> queue = new LinkedList<>();

        startVertex.setDistance(0.0); // Set distance of the start vertex to 0

        visited.add(startVertex);
        queue.add(startVertex);

        while (!queue.isEmpty()) {
            Vertex<V> currentVertex = queue.poll();
            System.out.println("Visited: " + currentVertex.getData() + ", Distance: " + currentVertex.getDistance());

            List<Vertex<V>> neighbors = graph.getNeighbors(currentVertex);
            for (Vertex<V> neighbor : neighbors) {
                if (!visited.contains(neighbor)) {
                    visited.add(neighbor);
                    neighbor.setDistance(currentVertex.getDistance() + 1); // Increment distance by 1
                    queue.add(neighbor);
                }
            }
        }
    }
}
```
### - DijkstraSearch:
The DijkstraSearch class implements Dijkstra's algorithm on the weighted graph. It extends the Search class and provides a method to perform Dijkstra's algorithm starting from a given vertex.
```
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
```
### Usage example:
        WeightedGraph<String> graph = new WeightedGraph<>();
        Vertex<String> vertexA = new Vertex<>("A");
        Vertex<String> vertexB = new Vertex<>("B");
        Vertex<String> vertexC = new Vertex<>("C");
        Vertex<String> vertexD = new Vertex<>("D");
        Vertex<String> vertexE = new Vertex<>("E");
        graph.addVertex(vertexA);
        graph.addVertex(vertexB);
        graph.addVertex(vertexC);
        graph.addVertex(vertexD);
        graph.addVertex(vertexE);
        graph.addEdge(vertexA, vertexB, 7.0);
        graph.addEdge(vertexA, vertexC, 4.0);
        graph.addEdge(vertexB, vertexD, 3.0);
        graph.addEdge(vertexC, vertexD, 8.0);
        graph.addEdge(vertexD, vertexE, 5.0);
        BreadthFirstSearch<String> bfs = new BreadthFirstSearch<>(graph);
        System.out.println("BFS traversal:");
        bfs.bfs(vertexA);
        DijkstraSearch<String> dijkstra = new DijkstraSearch<>(graph);
        System.out.println("\nDijkstra's algorithm:");
        dijkstra.dijSearch(vertexA);
