public class Main {
    public static void main(String[] args) {
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
    }
}
