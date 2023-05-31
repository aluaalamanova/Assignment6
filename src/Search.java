public abstract class Search<V> {
    protected WeightedGraph<V> graph;

    public Search(WeightedGraph<V> graph) {
        this.graph = graph;
    }
}
