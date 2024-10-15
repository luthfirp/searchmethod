import java.util.*;

class SearchAlgorithms {
    Graph graph;

    public SearchAlgorithms(Graph graph) {
        this.graph = graph;
    }

    // Breath First Search
    public void bfs(String start, String goal) {
        long startTime = System.nanoTime();
        Runtime runtime = Runtime.getRuntime();
        runtime.gc();
        
        long memoryBefore = runtime.totalMemory() - runtime.freeMemory();

        Queue<String> frontier = new LinkedList<>();
        Map<String, String> cameFrom = new HashMap<>();
        List<String> path = new ArrayList<>();

        frontier.add(start);
        cameFrom.put(start, null);

        while (!frontier.isEmpty()) {
            String current = frontier.poll();
            if (current.equals(goal)) break;

            for (String neighbor : graph.getNeighbors(current)) {
                if (!cameFrom.containsKey(neighbor)) {
                    frontier.add(neighbor);
                    cameFrom.put(neighbor, current);
                }
            }
        }

        path = reconstructPath(cameFrom, start, goal);
        double totalDistance = calculateTotalDistance(path);

        long memoryAfter = runtime.totalMemory() - runtime.freeMemory();
        long memoryUsed = (memoryAfter - memoryBefore) / 1024;
        
        long endTime = System.nanoTime();
        long duration = (endTime - startTime) / 1_000_000;

        
        System.out.println("BFS Path found: " + path);
        System.out.println("Total distance: " + totalDistance + " km");
        System.out.println("Time taken: " + duration + " ms");
        System.out.println("Memory used: " + memoryUsed + " KB");
    }

    // Depth-First Search
    public void dfs(String start, String goal) {
        long startTime = System.nanoTime();
        Runtime runtime = Runtime.getRuntime();
        runtime.gc();

        long memoryBefore = runtime.totalMemory() - runtime.freeMemory();

        Stack<String> frontier = new Stack<>();
        Map<String, String> cameFrom = new HashMap<>();
        List<String> path = new ArrayList<>();

        frontier.add(start);
        cameFrom.put(start, null);

        while (!frontier.isEmpty()) {
            String current = frontier.pop();
            if (current.equals(goal)) break;

            for (String neighbor : graph.getNeighbors(current)) {
                if (!cameFrom.containsKey(neighbor)) {
                    frontier.add(neighbor);
                    cameFrom.put(neighbor, current);
                }
            }
        }

        path = reconstructPath(cameFrom, start, goal);
        double totalDistance = calculateTotalDistance(path);

        long memoryAfter = runtime.totalMemory() - runtime.freeMemory();
        long memoryUsed = (memoryAfter - memoryBefore) / 1024;

        long endTime = System.nanoTime();
        long duration = (endTime - startTime) / 1_000_000;

        
        System.out.println("DFS Path found: " + path);
        System.out.println("Total distance: " + totalDistance + " km");
        System.out.println("Time taken: " + duration + " ms");
        System.out.println("Memory used: " + memoryUsed + " KB");
    }

    // Iterative Deepening Depth-First Search (ID-DFS)
    public void iddfs(String start, String goal) {
        long startTime = System.nanoTime();
        Runtime runtime = Runtime.getRuntime();
        runtime.gc();

        long memoryBefore = runtime.totalMemory() - runtime.freeMemory();

        List<String> path = null;
        for (int depth = 0; depth < Integer.MAX_VALUE; depth++) {
            path = dls(start, goal, depth);
            if (path != null) break;
        }

        double totalDistance = calculateTotalDistance(path);

        long memoryAfter = runtime.totalMemory() - runtime.freeMemory();
        long memoryUsed = (memoryAfter - memoryBefore) / 1024;

        long endTime = System.nanoTime();
        long duration = (endTime - startTime) / 1_000_000;

        
        System.out.println("IDDFS Path found: " + path);
        System.out.println("Total distance: " + totalDistance + " km");
        System.out.println("Time taken: " + duration + " ms");
        System.out.println("Memory used: " + memoryUsed + " KB");
    }

    // Depth-Limited Search helper for IDDFS
    private List<String> dls(String current, String goal, int depth) {
        if (depth == 0 && current.equals(goal)) {
            return new ArrayList<>(Collections.singletonList(goal));
        }
        if (depth > 0) {
            for (String neighbor : graph.getNeighbors(current)) {
                List<String> result = dls(neighbor, goal, depth - 1);
                if (result != null) {
                    result.add(0, current);
                    return result;
                }
            }
        }
        return null;
    }

    // Best-First Search
    public void bestFirstSearch(String start, String goal) {
        long startTime = System.nanoTime();
        Runtime runtime = Runtime.getRuntime();
        runtime.gc();

        long memoryBefore = runtime.totalMemory() - runtime.freeMemory();

        PriorityQueue<Node> frontier = new PriorityQueue<>(Comparator.comparingDouble(n -> n.heuristic));
        Map<String, String> cameFrom = new HashMap<>();
        List<String> path = new ArrayList<>();

        frontier.add(new Node(start, 0, heuristic(start, goal)));
        cameFrom.put(start, null);

        while (!frontier.isEmpty()) {
            Node current = frontier.poll();
            if (current.cityName.equals(goal)) break;

            for (String neighbor : graph.getNeighbors(current.cityName)) {
                if (!cameFrom.containsKey(neighbor)) {
                    frontier.add(new Node(neighbor, 0, heuristic(neighbor, goal)));
                    cameFrom.put(neighbor, current.cityName);
                }
            }
        }

        path = reconstructPath(cameFrom, start, goal);
        double totalDistance = calculateTotalDistance(path);

        long memoryAfter = runtime.totalMemory() - runtime.freeMemory();
        long memoryUsed = (memoryAfter - memoryBefore) / 1024;

        long endTime = System.nanoTime();
        long duration = (endTime - startTime) / 1_000_000;

        System.out.println("Best-First Search Path found: " + path);
        System.out.println("Total distance: " + totalDistance + " km");
        System.out.println("Time taken: " + duration + " ms");
        System.out.println("Memory used: " + memoryUsed + " KB");
    }

    // A* Search
    public void aStarSearch(String start, String goal) {
        long startTime = System.nanoTime();
        Runtime runtime = Runtime.getRuntime();
        runtime.gc();

        long memoryBefore = runtime.totalMemory() - runtime.freeMemory();

        PriorityQueue<Node> frontier = new PriorityQueue<>(Comparator.comparingDouble(n -> n.cost + n.heuristic));
        Map<String, String> cameFrom = new HashMap<>();
        Map<String, Double> costSoFar = new HashMap<>();
        List<String> path = new ArrayList<>();

        frontier.add(new Node(start, 0, heuristic(start, goal)));
        cameFrom.put(start, null);
        costSoFar.put(start, 0.0);

        while (!frontier.isEmpty()) {
            Node current = frontier.poll();
            if (current.cityName.equals(goal)) break;

            for (String neighbor : graph.getNeighbors(current.cityName)) {
                double newCost = costSoFar.get(current.cityName) + distance(current.cityName, neighbor);
                if (!costSoFar.containsKey(neighbor) || newCost < costSoFar.get(neighbor)) {
                    costSoFar.put(neighbor, newCost);
                    frontier.add(new Node(neighbor, newCost, heuristic(neighbor, goal)));
                    cameFrom.put(neighbor, current.cityName);
                }
            }
        }

        path = reconstructPath(cameFrom, start, goal);
        double totalDistance = calculateTotalDistance(path);

        long memoryAfter = runtime.totalMemory() - runtime.freeMemory();
        long memoryUsed = (memoryAfter - memoryBefore) / 1024;

        long endTime = System.nanoTime();
        long duration = (endTime - startTime) / 1_000_000;

        
        System.out.println("A* Path found: " + path);
        System.out.println("Total distance: " + totalDistance + " km");
        System.out.println("Time taken: " + duration + " ms");
        System.out.println("Memory used: " + memoryUsed + " KB");
    }

    // Helper methods

    private List<String> reconstructPath(Map<String, String> cameFrom, String start, String goal) {
        List<String> path = new ArrayList<>();
        String current = goal;
        while (current != null) {
            path.add(current);
            current = cameFrom.get(current);
        }
        Collections.reverse(path);
        return path;
    }

    private double calculateTotalDistance(List<String> path) {
        double totalDistance = 0.0;
        for (int i = 0; i < path.size() - 1; i++) {
            totalDistance += distance(path.get(i), path.get(i + 1));
        }
        return totalDistance;
    }

    private double heuristic(String cityA, String cityB) {
        City a = graph.cities.get(cityA);
        City b = graph.cities.get(cityB);
        return haversine(a.latitude, a.longitude, b.latitude, b.longitude);
    }

    private double distance(String cityA, String cityB) {
        return heuristic(cityA, cityB);
    }

    private double haversine(double lat1, double lon1, double lat2, double lon2) {
        final double R = 6371; // Earth radius in km
        double dLat = Math.toRadians(lat2 - lat1);
        double dLon = Math.toRadians(lon2 - lon1);
        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                   Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) *
                   Math.sin(dLon / 2) * Math.sin(dLon / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return R * c;
    }
}
