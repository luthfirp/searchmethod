import java.io.IOException;
import java.util.Scanner;

public class Main {
    public static void main(String[] args) throws IOException {
        Graph graph = new Graph();
        graph.loadCoordinates("coordinates.csv");
        graph.loadAdjacency("adjacencies.txt");

        SearchAlgorithms searchAlgorithms = new SearchAlgorithms(graph);

        Scanner input = new Scanner(System.in);
        System.out.println("Enter start city: ");
        String start = input.nextLine();
        System.out.println("Enter goal city: ");
        String goal = input.nextLine();

        boolean condition = true;
        while (condition) {
            System.out.println("=========================================================================================");
            System.out.println("Select search method (1: BFS, 2: DFS, 3: IDDFS, 4: Best-First Search, 5: A*, 0: Exit): ");
            int choice = input.nextInt();

            switch (choice) {
                case 1:
                    searchAlgorithms.bfs(start, goal);
                    break;
                case 2:
                    searchAlgorithms.dfs(start, goal);
                    break;
                case 3:
                    searchAlgorithms.iddfs(start, goal);
                    break;
                case 4:
                    searchAlgorithms.bestFirstSearch(start, goal);
                    break;
                case 5:
                    searchAlgorithms.aStarSearch(start, goal);
                    break;
                case 0:
                    condition = false;
                    break;
                default:
                    System.out.println("Invalid choice. Please try again.");
                    break;
            }
        }
        input.close();
    }
}
