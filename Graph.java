import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Graph {
    Map<String, City> cities = new HashMap<>();
    Map<String, List<String>> adjList = new HashMap<>();

    public void loadCoordinates(String filePath) throws IOException {
        BufferedReader reader = new BufferedReader(new FileReader(filePath));
        String line;
        while ((line = reader.readLine()) != null) {
            String[] data = line.split(",");
            String name = data[0];
            double latitude = Double.parseDouble(data[1]);
            double longitude = Double.parseDouble(data[2]);
            cities.put(name, new City(name, latitude, longitude));
        }
        reader.close();
    }


    public void loadAdjacency(String filePath) throws IOException {
        BufferedReader reader = new BufferedReader(new FileReader(filePath));
        String line;

        while ((line = reader.readLine()) != null) {
            line = line.trim();

            if (line.isEmpty())
                continue;

            String[] cities = line.split("\\s+"); 

            if (cities.length != 2) {
                System.err.println("Skipping invalid line: " + line);
                continue;
            }

            String cityA = cities[0].trim();
            String cityB = cities[1].trim();

            adjList.computeIfAbsent(cityA, k -> new ArrayList<>()).add(cityB);
            adjList.computeIfAbsent(cityB, k -> new ArrayList<>()).add(cityA);
        }

        reader.close();
    }

    public City getCity(String name) {
        return cities.get(name);
    }

    public List<String> getNeighbors(String city) {
        return adjList.getOrDefault(city, new ArrayList<>());
    }
}
