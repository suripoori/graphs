/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import geography.RoadSegment;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private Map<GeographicPoint,ArrayList<RoadSegment>> adjListsMap;
	private int numVertices;
	private int numEdges;
	private enum searchType {DIJKSTRA, ASTAR};
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		adjListsMap = new HashMap<GeographicPoint,ArrayList<RoadSegment>>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return new HashSet<GeographicPoint>(adjListsMap.keySet());
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return numEdges;
	}
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if (location.equals(null) || adjListsMap.get(location) != null){
			return false;
		}
		try {
			ArrayList<RoadSegment> neighbors = new ArrayList<RoadSegment>();
			adjListsMap.put(location,  neighbors);
			numVertices++;
			return true;
		}
		catch (Exception e) {
			return false;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		//TODO: Implement this method in WEEK 2
		if(adjListsMap.containsKey(from) && adjListsMap.containsKey(to) && length >= 0){
			RoadSegment rs = new RoadSegment(from, to, 
				new ArrayList<GeographicPoint>(), roadName, 
				roadType, length);
			adjListsMap.get(from).add(rs);
			numEdges++;
		}
		else{
			throw new IllegalArgumentException("Either the from point " + from + 
					" or the to point " + to + " are not present in the graph");
		}
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		if ((!adjListsMap.containsKey(start)) || (!adjListsMap.containsKey(goal))) {
			System.out.println("The point " + start + " or the point " + goal + 
					" do not exist in this graph");
			return(null);
		}
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Queue<GeographicPoint> bfsQueue = new LinkedList<GeographicPoint>();
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
		bfsQueue.add(start);
		// Keep looping until we reach the goal or the queue is empty
		while(!bfsQueue.isEmpty()){
			GeographicPoint next = bfsQueue.remove();
			//Hook for visualization.  See writeup.
			nodeSearched.accept(next);
			if (next.equals(goal)){
				return reconstructPath(parent, start, goal);
			}
			for (RoadSegment r : getNeighbors(next)){
				if (!visited.contains(r.getOtherPoint(next))){
					visited.add(r.getOtherPoint(next));
					parent.put(r.getOtherPoint(next), next);
					bfsQueue.add(r.getOtherPoint(next));
				}
			}
		}
		return(null);
	}
	
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		return heuristicSearch(start, goal, nodeSearched, searchType.DIJKSTRA);
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		return heuristicSearch(start, goal, nodeSearched, searchType.ASTAR);
	}

	private List<GeographicPoint> heuristicSearch(GeographicPoint start, 
			GeographicPoint goal, Consumer<GeographicPoint> nodeSearched, 
			searchType type)
	{	
		if ((!adjListsMap.containsKey(start)) || (!adjListsMap.containsKey(goal))) {
			System.out.println("The point " + start + " or the point " + goal + 
					" do not exist in this graph");
			return(null);
		}
		PriorityQueue<costOfVertex> pq = new PriorityQueue<costOfVertex>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
		HashMap<GeographicPoint, costOfVertex> vertexCosts = initializeVertexCosts();
		costOfVertex s = new costOfVertex(0, start);
		vertexCosts.put(start, s);
		pq.add(s);
		int numNodesSearched = 0;
		// Keep looping until we reach the goal or the priority queue is empty
		while(!pq.isEmpty()){
			costOfVertex n = pq.remove();
			GeographicPoint next = n.getVertex(); 
			if(!visited.contains(next)){
				numNodesSearched++;
				visited.add(next);
				//Hook for visualization.  See writeup.
				nodeSearched.accept(next);
				if (next.equals(goal)){
					System.out.println(numNodesSearched);
					return reconstructPath(parent, start, goal);
				}
				for (RoadSegment r : getNeighbors(next)){
					GeographicPoint neighbor = r.getOtherPoint(next);
					if (!visited.contains(neighbor)){
						if (r.getLength() + n.getCost() < vertexCosts.get(neighbor).getCost()){
							double totalCost = 0;
							if (type == searchType.DIJKSTRA){
								totalCost = n.getCost() + r.getLength();
							}
							else if (type == searchType.ASTAR){
								totalCost = n.getCost() + r.getLength() + neighbor.distance(goal);
							}
							costOfVertex updatedCost = new costOfVertex(totalCost, neighbor);
							pq.add(updatedCost);
							parent.put(neighbor, next);
							vertexCosts.put(neighbor, updatedCost);
						}
					}
				}
			}
		}
		System.out.println(numNodesSearched);
		return(null);
	}
	
	
	public static void main(String[] args)
	{
		/*System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		GeographicPoint start = new GeographicPoint(1, 1);
		GeographicPoint end = new GeographicPoint(8, -1);
		
		
		List<GeographicPoint> route = theMap.bfs(start,end);
		System.out.println(route);*/
		// You can use this method for testing.  
		
		//Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
	}
	
	/* A method to get the neighbors of an intersection */
	private List<RoadSegment> getNeighbors(GeographicPoint v){
		return adjListsMap.get(v);
	}
	
	/* A method to reconstruct the path from a parent hashmap */
	private List<GeographicPoint> reconstructPath(HashMap<GeographicPoint, GeographicPoint> parent, GeographicPoint start, GeographicPoint goal){
		GeographicPoint curr = goal;
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		path.add(curr);
		while(curr != start){
			path.add(parent.get(curr));
			curr = parent.get(curr);
		}
		Collections.reverse(path);
		return(path);
	}
	
	/* A method which creates an empty hashmap and adds all the vertexes into it.
	 * Its a map of GeographicPoint to costOfVertex
	 */
	private HashMap<GeographicPoint, costOfVertex> initializeVertexCosts(){
		HashMap<GeographicPoint, costOfVertex> vertexCosts = new HashMap<GeographicPoint, costOfVertex>();
		for(GeographicPoint vertex : getVertices()){
			vertexCosts.put(vertex, new costOfVertex(Double.MAX_VALUE, vertex));
		}
		return(vertexCosts);
	}
	
	/*A class to keep track of the heuristic cost of going to a vertex*/
	private class costOfVertex implements Comparable<costOfVertex>{
		private double cost;
		private GeographicPoint vertex;
		
		public double getCost(){
			return cost;
		}

		public GeographicPoint getVertex(){
			return vertex;
		}
		
		public void setCost(double cost){
			this.cost = cost;
		}
		
		public costOfVertex(double cost, GeographicPoint vertex){
			this.cost = cost;
			this.vertex = vertex;
		}
		
		@Override
		public int compareTo(costOfVertex o) {
			// TODO Auto-generated method stub
			if (o.getCost() == this.getCost()){
				return(0);
			}
			else {
				return this.getCost() > o.getCost() ? 1 : -1;
			}
		}
	}
}
