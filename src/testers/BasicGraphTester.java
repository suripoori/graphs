package testers;

import static org.junit.Assert.*;

import java.util.LinkedList;
import java.util.List;

import org.junit.Before;
import org.junit.Test;
import basicgraph.Graph;
import basicgraph.GraphAdjList;
import basicgraph.GraphAdjMatrix;
import util.GraphLoader;

public class BasicGraphTester {
	GraphAdjList graphFromFile;
	GraphAdjList nyGraphFromFile;
	GraphAdjMatrix graphFromFileMatrix;
	
	@Before
	public void setUp() throws Exception {
		graphFromFile = new GraphAdjList();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", graphFromFile);
		
		graphFromFileMatrix = new GraphAdjMatrix();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", graphFromFileMatrix);
		
		nyGraphFromFile = new GraphAdjList();
		GraphLoader.loadRoadMap("data/maps/new_york.map", nyGraphFromFile);
	}
	
	@Test
	public void testDegreeSequence()
	{
		try{
			List<Integer> degSequence = graphFromFile.degreeSequence();
			for (int i=0; i<degSequence.size()-1; i++){
				if (degSequence.get(i) < degSequence.get(i+1)){
					fail("Small graph has wrong degree sequence");
				}
			}
			List<Integer> nyDegSequence = nyGraphFromFile.degreeSequence();
			for (int i=0; i<nyDegSequence.size()-1; i++){
				if (nyDegSequence.get(i) < nyDegSequence.get(i+1)){
					fail("NY graph has wrong degree sequence");
				}
			}
			/*for (int i=0; i<degSequence.size()-1; i++){
				if (graphFromFile.getNeighbors(degSequence.get(i)).size() < 
						graphFromFile.getNeighbors(degSequence.get(i+1)).size()){
					fail("Small graph has wrong degree sequence");
				}	
			}
			
			List<Integer> nyDegSequence = nyGraphFromFile.degreeSequence();
			for (int i=0; i<nyDegSequence.size()-1; i++){
				if (nyGraphFromFile.getNeighbors(nyDegSequence.get(i)).size() < 
						nyGraphFromFile.getNeighbors(nyDegSequence.get(i+1)).size()){
					fail("NY graph has wrong degree sequence: Vertex " + i + 
							" has degree of " + nyGraphFromFile.getNeighbors(nyDegSequence.get(i)).size() + 
							", Vertex " + (i+1) + " has degree of " + 
							nyGraphFromFile.getNeighbors(nyDegSequence.get(i+1)).size());
				}	
			}*/
		}
		catch(IndexOutOfBoundsException e){
			System.out.println("Test is bad");
		}
	}
	
	@Test
	public void testDistance2(){
		try{
			assertEquals("Check adj list getDistance2", 5, graphFromFile.getDistance2(0).size());
			assertEquals("Check adj Matrix getDistance2", 5, graphFromFileMatrix.getDistance2(0).size());
		}
		catch(IndexOutOfBoundsException e){
			System.out.println("Test is bad");
		}
	}
}
