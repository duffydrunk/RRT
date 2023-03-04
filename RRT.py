import pygame
from RRTbase import RRTGraph
from RRTbase import RRTMAP
import time

def main():
    dimensions = (900,900)
    start = (450,450)
    goal = (850,850)

    pygame.init()
    map = RRTMAP(start,goal,dimensions)
    graph = RRTGraph(start,goal,dimensions)

    # obstacles = graph.makeobs()
    graph.createObs((300,0),20,400)
    graph.createObs((600,0),20,400)
    graph.createObs((300,500),20,400)
    graph.createObs((600,500),20,400)
    graph.createObs((700,800),200,20)
    
    map.drawMap()

    i = 0

    while not graph.goalFlag and i < 5000:

        x,y,px,py = graph.expand() 
        pygame.draw.circle(map.map,"darkorange",(x,y),map.nodeRad+2,0)
        pygame.draw.line(map.map,"royalblue1",(x,y),(px,py),map.edgeThickness+2)
        pygame.display.update()
        i += 1
        path = graph.checkGoal()
        time.sleep(0.1)
        
    
    if path:
        for node in path:
            pygame.draw.circle(map.map,"red3",(node.x,node.y),map.nodeRad+5,0)
            if node.p:
                pygame.draw.line(map.map,"red3",(node.x,node.y),(node.p.x,node.p.y),map.edgeThickness+3)

    
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == "__main__":
     main()
