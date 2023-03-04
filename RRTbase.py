import random
import math
import pygame
import line_inter as li
import numpy as np

class Node:
    """
    Node class that represent the data points 
    """
    id = 0
    instances = []

    def __init__(self,x,y,p):
        self.x = x
        self.y = y
        self.p = p
        self.id = Node.id 
        Node.id += 1
        Node.instances.append(self)
        
    def totalNodes(self):
        return len(Node.instances)
    
class Obstacle:
    """
    Obstacle class that represents the obstacles. Only Rectangular at the moment.
    """
    id = 0
    instances = []

    def __init__(self,upper,w,h):
        x,y = upper
        self.dim = (w,h)
        self.center = (x + w/2, y + h/2)
        self.corners = [upper,(x+w,y),(x+w,y+h),(x,y+h)]
        self.edges = [(upper,(x+w,y)),((x+w,y),(x+w,y+h)),((x+w,y+h),(x,y+h)),((x,y+h),upper)]
        self.id = Obstacle.id
        Obstacle.id += 1
        Obstacle.instances.append(self)
        self.rect = pygame.Rect(self.corners[0],(self.dim[0],self.dim[1]))

class RRTMAP:
    """
    Class that visualize the code
    """
    def __init__(self,start,goal,MapDimensions):
        #Start,goal & map informations
        self.start = start
        self.goal = goal
        self.Maph , self.Mapw = MapDimensions

        #window settings
        self.MapWindowName = "RRT Path Planning"
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw,self.Maph))
        self.map.fill((255,255,255))
        self.nodeRad = 2
        self.edgeThickness = 1

    def drawObs(self):
        """
        Draw obstacles onto map
        """
        for obs in Obstacle.instances:
            pygame.draw.rect(self.map,"gray",obs)

    def drawMap(self):
        """
        Draw elements onto map
        """
        pygame.draw.circle(self.map,"green3",self.start,self.nodeRad+5,0)
        pygame.draw.circle(self.map,"red1",self.goal,self.nodeRad+20,0)
        self.drawObs()

    def drawPath(self):
        pass

class RRTGraph:
    """
    Class that generates the map and data points
    """
    def __init__(self,start,goal,MapDimensions):

        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.maph , self.mapw = MapDimensions

        #initialize nodes
        Node(start[0],start[1],None)

        #path
        self.path = []

    def randomCoord(self):
        """
        Random coordinate generator
        """
        xLen = random.randint(30,100)
        yLen = random.randint(30,100)
        uppercornerx = int(random.uniform(0,self.mapw-xLen))
        uppercornery = int(random.uniform(0,self.maph-yLen))

        return (uppercornerx,uppercornery) , (xLen,yLen)

    def makeobs(self,obsNum):
        """
        Generate random obstacles with random sizes
        """
        for _ in range(obsNum):
            c_ = 0
            collision = True
            while c_ < 100:
                upper , xLen, yLen = self.randomCoord()
                newRect = pygame.Rect(upper,(xLen,yLen))
                if newRect.collidepoint(self.start) or newRect.collidepoint(self.goal):
                    continue
                else:
                    collision = False
            if not collision:
                Obstacle(upper,xLen,yLen)

    def createObs(self,upper,width,height):
        """
        Create desired rectangular obstacle
        """
        Obstacle(upper,width,height)

        return

    def add_node(self,x,y,p=None):
        """
        Create Node
        """
        Node(x,y,p)

        return

    def findParent(self,nodeCandidate):
        """
        A function that find the parent of a given nodeCandidate. 
        """
        dmin = self.maph * self.mapw #A value that exceeds the distance between any points from the map
        for n_ in Node.instances[:-1]:
            distance = self.distance(nodeCandidate,n_)
            if distance < dmin:
                dmin = distance
                nearest_ = n_
        
        nodeCandidate.p = nearest_

        return

    def removeNode(self,nodeIndex): #TODO BURASI DÜZELTİLECEK NODE INDEXTEN POPLADIĞIMIZDA SONRAKİLERiN İNDEXİ KAYIYOR
        """
        Remove Node
        """
        Node.instances.pop(nodeIndex)
        Node.id -= 1

        return

    def trim(self,node,dmax = 35):
        """
        Trims the distanec between node and its parent by placing the node to the distance
        dmax.
        """
        d = self.distance(node,node.p)
        if d > dmax:
            px,py = node.p.x-node.x, node.p.y-node.y
            px,py = node.x-node.p.x, node.y-node.p.y
            theta = math.atan2(py,px)
            x,y = int(node.p.x+dmax*math.cos(theta)),int(node.p.y+dmax*math.sin(theta))
            newIdBox = (x,y,node.p)
            self.removeNode(-1)
            self.add_node(newIdBox[0],newIdBox[1],newIdBox[2])
        
        return

    def addParent(self,parent,child):
        """
        Add a connection btw parent and child
        """
        Node.instances[child].p = parent

        return

    def expand(self):  
        """
        Pick a random x,y coordinates for a node candidate. Find nearest Node. Check if path is feasible. If not find the feaseble point
        that lies in the same direction.
        """
        x,y = self.sample_envir()

        self.add_node(x,y,None)
        node = Node.instances[-1]
        self.findParent(node)
        parent = node.p
        p1 = li.Point(node.x,node.y)
        p2 = li.Point(parent.x,parent.y)
        
        collidedEdges = []
        for obs in Obstacle.instances:
            for edge in obs.edges:
                p3 = li.Point(edge[0][0],edge[0][1])
                p4 = li.Point(edge[1][0],edge[1][1])
                if li.doIntersect(p1, p2, p3, p4):
                    collidedEdges.append((p3,p4))

        if collidedEdges:
            distances = []
            for colEdge in collidedEdges:
                E1 = np.array([colEdge[0].x,colEdge[0].y])
                E2 = np.array([colEdge[1].x,colEdge[1].y])
                E3 = np.array([parent.x,parent.y])
                dist = np.linalg.norm(np.cross(E2-E1, E1-E3))/np.linalg.norm(E2-E1)
                distances.append(dist)
            minDistance = min(distances)
            minDistance = min(minDistance,35)
            
            self.trim(Node.instances[-1],minDistance)
        
        else:
            self.trim(Node.instances[-1])

        return Node.instances[-1].x, Node.instances[-1].y, parent.x, parent.y

    def distance(self,Node1,Node2):
        """
        Distance btw Node1 and Node2
        """
        deltaX = (Node1.x-Node2.x)**2
        deltaY = (Node1.y-Node2.y)**2
        return (deltaX + deltaY)**(0.5)
    
    def sample_envir(self):
        """
        Pick a random point coordinate from the map
        """
        x = int(random.uniform(0,self.mapw))
        y = int(random.uniform(0,self.maph))

        return x,y

    def path_to_goal(self):
        pass

    def getPathCoords(self):
        pass

    def bias(self,ngoal): #TODO CREATE BIAS FUNCTION
        pass

    def cost(self):
        pass

    def checkGoal(self,dmax = 35):
        """
        function that checks if the goal is found
        """
        lastNode = Node.instances[-1]
        if abs(lastNode.x-self.goal[0]) < dmax and abs(lastNode.y-self.goal[1]) < dmax:
            self.add_node(self.goal[0],self.goal[1],lastNode)
            self.goalFlag = True
            # self.add_node(lastNode.x,lastNode.y,lastNode)
            
            goalNode = Node.instances[-1]
            path = [goalNode]
            currentNode = goalNode
            while currentNode.p:
                path.append(currentNode.p)
                currentNode = currentNode.p
            return path
        
        return []



