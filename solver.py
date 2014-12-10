from copy import deepcopy
from PIL import Image, ImageDraw
import pprint
import Queue
import sys

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        else:
            return False

    def __hash__(self):
        return hash(str(self.x) + "," + str(self.y))

class Node:
    maxX  = None
    maxY = None

    def __init__(self, parent, point, heuristicCost, movementCost):
        self.parent        = parent
        self.point         = point
        self.heuristicCost = heuristicCost
        self.movementCost  = movementCost

    def __cmp__(self, other):
        return cmp(self.cost(), other.cost())

    def __hash__(self):
        return self.point.__hash__()

    def cost(self):
        return self.heuristicCost + self.movementCost

    def adjacentNodes(self):
        adjacent = []
        if self.point.x > 0:
            adjacent.append(Node(self,
                                 Point(self.point.x-1, self.point.y),
                                 0,
                                 0
                            ))
        if self.point.x < Node.maxX:
            adjacent.append(Node(self,
                                 Point(self.point.x+1, self.point.y),
                                 0,
                                 0
                            ))
        if self.point.y > 0:
            adjacent.append(Node(self,
                                 Point(self.point.x, self.point.y-1),
                                 0,
                                 0
                            ))
        if self.point.y < Node.maxY:
            adjacent.append(Node(self,
                                 Point(self.point.x, self.point.y+1),
                                 0,
                                 0
                            ))
        return adjacent

class AStarSearch:
    def __init__(self):
        self.openSet         = Queue.PriorityQueue()
        self.openSetMirror   = set()
        self.closedSet       = set()
        self.closedSetMirror = set()

################################################################################


def convertBW(imgPath):
    '''
    Converts and image to just pure black and pure white.
    without any grays or colours.
    '''

    if not type(imgPath) is str:
        raise TypeError('must call convertBW with string containing image path')
    if not imgPath:
        raise ValueError('cannot call convertBW with no image path')

    colourImg    = Image.open(imgPath)
    grayscaleImg = colourImg.convert('L')
    bwImg        = grayscaleImg.point(lambda x: 0 if x<128 else 255, '1')
    return bwImg, colourImg

def manhattanHeuristic(point, endPoint):
    xDifference = abs(point.x - endPoint.x)
    yDifference = abs(point.y - endPoint.y)
    return xDifference + yDifference

def walkable(node, imgPixelMap):
    if imgPixelMap[node.point.x, node.point.y] == 255:
        return True
    else:
        return False

def adjacentWalkableOpenNodes(centralNode, closedSet, imgPixelMap):
    adjacentNodes = centralNode.adjacentNodes()
    validNodes = []
    for node in adjacentNodes:
        if node.point in closedSet:
            continue
        if not walkable(node, imgPixelMap):
            continue
        validNodes.append(node)

    return validNodes

def solve(imgPath, startPoint, endPoint):
    try:
        bwImg, colourImg = convertBW(imgPath)
    except IOError:
        print imgPath, 'is not a valid image path'
        return
    except (ValueError, TypeError) as e:
        print e
        return

    Node.maxX, Node.maxY = bwImg.size[0] - 1, bwImg.size[1] - 1
    pixelMap = bwImg.load()
    draw = ImageDraw.Draw(colourImg)

    search = AStarSearch()

    startNode = Node(None, startPoint, 0, 0)
    adjacentToStart = adjacentWalkableOpenNodes(startNode,
                                                search.closedSet,
                                                pixelMap)
    for node in adjacentToStart:
        node.heuristicCost = manhattanHeuristic(node.point, endPoint)
        node.movementCost = 10
        node.parent = startNode
        search.openSet.put(deepcopy(node))
        search.openSetMirror.add(deepcopy(node.point))

    # put start point in closed set
    search.closedSet.add(deepcopy(startNode.point))
    search.closedSetMirror.add(deepcopy(startNode))

    while not search.openSet.empty():
        currentNode = search.openSet.get()
        draw.point((currentNode.point.x, currentNode.point.y), 'red')
        search.openSetMirror.remove(currentNode.point)
        search.closedSet.add(deepcopy(currentNode.point))
        search.closedSetMirror.add(deepcopy(currentNode))
        if currentNode.point == endPoint:
            break

        adjacents = adjacentWalkableOpenNodes(currentNode,
                                              search.closedSet,
                                              pixelMap)
        for node in adjacents:
            if not (node.point in search.openSetMirror):
                node.parent        = currentNode
                node.heuristicCost = manhattanHeuristic(node.point, endPoint)
                node.movementCost  = 10 + currentNode.movementCost
                search.openSet.put(deepcopy(node))
                search.openSetMirror.add(deepcopy(node.point))
            else:
                if (10 + currentNode.movementCost) < node.movementCost:
                    node.parent       = currentNode
                    node.movementCost = 10 + currentNode.movementCost

    #pprint.pprint(search.closedSetMirror)
    #pprint.pprint(search.closedSet)
    endNode = None
    for node in search.closedSetMirror:
        if node.point == endPoint:
            endNode = node
            break

    currentNode = endNode
    while True:
        if not currentNode:
            break

        print currentNode.point.x, currentNode.point.y
        draw.point((currentNode.point.x, currentNode.point.y), 'green')
        currentNode = currentNode.parent

    del draw
    colourImg.save(imgPath.split('.')[0] + '-result.png')
    colourImg.show()


################################################################################


if __name__ == "__main__":
    if len(sys.argv) == 1:
        print 'Must call this program with an image path, \
               start point and end point'

    imgPath, startPointX, startPointY, endPointX, endPointY = sys.argv[1:]
    startPoint = Point(int(startPointX), int(startPointY))
    endPoint = Point(int(endPointX), int(endPointY))
    solve(imgPath, startPoint, endPoint)
