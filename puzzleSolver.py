from _heapq import heappush, heappop
import numpy
import sys

from numpy import math

from datetime import datetime
from astropy.units import count


# #function required for tile manipulation
def printBoard(board):

    print("")
    for row in board:
        row_str = ""
        for cell in row:
            row_str += str(cell) + " "
        print(row_str)
        
        
def possibleMoves(board):

    global moves
    x, y = findGap(board)

    res = []
    for mv in moves:
        x2, y2 = nextPos(x, y, mv)
        if isPositionLegal(board, x2, y2):
            res.append(mv)

    return res


moves = [[-1, 0], [0, 1], [1, 0], [0, -1]]
checkaction = ['U', 'R', 'D', 'L', 's']


def isPositionLegal(board, x, y):
    n = len(board)
    return ((x >= 0) and (x < n) and (y >= 0) and (y < n))


def nextPos(x, y, move):
    nextX = x + move[0]
    nextY = y + move[1]

    return nextX, nextY


def canMove(board, direction):

    mv = moves[direction]
    x, y = findGap(board)
    x2, y2 = nextPos(x, y, mv)

    return isPositionLegal(board, x2, y2)


def moveGap(board, move):
    f = []
    for i in range(len(board)):
        f.append([])
        for j in range(len(board[i])):
            f[i].append(board[i][j])
            
    x, y = findGap(board)
    x2, y2 = nextPos(x, y, move)

    tmp = f[x][y]
    f[x][y] = f[x2][y2]
    f[x2][y2] = tmp
    # #change
    return f


def findGap(board):
    for i in range(len(board)):
        for j in range(len(board[i])):
            if board[i][j] == 0:
                return i, j
    return -1, -1


def checkIfgoal (boardCheck):
    for i in range(n):
        for j in range(n):
            if (goalState[i][j] != boardCheck[i][j]):
                return False
    return True 


# # calculate hamming distance/Misplaced Tiles
def calculateHammingDist(board):
    countH = 0
    for i in range(n):
        for j in range(n):
            if goalState[i][j] != board[i][j] :
                countH = countH + 1
    # #print("countH")
    # #print(countH)
    return countH


# # Calculate manhattan distance
def calculateManhattan(board):
    value = 0
    for i in range(n):
        for j in range(n):
            if(board[i][j] != 0):
                targetX = int((board[i][j] - 1) / n)
                targetY = (board[i][j] - 1) % n
                x = i - targetX
                y = j - targetY
                value = value + abs(x) + abs(y)
    return value


# # Check if record is present in explore
def isInExplored(board):
    # ## implement this
    if(len(Explored) != 0):
        for i in range(len(Explored)):
            if(numpy.array_equal(Explored[i]['board'], board)):
                return True;
    
    return False


# #implementation of priority queue
class MyPriorityEntry(object):
    f = 0;
    board = {}

    def __init__(self, recordForQueue):
        
        self.recordForQueue = recordForQueue

    def __lt__(self, other):
        return self.recordForQueue['f'] < other.recordForQueue['f']


def findPath(goal):
    path = []
    countcheck = 0
    print("goal achieved")
    stateCheck = goal['board']
    ##print(checkaction[goal['action']])
    path.append(checkaction[goal['action']])
    countcheck = countcheck + 1
    for i in range(len(Explored), 1, -1):
        # #print(i)
        # #print(len(Explored))
        # #print(Explored)
        checkposs = possibleMoves(stateCheck)
        
        for movstr in checkposs:
            new = moveGap(stateCheck, movstr)
            if(numpy.array_equal(Explored[i - 1]['board'], new)):
                countcheck = countcheck + 1
                # #print('Action')
                # #print(checkaction[Explored[i - 1]['action']])
                path.append(checkaction[Explored[i - 1]['action']])
                stateCheck = Explored[i - 1]['board']
    print("path lenght")
    print(len(path))
    outputFile = open(outputFileName, "w")
    for j in range(len(path), 0, -1):
        outputFile.write(path[j - 1])
        if(j!=1):
            outputFile.write(",")
    outputFile.close()      

   
def aStartAlgorithm(myBoard):
    if(heuristic == 2):
        hforCurrentState = calculateHammingDist(myBoard)
        # #print(" hamming distance heuristic ")
        # #print(hforCurrentState)
        
    else:
        if(heuristic == 1):
            hforCurrentState = calculateManhattan(myBoard)
            # #print(" manhattan distance heuristic ")
            # #print(hforCurrentState)
    recordForQueue = {'f':0, 'g':0, 'h':hforCurrentState, 'board':myBoard , 'action' :4}
    # #print("data strucutre for record")    
    # #print(recordForQueue)
    # #print("------------------------")
    # ## root node to frontier to furthe explore
    heappush(frontier, MyPriorityEntry(recordForQueue))
    # #print("frontier priority queue")
    # #print(frontier[0].recordForQueue)
    
    while (len(frontier) != 0):
        # #print("---------");
        current = heappop(frontier).recordForQueue
        # #print(current)
        currentBoard = current['board']
        
    # #    print(" board state")
    # #    print(currentBoard)
       
        if(checkIfgoal(currentBoard)):
            print(currentBoard)
            print(" goal achieved")
            findPath(current)
            break
    # #    print(isInExplored(currentBoard))
        if(not isInExplored(currentBoard)):
            ##print("appending in explored")
            ##print(current)
            Explored.append(current)
            # #print("action")
            # #print(current['action'])
            # ## check possible moves
            possibleMovesArray = possibleMoves(currentBoard)
    # #        print("Possible moves...")
    # #        print(possibleMovesArray)
            # #print("----------------------------------------------------")
            for moveRestr in possibleMovesArray:            
                new = moveGap(currentBoard, moveRestr)
                
                # # calculate new distances
                if(heuristic == 2):
                    hforCurrentState = calculateHammingDist(new)
                    # #print(" hamming distance heuristic ")
                    # #print(hforCurrentState)
                else:
                    if(heuristic == 1):
                        hforCurrentState = calculateManhattan(new)
                        # #print(" manhattan distance heuristic ")
                        # #print(hforCurrentState)
                           
                recordForQueue = {'f':0, 'g':0, 'h':hforCurrentState, 'board':new, 'action' :moves.index(moveRestr)}
                recordForQueue['g'] = current['g'] + 1
                recordForQueue['f'] = recordForQueue['g'] + recordForQueue['h']
                # #print("data strucutre for record")    
                # #print(recordForQueue)
                
                heappush(frontier, MyPriorityEntry(recordForQueue))

    
# # Check if record is present in explore
def isInPath(path, board):
    # ## implement this
    if(len(path) != 0):
        for i in range(len(path)):
            if(numpy.array_equal(path[i]['board'], board)):
                return True;
    
    return False


bound = 0
global count

def IDAStart(rootNode):
    global count
    count=0
    print(rootNode['board'])
    path = []
    if(heuristic == 2):
         bound = calculateHammingDist(rootNode['board'])
    else:
        if(heuristic == 1):
            bound = calculateManhattan(rootNode['board'])
    path.append(rootNode)
    
    while(1):
        
        ##print("path")
        ##print(path)
        temp = search(path, 0, bound)
        if (temp == "FOUND"):
            return path, bound
        if (temp == -1):
            return "NOT FOUND"
        bound = temp

        
def search(path, g, bound):
    global count
    count= count +1
    node = path[len(path) - 1]
    if(heuristic == 2):
        f = g + calculateHammingDist(node['board'])
    else:
        if(heuristic == 1):
            f = g + calculateManhattan(node['board'])
    
    if(f > bound):
        return f
    if(checkIfgoal(node['board'])):
        print(len(path))
        return "FOUND"
    minimu = math.inf
    possibleMovesArray = possibleMoves(node['board'])
    for moveRestr in possibleMovesArray:            
        new = moveGap(node['board'], moveRestr)
        newRecord = {'f':0, 'g':0, 'h':0, 'board':new, 'action' :moves.index(moveRestr)}
        if(isInPath(path, new) != True):
            path.append(newRecord)
            ##print("path")
            ##print(path)
            temp = search(path, g + 1, bound)
            if (temp == "FOUND"):
                return "FOUND"
            if(temp < minimu):
                minimu = temp
            path.pop()
    return minimu        


frontier = []  # # priority queue

Explored = []
# ## generating goal state

goalState = []
heuristic = 0
outputFile = ""
if __name__ == "__main__":
    print(str(sys.argv[1]))
    outputFileName = sys.argv[5]
    if(int(sys.argv[1]) == 1):
        
        if(int(sys.argv[2]) == 3):
            n = 3
            data = open(sys.argv[4], "r")
            for i in range(n):
                goalState.append([])
                for j in range(n):
                    if (n * i + j + 1) == n * n:
                        goalState[i].append(0)
                    else:
                        goalState[i].append(n * i + j + 1)
            print("Goal state")
            print(goalState)
            printBoard(goalState)
            
            # ## Read matrix from filedata = open("priority_dict.txt","r")
            
            lines = data.readlines()
            myBoard = []
            for i in range(n):
                line = lines[i].split(",")
                
                for j in range(len(line)):
                    ##print(i, j)
                    if(line[j] == ""):
                        line[j] = 0
                    else:
                        if(line[j] != "\n"):
                            line[j] = int(line[j])
                        else:
                            line[j] = 0
                        
                myBoard.append(line)
                            
            print("Available puzzle problem")    
            print(myBoard)
            printBoard(myBoard)
            # #aStartAlgorithm(myBoard)
        else: 
            if(int(sys.argv[2]) == 4):
                # # done reading file
                n = 4
                data = open(sys.argv[4], "r")
                for i in range(n):
                    goalState.append([])
                    for j in range(n):
                        if (n * i + j + 1) == n * n:
                            goalState[i].append(0)
                        else:
                            goalState[i].append(n * i + j + 1)
                print("Goal state")
                print(goalState)
                printBoard(goalState)
                
                # ## Read matrix from filedata = open("priority_dict.txt","r")
                
                lines = data.readlines()
                myBoard = []
                for i in range(n):
                    line = lines[i].split(",")
                    
                    for j in range(len(line)):
                        ##print(i, j)
                        if(line[j] == ""):
                            line[j] = 0
                        else:
                            if(line[j] != "\n"):
                                line[j] = int(line[j])
                            else:
                                line[j] = 0
                            
                    myBoard.append(line)
                                
                print("Available puzzle problem")    
                print(myBoard)
                printBoard(myBoard)
        heuristic = int(sys.argv[3])
        
        ts1 = datetime.utcnow()
        aStartAlgorithm(myBoard)
        
        te1 = datetime.utcnow()
        t = te1 - ts1
        
        # #timerequired=te-ts
        print("heuristic")
        print(heuristic)
        print("timerequired")
        print(float("0." + str(t).split(".")[1]) * 1000)
        print("EXplored size")
        print(len(Explored))
    
               # ## form dict for the first record
    else :
        if (int(sys.argv[1]) == 2):
            if(int(sys.argv[2]) == 3):
                n = 3
                data = open(sys.argv[4], "r")
                for i in range(n):
                    goalState.append([])
                    for j in range(n):
                        if (n * i + j + 1) == n * n:
                            goalState[i].append(0)
                        else:
                            goalState[i].append(n * i + j + 1)
                print("Goal state")
                print(goalState)
                printBoard(goalState)
                
                # ## Read matrix from filedata = open("priority_dict.txt","r")
                
                lines = data.readlines()
                myBoard = []
                for i in range(n):
                    line = lines[i].split(",")
                    
                    for j in range(len(line)):
                        ##print(i, j)
                        if(line[j] == ""):
                            line[j] = 0
                        else:
                            if(line[j] != "\n"):
                                line[j] = int(line[j])
                            else:
                                line[j] = 0
                            
                    myBoard.append(line)
                                
                print("Available puzzle problem")    
                print(myBoard)
                printBoard(myBoard)
                # #aStartAlgorithm(myBoard)
            else: 
                if(int(sys.argv[2]) == 4):
                    # # done reading file
                    n = 4
                    data = open(sys.argv[4], "r")
                    for i in range(n):
                        goalState.append([])
                        for j in range(n):
                            if (n * i + j + 1) == n * n:
                                goalState[i].append(0)
                            else:
                                goalState[i].append(n * i + j + 1)
                    print("Goal state")
                    print(goalState)
                    printBoard(goalState)
                    
                    # ## Read matrix from filedata = open("priority_dict.txt","r")
                    
                    lines = data.readlines()
                    myBoard = []
                    for i in range(n):
                        line = lines[i].split(",")
                        
                        for j in range(len(line)):
                            ##print(i, j)
                            if(line[j] == ""):
                                line[j] = 0
                            else:
                                if(line[j] != "\n"):
                                    line[j] = int(line[j])
                                else:
                                    line[j] = 0
                                
                        myBoard.append(line)
                                    
                    print("Available puzzle problem")    
                    print(myBoard)
                    printBoard(myBoard)
            heuristic = int(sys.argv[3])
            
            newRecord = {'f':0, 'g':0, 'h':0, 'board':myBoard, 'action' :4}
            ts1 = datetime.utcnow()
            path, bound = IDAStart(newRecord)
            te1 = datetime.utcnow()
            t = te1 - ts1
            print("heuristic")
            print(heuristic)
            print("path length")
            print(len(path)-1)
            print("Explored")
            print(count)
            print("timerequired")
            print(float("0." + str(t).split(".")[1]) * 1000)
            
            outputFile = open(outputFileName, "w")
            for i in range(len(path) - 1):
                outputFile.write(checkaction[path[i + 1]['action']])
                if(i != len(path)-2):
                    outputFile.write(",")
            outputFile.close() 
        
