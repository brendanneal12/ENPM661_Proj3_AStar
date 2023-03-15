## Brendan Neal and Adam Lobo
## ENPM661 Project 3 A*

#STILL NEED:
#OBSTACLES, PLOTTING, VIDEO, CHECK IN WS AND OBSTACLES


##------------------------Importing Libraries-------------------------##
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import math
import timeit
import queue
from queue import PriorityQueue

##------------Defining Node Class (From Previous Lab)-----------------##
class Node():
    #Initializing Function
    def __init__(self, state, parent, move, C2C, TotalCost):
        self.state = state
        self.parent = parent
        self.move = move
        self.C2C = C2C
        self.TotalCost = TotalCost
    #---Methods for this Class---#
    def ReturnState(self): #Returns Node State X and Y
        return self.state
    
    def ReturnMove(self): #Returns the Move of that Node (change from prior node)
        return self.move
    
    def ReturnParent(self): #Returns the Parent Node
        return self.parent
    
    def ReturnParentState(self): #Returns the Parent Node's State
        if self.ReturnParent() is None:
            return None
        return self.ReturnParent().ReturnState()
    
    def ReturnCost(self): #Returns the Cost Leading up to the Node
        return self.TotalCost
    
    def ReturnC2C(self):
        return self.C2C
    
    def __lt__(self, other): #OOP Definition for Less than. Required for Priority Queue.
        return self.TotalCost < other.TotalCost
    

    
    ##--------------BACKTRACKING FUNCTION Integrated into Class--------##
    def ReturnPath(self):
        CompletedMoves = [] #Initialize Move Array
        NodePath = [] #Initialize the Node Path
        CurrentNode = self
        while(CurrentNode.ReturnMove() is not None): #For move that a Node has made
            CompletedMoves.append(CurrentNode.ReturnMove()) #Append the previous move
            NodePath.append(CurrentNode) #Append Node to Path
            CurrentNode = CurrentNode.ReturnParent() #Backtrack to the Parent before repeating Process
        NodePath.append(CurrentNode) #Append the starting point after path is derived.
        NodePath.reverse() #Reverse Order to get front to back path
        CompletedMoves.reverse() #Reverse Order to get front to back path

        return CompletedMoves, NodePath
    

##------------------Defining my Check in Workspace? Function-------##   
def CheckInWorkspace(CurrentX, CurrentY):
    WsX_Extended = 600-1 #Index the workspace back 1 slot for indexing
    WsY_Extended = 250-1 #Index the workspace back 1 slot for indexing
    if (CurrentX > WsX_Extended or int(CurrentX)<1 or int(CurrentY)<1 or CurrentY>WsY_Extended): #If outside of workspace
        return 1 # Logic will be used later!!!
    return 0
##---------------------Mathematically Defining Action Set-------------------------##
def MoveMaxTurnLeft(Current_State, Step_Size, AngleOffset):
    curr_theta = Current_State[2]
    adjusted_theta = curr_theta + 2*AngleOffset
    if adjusted_theta >= 360:
        adjusted_theta = adjusted_theta - 360
    if adjusted_theta <= 0:
        adjusted_theta = adjusted_theta + 180

    ChangeX = float(Step_Size)*np.cos(np.deg2rad(adjusted_theta))
    ChangeY = Step_Size*np.cos(np.deg2rad(adjusted_theta))

    NewNodeState = [Current_State[0] + ChangeX, Current_State[1] + ChangeY, adjusted_theta]

    if CheckInWorkspace(NewNodeState[0], NewNodeState[1]):
        return None

    # if CheckInObstacle(NewNodeState[0], NewNodeState[1]): ############################################AdjustObstacleCheck
    #     return None
    
    return NewNodeState

def MoveTurnLeft(Current_State, Step_Size, AngleOffset):
    curr_theta = Current_State[2]
    adjusted_theta = curr_theta + AngleOffset
    if adjusted_theta >= 360:
        adjusted_theta = adjusted_theta - 360
    if adjusted_theta <= 0:
        adjusted_theta = adjusted_theta + 180

    ChangeX = Step_Size*np.cos(np.deg2rad(adjusted_theta))
    ChangeY = Step_Size*np.cos(np.deg2rad(adjusted_theta))

    NewNodeState = [Current_State[0] + ChangeX, Current_State[1] + ChangeY, adjusted_theta]

    if CheckInWorkspace(NewNodeState[0], NewNodeState[1]):
        return None

    # if CheckInObstacle(NewNodeState[0], NewNodeState[1]): ############################################AdjustObstacleCheck
    #     return None
    
    return NewNodeState

def MoveStraight(Current_State, Step_Size):
    curr_theta = Current_State[2]
    adjusted_theta = curr_theta

    ChangeX = Step_Size*np.cos(np.deg2rad(adjusted_theta))
    ChangeY = Step_Size*np.cos(np.deg2rad(adjusted_theta))

    NewNodeState = [Current_State[0] + ChangeX, Current_State[1] + ChangeY, adjusted_theta]
    if CheckInWorkspace(NewNodeState[0], NewNodeState[1]):
        return None

    # if CheckInObstacle(NewNodeState[0], NewNodeState[1]): ############################################AdjustObstacleCheck
    #     return None
    
    return NewNodeState

def MoveMaxTurnRight(Current_State, Step_Size, AngleOffset):
    curr_theta = Current_State[2]
    adjusted_theta = curr_theta - 2*AngleOffset
    if adjusted_theta >= 360:
        adjusted_theta = adjusted_theta - 360
    if adjusted_theta <= 0:
        adjusted_theta = adjusted_theta +180

    ChangeX = Step_Size*np.cos(np.deg2rad(adjusted_theta))
    ChangeY = Step_Size*np.cos(np.deg2rad(adjusted_theta))

    NewNodeState = [Current_State[0] + ChangeX, Current_State[1] + ChangeY, adjusted_theta]

    if CheckInWorkspace(NewNodeState[0], NewNodeState[1]):
        return None

    # if CheckInObstacle(NewNodeState[0], NewNodeState[1]): ############################################AdjustObstacleCheck
    #     return None
    
    return NewNodeState

def MoveTurnRight(Current_State, Step_Size, AngleOffset):
    curr_theta = Current_State[2]
    adjusted_theta = curr_theta - AngleOffset
    if adjusted_theta >= 360:
        adjusted_theta = adjusted_theta - 360
    if adjusted_theta <= 0:
        adjusted_theta = adjusted_theta + 180

    ChangeX = Step_Size*np.cos(np.deg2rad(adjusted_theta))
    ChangeY = Step_Size*np.cos(np.deg2rad(adjusted_theta))

    NewNodeState = [Current_State[0] + ChangeX, Current_State[1] + ChangeY, adjusted_theta]

    if CheckInWorkspace(NewNodeState[0], NewNodeState[1]):
        return None

    # if CheckInObstacle(NewNodeState[0], NewNodeState[1]): ############################################AdjustObstacleCheck
    #     return None
    
    return NewNodeState

    

##-----------------Defining Possible Moves Based on Theta and Step Size---------------##
def GeneratePossibleMoves(Current_Node, Step_Size, Theta):
    Poss_Move_List = ["MaxTurnLeft", "LeftTurn", "Straight", "RightTurn", "MaxTurnRight"]
    CurrNodeState = Current_Node.ReturnState()
    actionmoves = []
    actionmoves.append(Node(MoveMaxTurnLeft(CurrNodeState, Step_Size, Theta), Current_Node, Poss_Move_List[0], Current_Node.ReturnCost() + Step_Size, Current_Node.ReturnCost() + Step_Size + Calc_Cost2Go(MoveMaxTurnLeft(CurrNodeState, Step_Size, Theta), GoalState)))
    actionmoves.append(Node(MoveTurnLeft(CurrNodeState, Step_Size, Theta), Current_Node, Poss_Move_List[1], Current_Node.ReturnCost() + Step_Size, Current_Node.ReturnCost() + Step_Size + Calc_Cost2Go(MoveTurnLeft(CurrNodeState, Step_Size, Theta), GoalState)))
    actionmoves.append(Node(MoveStraight(CurrNodeState, Step_Size), Current_Node, Poss_Move_List[2], Current_Node.ReturnCost() + Step_Size,Current_Node.ReturnCost() + Step_Size+ Calc_Cost2Go(MoveStraight(CurrNodeState, Step_Size), GoalState)))
    actionmoves.append(Node(MoveMaxTurnRight(CurrNodeState, Step_Size, Theta), Current_Node, Poss_Move_List[3], Current_Node.ReturnCost() + Step_Size, Current_Node.ReturnCost() + Step_Size + Calc_Cost2Go(MoveMaxTurnRight(CurrNodeState, Step_Size, Theta), GoalState)))
    actionmoves.append(Node(MoveTurnRight(CurrNodeState, Step_Size, Theta), Current_Node, Poss_Move_List[4], Current_Node.ReturnCost() + Step_Size, Current_Node.ReturnCost() + Step_Size + Calc_Cost2Go(MoveTurnRight(CurrNodeState, Step_Size, Theta), GoalState)))


    PossibleMoves = [NewNode for NewNode in actionmoves if NewNode.ReturnState() is not None]

    return PossibleMoves

##------------------Defining my Cost to Go Calculation Function------------------------##

def Calc_Cost2Go(Current_Node_Position, Goal_Node_Position):
    C2G = 0.0

    if Current_Node_Position is not None:
        C2G = np.sqrt((Goal_Node_Position[0]-Current_Node_Position[0])**2 + (Goal_Node_Position[1]-Current_Node_Position[1])**2)

    return C2G

##-----------------------Defining my Compare to Goal Function---------------------------##

def CompareToGoal(Current_Node_Position, Goal_Node_Position, Threshold):

    Dist2Goal = (Goal_Node_Position[0] - Current_Node_Position[0])**2 + (Goal_Node_Position[1] - Current_Node_Position[1])**2

    if Dist2Goal < Threshold**2:
        return True
    else:
        return False
    
##-------------------------Defining my Round to Half Function-------------------------##

def Round2Half(number):
    testvalue = np.round(2*number)/2
    if (testvalue == 10):
        testvalue = testvalue - 0.5
    return testvalue

##---------------------------Defining my Check Visited Function-----------------------##
def CheckIfVisited(Current_Node, Node_Array, XYThreshold, ThetaThreshold):
    curr_node_state = Current_Node.ReturnState()
    X = curr_node_state[0]
    Y = curr_node_state[1]
    Theta = curr_node_state[2]
    X = int(Round2Half(X)/XYThreshold)
    Y = int(Round2Half(Y)/XYThreshold)
    Theta = int(Round2Half(Theta)/ThetaThreshold)

    if Node_Array[Y,X,Theta] == 1:
        result = True
    else:
        result = False

    return result

##------------------------Defining my GetInitialState Function-----------------------##
def GetInitialState():
    print("Enter Initial Node X, Y, and Theta separated by spaces: ")
    Init_State=[int(x) for x in input().split()]
    return Init_State


##------------------------Defining my GetGoalState Function--------------------------##
def GetGoalState():
    print("Enter Goal Node X and Y, and Theta separated by spaces: ")
    Goal_State=[int(x) for x in input().split()]
    return  Goal_State

def GetClearance():
    print("Enter Robot Clearance and Robot Radius separated by spaces:")
    Clearance=[int(x) for x in input().split()]
    return  Clearance

def GetStepSize():
    print("Enter Robot Step Size (L = 1 to L = 10)")
    StepSize=int(input())
    return  StepSize

##------------------------Defining my Vector Plotter Function--------------------------##
def VectorPlotter(CurrentNodeState, ParentNodeState, Color):
    #plt.quiver(ParentNodeState[0], ParentNodeState[1], CurrentNodeState[0], CurrentNodeState[1] ,units='xy' ,scale=100 ,color= Color)
    plt.plot([ParentNodeState[0], CurrentNodeState[0]],[ParentNodeState[1], CurrentNodeState[1]], 'm')
    return plt

#----------------Defining my Map Coloring Function---------------##
def WSColoring(Workspace, Location, Color):
    x,_,_ = Workspace.shape #Get Shape of Workspace
    translation_y = Location[0] #Where in Y
    translation_x = x - Location[1] - 1 #Where in X - (Shifts origin from top left to bottom right when plotting!)
    Workspace[translation_x,translation_y,:] = Color #Change the Color to a set Color
    return Workspace  

##---------------------------------MAIN Function---------------------------------------##
SizeAreaX = 600
SizeAreaY = 250
ThreshXY = 0.5
ThreshTheta = 30
ThreshGoalState = 1.5
Angles_Offset = 30
Workspace = np.zeros((SizeAreaY, SizeAreaX,3), dtype = np.uint8) #Initialize the workspace as 0s at first. Integer data type to write to video.
Workspace[:,:] = (0,0,0) #Set all colors to black.
node_array = np.array([[[ 0 for k in range(int(360/ThreshTheta))] for j in range(int(SizeAreaX/ThreshXY))] for i in range(int(SizeAreaY/ThreshXY))])
print(node_array.shape)

InitState = GetInitialState() #Grab Initial State
GoalState = GetGoalState() #Grab Goal State
#Clearance = GetClearance()
StepSize = GetStepSize()

Workspace= WSColoring(Workspace, InitState[0:2], [0,255,0]) #Plot initial state in GREEN on Workspace.
Workspace = WSColoring(Workspace, GoalState[0:2], [0,255,0]) #Plot goal state in GREEN on Workspace.
plt.imshow(Workspace)
plt.show()

Open_List = PriorityQueue() #Initialize list using priority queue.

starting_node_Temp = Node(InitState, None, None, 0, 0) #Generate starting node based on the initial state given above.
starting_node = Node(InitState, starting_node_Temp, None, 0, Calc_Cost2Go(InitState, GoalState)) #Generate starting node based on the initial state given above.
Open_List.put((starting_node.ReturnCost(), starting_node)) #Add to Open List

GoalReach = False #Initialze Goal Check Variable



Closed_List= []#Initialize Closed List of nodes, size of workspace, and setting their cost to infinity to allow for Dijkstra searching.


##-----------------------CONDUCT A*---------------------##

starttime = timeit.default_timer() #Start the Timer when serch starts
print("A* Search Starting!!!!")

while not (Open_List.empty()):
    current_node = Open_List.get()[1] #Grab first (lowest cost) item from Priority Queue.
    print(current_node.ReturnState())
    Closed_List.append(current_node) #Add popped node location to Closed List


    VectorPlotter(current_node.ReturnState(), current_node.ReturnParentState(), 'g')
    goalreachcheck = CompareToGoal(current_node.ReturnState(), GoalState, ThreshGoalState) #Check if we have reached goal.

    if goalreachcheck: #If we have reached goal node.
        print("Goal Reached!")
        print("Total Cost:", current_node.ReturnCost()) #Print Total Cost
        MovesPath, Path = current_node.ReturnPath() #BackTrack to find path.
        for nodes in Path:
            VectorPlotter(nodes.ReturnState(), nodes.ReturnParentState(), 'm')


    else: #If you have NOT reached the goal node
        NewNodes = GeneratePossibleMoves(current_node, StepSize, 30)#Generate New Nodes from the possible moves current node can take.
        if NewNodes not in Closed_List: #Check to see if the new node position is currently in the closed list
            for TestNode in NewNodes: #For each new node generated by the possible moves.
                if CheckIfVisited(TestNode, node_array, ThreshXY, ThreshTheta) == False:
                    node_array[int(Round2Half(TestNode.ReturnState()[1]/ThreshXY)), int(Round2Half(TestNode.ReturnState()[0]/ThreshXY)), int(Round2Half(TestNode.ReturnState()[2]/ThreshTheta))] = 1
                    Open_List.put((TestNode.ReturnCost() , TestNode))

                if CheckIfVisited(TestNode, node_array, ThreshXY, ThreshTheta) == True: #If Visited, Update
                    if TestNode.ReturnCost() > current_node.ReturnCost():
                        TestNode.parent = current_node
                        TestNode.TotalCost = current_node.ReturnCost()

    if goalreachcheck: #If you reach goal
        break #Break the Loop

stoptime = timeit.default_timer() #Stop the Timer, as Searching is complete.
plt.imshow(Workspace, origin = 'lower')
plt.show()
    



