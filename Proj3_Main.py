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
    def __init__(self, state, parent, move, TotalCost):
        self.state = state
        self.parent = parent
        self.move = move
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
    
    def __lt__(self, other): #OOP Definition for Less than. Required for Priority Queue.
        return self.TotalCost < other.TotalCost
    
    def __gt__(self, other): #OOP Definition for Greater than.
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
    
##---------------------Mathematically Defining Action Set-------------------------##
def MoveMaxTurnLeft(Current_State, Step_Size, robot_angle):
    curr_theta = Current_State[2]
    adjusted_theta = curr_theta + 2*robot_angle
    if adjusted_theta >= 360:
        adjusted_theta = adjusted_theta - 360

    ChangeX = Step_Size*np.cos(np.deg2rad(adjusted_theta))
    ChangeY = Step_Size*np.cos(np.deg2rad(adjusted_theta))

    NewNodeState = [Current_State[0] + ChangeX, Current_State[1] + ChangeY, adjusted_theta]

    if CheckInObstacle(NewNodeState[0], NewNodeState[1]): ############################################AdjustObstacleCheck
        return None
    
    return NewNodeState

def MoveTurnLeft(Current_State, Step_Size, robot_angle):
    curr_theta = Current_State[2]
    adjusted_theta = curr_theta + robot_angle
    if adjusted_theta >= 360:
        adjusted_theta = adjusted_theta - 360

    ChangeX = Step_Size*np.cos(np.deg2rad(adjusted_theta))
    ChangeY = Step_Size*np.cos(np.deg2rad(adjusted_theta))

    NewNodeState = [Current_State[0] + ChangeX, Current_State[1] + ChangeY, adjusted_theta]

    if CheckInObstacle(NewNodeState[0], NewNodeState[1]): ############################################AdjustObstacleCheck
        return None
    
    return NewNodeState

def MoveStraight(Current_State, Step_Size):
    curr_theta = Current_State[2]
    adjusted_theta = curr_theta

    ChangeX = Step_Size*np.cos(np.deg2rad(adjusted_theta))
    ChangeY = Step_Size*np.cos(np.deg2rad(adjusted_theta))

    NewNodeState = [Current_State[0] + ChangeX, Current_State[1] + ChangeY, adjusted_theta]

    if CheckInObstacle(NewNodeState[0], NewNodeState[1]): ############################################AdjustObstacleCheck
        return None
    
    return NewNodeState

def MoveMaxTurnRight(Current_State, Step_Size, robot_angle):
    curr_theta = Current_State[2]
    adjusted_theta = curr_theta - 2*robot_angle
    if adjusted_theta >= 360:
        adjusted_theta = adjusted_theta - 360

    ChangeX = Step_Size*np.cos(np.deg2rad(adjusted_theta))
    ChangeY = Step_Size*np.cos(np.deg2rad(adjusted_theta))

    NewNodeState = [Current_State[0] + ChangeX, Current_State[1] + ChangeY, adjusted_theta]

    if CheckInObstacle(NewNodeState[0], NewNodeState[1]): ############################################AdjustObstacleCheck
        return None
    
    return NewNodeState

def MoveTurnRight(Current_State, Step_Size, robot_angle):
    curr_theta = Current_State[2]
    adjusted_theta = curr_theta - robot_angle
    if adjusted_theta >= 360:
        adjusted_theta = adjusted_theta - 360

    ChangeX = Step_Size*np.cos(np.deg2rad(adjusted_theta))
    ChangeY = Step_Size*np.cos(np.deg2rad(adjusted_theta))

    NewNodeState = [Current_State[0] + ChangeX, Current_State[1] + ChangeY, adjusted_theta]

    if CheckInObstacle(NewNodeState[0], NewNodeState[1]): ############################################AdjustObstacleCheck
        return None
    
    return NewNodeState

    

##-----------------Defining Possible Moves Based on Theta and Step Size---------------##
def GeneratePossibleMoves(Current_Node, Step_Size, Theta):
    Poss_Move_List = ["MaxTurnLeft", "LeftTurn", "Straight", "RightTurn", "MaxTurnRight"]
    CurrNodeState = Current_Node.ReturnState()
    actionmoves = []
    actionmoves.append(MoveMaxTurnLeft(CurrNodeState, Step_Size, Theta), Current_Node, Poss_Move_List[0], Current_Node.ReturnCost() + Step_Size)
    actionmoves.append(MoveTurnLeft(CurrNodeState, Step_Size, Theta), Current_Node, Poss_Move_List[0], Current_Node.ReturnCost() + Step_Size)
    actionmoves.append(MoveStraight(CurrNodeState, Step_Size), Current_Node, Poss_Move_List[0], Current_Node.ReturnCost() + Step_Size)
    actionmoves.append(MoveMaxTurnRight(CurrNodeState, Step_Size, Theta), Current_Node, Poss_Move_List[0], Current_Node.ReturnCost() + Step_Size)
    actionmoves.append(MoveTurnRight(CurrNodeState, Step_Size, Theta), Current_Node, Poss_Move_List[0], Current_Node.ReturnCost() + Step_Size)

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
def CheckIfVisited(Current_Node, Node_Array, Goal_State, Threshold):
    curr_node_state = Current_Node.ReturnState()
    X = curr_node_state[0]
    Y = curr_node_state[1]
    Theta = curr_node_state[2]
    X = int(Round2Half(X)/Threshold)
    Y = int(Round2Half(Y)/Threshold)

    if (Current_Node.ReturnCost() + Calc_Cost2Go(curr_node_state, Goal_State) < Node_Array[X,Y,Theta]):
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
    StepSize=[int(x) for x in input().split()]
    return  StepSize

    
##---------------------------------MAIN Function---------------------------------------##
SizeAreaX = 600
SizeAreaY = 250
ThreshXY = 0.5
ThreshTheta = 30
ThreshGoalState = 1.5
Workspace = np.zeros((SizeAreaY, SizeAreaX,3), dtype = np.uint8) #Initialize the workspace as 0s at first. Integer data type to write to video.
Workspace[:,:] = (0,0,0) #Set all colors to black.
node_array = np.array([[[ math.inf for k in range(int(360/ThreshTheta))] for j in range(int(SizeAreaX/ThreshXY))] for i in range(int(SizeAreaY/ThreshXY))])
print(node_array.shape)

InitState = GetInitialState() #Grab Initial State
GoalState = GetGoalState() #Grab Goal State
Clearance = GetClearance()
StepSize = GetStepSize()

Open_List = PriorityQueue() #Initialize list using priority queue.
starting_node = Node(InitState, None, None, 0) #Generate starting node based on the initial state given above.
Open_List.put((starting_node.ReturnCost(), starting_node)) #Add to Open List
GoalReach = False #Initialze Goal Check Variable


Closed_List= []#Initialize Closed List of nodes, size of workspace, and setting their cost to infinity to allow for Dijkstra searching.


##-----------------------CONDUCT A*---------------------##

starttime = timeit.default_timer() #Start the Timer when serch starts
print("A* Search Starting!!!!")

while not (Open_List.empty()):
    current_node = Open_List.get()[1] #Grab first (lowest cost) item from Priority Queue.
    Closed_List.append(current_node) #Add popped node location to Closed List

    goalreachcheck = CompareToGoal(current_node.ReturnState(), GoalState, ThreshGoalState) #Check if we have reached goal.

    if goalreachcheck: #If we have reached goal node.
        print("Goal Reached!")
        print("Total Cost:", current_node.ReturnCost()) #Print Total Cost
        MovesPath, Path = current_node.ReturnPath() #BackTrack to find path.

    else: #If you have NOT reached the goal node


        NewNodes = GeneratePossibleMoves(current_node, StepSize, current_node.ReturnState()[2]) #Generate New Nodes from the possible moves current node can take.
        if NewNodes not in Closed_List: #Check to see if the new node position is currently in the closed list
            for move in NewNodes: #For each new node generated by the possible moves.
                TestNode = Node(move[0], move[1], move[2], move[3])
                if CheckIfVisited(TestNode, node_array, GoalState, ThreshXY):
                    node_array[int(Round2Half(TestNode.returnState()[0]/ThreshXY)), int(Round2Half(TestNode.returnState()[1]/ThreshXY)), TestNode.ReturnState()[2]] = TestNode.ReturnCost() + Calc_Cost2Go(TestNode.returnState(), GoalState)
                    Open_List.put((TestNode.ReturnCost() + Calc_Cost2Go(TestNode.returnState(), GoalState)), TestNode)
        if goalreachcheck: #If you reach goal
            break #Break the Loop

stoptime = timeit.default_timer() #Stop the Timer, as Searching is complete.
    



