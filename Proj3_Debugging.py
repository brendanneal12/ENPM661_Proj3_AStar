#------------------------Importing Libraries-------------------------##
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
    
    def ReturnC2C(self):
        return self.C2C
    
    def ReturnTotalCost(self): #Returns the Cost Leading up to the Node
        return self.TotalCost

    
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
    if adjusted_theta < 0:
        adjusted_theta = adjusted_theta + 360

    ChangeX = Step_Size*np.cos(np.deg2rad(adjusted_theta))
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
    if adjusted_theta < 0:
        adjusted_theta = adjusted_theta + 360

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
    if adjusted_theta >= 360:
        adjusted_theta = adjusted_theta - 360
    if adjusted_theta < 0:
        adjusted_theta = adjusted_theta +360


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
    if adjusted_theta < 0:
        adjusted_theta = adjusted_theta +360

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
    if adjusted_theta < 0:
        adjusted_theta = adjusted_theta + 360

    ChangeX = Step_Size*np.cos(np.deg2rad(adjusted_theta))
    ChangeY = Step_Size*np.cos(np.deg2rad(adjusted_theta))

    NewNodeState = [Current_State[0] + ChangeX, Current_State[1] + ChangeY, adjusted_theta]

    if CheckInWorkspace(NewNodeState[0], NewNodeState[1]):
        return None

    # if CheckInObstacle(NewNodeState[0], NewNodeState[1]): ############################################AdjustObstacleCheck
    #     return None
    
    return NewNodeState



def GeneratePossibleMoves(Current_Node, Step_Size, Theta):
    Poss_Move_List = ["MaxTurnLeft", "LeftTurn", "Straight", "RightTurn", "MaxTurnRight"]
    CurrNodeState = Current_Node.ReturnState()
    actionmoves = []
    actionmoves.append(MoveMaxTurnLeft(CurrNodeState, Step_Size, Theta))
    actionmoves.append(MoveTurnLeft(CurrNodeState, Step_Size, Theta))
    actionmoves.append(MoveStraight(CurrNodeState, Step_Size))
    actionmoves.append(MoveMaxTurnRight(CurrNodeState, Step_Size, Theta))
    actionmoves.append(MoveTurnRight(CurrNodeState, Step_Size, Theta))
    
    PossibleMoves = [NewNode for NewNode in actionmoves if NewNode is not None]

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


    if Dist2Goal < Threshold**2 and Current_Node_Position[2] == Goal_Node_Position[2]:
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
    #plt.quiver(ParentNodeState[0], ParentNodeState[1], ParentNodeState[0] - CurrentNodeState[0], ParentNodeState[1]- CurrentNodeState[1] ,units='xy' ,scale=10000 ,color= Color)
    #plt.plot([ParentNodeState[0], CurrentNodeState[0]],[ParentNodeState[1], CurrentNodeState[1]], Color, linewidth = 0.5)
    plt.plot(CurrentNodeState[0], CurrentNodeState[1], marker="o", markersize=2, markeredgecolor=Color, markerfacecolor=Color)
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
Open_List.put((starting_node.ReturnTotalCost(), starting_node)) #Add to Open List

GoalReach = False #Initialze Goal Check Variable



Closed_List= np.array([])#Initialize Closed List of nodes, size of workspace, and setting their cost to infinity to allow for Dijkstra searching.


##-----------------------CONDUCT A*---------------------##

starttime = timeit.default_timer() #Start the Timer when serch starts
print("A* Search Starting!!!!")

while not (Open_List.empty()):
    current_node = Open_List.get()[1] #Grab first (lowest cost) item from Priority Queue.
    print(current_node.ReturnState(), current_node.ReturnTotalCost())
    np.append(Closed_List, current_node.ReturnState())



    goalreachcheck = CompareToGoal(current_node.ReturnState(), GoalState, ThreshGoalState) #Check if we have reached goal.

    if goalreachcheck: #If we have reached goal node.
        print("Goal Reached!")
        print("Total Cost:", current_node.ReturnTotalCost()) #Print Total Cost
        MovesPath, Path = current_node.ReturnPath() #BackTrack to find path.
        for nodes in Path:
            VectorPlotter(nodes.ReturnState(), nodes.ReturnParentState(), 'm')


    else: #If you have NOT reached the goal node
        NewNodeStates = GeneratePossibleMoves(current_node, StepSize, 30)#Generate New Nodes from the possible moves current node can take.
        ParentC2C = current_node.ReturnC2C()
        if NewNodeStates not in Closed_List: #Check to see if the new node position is currently in the closed list
            for State in NewNodeStates: #For each new node generated by the possible moves.
                ChildNode_C2C = ParentC2C + StepSize
                ChildNode_Total_Cost = ChildNode_C2C + Calc_Cost2Go(State, GoalState)
                NewChild = Node(State, current_node, "Move",ChildNode_C2C, ChildNode_Total_Cost)
                if CheckIfVisited(NewChild, node_array, ThreshXY, ThreshTheta) ==  False:
                    node_array[int(Round2Half(NewChild.ReturnState()[1]/ThreshXY)), int(Round2Half(NewChild.ReturnState()[0]/ThreshXY)), int(Round2Half(NewChild.ReturnState()[2]/ThreshTheta))] = 1
                    Open_List.put((NewChild.ReturnTotalCost() , NewChild))

                if CheckIfVisited(NewChild, node_array, ThreshXY, ThreshTheta) ==  True:
                        if NewChild.ReturnTotalCost() > current_node.ReturnC2C() + StepSize:
                            NewChild.parent = current_node
                            NewChild.C2C = current_node.ReturnC2C() + StepSize
                            NewChild.TotalCost = NewChild.ReturnC2C() + Calc_Cost2Go(NewChild.ReturnState(), GoalState)

    if goalreachcheck: #If you reach goal
        break #Break the Loop

stoptime = timeit.default_timer() #Stop the Timer, as Searching is complete.
plt.imshow(Workspace,origin = 'lower')
plt.show()