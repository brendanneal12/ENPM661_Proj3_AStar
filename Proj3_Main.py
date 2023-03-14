## Brendan Neal and Adam Lobo
## ENPM661 Project 3 A*


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
    actionmoves.append(MoveMaxTurnLeft(Current_Node, Step_Size, Theta), Current_Node, Poss_Move_List[0], Current_Node.ReturnCost() + Step_Size)
    actionmoves.append(MoveTurnLeft(Current_Node, Step_Size, Theta), Current_Node, Poss_Move_List[0], Current_Node.ReturnCost() + Step_Size)
    actionmoves.append(MoveStraight(Current_Node, Step_Size), Current_Node, Poss_Move_List[0], Current_Node.ReturnCost() + Step_Size)
    actionmoves.append(MoveMaxTurnRight(Current_Node, Step_Size, Theta), Current_Node, Poss_Move_List[0], Current_Node.ReturnCost() + Step_Size)
    actionmoves.append(MoveTurnRight(Current_Node, Step_Size, Theta), Current_Node, Poss_Move_List[0], Current_Node.ReturnCost() + Step_Size)

    PossibleMoves = [NewNode for NewNode in actionmoves if NewNode.ReturnState() is not None]

    return PossibleMoves

def Calc_Cost2Go(Current_Node_Position, Goal_Node_Position):
    C2G = 0.0

    if Current_Node_Position is not None:
        C2G = np.sqrt((Goal_Node_Position[0]-Current_Node_Position[0])**2 + (Goal_Node_Position[1]-Current_Node_Position[1])**2)

    return C2G

def CompareToGoal(Current_Node_Position, Goal_Node_Position, Threshold):

    Dist2Goal = (Goal_Node_Position[0] - Current_Node_Position[0])**2 + (Goal_Node_Position[1] - Current_Node_Position[1])**2

    if Dist2Goal < Threshold**2:
        return True
    else:
        return False
    
    


