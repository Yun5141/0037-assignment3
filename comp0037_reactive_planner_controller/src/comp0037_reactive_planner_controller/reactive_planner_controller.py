# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot motion.

import rospy
import threading
from cell import CellLabel
from planner_controller_base import PlannerControllerBase
from comp0037_mapper.msg import *
from comp0037_reactive_planner_controller.aisle import Aisle

class ReactivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)
        
        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.gridUpdateLock =  threading.Condition()
        self.aisleToDriveDown = None

        # Part 2-2
        self.Lw = 2

        # Part 2-3
        self.p = 0.8   

    def mapUpdateCallback(self, mapUpdateMessage):

        # Update the occupancy grid and search grid given the latest map update
        self.gridUpdateLock.acquire()
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()
        self.gridUpdateLock.release()

        # If we are not currently following any route, drop out here.
        if self.currentPlannedPath is None:
            return

        self.checkIfPathCurrentPathIsStillGood()

    def checkIfPathCurrentPathIsStillGood(self):

        # This methods needs to check if the current path, whose
        # waypoints are in self.currentPlannedPath, can still be
        # traversed
                
        # If the route is not viable any more, call
        # self.controller.stopDrivingToCurrentGoal()

        for waypoint in self.currentPlannedPath.waypoints:
            label = self.planner.searchGrid.getCellFromCoords(waypoint.coords).label
            if label == CellLabel.OBSTRUCTED:
                rospy.loginfo("Meet Obstacle")
                self.controller.stopDrivingToCurrentGoal()

    # ------- Part 2-3 -------
    # Choose the first aisle the robot will initially drive down.
    # This is based on the prior.
    def chooseInitialAisle(self, startCellCoords, goalCellCoords):

        pathB = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, Aisle.B)
        pathC = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, Aisle.C)

        self.planner.searchGridDrawer.drawPathGraphicsWithCustomColour(pathB,'yellow')
        self.planner.searchGridDrawer.drawPathGraphicsWithCustomColour(pathC,'blue')

        pathCost_B = pathB.travelCost
        pathCost_C = pathC.travelCost

        # output threshold values and ask for waiting time input
        print("\n---Part 2-3---")
        print('pathCost_B: '+str(pathCost_B))
        print('pathCost_C: '+str(pathCost_C))
        threshold_ExpectedTw = (pathCost_C - pathCost_B) / self.Lw
        threshold_lambdaB = self.p / threshold_ExpectedTw
        print("threshold expected Tw (= p / lambdaB): "+str(threshold_ExpectedTw))
        print("threshold lambdaB: "+str(threshold_lambdaB))
        print("If Tw <= the threshold value then choose aisle B (yellow path), else choose aisle C (blue path).")
        Tw = input("Tw: ")
        if Tw <= threshold_ExpectedTw:
            return Aisle.B
        else:
            return Aisle.C
    
    # ---- end part 2-3 ------

    # Choose the subdquent aisle the robot will drive down
    def chooseAisle(self, startCellCoords, goalCellCoords):
        return Aisle.E

    # ------- Part 2-2 -------
    def getPathCost(self,startCellCoords,goalCellCoords):
        pathFound = self.planner.search(startCellCoords, goalCellCoords)
        path = self.planner.extractPathToGoal()
        return path.travelCost

    # Return whether the robot should wait for the obstacle to clear or not.
    def shouldWaitUntilTheObstacleClears(self, startCellCoords, goalCellCoords):

        # --- remaining original path cost ---
        remainingOriginalPathCost = 0
        oldFullPathCost = self.currentPlannedPath.travelCost
        originalStartCell = self.currentPlannedPath.waypoints[0]
        remainingOriginalPathCost = oldFullPathCost - self.getPathCost(originalStartCell.coords, startCellCoords)

        # --- new path cost ---
        newPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords,self.chooseAisle(startCellCoords,goalCellCoords))
        newPathCost = newPath.travelCost

        # Draw the two paths
        self.planner.searchGridDrawer.drawPathGraphicsWithCustomColour(self.currentPlannedPath,'yellow')
        self.planner.searchGridDrawer.drawPathGraphicsWithCustomColour(newPath,'blue')

        # Calculate and output the threshold values
        print("\n---Part 2-2---")
        print("remaining original Path Cost: "+str(remainingOriginalPathCost))
        print("New Path Cost: "+str(newPathCost))
        threshold_ExpectedTw = (newPathCost - remainingOriginalPathCost) / self.Lw
        threshold_lambdaB = 1.0/threshold_ExpectedTw
        print("threshold expected Tw (= 1.0 / lambdaB): "+str(threshold_ExpectedTw))
        print("threshold lambdaB: "+str(threshold_lambdaB))
        print("If Tw <= the threshold value then wait, else move along the new (blue) path.")
        Tw = input("Tw: ")
        if Tw <= threshold_ExpectedTw:
            return True
        else:
            return False

    # This method will wait until the obstacle has cleared and the robot can move.
    def waitUntilTheObstacleClears(self):

        print("Waiting until the obstacle clear...")

        # find the obstacle cell
        waypoints = self.currentPlannedPath.waypoints
        obstacleCell = waypoints[0]
        for cell in waypoints:
            if self.occupancyGrid.getCell(cell.coords[0], cell.coords[1]) == 1:
                obstacleCell = cell
                break

        # keep checking if the obstacle is clear or not
        while True:
            if self.occupancyGrid.getCell(obstacleCell.coords[0], obstacleCell.coords[1]) == 0:
                break
    
    # ---- end part 2-2 ------

    # ------ Part 2-1 -------
    def getAisleCellCoords(self, aisle):
        if(aisle == Aisle.A):
            return (29,15)      # To enter the aisle from the bottom, the y value must be lower than the mid point which is 38
        elif(aisle == Aisle.B):
            return (43,15)
        elif(aisle == Aisle.C):
            return (58,15)
        elif(aisle == Aisle.D):
            return (74,15)
        elif(aisle == Aisle.E):
            return (88,15)
        else:
            rospy.logerr("Undefined Aisle")
    
    # Plan a path to the goal which will go down the designated aisle. The code, as
    # currently implemented simply tries to drive from the start to the goal without
    # considering the aisle.
    def planPathToGoalViaAisle(self, startCellCoords, goalCellCoords, aisle):

        # Note that, if the robot has waited, it might be tasked to drive down the
        # aisle it's currently on. Your code should handle this case.
        if self.aisleToDriveDown is None:
            self.aisleToDriveDown = aisle

        aisleCellCoords = self.getAisleCellCoords(aisle)

        print("Goal cell coords:" + str(goalCellCoords))
        print("Aisle cell coords:" + str(aisleCellCoords))

        # Implement your method here to construct a path which will drive the robot
        # from the start to the goal via the aisle.

        # get the path 1 which is from the start to the aisle
        pathToAisleFound = self.planner.search(startCellCoords, aisleCellCoords)    
        # If we can't reach the goal, give up and return
        if pathToAisleFound is False:
            rospy.logwarn("Could not find a path to the goal at (%d, %d)", \
                            aisleCellCoords[0], aisleCellCoords[1])
            return None
        # Extract the path
        path1 = self.planner.extractPathToGoal()
        currentPlannedPath = path1

        # get the path 2 which is from the aisle to the goal
        pathFromAisleToGoalFound = self.planner.search(aisleCellCoords, goalCellCoords)    
        # If we can't reach the goal, give up and return
        if pathFromAisleToGoalFound is False:
            rospy.logwarn("Could not find a path to the goal at (%d, %d)", \
                            goalCellCoords[0], goalCellCoords[1])
            return None
        # Extract the path
        currentPlannedPath.addToEnd(self.planner.extractPathToGoal())

        self.planner.searchGridDrawer.drawPathGraphicsWithCustomColour(path1,'yellow')
        self.planner.searchGridDrawer.rectangles[startCellCoords[0]][startCellCoords[1]].setFill('purple');

        return currentPlannedPath

    # ----- end part 2-1 ------

    # This method drives the robot from the start to the final goal. It includes
    # choosing an aisle to drive down and both waiting and replanning behaviour.
    # Note that driving down an aisle is like introducing an intermediate waypoint.

    def driveToGoal(self, goal):

        # Get the goal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Set the start conditions to the current position of the robot
        pose = self.controller.getCurrentPose()
        start = (pose.x, pose.y)
        startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

        # Work out the initial aisle to drive down
        aisleToDriveDown = self.chooseInitialAisle(startCellCoords, goalCellCoords)

        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.
        
        while rospy.is_shutdown() is False:

            # Plan a path from the robot's current position to the goal. This is called even
            # if the robot waited and used its existing path. This is more robust than, say,
            # stripping cells from the existing path.           
            
            print 'Planning a new path: start=' + str(start) + '; goal=' + str(goal)
            
            # Plan a path using the current occupancy grid
            self.gridUpdateLock.acquire()
            self.currentPlannedPath = self.planPathToGoalViaAisle(startCellCoords, goalCellCoords, aisleToDriveDown)
            self.gridUpdateLock.release()

            # If we couldn't find a path, give up
            if self.currentPlannedPath is None:
                return False

            # Drive along the path towards the goal. This returns True
            # if the goal was successfully reached. The controller
            # should stop the robot and return False if the
            # stopDrivingToCurrentGoal method is called.
            goalReached = self.controller.drivePathToGoal(self.currentPlannedPath, \
                                                          goal.theta, self.planner.getPlannerDrawer())

            rospy.logerr('goalReached=%d', goalReached)
            # If we reached the goal, return
            if goalReached is True:
                return True

            # An obstacle blocked the robot's movement. Determine whether we need to
            # wait or replan.

            # Figure out where we are
            pose = self.controller.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)
            # See if we should wait
            waitingGame = self.shouldWaitUntilTheObstacleClears(startCellCoords, goalCellCoords)
            # Depending upon the decision, either wait or determine the new aisle
            # we should drive down.
            if waitingGame is True:
                self.waitUntilTheObstacleClears()
            else:
                aisleToDriveDown = self.chooseAisle(startCellCoords, goalCellCoords)

        return False
            
            
