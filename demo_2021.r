# File:         demo.r 
# Description:  Naive demo-solution given at classroom session
#               for Project 1, Artificial Intelligence 2019, UU
# Author:       Fredrik Nilsson
# Modified by:  Marcello Vendruscolo (2021)

# Install the package
# install.packages("DeliveryMan_1.1.0.tar.gz", repos = NULL, type="source")

# Load the library
library("DeliveryMan")

myFunction <- function(trafficMatrix, carInfo, packageMatrix) {
  # What is our goal?
  if(carInfo$load == 0 && is.null(carInfo$mem$goal)) {
      carInfo$mem <- nextPickup(trafficMatrix, 
                                     carInfo, 
                                     packageMatrix)
    #} else {
    #  carInfo$mem$goal <- list(packageMatrix[carInfo$load, c(3,4)])#,distance)
  }
  
  # How do we get there?
  carInfo <- nextMove(trafficMatrix,
                               carInfo,
                               packageMatrix)
  return(carInfo)
}

# Find the nearest pickup location for an undelivered package
nextPickup <- function(trafficMatrix, carInfo, packageMatrix) {
  frontier <- list(list(pos = list(x = carInfo$x, y = carInfo$y), cost = 0))
  carInfo$mem$frontier <- frontier
  carInfo$mem$closed <- list()
  carInfo$mem$opened <- list()
  
  
  distanceVector = abs(packageMatrix[,1] - carInfo$x) + abs(packageMatrix[,2] - carInfo$y)
  distanceVector[packageMatrix[,5] != 0] = Inf
  
  goal <- list(as.list(packageMatrix[which.min(distanceVector), c(1,2)]), which.min(distanceVector))
  carInfo$mem$goal <- goal
  carInfo$mem$opened[paste(carInfo$x,carInfo$y,sep=",")] <- list(list(f = which.min(distanceVector), c = 0))
  return(carInfo$mem)
}

getCost <- function(trafficMatrix, pos){
  return(trafficMatrix[pos[1], pos[2]])
}

getHeuristic <- function(pos, package){
  return(abs(package$x - pos[1]) + abs(package$y - as.numeric(pos[2])))
}

getF <- function(cost, h){
  return(cost + h)
}

getG <- function(cost, parentCost){
  return(cost+parentCost)
}

# Find the move to get to carInfo$mem$goal
nextMove <- function(trafficMatrix, carInfo, packageMatrix) {
  frontier <- list(list(pos = list(x = carInfo$x, y = carInfo$y), cost = 0))
  
  #closed list are the expanded nodes
  closed <- list()
  #opened list are nodes in frontier
  opened <- list()
  
  goalNode <- carInfo$mem$goal
  
  while(length(frontier) > 0){
    #get current node where the car is and delete it from the frontier and from opened nodes
    curNode <- frontier[[1]]
    curPosx <- curNode$pos$x
    curPosy <- curNode$pos$y
    closed <- append(closed, paste(curPosx,curPosy,sep=","))
  
    curCost <- opened[[paste(curPosx,curPosy,sep=",")]]$c
    curF <- opened[[paste(curPosx,curPosy,sep=",")]]$f
    opened[[paste(curPosx,curPosy,sep=",")]] <- NULL
    frontier <- frontier[-1]
    
    for(pos in list(c(1,0,'h'),c(-1,0,'h'),c(0,1,'v'),c(0,-1,'v'))){
      newPos <- list(x = carInfo$x + as.numeric(pos[1]), y = carInfo$y + as.numeric(pos[2]))
      
      if(!isPosInGrid(newPos)){
        next
      }
      
      if(paste(newPos$x,newPos$y,sep=",") %in% closed){
        next
      }
      
      if(newPos$x == goalNode$x && newPos$y == goalNode$y){
        #found the goal node -> end the while cycle and return the first node of the path
      }
      
      #get cost of newPos based on traffic ->
      #TODO traffic matrices should be checked if the value is retrieved correctly
      if(pos[3] == 'h'){
        if(pos[1] == "1")
          cost <- getCost(trafficMatrix$hroads, c(carInfo$x, carInfo$y))
        else
          cost <- getCost(trafficMatrix$hroads, c(carInfo$x + newPos$x, carInfo$y))
      }
      else{
        if(pos[2] == "1")
          cost <- getCost(trafficMatrix$vroads,c(carInfo$x, carInfo$y))
        else
          cost <- getCost(trafficMatrix$vroads,c(carInfo$x, carInfo$y + newPos$y))
      }
      
      g <- getG(cost, curCost)
      h <- getHeuristic(unlist(goalNode[1]), newPos)
      f <- getF(g, h)
      
      #if we can already get to this node with lower cost -> continue, otherwise update
      if(paste(newPos$x,newPos$y,sep=",") %in% opened)
      {
        if(opened[paste(newPos$x,newPos$y,sep=",")]$c >= cost){
          tempCost <- opened[paste(newPos$x,newPos$y,sep=",")]
          opened[paste(newPos$x,newPos$y,sep=",")] <- list(f = tempCost$f, c = cost)
          #TODO update cost in frontier
          #frontier <- frontier[order(sapply(frontier, `[[`, i=2))]
        }
        next
      }
      opened[paste(newPos$x,newPos$y,sep=",")] <- list(list(f = f, c = cost))
      frontier <- append(frontier, list(list(pos = list(x = newPos$x, y = newPos$y), cost = f)))
    }
    frontier <- frontier[ order( sapply(frontier, "[[", 2) ) ]
  }
  
  #TODO return the first node
  carInfo$nextMove <- getNextMove(curPosx, curPosy, carInfo)
  return(carInfo)
}

getNextMove <- function(curPosx, curPosy, carInfo){
  if(carInfo$x < curPosx) {
    return(6)
  } else if (carInfo$x > curPosx) {
    return(4)
  } else if (carInfo$y < curPosy) {
    return(8)
  } else if (carInfo$y > curPosy) {
    return(2)
  } else {
    return(5)
  }
}

isPosInGrid <- function(newPos){
  if(newPos[1] < 1 || newPos[2] < 1 || newPos[1] > 10 || newPos[2] > 10){
    return(FALSE)
  }
  return(TRUE)
}