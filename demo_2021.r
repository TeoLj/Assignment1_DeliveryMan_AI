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
  carInfo$mem <- nextPickup(trafficMatrix, 
                              carInfo, 
                              packageMatrix)
  
  # How do we get there?
  carInfo <- nextMove(trafficMatrix,
                        carInfo,
                        packageMatrix)
  return(carInfo)
}

# Find the nearest pickup location for an undelivered package
nextPickup <- function(trafficMatrix, carInfo, packageMatrix) {
  if (is.null(carInfo$mem$goal)){
    carInfo$mem <- nextPickupPackageStartPosition(trafficMatrix, carInfo, packageMatrix)
  }
  else if(carInfo$mem$goal$pos$x == carInfo$x && carInfo$mem$goal$pos$y == carInfo$y){
    if(carInfo$load == 0){
      carInfo$mem$goal <- NULL
    }else{
      carInfo$mem <- nextPickupPackageEndPosition(carInfo, packageMatrix)
    }
    if (is.null(carInfo$mem$goal)){
      carInfo$mem <- nextPickupPackageStartPosition(trafficMatrix, carInfo, packageMatrix)
    }
  }
  return(carInfo$mem)
}

nextPickupPackageStartPosition <- function(trafficMatrix, carInfo, packageMatrix){
  startingNode <- list(pos = list(x = carInfo$x, y = carInfo$y), cost = 0, path = {})
  
  distanceVector = abs(packageMatrix[,1] - carInfo$x) + abs(packageMatrix[,2] - carInfo$y)
  distanceVector[packageMatrix[,5] != 0] = Inf
  
  goalPos <- packageMatrix[which.min(distanceVector), c(1,2)]
  goalCost <- which.min(distanceVector)
  goalNode <- list(pos = list(x = goalPos[1], y = goalPos[2]), cost = goalCost)
  carInfo$mem$goal <- goalNode
  return(carInfo$mem)
}


nextPickupPackageEndPosition <- function(carInfo, packageMatrix){
  goalPos <- c(0,0)
  for (row in 1:nrow(packageMatrix)){
    if(packageMatrix[row,5] == 1){
      #package is loaded
      goalPos <- packageMatrix[row, c(3,4)]
    }
  }
  goalNode <- list(pos = list(x = goalPos[1], y = goalPos[2]), cost = 0)
  carInfo$mem$goal <- goalNode
  return(carInfo$mem)
}

getCost <- function(trafficMatrix, pos){
  return(trafficMatrix[pos[1], pos[2]])
}

getHeuristic <- function(pos, package){
  return(as.numeric(abs(package$x - pos[1]) + abs(package$y - as.numeric(pos[2]))))
}

getF <- function(cost, h){
  return(as.numeric(cost + h))
}

getG <- function(cost, parentCost){
  return(as.numeric(cost+parentCost))
}

# Find the move to get to carInfo$mem$goal
nextMove <- function(trafficMatrix, carInfo, packageMatrix) {
  startingNode <- list(pos = list(x = carInfo$x, y = carInfo$y), cost = 0, path = list(paste(carInfo$x,carInfo$y,sep=",")))
  frontier <- list(startingNode)
  
  #closed list are the expanded nodes
  closed <- list()
  #opened list are nodes in frontier
  opened <- list()
  opened[paste(carInfo$x,carInfo$y,sep=",")] <- list(list(f = 0, c = 0))
  
  goalNode <- carInfo$mem$goal
  
  while(length(frontier) > 0){
    #get current node where the car is and delete it from the frontier and from opened nodes
    scores=sapply(frontier,function(item)item$cost)
    best_index=which.min(scores)
    
    curNode <- frontier[[best_index]]
    frontier <- frontier[-best_index]
    
    curPos <- curNode$pos
    closed <- append(closed, paste(curPos$x,curPos$y,sep=","))
    
    curCost <- opened[[paste(curPos$x,curPos$y,sep=",")]]$c
    curF <- opened[[paste(curPos$x,curPos$y,sep=",")]]$f
    opened[[paste(curPos$x,curPos$y,sep=",")]] <- NULL
    
    #get every neighbor of the current node and compute their cost and add to the frontier
    for(pos in list(c(1,0,'h'),c(-1,0,'h'),c(0,1,'v'),c(0,-1,'v'))){
      newPos <- list(x = curPos$x + as.numeric(pos[1]), y = curPos$y + as.numeric(pos[2]))
      
      if(!isPosInGrid(newPos)){
        next
      }
      
      if(paste(newPos$x,newPos$y,sep=",") %in% closed){
        next
      }
      
      #get cost of newPos based on traffic ->
      #TODO traffic matrices should be checked if the value is retrieved correctly
      if(pos[3] == 'h'){
        if(pos[1] == "1")
          cost <- getCost(trafficMatrix$hroads, c(curPos$x, curPos$y))
        else
          cost <- getCost(trafficMatrix$hroads, c(curPos$x - 1, curPos$y))
      }
      else{
        if(pos[2] == "1")
          cost <- getCost(trafficMatrix$vroads,c(curPos$x, curPos$y))
        else
          cost <- getCost(trafficMatrix$vroads,c(curPos$x, curPos$y - 1))
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
          #TODO update cost in frontier and order it
          #frontier <- frontier[ order( sapply(frontier, "[[", 2) ) ]
        }
        next
      }
      opened[paste(newPos$x,newPos$y,sep=",")] <- list(list(f = f, c = cost))
      newPath <- append(curNode$path, paste(newPos$x,newPos$y,sep=","))
      newNode <- list(pos = list(x = newPos$x, y = newPos$y), cost = f, path = newPath)
      frontier <- append(frontier, list(newNode))
      
      if(newPos$x == goalNode$pos$x && newPos$y == goalNode$pos$y){
        #found the goal node -> end the while cycle and return the first node of the path
        #needs to be at the end of the cycle to get the whole path
        posOfFirstNode <- getPosFromString(newPath[[2]])
        carInfo$nextMove <- getNextMove(posOfFirstNode$x, posOfFirstNode$y, carInfo)
        return(carInfo)
      }
    }
  }
  
  #TODO return the first node
  carInfo$nextMove <- getNextMove(carInfo$x, carInfo$y, carInfo)
  return(carInfo)
}

getPosFromString <- function(text){
  splittedText <- unlist(strsplit(text, split=","))
  x <- as.numeric(splittedText[1])
  y <- as.numeric(splittedText[2])
  return(list(x = x,y = y))
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