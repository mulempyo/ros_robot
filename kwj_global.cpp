#include "kwj_global_planner/kwj_global.h"


//cost of non connected nodes
float infinity = std::numeric_limits<float>::infinity();
int mapSize = 0;
int value = 0;
bool *occupancyGridMap = nullptr;
float tBreak = 0;

namespace kwj
{
  Kwj::Kwj()
  {
  width = costmap_->getSizeInCellsX();
  goal[0] = goal[1] = 0;
  start[0] = start[1] = 0;
  }

  void Kwj::setGoal(int *g)
    {
      goal[0] = g[0];
      goal[1] = g[1];
    }

  void Kwj::setStart(int *g)
    {
      start[0] = g[0];
      start[1] = g[1];
    }


vector<int> Kwj::runAStarOnGrid(int startGridSquare, int goalGridSquare)
{

  vector<int> bestPath;

  float g_score[mapSize];

  for (uint i = 0; i < mapSize; i++)
    g_score[i] = infinity;

  bestPath = findPath(startGridSquare, goalGridSquare, g_score);

  return bestPath;
}

/**

  Generates the path for the bot towards the goal

**/
vector<int> Kwj::findPath(int startGridSquare, int goalGridSquare, float g_score[])
{
  value++;
  vector<int> bestPath;
  vector<int> emptyPath;
  cell gridSq;

  multiset<cell> openSquaresList;
  int currentGridSquare;

  //calculate g_score and f_score of the start position
  g_score[startGridSquare] = 0;
  gridSq.currentCell = startGridSquare;
  gridSq.fCost = g_score[startGridSquare] + calculateHScore(startGridSquare, goalGridSquare);

  //add the start gridSquare to the open list
  openSquaresList.insert(gridSq);
  currentGridSquare = startGridSquare;

  //while the open list is not empty and till goal square is reached continue the search
  while (!openSquaresList.empty() && g_score[goalGridSquare] == infinity)
  {
    //choose the gridSquare that has the lowest cost fCost in the open set
    currentGridSquare = openSquaresList.begin()->currentCell;
    //remove that gridSquare from the openList
    openSquaresList.erase(openSquaresList.begin());
    //search the neighbors of that gridSquare
    vector<int> neighborGridSquares;
    neighborGridSquares = findFreeNeighborGridSquare(currentGridSquare);
    for (uint i = 0; i < neighborGridSquares.size(); i++) //for each neighbor v of gridSquare
    {
      // if the g_score of the neighbor is equal to INF: unvisited gridSquare
      if (g_score[neighborGridSquares[i]] == infinity)
      {
        g_score[neighborGridSquares[i]] = g_score[currentGridSquare] + getMoveCost(currentGridSquare, neighborGridSquares[i]);
        addNeighborGridSquareToOpenList(openSquaresList, neighborGridSquares[i], goalGridSquare, g_score);
      }
    }
  }

  if (g_score[goalGridSquare] != infinity) // if goal gridSquare has been reached
  {
    bestPath = constructPath(startGridSquare, goalGridSquare, g_score);
    return bestPath;
  }
  else
  {
    ROS_INFO("Failure to find a path !");
    return emptyPath;
  }
}

/**
  Function constructs the path found by findPath function by returning vector of
  gridSquare indices that lie on path.

**/
vector<int> Kwj::constructPath(int startGridSquare, int goalGridSquare, float g_score[])
{
  vector<int> bestPath;
  vector<int> path;

  path.insert(path.begin() + bestPath.size(), goalGridSquare);
  int currentGridSquare = goalGridSquare;

  while (currentGridSquare != startGridSquare)
  {
    vector<int> neighborGridSquares;
    neighborGridSquares = findFreeNeighborGridSquare(currentGridSquare);

    vector<float> gScoresNeighbors;
    for (uint i = 0; i < neighborGridSquares.size(); i++)
      gScoresNeighbors.push_back(g_score[neighborGridSquares[i]]);

    int posMinGScore = distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
    currentGridSquare = neighborGridSquares[posMinGScore];

    //insert the neighbor in the path
    path.insert(path.begin() + path.size(), currentGridSquare);
  }
  for (uint i = 0; i < path.size(); i++)
    bestPath.insert(bestPath.begin() + bestPath.size(), path[path.size() - (i + 1)]);

  return bestPath;
}

/**
  Helper function to add unexplored neighbours of currentGridSquare to openlist
**/
void Kwj::addNeighborGridSquareToOpenList(multiset<cell> &openSquaresList, int neighborGridSquare, int goalGridSquare, float g_score[])
{
  cell gridSq;
  gridSq.currentCell = neighborGridSquare; //insert the neighborGridSquare
  gridSq.fCost = g_score[neighborGridSquare] + calculateHScore(neighborGridSquare, goalGridSquare);
  openSquaresList.insert(gridSq);
}

/**
  Helper function to find free neighbours of currentGridSquare 
**/

vector<int> Kwj::findFreeNeighborGridSquare(int gridSquare)
{

  int rowIndex = getGridSquareRowIndex(gridSquare);
  int colIndex = getGridSquareColIndex(gridSquare);
  int neighborIndex;
  vector<int> freeNeighborGridSquares;

  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++)
    {
      //check whether the index is valid
      if ((rowIndex + i >= 0) && (rowIndex + i < costmap_->getSizeInCellsY()) && (colIndex + j >= 0) && (colIndex + j < width) && (!(i == 0 && j == 0)))
      {
        neighborIndex = ((rowIndex + i) * width) + (colIndex + j);

        if (isFree(neighborIndex))
          freeNeighborGridSquares.push_back(neighborIndex);
      }
    }
  return freeNeighborGridSquares;
}

/**
  Checks if start and goal positions are valid and not unreachable.
**/
bool Kwj::isStartAndGoalValid(int startGridSquare, int goalGridSquare)
{
  bool isvalid = true;
  bool isFreeStartGridSquare = isFree(startGridSquare);
  bool isFreeGoalGridSquare = isFree(goalGridSquare);
  if (startGridSquare == goalGridSquare)
  {

    isvalid = false;
  }
  else
  {
    if (!isFreeStartGridSquare && !isFreeGoalGridSquare)
    {

      isvalid = false;
    }
    else
    {
      if (!isFreeStartGridSquare)
      {

        isvalid = false;
      }
      else
      {
        if (!isFreeGoalGridSquare)
        {

          isvalid = false;
        }
        else
        {
          if (findFreeNeighborGridSquare(goalGridSquare).size() == 0)
          {

            isvalid = false;
          }
          else
          {
            if (findFreeNeighborGridSquare(startGridSquare).size() == 0)
            {

              isvalid = false;
            }
          }
        }
      }
    }
  }
  return isvalid;
}

/**
  Helper function to calculate cost of moving from currentGridSquare to neighbour

**/
float Kwj::getMoveCost(int i1, int j1, int i2, int j2)
{
  float moveCost = infinity; //start cost with maximum value. Change it to real cost of gridSquares are connected
  //if gridSquare(i2,j2) exists in the diagonal of gridSquare(i1,j1)
  if ((j2 == j1 + 1 && i2 == i1 + 1) || (i2 == i1 - 1 && j2 == j1 + 1) || (i2 == i1 - 1 && j2 == j1 - 1) || (j2 == j1 - 1 && i2 == i1 + 1))
  {

    moveCost = 1.4;
  }
  //if gridSquare(i2,j2) exists in the horizontal or vertical line with gridSquare(i1,j1)
  else
  {
    if ((j2 == j1 && i2 == i1 - 1) || (i2 == i1 && j2 == j1 - 1) || (i2 == i1 + 1 && j2 == j1) || (i1 == i2 && j2 == j1 + 1))
    {

      moveCost = 1;
    }
  }
  return moveCost;
}

/**
  Wrapper function to calculate cost of moving from currentGridSquare to neighbour

**/
float Kwj::getMoveCost(int gridSquareIndex1, int gridSquareIndex2)
{
  int i1 = 0, i2 = 0, j1 = 0, j2 = 0;

  i1 = getGridSquareRowIndex(gridSquareIndex1);
  j1 = getGridSquareColIndex(gridSquareIndex1);
  i2 = getGridSquareRowIndex(gridSquareIndex2);
  j2 = getGridSquareColIndex(gridSquareIndex2);

  return getMoveCost(i1, j1, i2, j2);
}

float Kwj::calculateHScore(int gridSquareIndex, int goalGridSquare)
{
  int x1 = getGridSquareRowIndex(goalGridSquare);
  int y1 = getGridSquareColIndex(goalGridSquare);
  int x2 = getGridSquareRowIndex(gridSquareIndex);
  int y2 = getGridSquareColIndex(gridSquareIndex);
  return abs(x1 - x2) + abs(y1 - y2);
}

/**
  Calculates the gridSquare index from square coordinates
**/
int Kwj::calculateGridSquareIndex(float i, float j) 
{
  return (i * costmap_->getSizeInCellsX()) + j;
}

/**

  Calculates gridSquare row from square index

**/
int Kwj::getGridSquareRowIndex(int index) //get the row index from gridSquare index
{
  return index / costmap_->getSizeInCellsX();
}

/**

  Calculates gridSquare column from square index

**/
int Kwj::getGridSquareColIndex(int index) //get column index from gridSquare index
{
  return index % costmap_->getSizeInCellsX();
}

void Kwj::getGridSquareCoordinates(int index, float &x, float &y)
  {
     x = getGridSquareColIndex(index) * costmap_->getResolution();
     y = getGridSquareRowIndex(index) * costmap_->getResolution();
     x = x + costmap_->getOriginX();
     y = y + costmap_->getOriginY();
  }

/**

  Checks if gridSquare at (i,j) is free

**/
bool Kwj::isFree(int i, int j)
{
  int gridSquareIndex = (i * costmap_->getSizeInCellsX()) + j;

  return occupancyGridMap[gridSquareIndex];
}

/**

  Checks if gridSquare at index gridSquareIndex is free

**/
bool Kwj::isFree(int gridSquareIndex)
{
  return occupancyGridMap[gridSquareIndex];
}

};
bool operator<(cell const &c1, cell const &c2) { return c1.fCost < c2.fCost; }
