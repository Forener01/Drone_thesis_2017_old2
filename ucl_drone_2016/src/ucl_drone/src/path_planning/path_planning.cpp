/* NOT THE ORIGINAL FILE */

/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 * This file receives information from Strategy and the Pose_estimation and publishes to the
 * Controller.
 * It tells the controller where the drone must go as function of the strategy and the position of
 * the drone.
 *
 *  \authors Julien Gérardy & Félicien Schiltz
 *  \date 2016
 *
 */

#include "ucl_drone/path_planning.h"

PathPlanning::PathPlanning()
{
  // drone prefix from the launch file.
  std::string drone_prefix;
  ros::param::get("~drone_prefix", drone_prefix);

  // List of subscribers and publishers. This node subscribes to pose_estimation to get the
  // real-time
  // position of the drone. It also subsribes to Stragegy in order to know in what the drone must do
  // and so where it has to go.
  // This node publishes in the node path_planning that communicates with the controller and to the
  // node mapcell that is just used to export data from tests.

  // Subscribers
  pose_channel = nh.resolveName("pose_estimation");
  pose_sub = nh.subscribe(pose_channel, 10, &PathPlanning::poseCb, this);

  strategy_channel = nh.resolveName("strategy");
  strategy_sub = nh.subscribe(strategy_channel, 10, &PathPlanning::strategyCb, this);

  // Publishers
  poseref_channel = nh.resolveName("path_planning");
  poseref_pub = nh.advertise< ucl_drone::PoseRef >(poseref_channel, 1);

  // Just for some tests
  // mapcell_channel = nh.resolveName("mapcell");
  // mapcell_pub = nh.advertise< ucl_drone::cellUpdate >(mapcell_channel, 500);

  // instruction_publishing = false;
}

// Destructor

PathPlanning::~PathPlanning()
{
}

// Function to reset the desired pose sent to the controller
void PathPlanning::reset()
{
  next_x = 0;
  next_y = 0;
  next_z = 1;
  next_rotZ = 0;
  i = 0;
  landing = false;
  takeoff = false;
  gridInitialized = false;

  // XMax and YMax are the half of the distance between the horizontal and vertical border lines of
  // the vision of the drone. Those values come from bottom camera data and 0.8 is used as a
  // security factor in the case of the drone is not well oriented.

  XMax = next_z * 0.435 * 0.8 * 0.5;
  YMax = next_z * 0.326 * 0.8 * 0.5;
}

// This function publish the position where the drone has to get. It is used as pose_ref in the
// controller node.
void PathPlanning::publish_poseref()
{
  // instantiate the poseref message
  ucl_drone::PoseRef poseref_msg;

  poseref_msg.x = next_x;
  poseref_msg.y = next_y;
  poseref_msg.z = next_z;
  poseref_msg.rotZ = next_rotZ;
  poseref_msg.landAndStop = landing;
  poseref_msg.takeoffAndStart = takeoff;

  poseref_pub.publish(poseref_msg);
}

// This function is called when this node receives a message from the topic "pose_estimation". So it
// takes this message and put it in a variable where it will be used in the other functions.

void PathPlanning::poseCb(const ucl_drone::Pose3D::ConstPtr posePtr)
{
  lastPoseReceived = *posePtr;
}

// This function is called when this node receives a message from the topic "strategy". So it
// takes this message and put it in a variable where it will be used in the other functions.

void PathPlanning::strategyCb(const ucl_drone::StrategyMsg::ConstPtr strategyPtr)
{
  lastStrategyReceived = *strategyPtr;
}

// First approach to explore a map. The drones makes a kind of labyrinth

bool PathPlanning::xy_desired()
{
  // The drone is considered as "arrived on poseref if it in a radius of 0.35m"
  if (sqrt((lastPoseReceived.x - next_x) * (lastPoseReceived.x - next_x) +
           (lastPoseReceived.y - next_y) * (lastPoseReceived.y - next_y)) < 0.35)
  {
    printf("i: %d\n", i);
    if (i % 2 == 0)
    {
      next_x += 1;
    }
    else if (i % 4 == 1)
    {
      next_y += 1;
    }
    else  //(i%4==3)
    {
      next_y -= 1;
    }
    i++;
    return true;
  }
  else
  {
    return false;
  }
}

// In our final approach to expore the map, the pathplanning creates a grid. This grid is made of
// 0.10*0.10 m² cells that can be in 3 different states :
// 0 means unseen/unexplored
// 1 means seen/explored
// 2 means border cell explored (there is cells behind but we haven't seen them yet)
// 3 means wall cell detected

void PathPlanning::InitializeGrid()
{
  // Set all the cells to "unexplored"
  for (i = 0; i < SIDE * 10; i++)
  {
    for (j = 0; j < SIDE * 10; j++)
    {
      myGrid[i][j] = 0;
    }
  }
  gridInitialized = true;
}

// This function creates a virtual wall. It returns true if the cell (i,j) contains a wall. This
// function has to be replaced by a true wall identification function via a camera and image process
// (for example). This function has been made in order where it can easily be replaced by something
// real and with more performances.

bool PathPlanning::ThereIsAWallCell(int i, int j)  // modified
{
  yfromcell2 = -((j / 10.0) - SIDE / 2.0);
  xfromcell2 = SIDE - (i / 10.0);

  // ROS_INFO("i is :  %d    , j is : %d \n", i, j);
  // ROS_INFO("xfromcell2 is : %lf    , yfromcell2 is : %lf \n", xfromcell2, yfromcell2);

  if (yfromcell2 >= (SIDE / 2.0 - 0.2) || yfromcell2 <= -(SIDE - 0.4) / 2.0)
  {
    return true;
  }
  else if (xfromcell2 >= SIDE - 0.2 || xfromcell2 <= 0.2)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// This function trasnforms the XMax and Ymax into the panel of cells that are seen by the drone in
// function of its position.
// !! abscisse = I and ordoonee = J
void PathPlanning::AbsOrdMinMax(double x, double y, int* iMin, int* iMax, int* jMin, int* jMax)
{
  printf("I'm calculating AbsOrdMinMax\n");
  *jMin = fmax(0, (int)(10 * SIDE / 2.0 - (y + YMax) * 10));
  *jMax = fmin((int)(10 * SIDE / 2.0 - (y - YMax) * 10), SIDE * 10);
  *iMin = fmax(0, (int)(10 * SIDE - (x + XMax) * 10));
  *iMax = fmin((int)(10 * SIDE - (x - XMax) * 10), SIDE * 10);
}

// This function transforms i and j coordinates to x and y position in the real playground.
void PathPlanning::CellToXY(int i, int j, double* xfromcell, double* yfromcell)
{
  *yfromcell = (double)(SIDE / 2.0 - j / 10.0);
  *xfromcell = (double)SIDE - (i / 10.0);
}

// This function is the main one of the grid. It checks all the cells that the drone is seeing and
// modify their value in regard with the previous grid and the "ThereIsAWallCell" function.
void PathPlanning::UpdateMap(double x, double y)
{
  ROS_INFO("I'm in the updatemap function");
  AbsOrdMinMax(x, y, &myAbsMin, &myAbsMax, &myOrdMin, &myOrdMax);
  ROS_INFO("I'm in the updatemap function after absOrdMinMax function");
  printf("myOrdMin: %d     myOrdMax: %d      myAbsMin: %d    myAbsMax: %d   \n", myOrdMin, myOrdMax,
         myAbsMin, myAbsMax);
  ucl_drone::cellUpdate cellUpdateMsg;
  for (i = myAbsMin; i < myAbsMax; i++)
  {
    for (j = myOrdMin; j < myOrdMax; j++)
    {
      // ROS_INFO("I'm in the updatemap loop for (i,j) %d, %d \n", i, j);

      if (myGrid[i][j] == 0 || myGrid[i][j] == 2)
      {
        if (!ThereIsAWallCell(i, j))
        {
          if (i == myAbsMin || i == myAbsMax - 1 || j == myOrdMin || j == myOrdMax - 1)
          {
            if (myGrid[i][j] == 2)
            {
              printf("I put a border cell\n");
            }
            else
            {
              printf("I put a NEW border cell\n");
              cellUpdateMsg.i = i;
              cellUpdateMsg.j = j;
              cellUpdateMsg.type = 2;
              mapcell_pub.publish(cellUpdateMsg);
            }
            myGrid[i][j] = 2;
          }
          else
          {
            myGrid[i][j] = 1;
            printf("I put an explored cell\n");
            cellUpdateMsg.i = i;
            cellUpdateMsg.j = j;
            cellUpdateMsg.type = 1;
            mapcell_pub.publish(cellUpdateMsg);
          }
        }
        else
        {
          myGrid[i][j] = 3;
          printf("I put a wall cell\n");
          cellUpdateMsg.i = i;
          cellUpdateMsg.j = j;
          cellUpdateMsg.type = 3;
          mapcell_pub.publish(cellUpdateMsg);
        }
      }
    }
  }
}

// This function returns the distance between two cells.
double PathPlanning::distance(int i, int j, int k, int l)
{
  return sqrt((i - k) * (i - k) + (j - l) * (j - l));
}

// This function replaced the labyrtinth one. It takes as argument the position of the drone and
// gives back the best cell where the drone must go. In order to do that, it compares all the border
// cells of the map and tell the drone to go to the nearrest one.
void PathPlanning::advanced_xy_desired(double x, double y, double* k, double* l)
{
  int absc = (int)(SIDE - x) * 10;
  int ord = (int)(SIDE / 2.0 - y) * 10;
  bestDist = 1000000.0;
  closestJ = SIDE * 10 / 2.0;
  closestI = SIDE * 10;
  for (i = 0; i < SIDE * 10; i++)
  {
    for (j = 0; j < SIDE * 10; j++)
    {
      if (myGrid[i][j] == 2 && distance(absc, ord, i, j) < bestDist)
      {
        bestDist = distance(absc, ord, i, j);
        closestJ = j;
        closestI = i;
      }
    }
  }
  if (bestDist == 1000000.0)
  {
    ROS_INFO("The map was explored completely, there are no more border cells!");
  }
  // printf("closestI should be  50 and is: %d \n", closestI);
  CellToXY(closestI, closestJ, k, l);
}

// This function give to the object variable the computed next reference position.
void PathPlanning::SetRef(double x_ref, double y_ref, double z_ref, double rotZ_ref)
{
  this->next_x = x_ref;
  this->next_y = y_ref;
  this->next_z = z_ref;
  this->next_rotZ = rotZ_ref;
}

// Main function, will publish pose_ref with regard to the selected strategy comming from the
// Strategy node.
int main(int argc, char** argv)
{
  ROS_INFO_STREAM("poseref_sending started!");
  ros::init(argc, argv, "path_planning");
  PathPlanning myPath;
  ros::Rate r(20);  // Refresh every 1/20 second.
  printf("Pathplanning launched");
  ROS_DEBUG("path planning initialized");

  myPath.reset();
  myPath.publish_poseref();
  ros::spinOnce();
  // r.sleep();
  ROS_INFO_STREAM("poseref initialized and launched");
  while (ros::ok())
  {
    TIC(path);
    // if (myPath.lastStrategyReceived.type == 1.0)  // This corresponds to the takeoff strategy.
    // {
    //   myPath.takeoff = true;
    //   myPath.publish_poseref();
    // }
    //
    // // Strategy == 7.0 and == 8.0 are never true. Those numbers are just used to change the
    // response
    // // we want from the pathplanning between 3 different functions. (test - labyrinth - grid). We
    // // just have to put 2.0 to the one we want and other numbers to the others.
    //
    // // This function calls the labyrinth pathplanning.
    // else if (myPath.lastStrategyReceived.type == 7.0)
    // {
    //   if (myPath.xy_desired() == true)
    //   {
    //     myPath.takeoff = true;
    //     ros::Duration(5).sleep();
    //     myPath.publish_poseref();
    //   }
    // }
    //
    // // Strategy = GOTO, the drone goes directly to the position writed in the message from the
    // // strategy.
    // else if (myPath.lastStrategyReceived.type == 3.0)
    // {
    //   myPath.takeoff = true;
    //   myPath.SetRef(myPath.lastStrategyReceived.x, myPath.lastStrategyReceived.y + 0.5,
    //                 myPath.next_z,
    //                 myPath.next_rotZ);  // Pas correct, ne tient pas compte de la
    //                                     // position du drone maitre
    //   myPath.publish_poseref();
    // }
    //
    // // Strategy = Land, the drone lands.
    // else if (myPath.lastStrategyReceived.type == 4.0)
    // {
    //   myPath.takeoff = false;
    //   myPath.landing = true;
    //   myPath.publish_poseref();
    // }
    //
    // // Strategy = Follow, the drone goes to the coordinates writed in the message from the
    // strategy.
    // // Those ones come from the target_detected channel in the strategy node.
    // else if (myPath.lastStrategyReceived.type == 5.0)
    // {
    //   myPath.takeoff = true;
    //   myPath.SetRef(myPath.lastStrategyReceived.x, myPath.lastStrategyReceived.y, myPath.next_z,
    //                 myPath.next_rotZ);
    //   myPath.publish_poseref();
    // }
    //
    // // Strategy = BackToBase, the drone rises up to 2m in order to avoid the other drone coming
    // to
    // // replace him. Then it goes back to the base and land when it is in a radius of 0.15m from
    // // where it took off.
    // else if (myPath.lastStrategyReceived.type == 6.0)
    // {
    //   myPath.next_z = 2;
    //   if (sqrt((myPath.lastPoseReceived.x) * (myPath.lastPoseReceived.x) +
    //            (myPath.lastPoseReceived.y) * (myPath.lastPoseReceived.y)) < 0.15)
    //   {
    //     myPath.takeoff = false;
    //     myPath.landing = true;
    //   }
    //   else if (!myPath.landing)
    //   {
    //     myPath.takeoff = true;
    //     myPath.SetRef(0, 0, myPath.next_z, myPath.next_rotZ);
    //   }
    //   myPath.publish_poseref();
    // }
    //
    // // Strategy = Seek (explore), the drone initializes the grid. Then this is the
    // // advanced_xy_desired function that tells the drone where to go.
    // else if (myPath.lastStrategyReceived.type == 2.0)
    // {
    //   if (!myPath.gridInitialized)
    //   {
    //     ROS_INFO("I am initializing the grid");
    //
    //     myPath.InitializeGrid();
    //   }
    //
    //   myPath.takeoff = true;
    //   myPath.UpdateMap(myPath.lastPoseReceived.x, myPath.lastPoseReceived.y);
    //   ROS_INFO("I have updated the map");
    //
    //   myPath.advanced_xy_desired(myPath.lastPoseReceived.x, myPath.lastPoseReceived.y,
    //                              &myPath.poseRefX, &myPath.poseRefY);
    //   myPath.SetRef(myPath.poseRefX, myPath.poseRefY, myPath.next_z, myPath.next_rotZ);
    //   ROS_INFO("poseref is : (%lf, %lf)", myPath.poseRefX, myPath.poseRefY);
    //   ros::Duration(1).sleep();
    //   myPath.publish_poseref();
    // }

    // Test of command for the report.
    // else if (myPath.lastStrategyReceived.type == 8.0)
    // {
    ros::Duration(20).sleep();
    ROS_INFO_STREAM("WORKING !!")
    myPath.takeoff = true;
    myPath.SetRef(0.0, 0.0, 1.0, 0.0);
    myPath.publish_poseref();
    ros::Duration(15).sleep();
    myPath.SetRef(2.0, 0.0, 1.0, 0.0);
    myPath.publish_poseref();
    ros::Duration(35).sleep();
    myPath.SetRef(0.0, 0.0, 1.0, 0.0);
    myPath.publish_poseref();
    ros::Duration(35).sleep();
    myPath.SetRef(-2.0, 0.0, 1.0, 0.0);
    myPath.publish_poseref();
    ros::Duration(35).sleep();
    // }

    TOC(path, "path planning");
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

// Function that gives a yaw angle in order to always face to the direction to go. We don't use it
// at this moment.

/*void PathPlanning::yaw_desired()
{
  double delta_x=next_x-lastPoseReceived.x;
  double delta_y=next_y-lastPoseReceived.y;
  if(sqrt(delta_x*delta_x+delta_y*delta_y)<1.5)
  {
  next_yaw=old_next_yaw;
  }
  else
  {
  double theta;
  if(delta_x!=0)
  {
  theta=atan(delta_y/delta_x);
  }
  else
  {
  theta=3.14159;
  }
  if(delta_y<0 && delta_x<0)
  {
  theta+=3.14159;
      if(theta>3.14159)
      {
          theta=theta-2*3.14159;
      }
  }
  else if(delta_y>0 && delta_x<0)
  {
  theta=abs(theta);
  }
  old_next_yaw=next_yaw;
  next_yaw=theta;
  }

}
*/
