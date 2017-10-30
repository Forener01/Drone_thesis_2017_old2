void PathPlanning::reset()
void PathPlanning::publish_poseref()
void PathPlanning::poseCb(const ucl_drone::Pose3D::ConstPtr posePtr)
void PathPlanning::strategyCb(const ucl_drone::StrategyMsg::ConstPtr strategyPtr)
bool PathPlanning::xy_desired()
void PathPlanning::InitializeGrid()
bool PathPlanning::ThereIsAWallCell(int i, int j)
void PathPlanning::AbsOrdMinMax(double x, double y, int *iMin, int *iMax, int *jMin, int *jMax)
void PathPlanning::CellToXY(int i, int j, double *xfromcell, double *yfromcell)
void PathPlanning::UpdateMap(double x, double y)
double PathPlanning::distance(int i, int j, int k, int l)
void PathPlanning::advanced_xy_desired(double x, double y, double *k, double *l)
void PathPlanning::SetRef(double x_ref, double y_ref, double z_ref, double rotZ_ref)

/* Strategy:
0 = reset;
1 = takeoff;
2 = seek; => myPath.xy_desired()
3 = goto;
4 = land;
5 = follow;
6 = backtobase;
7 = seek 2; => myPath.advanced_xy_desired()
8 = test command; => hardcoding
*/
