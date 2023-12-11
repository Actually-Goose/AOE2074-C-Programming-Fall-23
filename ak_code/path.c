/**
 * @file path.c
 **/

#include <fcntl.h>  // for F_OK
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  // for access()i
#include <ctype.h> 

#include <path.h>

path_t path = PATH_INITIALIZER;

/*********************************
 * Functions for internal use only
 *
 */

/**
 * @brief   Count the number of lines in a file, indicates number of waypoints
 *
 * @return  Number of lines in the file
 */
//static int __count_file_lines(const char* file_path);

/**
 * @brief   Read all of the waypoints from a file into the path variable
 *
 * @return  0 on success, -1 on failure
 */
//static int __read_waypoints(FILE* fd);


//AK CODE
//Bounds of the motion capture room, asssumes that origin is the exact center of the room
//when used in method compare values against postive and negative to account for either
//side of the origin, statics so not accidentally modified later, global for easy change of value/bounds
int xBound = 5;
int yBound = 5;
int zBound = 5;
/**
 * ********************************
 */

//read in the file passed from keyboard/NLP
int pathCreation(FILE* waypointFile) {
    int position = 0;
    int waypointValue = 0;
    while (position != EOF) {
        position = fscanf(waypointFile, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %i",
            &path.waypoints[waypointValue].t, &path.waypoints[waypointValue].x,
            &path.waypoints[waypointValue].y, &path.waypoints[waypointValue].z,
            &path.waypoints[waypointValue].xd, &path.waypoints[waypointValue].yd,
            &path.waypoints[waypointValue].zd, &path.waypoints[waypointValue].roll,
            &path.waypoints[waypointValue].pitch, &path.waypoints[waypointValue].yaw,
            &path.waypoints[waypointValue].p, &path.waypoints[waypointValue].q,
            &path.waypoints[waypointValue].r, &path.waypoints[waypointValue].flag);
        //if fscan finds an invalid character that is not the end of the file marker
        //position 14 is the flag value, and would be an invalid waypoint, hence why its excluded
        if (position != EOF && position != 14)
        {
            printf("There was an invalid entry at line %i of the waypoint file.\n", waypointValue + 1);
            return -1;
        }
        ++waypointValue;
    }
    return 0;
}

//use path creation to acutally create the path, check for file validity, and success of memory allocation, default mode
int pathFromFile(const char* file_path)
{
    clearList(path); //make sure no leftover values are present
    // Check for valid file
    FILE * waypointFile = fopen(file_path, "r");
    if (file_path == NULL) {
        printf("The file could not be opened. Method: pathFromFile(); line: 78 \n");
        return -1;
    }
    // Open file for waypoint reading
   // FILE* waypointFile = fopen(file_path, "r");

    //allocate memory for the waypoints by multiplying the len of the waypoint
    //and the number of waypoints
    path.len = numOfLines(file_path);
    path.waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path.len);

    if (path.waypoints == NULL)
    {
        printf("Memory allocation for waypoints failed. Method: pathFromFile(); line: 91\n");
        return -1;
    }

    // Read waypoints from file
    if (pathCreation(waypointFile) < 0)
    {
        clearList(path); //allocation failed, clear so can start again/not reading bad waypoints
        printf("Waypoint file could not be read. Method: pathFromFile(); line: 99 \n ");
        return -1;
    }

    fclose(waypointFile);

    path.initialized = 1;
    return 0;
}

//take in command either from keyboard or NLP team, decide what method to use
int pathSelection(char selection) {

    if (tolower(selection) == 'l') {
        return 5;
    }
    else if (tolower(selection) == 't') {
        return 6;
    }
    else if (tolower(selection) == 'x') {
        return 1;
    }
    else if (tolower(selection) == 'y') {
        return 2;
    }
    else if (tolower(selection) == 'z') {
        return 3;
    }
    else {
        printf("Invalid path selection. Method: pathSelection(); line: 128\n");
        return 0;
    }
}


/*clear the contents of the list*/
//we ended up renaming these variables from my original variable names to match path.c's format
void clearList(path_t path) {
   free(path.waypoints); //free memory
    path.waypoints = NULL; 
    path.len = 0; //set size zero (empty)
    path.initialized = 0; //path is no longer created
}

//figure out number of expected waypoints
int numOfLines(const char* file_path)
{
    FILE* lineCounter = fopen(file_path, "r");
    int character = 0;
    size_t lineNumber = 0;

    //gets the next character (an unsigned char) from the specified
    //stream and advances the position indicator for the stream.
    character = getc(lineCounter);
    //while it is not at the end of file marker...
    while (character != EOF)
    {   //if it is a new line add a line to the count
        if (character == '\n' )
        {
            ++lineNumber;
        }
        character = getc(lineCounter);
    }
    fclose(lineCounter);
    return lineNumber;
}

//NOT AK CODE
quintic_spline_1d_t make_1d_quintic_spline(float dx, float dt)
{
    quintic_spline_1d_t simple_quintic_spline;

    simple_quintic_spline.c0 = 0;
    simple_quintic_spline.c1 = 0;
    simple_quintic_spline.c2 = 0;
    simple_quintic_spline.c3 =  10 * dx / pow(dt,3);
    simple_quintic_spline.c4 = -15 * dx / pow(dt,4);
    simple_quintic_spline.c5 =   6 * dx / pow(dt,5);

    return simple_quintic_spline;
}

float compute_spline_position(quintic_spline_1d_t* the_spline, float t)
{
    return the_spline->c0 
         + the_spline->c3 * pow(t,3)
         + the_spline->c4 * pow(t,4)
         + the_spline->c5 * pow(t,5);
}


int path_plan_from_rsp_cmd_3pts()
{
    // Clear any previously stored path, set init to 0
    path_cleanup();

    // Simple 3-waypoint path
    path.len = 3;
    path.waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path.len);
        if (path.waypoints == NULL)
    {
        fprintf(stderr, "ERROR: failed allocating memory for path\n");
        return -1;
    }

    // Pt - 1 (current location)
    path.waypoints[0].t = 1;
    path.waypoints[0].x = state_estimate.X;
    path.waypoints[0].y = state_estimate.Y;
    path.waypoints[0].z = state_estimate.Z;
    path.waypoints[0].xd = 0;
    path.waypoints[0].yd = 0;
    path.waypoints[0].zd = 0;
    path.waypoints[0].roll = 0;
    path.waypoints[0].pitch = 0;
    path.waypoints[0].yaw = 0;
    path.waypoints[0].p = 0;
    path.waypoints[0].q = 0;
    path.waypoints[0].r = 0;
    path.waypoints[0].flag = 0;

    // Pt - 2 (above landing spot)
    path.waypoints[1].t = 6;
    path.waypoints[1].x = rspLandingCommandMsg.x;
    path.waypoints[1].y = rspLandingCommandMsg.y;
    path.waypoints[1].z = state_estimate.Z;
    path.waypoints[1].xd = 0;
    path.waypoints[1].yd = 0;
    path.waypoints[1].zd = 0;
    path.waypoints[1].roll = 0;
    path.waypoints[1].pitch = 0;
    path.waypoints[1].yaw = 0;
    path.waypoints[1].p = 0;
    path.waypoints[1].q = 0;
    path.waypoints[1].r = 0;
    path.waypoints[1].flag = 0;

    // Pt - 3 (at landing spot)
    path.waypoints[2].t = 11;
    path.waypoints[2].x = rspLandingCommandMsg.x;
    path.waypoints[2].y = rspLandingCommandMsg.y;
    path.waypoints[2].z = 0;
    path.waypoints[2].xd = 0;
    path.waypoints[2].yd = 0;
    path.waypoints[2].zd = 0;
    path.waypoints[2].roll = 0;
    path.waypoints[2].pitch = 0;
    path.waypoints[2].yaw = 0;
    path.waypoints[2].p = 0;
    path.waypoints[2].q = 0;
    path.waypoints[2].r = 0;
    path.waypoints[2].flag = 0;

    path.initialized = 1;
    return 0;
}

int path_plan_from_rsp_cmd()
{
    // x1 is our current position
    rc_vector_t x1  = RC_VECTOR_INITIALIZER;
    rc_vector_alloc(&x1, 3);
    x1.d[0] = state_estimate.X;
    x1.d[1] = state_estimate.Y;
    x1.d[2] = state_estimate.Z;
    double t1 = 0.0;

    // x2 is our current z but directly above the landing point
    rc_vector_t x2 = RC_VECTOR_INITIALIZER;
    rc_vector_alloc(&x2, 3);
    x2.d[0] = rspLandingCommandMsg.x;
    x2.d[1] = rspLandingCommandMsg.y;
    x2.d[2] = state_estimate.Z;
    double t2 = 5.0;

    // x3 is the landing point with z=0 so no funny business occurs
    rc_vector_t x3  = RC_VECTOR_INITIALIZER;
    rc_vector_alloc(&x3, 3);
    x3.d[0] = rspLandingCommandMsg.x;
    x3.d[1] = rspLandingCommandMsg.y;
    x3.d[2] = 0;
    double t3 = 10.0;

    // 
    int num_pts_per_spline = 500; // 5 sec, 500 pts, 100 pts/sec = 100Hz
    int num_pts_total = num_pts_per_spline * 2; // 2 segments

    path_cleanup(); // Clear any previously stored path, set init to 0

    path.len = num_pts_total;
    path.waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path.len);
        if (path.waypoints == NULL)
    {
        fprintf(stderr, "ERROR: failed allocating memory for path\n");
        return -1;
    }

    // Start at t=0
    double t_curr = 0;
    double s_curr = 0;
    double x_curr = 0;
    double y_curr = 0;
    double z_curr = 0;

    // Setup Segment #1
    double dx = x2.d[0] - x1.d[0];
    double dy = x2.d[1] - x1.d[1];
    double dz = x2.d[2] - x1.d[2];
    double d_len = sqrt(dx*dx + dy*dy + dz*dz);
    quintic_spline_1d_t q_spline_1  = make_1d_quintic_spline(d_len, t2 - t1);

    // Run through Segment #1
    for (int i=0; i < num_pts_per_spline; i++) 
    {
        // 1) Get 1d position
        t_curr = ( ((double) i) / ((double) (num_pts_per_spline-1))) * (t2 - t1);
        s_curr = compute_spline_position(&q_spline_1, t_curr);

        // 2) Convert to 3d position
        if (d_len > 0) 
        {
            x_curr = (s_curr / d_len) * dx  + x1.d[0];
            y_curr = (s_curr / d_len) * dy  + x1.d[1];
            z_curr = (s_curr / d_len) * dz  + x1.d[2];
        }   
        else
        {
            // If d_len is 0, avoid NaNs
            x_curr = x1.d[0];
            y_curr = x1.d[1];
            z_curr = x1.d[2];
        }
        

        // 3) Write to the path
        path.waypoints[i].t = t_curr + t1;
        path.waypoints[i].x = x_curr;
        path.waypoints[i].y = y_curr;
        path.waypoints[i].z = z_curr;
        path.waypoints[i].xd = 0;
        path.waypoints[i].yd = 0;
        path.waypoints[i].zd = 0;
        path.waypoints[i].roll = 0;
        path.waypoints[i].pitch = 0;
        path.waypoints[i].yaw = state_estimate.continuous_yaw;
        path.waypoints[i].p = 0;
        path.waypoints[i].q = 0;
        path.waypoints[i].r = 0;
        path.waypoints[i].flag = 0;
    }

    // Setup Segment #2
    dx = x3.d[0] - x2.d[0];
    dy = x3.d[1] - x2.d[1];
    dz = x3.d[2] - x2.d[2];
    d_len = sqrt(dx*dx + dy*dy + dz*dz);
    quintic_spline_1d_t q_spline_2  = make_1d_quintic_spline(d_len, t3 - t2);

    // Run through Segment #2
    for (int i=0; i < num_pts_per_spline; i++) 
    {
        // 1) Get 1d position
        t_curr = ( ((double) i) / ((double) (num_pts_per_spline-1))) * (t3 - t2);
        s_curr = compute_spline_position(&q_spline_2, t_curr);

        // 2) Convert to 3d position
        

        // 2) Convert to 3d position
        if (d_len > 0) 
        {
            x_curr = (s_curr / d_len) * dx  + x2.d[0];
            y_curr = (s_curr / d_len) * dy  + x2.d[1];
            z_curr = (s_curr / d_len) * dz  + x2.d[2];
        }   
        else
        {
            // If d_len is 0, avoid NaNs
            x_curr = x2.d[0];
            y_curr = x2.d[1];
            z_curr = x2.d[2];
        }

        // 3) Write to the path
        int j = i + num_pts_per_spline;
        path.waypoints[j].t = t_curr + t2;
        path.waypoints[j].x = x_curr;
        path.waypoints[j].y = y_curr;
        path.waypoints[j].z = z_curr;
        path.waypoints[j].xd = 0;
        path.waypoints[j].yd = 0;
        path.waypoints[j].zd = 0;
        path.waypoints[j].roll = 0;
        path.waypoints[j].pitch = 0;
        path.waypoints[j].yaw = state_estimate.continuous_yaw;
        path.waypoints[j].p = 0;
        path.waypoints[j].q = 0;
        path.waypoints[j].r = 0;
        path.waypoints[j].flag = 0;
    }

    // for (unsigned int i=0; i < path.len; i++) {
    //     printf("|%+5.2f|%+5.2f|%+5.2f|%+5.2f|\n",
    //         path.waypoints[i].t, path.waypoints[i].x, path.waypoints[i].y, path.waypoints[i].z);
    // }


    path.initialized = 1;
    return 0;
}

//AK CODE
//move in the x direction
int moveX(double xPosition, char str) {
    //clear leftover paths
    clearList(path);
    //determine which flight mode from path selection command
    if (pathSelection(str) == 1) {
        //check if the new position would be out of the motion capture room
       double potentialPos = state_estimate.X + xPosition;
        if (potentialPos < xBound && potentialPos > (-xBound)) {
            //this is how many waypoints there will be in the path
            //two waypoints will be at the current position, and change z to the
            //distance from the ground
            path.len = 2;

            //allocate the memory for the path we're creating
            path.waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path.len);
            //check if memory allocation was successful, if not, print message and exit
            //path planning
            if (path.waypoints == NULL) {
                printf("Memory allocation for path failed. Method: moveX(); line: 430 \n");
                return -1;
            }
            //from state_estimator.c, determines where the quadcopter is loacted
            path.waypoints[0].t = 1;
            path.waypoints[0].x = state_estimate.X;
            path.waypoints[0].y = state_estimate.Y;
            path.waypoints[0].z = state_estimate.Z;
            path.waypoints[0].xd = 0;
            path.waypoints[0].yd = 0;
            path.waypoints[0].zd = 0;
            path.waypoints[0].roll = 0;
            path.waypoints[0].pitch = 0;
            path.waypoints[0].yaw = 0;
            path.waypoints[0].p = 0;
            path.waypoints[0].q = 0;
            path.waypoints[0].r = 0;
            path.waypoints[0].flag = 0;

            //second waypoint
            //in theory I think this shoukd access the second line of the file read in, and these values
            //do not need to be set equal to anything, since it would change the value of the given points?
            path.waypoints[1].t = 10;
            path.waypoints[1].x = state_estimate.X + xPosition;
            path.waypoints[1].y = state_estimate.Y;
            path.waypoints[1].z = state_estimate.Z;
            path.waypoints[1].xd  = 0;
            path.waypoints[1].yd  = 0;
            path.waypoints[1].zd  = 0;
            path.waypoints[1].roll  = 0;
            path.waypoints[1].pitch  = 0;
            path.waypoints[1].yaw  = 0;
            path.waypoints[1].p  = 0;
            path.waypoints[1].q  = 0;
            path.waypoints[1].r  = 0;
            path.waypoints[1].flag  = 0;
            printf("Your moveX() path was created. \n");
            return 1;
        }
    }
    else {
        return 0;
    }
    return 0;
}

//move in the y direction
int moveY(double yPostition, char str) {
    //clear leftover paths
    clearList(path);
    //determine which flight mode from path selection command
    if (pathSelection(str) == 2)
    {
        //check if the new position would be out of the motion capture room
        double potentialPos = state_estimate.Y + yPostition;
        if (potentialPos < yBound && potentialPos > (-yBound)) {
            //this is how many waypoints there will be in the path
            //two waypoints will be at the current position, and change z to the
            //distance from the ground
            path.len = 2;

            //allocate the memory for the path we're creating
            path.waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path.len);
            //check if memory allocation was successful, if not, print message and exit
            //path planning
            if (path.waypoints == NULL) {
                printf("Memory allocation for path failed. Method: moveY(); line: 496\n");
                return -1;
            }
            //from state_estimator.c, determines where the quadcopter is loacted
            path.waypoints[0].t = 1;
            path.waypoints[0].x = state_estimate.X;
            path.waypoints[0].y = state_estimate.Y;
            path.waypoints[0].z = state_estimate.Z;
            path.waypoints[0].xd = 0;
            path.waypoints[0].yd = 0;
            path.waypoints[0].zd = 0;
            path.waypoints[0].roll = 0;
            path.waypoints[0].pitch = 0;
            path.waypoints[0].yaw = 0;
            path.waypoints[0].p = 0;
            path.waypoints[0].q = 0;
            path.waypoints[0].r = 0;
            path.waypoints[0].flag = 0;

            //second waypoint
            path.waypoints[1].t = 10;
            path.waypoints[1].x = state_estimate.X;
            path.waypoints[1].y = state_estimate.Y + yPostition;
            path.waypoints[1].z = state_estimate.Z;
            path.waypoints[1].xd  = 0;
            path.waypoints[1].yd  = 0;
            path.waypoints[1].zd  = 0;
            path.waypoints[1].roll  = 0;
            path.waypoints[1].pitch  = 0;
            path.waypoints[1].yaw  = 0;
            path.waypoints[1].p  = 0;
            path.waypoints[1].q  = 0;
            path.waypoints[1].r  = 0;
            path.waypoints[1].flag  = 0;

            printf("Your moveY() path was created. \n");
            return 1;
        }
    }
    else{
        return 0;
    }
    return 0;
}

//move in the z direction
int moveZ(double zPosition, char str) {
    //clear leftover paths
    clearList(path);
    //determine which flight mode from path selection command
    if (pathSelection(str) == 3) {
        //check if the new position would be out of the motion capture room
        double potentialPos = state_estimate.Z + zPosition;
        if (potentialPos < zBound && potentialPos > (-zBound)) {
            //this is how many waypoints there will be in the path
            //two waypoints will be at the current position, and change z to the
            //distance from the ground
            path.len = 2;

            //allocate the memory for the path we're creating
            path.waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path.len);
            //check if memory allocation was successful, if not, print message and exit
            //path planning
            if (path.waypoints == NULL) {
                printf("Memory allocation for path failed. Method: moveZ(); line: 560\n");
                return -1;
            }
            //determine where it is currently located
            path.waypoints[0].t = 1; //time
            //from state_estimator.c, determines where the quadcopter is located
            path.waypoints[0].t = 1;
            path.waypoints[0].x = state_estimate.X;
            path.waypoints[0].y = state_estimate.Y;
            path.waypoints[0].z = state_estimate.Z;
            path.waypoints[0].xd = 0;
            path.waypoints[0].yd = 0;
            path.waypoints[0].zd = 0;
            path.waypoints[0].roll = 0;
            path.waypoints[0].pitch = 0;
            path.waypoints[0].yaw = 0;
            path.waypoints[0].p = 0;
            path.waypoints[0].q = 0;
            path.waypoints[0].r = 0;
            path.waypoints[0].flag = 0;

            //next waypoint
            path.waypoints[1].t = 10;
            path.waypoints[1].x = state_estimate.X;
            path.waypoints[1].y = state_estimate.Y;
            path.waypoints[1].z = 0;
            path.waypoints[1].xd  = 0;
            path.waypoints[1].yd  = 0;
            path.waypoints[1].zd  = 0;
            path.waypoints[1].roll  = 0;
            path.waypoints[1].pitch  = 0;
            path.waypoints[1].yaw  = 0;
            path.waypoints[1].p  = 0;
            path.waypoints[1].q  = 0;
            path.waypoints[1].r  = 0;
            path.waypoints[1].flag  = 0;

             printf("Your moveZ() path was created. \n");
             return 1;
        }
    }
    else{
        return 0;
    }
    return 0;
}

//land
int land(char str){
    //clear leftover paths
    clearList(path);
    //determine which flight mode from path selection command
    if (pathSelection(str) == 5)
    {
        //this is how many waypoints there will be in the path
        //two waypoints will be at the current position, and change z to the
        //distance from the ground
        path.len = 2;

        //allocate the memory for the path we're creating
        path.waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path.len);
        //check if memory allocation was successful, if not, print message and exit
        //path planning
        if (path.waypoints == NULL) {
            printf("Memory allocation for path failed. Method: land(); line: 624\n");
            return -1;
        }
        //determine where it is currently located
        path.waypoints[0].t = 1; //time
        //from state_estimator.c, determines where the quadcopter is loacted
        path.waypoints[0].t = 1;
        path.waypoints[0].x = state_estimate.X;
        path.waypoints[0].y = state_estimate.Y;
        path.waypoints[0].z = state_estimate.Z;
        path.waypoints[0].xd = 0;
        path.waypoints[0].yd = 0;
        path.waypoints[0].zd = 0.15;
        path.waypoints[0].roll = 0;
        path.waypoints[0].pitch = 0;
        path.waypoints[0].yaw = 0;
        path.waypoints[0].p = 0;
        path.waypoints[0].q = 0;
        path.waypoints[0].r = 0;
        path.waypoints[0].flag = 0;

        //determine where it will land (directly below current--at start of path plan--position)
        path.waypoints[1].t = 10; //gives 9 seconds to land (i think)
        path.waypoints[1].x = state_estimate.X;
        path.waypoints[1].y = state_estimate.Y;
        path.waypoints[1].z = 0;
        path.waypoints[1].xd = 0;
        path.waypoints[1].yd = 0;
        path.waypoints[1].zd = 0;
        path.waypoints[1].roll = 0;
        path.waypoints[1].pitch = 0;
        path.waypoints[1].yaw = 0;
        path.waypoints[1].p = 0;
        path.waypoints[1].q = 0;
        path.waypoints[1].r = 0;
        path.waypoints[1].flag = 0;
        printf("Your land() path was created. \n");
        return 1;
    }
    return 0;
}

//take off
int takeOff(char str) {
   //clear leftover paths
    clearList(path);
    //determine which flight mode from path selection command
    if (pathSelection(str) == 6)
    {
        //this is how many waypoints there will be in the path
        //two waypoints will be at the ground position, and 1 unit above the
        //current position
        path.len = 2;

        //allocate the memory for the path we're creating
        path.waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path.len);
        //check if memory allocation was successful, if not, print message and exit
        //path planning
        if (path.waypoints == NULL) {
            printf("Memory allocation for path failed, method: takeOFF(); line: 683\n");
            return -1;
        }
        //determine current location (this is waypoint 1)
        path.waypoints[0].t = 1;
        path.waypoints[0].x = state_estimate.X;
        path.waypoints[0].y = state_estimate.Y;
        path.waypoints[0].z = state_estimate.Z;
        path.waypoints[0].xd = 0;
        path.waypoints[0].yd = 0;
        path.waypoints[0].zd = 0;
        path.waypoints[0].roll = 0;
        path.waypoints[0].pitch = 0;
        path.waypoints[0].yaw = 0;
        path.waypoints[0].p = 0;
        path.waypoints[0].q = 0;
        path.waypoints[0].r = 0;
        path.waypoints[0].flag = 0;

        //determine where it will take off
        path.waypoints[1].t = 5;
        path.waypoints[1].x = state_estimate.X;
        path.waypoints[1].y = state_estimate.Y;
        path.waypoints[1].z = 1; //default to one unit above origin for take off
        path.waypoints[1].xd = 0;
        path.waypoints[1].yd = 0;
        path.waypoints[1].zd = 0.25;
        path.waypoints[1].roll = 0;
        path.waypoints[1].pitch = 0;
        path.waypoints[1].yaw = 0;
        path.waypoints[1].p = 0;
        path.waypoints[1].q = 0;
        path.waypoints[1].r = 0;
        path.waypoints[1].flag = 0;

        printf("Your takeOff() path was created. \n");
        return 1;
    }
    return 0;
}
