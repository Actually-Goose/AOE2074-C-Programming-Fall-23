/**
 * <path.h>
 *
 * @brief   Functions to read the waypoint file and handle the path
 *
 * Program expects waypoint file of the form (no line numbers in actual file):
 *
 * ~~~
 * 1:    t x y z xd yd zd r p y rd pd yd
 * 2:    t x y z xd yd zd r p y rd pd yd
 *         ...
 * n:    t x y z xd yd zd r p y rd pd yd
 * ~~~
 *
 * where line i contains waypoint i with position, velocity and time respectively.
 * Memory will be dynamically allocated for the path and the path will be stored
 * as a set of waypoints defined by the file
 *
 * TODO: Possibly rename (path maybe?), nameing conflicts with the
 *
 * @author Glen Haggin (ghaggin@umich.edu)
 *
 * @addtogroup Path
 * @{
 */

#ifndef __PATH__
#define __PATH__

#include <stddef.h>

#include <coordinates.h>
#include <realsense_payload_receive.h>
#include <state_estimator.h>
#include <math_utils.h>

/**
 * @brief       Read waypoint file and initialize path
 *
 * Checks for a valid file and counts the number of waypoints specified.  Based
 * on the number of waypoints, memory for the path is dynamically allocated (freed in cleanup).
 * Waypoints are then sequentially read into the path.  If any invalid waypoints are specified,
 * i.e. not in the form <x y z xd yd zd t> (all floats), then the intialization fails and the
 * path stays unitialized.
 *
 * Usage: setpoint_manager.c, line 204
 *
 * @param[in]   file_path   string containing the relative path to the waypoint file
 *
 * @return      0 on success, -1 on failure
 */
int path_load_from_file(const char* file_path);

/**
 * @brief       Frees memory allocated in path and "unitializes" path variable
 *
 * Usage: path.c, line 39
 *        path.c, line 65
 *        path.c, line 162
 *        path.c, line 255
 *        main.c, line 657
 */
void path_cleanup();

/**
 * @brief       Plan a path based on realsense payload landing command
 * 
 * Very simple version with no-splines. Allows incremental testing.
 * 
 * @return      0 on success, -1 on failure
 */
int path_plan_from_rsp_cmd_3pts();

/**
 * @brief       Plan a path based on realsense payload landing command
 * 
 * @return      0 on success, -1 on failure
 */
int path_plan_from_rsp_cmd();


typedef struct quintic_spline_1d_t
{
    float c0, c1, c2, c3, c4, c5;
} quintic_spline_1d_t;

/**
 * @brief       Compute quintic spline coefficients for simple 1d path.
 * 
 * Starting and ending velocity and acceleration are 0.
 * 
 * @return      quintic_spline_1d_t with proper coefficients
 */
quintic_spline_1d_t make_1d_quintic_spline(float dx, float dt);

/**
 * @brief       Compute position along spline based on time
 * 
 * @return      1d position along spline
 */
float compute_spline_position(quintic_spline_1d_t* the_spline, float t);



/* define the structure of path, pointer to the start of the array, 
* the number of waypoints, and if its intialISed
*/typedef struct path_t
{
    waypoint_t* waypoints;  ///< pointer to head of the waypoint array
    size_t len;             ///< length of the path (number of waypoints)

    int initialized;  ///< 1 if initialized, 0 if uninitialized
} path_t;


//START OF AK CODE, HAD TO BE REIMPLEMENTED INTO PATH.H AND PATH.C FROM THE ORIGNAL FILE FOR COMPATIBILITY

/* read the waypoints from the file, allocate memory, determine if there are issues
* from the file
*/
int pathFromFile(const char* file_path);

/*clear the contents of the list*/
void clearList(struct path_t path);

/* reads the file sent from the keyboard interface and determines whether
 * to move in the x direction - 1, y direction - 2, z direction - 3,
 * land - 5, takeoff - 6. A 0 value means there
 * was an invalid path selection entry, default mode (no string passed) is to follow
 * the waypoint file
 */
int pathSelection(char str);

/* calculate the number of lines in a file
* using const char* means that this function is treaded as read only
* in the main function
*/
int numOfLines(const char* file_path);

/* read waypoints from the file passed in, seperate the cooridnates into 
* three arrays: waypointsX, waypointsY, waypointsZ, if the file is emtpy 
* return 0 and print out waypoint file empty. if the file contains characters 
* other than numerical characters return 0 and print out waypoint file contains
* nvalid characters, path could not be planned
*/
int pathCreation(FILE* filename);

/*pathSelection()'s return value will match the trigger value for moveX/moveY/
* moveZ/waypoints/None if it matches moveX/moveY/moveZ then it will enter the
* specified method and use pathGuide()'s matching
* generated array to set the correct path
*/
int moveX(double xPos, char str);
int moveY(double yPos, char str);
int moveZ(double xPost, char str);

/* if pathSelection() return is 5, land
*/
int land(char str);

/* if pathSelection() return is 6, take off
*/
int takeOff(char str);


/**
 * @brief       Initial values for path_t
 */
#define PATH_INITIALIZER                              \
    {                                                 \
        .waypoints = NULL, .len = 0, .initialized = 0 \
    }

extern path_t path;

#define TIME_TRANSITION_FLAG 0
#define POS_TRANSITION_FLAG 1

#endif /*__PATH__ */

/* @} Waypoints */
