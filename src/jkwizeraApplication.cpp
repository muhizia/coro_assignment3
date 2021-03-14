/*******************************************************************************************************************
*   Example pick-and-place program for a LynxMotion AL5D robot arm
*   ---------------------------------------------------------------
*
*   This application implements a simple robot program to grasp a simple object (a block),
*   lift it up, and place it somewhere else.  
*   
*   The position and orientation (pose) of the object and the goal position are specified in the input file.
*   (The pickAndPlaceVision application uses a camera to determine the object pose.)
*
*   The program uses task-level programming using frames to specify the object, robot, and gripper poses.
*
*   This application reads three lines from an input file pickAndPlace.txt.
*
*   The first line contains a filename of the file with the robot calibration data, i.e. for the inverse kinematic solution.
*   This allows the program to be used with different robots (by specifying the corresponding calibration data file). 
*
*   The second line contains the object pose, i.e. the x, y, and z coordinates and the phi angle of the object (i.e. rotation about z).
*
*   The third line contains the destination pose, i.e. the x, y, and z coordinates and the phi angle of the destination (i.e. rotation about z).
*
*   It is assumed that the input file is located in a data directory given by the path ../data/ 
*   defined relative to the location of executable for this application.
*
*
*   David Vernon, Carnegie Mellon University Africa
*   4 February 2020
*
*   Audit Trail
*   -----------
*   Renamed approach and grasp frames to object_approach and object_grasp
*   Renamed setGripper() to grasp()
*   These changes were required to facilitate new re-factored implementation code
*   David Vernon
*   27 July 2020
*
*   Ported to ROS for use with the Lynxmotion_AL5D simulator in Gazebo
*   Vinny Adjibi
*   9 February 2021
*
*   The option to spawn and kill a brick in the simulator to aid with debugging is now available
*   To switch this option on, set the create_brick variable to true
*   David Vernon
*   25 February 2021
*
*   Implemented piece-wise continuous path control for approach and depart phases of both the pick and the place actions
*   To switch this option on, set the continuous_path variable to true 
*   David Vernon
*   26 February 2021
*
*   Fixed a bug with the type of the color and name strings: changed from const char* to string 
*   so that they are compatible with the parameters of the spawn_brick() and kill_brick() functions
*   David Vernon
*   4 March 2021
*
*   Adapted the program for 3 bricks. The number of bricks may also be read from the data file. The size and
*   content of the arrays name and colors should be changed to match the number of bricks
*   Jean Baptiste Kwizera
*   13 March 2021
*******************************************************************************************************************/

#include <stdlib.h>
#include <time.h>
#ifdef WIN32
    #include "pickAndPlace.h"
#else
    #include <assignment3/jkwizera.h>
#endif

int main(int argc, char ** argv) {
  
   #ifdef ROS
       ros::init(argc, argv, "jkwizera"); // Initialize the ROS system
   #endif

   extern robotConfigurationDataType robotConfigurationData;
   
   bool debug = true;
   
   FILE *fp_in;                    // pickAndPlace input file
   int  end_of_file; 
   char robot_configuration_filename[MAX_FILENAME_LENGTH];
   char filename[MAX_FILENAME_LENGTH] = {};
   char directory[MAX_FILENAME_LENGTH] = {};

   /* Frame objects */
   
   Frame E;
   Frame Z;
   Frame T6;
   Frame object;
   Frame object_grasp;
   Frame object_approach;
   Frame object_depart;
   Frame destination;

   /* data variables */

   float effector_length;           // this is initialized from robot configuration file

   float object_x          = -40;   // default values; actual values are read from the input file
   float object_y          = 150;   //                         
   float object_z          =   0;   //                         
   float object_phi        = -90;   // rotation in degrees about the z (vertical) axis 
      
   float destination_x     =  40;   // default values; actual values are read from the input file                        
   float destination_y     = 150;
   float destination_z     =   0;
   float destination_phi   = -90;   // rotation in degrees about the z (vertical) axis 

   float grasp_x           =   0;   // grasp pose relative to object and destination poses                        
   float grasp_y           =   0;
   float grasp_z           =   5;
   float grasp_theta       = 180;   // rotation in degrees about the y axis 
      
   float approach_distance;         // approach  distance from grasp pose in -z direction
   float depart_distance;           // departure distance from grasp pose in -z direction
   float initial_approach_distance; // start the approach from this distance
   float final_depart_distance;     // start the approach from this distance
   float delta;                     // increment in approach and depart distance 

   bool continuous_path = true;    // if true, implement approximation of continuous path control
                                    // when approaching and departing the grasp pose
                                    // otherwise just move directly from the initial approach pose to the grasp pose
                                    // and directly from the grasp pose to the final depart pose 

#ifdef ROS   
   bool create_brick = true;       // if true, spawn a brick at the specified location
   
   string name[3]    = {"brick1", "brick2", "brick3"};    // name and colors for option to spawn and kill a brick
   string colors[3]  = {"red", "green", "blue"};
#endif

   
   /* open the input file */
   /* ------------------- */

   /* Set the filename. Different directories for ROS and Windows versions */
   
#ifdef ROS
    strcat(directory, (ros::package::getPath(ROS_PACKAGE_NAME) + "/data/").c_str());
#else
    strcat(directory, "../data/");// On Windows the exec is in bin, so we go in the parent directory first
#endif

    strcpy(filename, directory);
    strcat(filename, "assignment3Input.txt"); // Input filename matches the application name
    if ((fp_in = fopen(filename, "r")) == 0) {
        printf("Error can't open input assignment3Input.txt\n");
        prompt_and_exit(0);
    }

   
    /* get the robot configuration data */
    /* -------------------------------- */

    end_of_file = fscanf(fp_in, "%s", robot_configuration_filename); // read the configuration filename   
    if (end_of_file == EOF) {   
	    printf("Fatal error: unable to read the robot configuration filename\n");
        prompt_and_exit(1);
    }
    if (debug) printf("Robot configuration filename %s\n", robot_configuration_filename);

    strcpy(filename, robot_configuration_filename);
    strcpy(robot_configuration_filename, directory);
    strcat(robot_configuration_filename, filename);

    readRobotConfigurationData(robot_configuration_filename);

   
    /* get the object pose data */
    /* ------------------------ */
    const int NUM_BRICKS = 3;           // we have 3 bricks. this parameter should perhaps be read from the input file?
                                        // name and colors arrays should be of size NUM_BRICKS

    float bricks_pose[NUM_BRICKS][4];   // read and store the bricks poses in an array
    for (int i = 0; i < NUM_BRICKS; i++) { 
        end_of_file = fscanf(fp_in, "%f %f %f %f", &object_x, &object_y, &object_z, &object_phi);
        if (end_of_file == EOF) {   
	        printf("Fatal error: unable to read the object position and orientation\n");
            prompt_and_exit(1);
        }
        if (debug) printf("Object pose %f %f %f %f\n", object_x, object_y, object_z, object_phi);
        bricks_pose[i][0] = object_x;
        bricks_pose[i][1] = object_y;
        bricks_pose[i][2] = object_z;
        bricks_pose[i][3] = object_phi;
    }
 
    /* get the destination pose data */
    /* ----------------------------- */

    end_of_file = fscanf(fp_in, "%f %f %f %f", &destination_x, &destination_y, &destination_z, &destination_phi);
    if (end_of_file == EOF) {   
	    printf("Fatal error: unable to read the destination position and orientation\n");
        prompt_and_exit(1);
    }
    if (debug) printf("Destination pose %f %f %f %f\n", destination_x, destination_y, destination_z, destination_phi);
   
    float bricks_dest_z[NUM_BRICKS]; // each bricks destination z coordinate will depend on its position in the stack given
    for (int i = 0; i < NUM_BRICKS; i++)
        bricks_dest_z[i] = destination_z + i*11.4; // all bricks have a height of 11.4 mm
    
    
#ifdef ROS
    for (int i = 0; i < NUM_BRICKS; i++) {
        /* If we are using the simulator on ROS, we can instantiate the brick here to help with debugging */
        /* Normally, we would instantiate the brick using the terminal to mimic the way we would do it    */
        /* when using the physical robot, i.e. manually positioning it for the robot to pick and place    */

        if (create_brick) {
        
            /* Spawn the brick at the specified position */
            /* Use the brick's name and corresponding color */

            srand(time(NULL));

            if (debug) {
                printf("Spawning brick with name %s at position (%.2f %.2f %.2f %.2f)\n",
	            name[i].c_str(), bricks_pose[i][0], bricks_pose[i][1], bricks_pose[i][2], bricks_pose[i][3]);
            }
     
            /* Call the utility function */
       
            spawn_brick(name[i], colors[i], bricks_pose[i][0], bricks_pose[i][1], bricks_pose[i][2], bricks_pose[i][3]);
        }
    }

    /* Call the utility function to pick and place the spawned bricks */
    for (int i = 0; i < NUM_BRICKS; i++) {
        pick_and_place(bricks_pose[i][0], bricks_pose[i][1], bricks_pose[i][2], bricks_pose[i][3],
                       destination_x, destination_y, bricks_dest_z[i], destination_phi,
                       grasp_x, grasp_y, grasp_z, grasp_theta);
    }

    
    if (create_brick) {
        
        prompt_and_continue();
	
        /* Remove the bricks for the next time  */
        /* ----------------------------------- */
        
        if (debug) {
            for (int i = 0; i < NUM_BRICKS; i++)
                printf("Killing brick named %s\n",name[i].c_str());
        }
        
        /* Call the utility function */

        for (int i = 0; i < NUM_BRICKS; i++)
            kill_brick(name[i]);
    }
#endif
   
    return 0;
}
