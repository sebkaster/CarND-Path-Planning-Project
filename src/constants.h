//
// Created by sebas on 23/12/2019.
//

#ifndef CARND_PATH_PLANNING_PROJECT_CONSTANTS_H
#define CARND_PATH_PLANNING_PROJECT_CONSTANTS_H


/* define constants */
const double TIME_STEP_SIZE = 0.02; // s
const double NUM_TIME_STEPS = 25.0;
const double SPEED_LIMIT = 22.0; // m/s
const double SPEED_LIMIT_LANE = 27.0; // m/s
const double MAX_ABS_ACC_EGO_VEHICLE = 5.0 * TIME_STEP_SIZE; // m/s^2
const double MAX_ACC_VEHICLES = 10.0; // m/s^2
const double MAX_DEC_VEHICLES = -10.0; // m/s^2
const double SAFETY_DISTANCE = 20.0; // m
const double SPEED_BUFFER = 1.0; // m/s
const double DISTANCE_BUFFER = 10.0; // m
const unsigned NUM_THREADS = 4;

#endif //CARND_PATH_PLANNING_PROJECT_CONSTANTS_H
