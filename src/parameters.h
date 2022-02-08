/**
 * @file parameters.h
 *
 * @brief Define parameters
 *
 * @date 02/05/2021
 *
 * @author Tina Tian (yutian)  
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

typedef enum cmd_type {
    NEW_JOB,
    CANCEL_JOB
} cmd_t;

typedef enum goal_num {
    ROOM_1,
    ROOM_2,
    ROOM_3
} goal_t;

#endif /* PARAMETERS_H_ */