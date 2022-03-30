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
    CANCEL_JOB = 0,
    GHC = 2, 
    HH = 3,
    UC = 4, 
    HOME = 5
} cmd_t;

#endif /* PARAMETERS_H_ */