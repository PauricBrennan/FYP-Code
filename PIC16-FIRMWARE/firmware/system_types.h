/**
 * I2C Generated Driver Types Header File
 *
 * @file i2c_client_types.h
 *
 * @ingroup i2c_client_interface
 *
 * @brief This file contains additional data types for the I2C module.
 *
 * @version I2C Driver Version 2.1.0
 */

#ifndef SYSTEM_TYPES_H
#define	SYSTEM_TYPES_H

/**
 * @ingroup i2c_client_interface
 * @enum i2c_client_transfer_event_t
 * @brief I2C notification event type
 */
typedef enum {
    I2C_NO_EVENT = 0,                         /**< I2C Bus Idle state */
    I2C_ADDR_MATCH_EVENT,                     /**< Address match event */
    I2C_RX_EVENT ,                            /**< Data sent by the I2C Host is available */
    I2C_TX_EVENT,                             /**< I2C client can respond to data read request from the I2C Host */
    I2C_STOP_BIT_EVENT,                       /**< I2C Stop bit received */
    I2C_NO_ERROR_EVENT,                       /**< I2C error none */        
    I2C_BUS_COLLISION_ERROR_EVENT,            /**< I2C Bus collision occurred */
    I2C_WRITE_COLLISION_ERROR_EVENT,        
} i2c_events;

typedef enum {
    I2C_NO_COMMAND = 0,                         /**< I2C Bus Idle state */
    I2C_MOTOR_FORWARD_COMMAND,                     /**< Address match event */
    I2C_MOTOR_REVERSE_COMMAND,                            /**< Data sent by the I2C Host is available */
    I2C_PWM_POLARITY_INV_COMMAND,                             /**< I2C client can respond to data read request from the I2C Host */
    I2C_PWM_POLARITY_NOT_INV_COMMAND,        
} i2c_commands;


#endif

