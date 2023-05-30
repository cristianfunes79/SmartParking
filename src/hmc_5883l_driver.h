#ifndef __HMC_5883L_DRIVER_H__
#define __HMC_5883L_DRIVER_H__

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// Macros ==========================================================

/* I2C device address */
#define HMC5883L_I2C_ADDR	0x1E

// Data types ======================================================
/* Hmc internal registers */
enum hmc_5883l_Registers
{
    eHMC5883L_REG_CONFIG_A,
    eHMC5883L_REG_CONFIG_B,
    eHMC5883L_REG_MODE,
    eHMC5883L_REG_DATA_OUTPUT_X_MSB,
    eHMC5883L_REG_DATA_OUTPUT_X_LSB,
    eHMC5883L_REG_DATA_OUTPUT_Z_MSB,
    eHMC5883L_REG_DATA_OUTPUT_Z_LSB,
    eHMC5883L_REG_DATA_OUTPUT_Y_MSB,
    eHMC5883L_REG_DATA_OUTPUT_Y_LSB,
    eHMC5883L_REG_STATUS,
    eHMC5883L_REG_ID_A,
    eHMC5883L_REG_ID_B,
    eHMC5883L_REG_ID_C,
    eHMC5883L_MAX_REGISTERS
};

/* Driver data type */
typedef struct hmc_5883l_driver_s hmc_5883l_driver_t;
struct hmc_5883l_driver_s
{
    void*   instance;
    uint8_t registers[eHMC5883L_MAX_REGISTERS];

    int (*read) (hmc_5883l_driver_t*, uint8_t*, uint8_t);
    int (*write)(hmc_5883l_driver_t*, uint8_t*, uint8_t);
};

// Public API ======================================================

/// \brief Initializes i2c driver.
/// \param driver hmc driver structure pointer.
/// \param instance pointer to hardware handler instance.
/// \param read callback to read function.
/// \param write callback to write function.
/// \retval 0 if initialization was success and less than 0 otherwise.
int hmc_driverInit( hmc_5883l_driver_t* driver, void* instance, 
                        int (*read) (hmc_5883l_driver_t*, uint8_t*, uint8_t),
                        int (*write)(hmc_5883l_driver_t*, uint8_t*, uint8_t));


/// \brief Configures hmc device in continues mode.
/// \param driver hmc driver structure pointer.
/// \retval 0 if success, less than 0 if error.
int hmc_configureContinuosMode(hmc_5883l_driver_t* driver);


/// \brief Reads hmc X field component.
/// \param driver hmc driver structure pointer.
/// \retval Raw data containing X field component raw value.
uint16_t hmc_readX_raw(hmc_5883l_driver_t* driver);


/// \brief Reads hmc Y field component.
/// \param driver hmc driver structure pointer.
/// \retval Raw data containing Y field component raw value.
uint16_t hmc_readY_raw(hmc_5883l_driver_t* driver);


/// \brief Reads hmc Z field component.
/// \param driver hmc driver structure pointer.
/// \retval Raw data containing Z field component raw value.
uint16_t hmc_readZ_raw(hmc_5883l_driver_t* driver);


/// \brief Reads hmc X field component.
/// \param driver hmc driver structure pointer.
/// \retval Raw data containing X field component value.
int hmc_readX(hmc_5883l_driver_t* driver);


/// \brief Reads hmc Y field component.
/// \param driver hmc driver structure pointer.
/// \retval Raw data containing Y field component value.
int hmc_readY(hmc_5883l_driver_t* driver);


/// \brief Reads hmc Z field component.
/// \param driver hmc driver structure pointer.
/// \retval Raw data containing Z field component value.
int hmc_readZ(hmc_5883l_driver_t* driver);


// Initialization Macros =============================================

#define HMC_5883L_DRIVER_INIT_DEFAULT { \
    .instance = NULL,                   \
    .registers = {0},                   \
    .read = NULL,                       \
    .write = NULL                       \
}

#endif