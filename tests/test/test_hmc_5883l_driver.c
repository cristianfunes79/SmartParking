#include "unity.h"

#include "mock_fake_i2c.h"
#include "hmc_5883l_driver.h"

// Aux functions ==========================================================================
static int fake_read_wrapper(hmc_5883l_driver_t* driver, uint8_t* data, uint8_t len)
{
    return fakeRead(driver, data, len);
}

static int fake_write_wrapper(hmc_5883l_driver_t* driver, uint8_t* data, uint8_t len)
{
    return fakeWrite(driver, data, len);
}

// Tests ===================================================================================

/*
    Tests if driver is initialized properly when all parameters are ok.
*/
void test_hmc_5883l_driver_init_successfully(void)
{
    hmc_5883l_driver_t hmc_driver = HMC_5883L_DRIVER_INIT_DEFAULT;
    void* fake_instance;

    TEST_ASSERT_NULL(hmc_driver.instance);
    TEST_ASSERT_NULL(hmc_driver.read);
    TEST_ASSERT_NULL(hmc_driver.write);
    TEST_ASSERT_EACH_EQUAL_UINT8(0, hmc_driver.registers, eHMC5883L_MAX_REGISTERS);

    TEST_ASSERT_EQUAL_INT(hmc_driverInit(&hmc_driver, fake_instance, fake_read_wrapper, fake_write_wrapper), 0);

    TEST_ASSERT_EQUAL_PTR(hmc_driver.instance, fake_instance);
    TEST_ASSERT_EQUAL_PTR(hmc_driver.read, fake_read_wrapper);
    TEST_ASSERT_EQUAL_PTR(hmc_driver.write, fake_write_wrapper);
}

/*
    Tests if initialization function returns error when any parameter is NULL
*/
void test_hmc_5883l_driver_init_with_some_parameters_invalid(void)
{
    hmc_5883l_driver_t hmc_driver = HMC_5883L_DRIVER_INIT_DEFAULT;
    void* fake_instance;

    TEST_ASSERT_EQUAL_INT(hmc_driverInit(NULL, fake_instance, fake_read_wrapper, fake_write_wrapper), -1);
    TEST_ASSERT_EQUAL_INT(hmc_driverInit(&hmc_driver, NULL, fake_read_wrapper, fake_write_wrapper), -1);
    TEST_ASSERT_EQUAL_INT(hmc_driverInit(&hmc_driver, fake_instance, NULL, fake_write_wrapper), -1);
    TEST_ASSERT_EQUAL_INT(hmc_driverInit(&hmc_driver, fake_instance, fake_read_wrapper, NULL), -1);

    TEST_ASSERT_NULL(hmc_driver.instance);
    TEST_ASSERT_NULL(hmc_driver.read);
    TEST_ASSERT_NULL(hmc_driver.write);
    TEST_ASSERT_EACH_EQUAL_UINT8(0, hmc_driver.registers, eHMC5883L_MAX_REGISTERS);
}

/*
    Tests if continuos mode is set properly. Real hardware is not used here but 
    we can check if all commands are properly send.
*/
void test_hmc_5883l_configure_continuos_mode_successfully(void)
{
    hmc_5883l_driver_t hmc_driver = HMC_5883L_DRIVER_INIT_DEFAULT;
    void* fake_instance;

    hmc_driverInit(&hmc_driver, fake_instance, fake_read_wrapper, fake_write_wrapper);

    uint8_t command_0[2] = {eHMC5883L_REG_CONFIG_A, 0x70};
    uint8_t command_1[2] = {eHMC5883L_REG_CONFIG_B, 0xE0};
    uint8_t command_2[2] = {eHMC5883L_REG_MODE,     0x00};

    fakeWrite_ExpectWithArrayAndReturn(&hmc_driver, 1, command_0, 2, 2, 0);
    fakeWrite_ExpectWithArrayAndReturn(&hmc_driver, 1, command_1, 2, 2, 0);
    fakeWrite_ExpectWithArrayAndReturn(&hmc_driver, 1, command_2, 2, 2, 0);

    TEST_ASSERT_EQUAL_INT(hmc_configureContinuosMode(&hmc_driver), 0);
}

/*
    Test if configuration function returns error as soon as it fails during command write.
*/
void test_hmc_5883l_configure_continuos_mode_with_some_write_error(void)
{
    hmc_5883l_driver_t hmc_driver = HMC_5883L_DRIVER_INIT_DEFAULT;
    void* fake_instance;

    hmc_driverInit(&hmc_driver, fake_instance, fake_read_wrapper, fake_write_wrapper);

    uint8_t command_0[2] = {eHMC5883L_REG_CONFIG_A, 0x70};
    uint8_t command_1[2] = {eHMC5883L_REG_CONFIG_B, 0xE0};
    uint8_t command_2[2] = {eHMC5883L_REG_MODE,     0x00};

    fakeWrite_ExpectWithArrayAndReturn(&hmc_driver, 1, command_0, 2, 2, 0);
    fakeWrite_ExpectWithArrayAndReturn(&hmc_driver, 1, command_1, 2, 2, -2);

    TEST_ASSERT_EQUAL_INT(hmc_configureContinuosMode(&hmc_driver), -1);
}

/*
    Test if raw data returned for X field is ok.
*/
void test_hmc_5883l_read_raw_x_successfully(void)
{
    hmc_5883l_driver_t hmc_driver = HMC_5883L_DRIVER_INIT_DEFAULT;
    void* fake_instance;
    uint8_t pData[2] = {0x1, 0x2};

    hmc_driverInit(&hmc_driver, fake_instance, fake_read_wrapper, fake_write_wrapper);

    uint8_t command[1] = {eHMC5883L_REG_DATA_OUTPUT_X_MSB};

    fakeWrite_ExpectWithArrayAndReturn(&hmc_driver, 1, command, 1, 1, 0);
    fakeRead_ExpectWithArrayAndReturn(&hmc_driver, 1, &hmc_driver.registers[eHMC5883L_REG_DATA_OUTPUT_X_MSB], 1, 2, 0);
    fakeRead_ReturnArrayThruPtr_data(pData, 2);

    TEST_ASSERT_EQUAL_UINT16((uint16_t)(pData[0] << 8 | pData[1]), hmc_readX_raw(&hmc_driver));
}

/*
    Test if raw data returned for Y field is ok.
*/
void test_hmc_5883l_read_raw_y_successfully(void)
{
    hmc_5883l_driver_t hmc_driver = HMC_5883L_DRIVER_INIT_DEFAULT;
    void* fake_instance;
    uint8_t pData[2] = {0x4, 0x1};

    hmc_driverInit(&hmc_driver, fake_instance, fake_read_wrapper, fake_write_wrapper);

    uint8_t command[1] = {eHMC5883L_REG_DATA_OUTPUT_Y_MSB};

    fakeWrite_ExpectWithArrayAndReturn(&hmc_driver, 1, command, 1, 1, 0);
    fakeRead_ExpectWithArrayAndReturn(&hmc_driver, 1, &hmc_driver.registers[eHMC5883L_REG_DATA_OUTPUT_Y_MSB], 1, 2, 0);
    fakeRead_ReturnArrayThruPtr_data(pData, 2);

    TEST_ASSERT_EQUAL_UINT16((uint16_t)(pData[0] << 8 | pData[1]), hmc_readY_raw(&hmc_driver));
}

/*
    Test if raw data returned for Z field is ok.
*/
void test_hmc_5883l_read_raw_z_successfully(void)
{
    hmc_5883l_driver_t hmc_driver = HMC_5883L_DRIVER_INIT_DEFAULT;
    void* fake_instance;
    uint8_t pData[2] = {0x45, 0x21};

    hmc_driverInit(&hmc_driver, fake_instance, fake_read_wrapper, fake_write_wrapper);

    uint8_t command[1] = {eHMC5883L_REG_DATA_OUTPUT_Z_MSB};

    fakeWrite_ExpectWithArrayAndReturn(&hmc_driver, 1, command, 1, 1, 0);
    fakeRead_ExpectWithArrayAndReturn(&hmc_driver, 1, &hmc_driver.registers[eHMC5883L_REG_DATA_OUTPUT_Z_MSB], 1, 2, 0);
    fakeRead_ReturnArrayThruPtr_data(pData, 2);

    TEST_ASSERT_EQUAL_UINT16((uint16_t)(pData[0] << 8 | pData[1]), hmc_readZ_raw(&hmc_driver));
}
