#include "atca_hal.h"
#include "hal_cc2650_i2c_contiki.h"
#include "board-i2c.h"
#include "rtimer.h"
#include "ti-lib.h"
#include <string.h>
#include <stdio.h>

/** This implementation uses the primitives offered by Contiki.
 * In this way, sharing the code between the two implementations
 * is possible to reduce the final memory footprint.
 */

/** \brief discover i2c buses available for this hardware
 * this maintains a list of logical to physical bus mappings freeing the application
 * of the a-priori knowledge
 * \param[in] i2c_buses - an array of logical bus numbers
 * \param[in] max_buses - maximum number of buses the app wants to attempt to discover
 */
ATCA_STATUS hal_i2c_discover_buses( int i2c_buses[], int max_buses )
{
	return ATCA_UNIMPLEMENTED;
}

/** \brief discover any CryptoAuth devices on a given logical bus number
 * \param[in]  busNum  logical bus number on which to look for CryptoAuth devices
 * \param[out] cfg     pointer to head of an array of interface config structures which get filled in by this method
 * \param[out] found   number of devices found on this bus
 */
ATCA_STATUS hal_i2c_discover_devices( int busNum, ATCAIfaceCfg cfg[], int *found )
{
	return ATCA_UNIMPLEMENTED;
}

/** \brief HAL implementation of I2C init
 *
 * this implementation assumes I2C peripheral has been enabled by user. It only initialize an
 * I2C interface using given config.
 *
 *  \param[in] hal pointer to HAL specific data that is maintained by this HAL
 *  \param[in] cfg pointer to HAL specific configuration data that is used to initialize this HAL
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_i2c_init( void* hal, ATCAIfaceCfg* cfg )
{
    board_i2c_wakeup();
    return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C post init
 * \param[in] iface  instance
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_i2c_post_init( ATCAIface iface )
{
    return ATCA_SUCCESS;
}

/** \brief HAL implementation of I2C send over ASF
 * \param[in] iface     instance
 * \param[in] txdata    pointer to space to bytes to send
 * \param[in] txlength  number of bytes to send
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_i2c_send( ATCAIface iface, uint8_t* txdata, int txlength)
{
    const ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    txdata[0] = 0x03;	// insert the Word Address Value, Command token
    txlength++;		// account for word address value byte.
    board_i2c_select(DEFAULT_I2C_INTERFACE, cfg->atcai2c.slave_address >> 1);
    bool success = board_i2c_write(txdata, txlength);
    board_i2c_deselect();
    return success? ATCA_SUCCESS: ATCA_COMM_FAIL;
}

/** \brief HAL implementation of I2C receive function for ASF I2C
 * \param[in] iface     instance
 * \param[in] rxdata    pointer to space to receive the data
 * \param[in] rxlength  ptr to expected number of receive bytes to request
 * \return ATCA_STATUS
 */
ATCA_STATUS hal_i2c_receive( ATCAIface iface, uint8_t *rxdata, uint16_t *rxlength )
{
    const ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    board_i2c_select(DEFAULT_I2C_INTERFACE, cfg->atcai2c.slave_address >> 1);
    bool success = board_i2c_read(rxdata, *rxlength);
    board_i2c_deselect();
    return success? ATCA_SUCCESS: ATCA_COMM_FAIL;
}

ATCA_STATUS hal_i2c_wake( ATCAIface iface) {
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    int retries = cfg->rx_retries;
    bool status = false;
    uint8_t data[16], expected[4] = { 0x04, 0x11, 0x33, 0x43 };

    /* Send the wake up sequence using the GPIO interface for tWLO */
    ti_lib_i2c_master_disable(I2C0_BASE);
    ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_SDA);
    ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_SCL);
    ti_lib_gpio_clear_dio(BOARD_IOID_SDA);
    ti_lib_gpio_clear_dio(BOARD_IOID_SCL);
    atca_delay_us(200);

    /* Restore the value of the SDA and SCL pins and wait tWHI */
    board_i2c_wakeup();
    ti_lib_i2c_master_disable(I2C0_BASE); 
    ti_lib_ioc_io_port_pull_set(BOARD_IOID_SDA, IOC_NO_IOPULL);
    ti_lib_ioc_io_port_pull_set(BOARD_IOID_SCL, IOC_NO_IOPULL);
    ti_lib_ioc_pin_type_i2c(I2C0_BASE, BOARD_IOID_SDA, BOARD_IOID_SCL);
    ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_SDA_HP);
    ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_SCL_HP);
    ti_lib_i2c_master_init_exp_clk(I2C0_BASE, ti_lib_sys_ctrl_clock_get(),true);
    atca_delay_us(cfg->wake_delay);

    /* Try to read the wake up value from the device */
    board_i2c_select(DEFAULT_I2C_INTERFACE, cfg->atcai2c.slave_address >> 1); 
    while (retries-- > 0 && status != true){
        status = board_i2c_read(data, 4);
    }
    board_i2c_deselect();

    if (status != true) {
        printf("The read was not successfull\n");
        return ATCA_TOO_MANY_COMM_RETRIES;
    }

    if (memcmp(data, expected, 4) == 0) {
        printf("Wake up successfull\n");
        return ATCA_SUCCESS;
    }
    printf("The received data was not correct\n");
    return ATCA_COMM_FAIL;
}

static ATCA_STATUS send_single(ATCAIface iface, uint8_t single) {
    const ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    board_i2c_select(DEFAULT_I2C_INTERFACE, cfg->atcai2c.slave_address >> 1);
    bool success = board_i2c_write_single(single);
    board_i2c_deselect();
    return success? ATCA_SUCCESS: ATCA_COMM_FAIL;
}

/** \brief idle CryptoAuth device using I2C bus
 * \param[in] iface  interface to logical device to idle
 */
ATCA_STATUS hal_i2c_idle( ATCAIface iface )
{
    printf("send idle\n");
    return send_single(iface, 0x02);
}

/** \brief sleep CryptoAuth device using I2C bus
 * \param[in] iface  interface to logical device to sleep
 */
ATCA_STATUS hal_i2c_sleep( ATCAIface iface )
{
    printf("send sleep\n");
    return send_single(iface, 0x01);
}

/** \brief manages reference count on given bus and releases resource if no more refences exist
 * \param[in] hal_data - opaque pointer to hal data structure - known only to the HAL implementation
 */
ATCA_STATUS hal_i2c_release( void *hal_data )
{
    board_i2c_shutdown();
    return ATCA_SUCCESS;
}
