#include "atca_hal.h"
#include "hal_cc2650_i2c.h"
#include "ti-lib.h"
#include "lpm.h"
#include "rtimer.h"
#include <string.h>

/** This implementation do not uses direcly the Contiki i2c module
 * to interact with the i2c device but it is inspired from it.
 * To make this code completely indipendent from Contiki you need
 * to implement the timer according to your platform or using the
 * hw timer api provided in driverlib/timer.h */

#define I2C_MAX_WAIT_TIME    (RTIMER_SECOND / 10)

#define LIMITED_BUSYWAIT(cond) do {                   \
    rtimer_clock_t end_time = RTIMER_NOW() + I2C_MAX_WAIT_TIME; \
    while(cond) {                                               \
        if(!RTIMER_CLOCK_LT(RTIMER_NOW(), end_time)) {          \
            return false;                                       \
        }                                                       \
    }                                                           \
} while(0)

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
   ti_lib_prcm_power_domain_on(PRCM_DOMAIN_SERIAL);
   while((ti_lib_prcm_power_domain_status(PRCM_DOMAIN_SERIAL)
         != PRCM_DOMAIN_POWER_ON));

   ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_I2C0);
   ti_lib_prcm_load_set();
   while(!ti_lib_prcm_load_get());

   ti_lib_i2c_master_init_exp_clk(I2C0_BASE, ti_lib_sys_ctrl_clock_get(),
                                  true);
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

static bool i2c_status() {
    uint32_t status;

    status = ti_lib_i2c_master_err(I2C0_BASE);
    if(status & (I2C_MSTAT_DATACK_N_M | I2C_MSTAT_ADRACK_N_M)) {
        ti_lib_i2c_master_control(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
    }

    return status == I2C_MASTER_ERR_NONE;
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
    bool success;
    uint32_t i;

    txdata[0] = 0x03;	/* Insert the Word Address Value, Command token */
    txlength++;		    /* account for word address value byte. */

    ti_lib_i2c_master_slave_addr_set(I2C0_BASE, cfg->atcai2c.slave_address >> 1, false);

    ti_lib_i2c_master_data_put(I2C0_BASE, txdata[0]);

    LIMITED_BUSYWAIT(ti_lib_i2c_master_bus_busy(I2C0_BASE));

    /* Assert RUN + START */
    ti_lib_i2c_master_control(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    LIMITED_BUSYWAIT(ti_lib_i2c_master_busy(I2C0_BASE));
    success = i2c_status();

    for(i = 1; i < txlength && success; i++) {
        ti_lib_i2c_master_data_put(I2C0_BASE, txdata[i]);
        if(i < txlength - 1) {
            ti_lib_i2c_master_control(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
            LIMITED_BUSYWAIT(ti_lib_i2c_master_busy(I2C0_BASE));
            success = i2c_status();
        }
    }

    if(success) {
        /* Assert STOP */
        ti_lib_i2c_master_control(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        LIMITED_BUSYWAIT(ti_lib_i2c_master_busy(I2C0_BASE));
        success = i2c_status();
        LIMITED_BUSYWAIT(ti_lib_i2c_master_bus_busy(I2C0_BASE));
    }
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
    bool success = false;
    uint8_t i;
    int retries = cfg->rx_retries;
    ti_lib_i2c_master_slave_addr_set(I2C0_BASE, cfg->atcai2c.slave_address >> 1, true);
    while (retries-- > 0 && !success) {
        /* Wait 5 ms between one read and the other */
        atca_delay_ms(5);

        /* Check if another master has access */
        LIMITED_BUSYWAIT(ti_lib_i2c_master_bus_busy(I2C0_BASE));
        /* Assert RUN + START + ACK */
        ti_lib_i2c_master_control(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

        i = 0;
        success = true;
        while(i < (*rxlength - 1) && success) {
            LIMITED_BUSYWAIT(ti_lib_i2c_master_busy(I2C0_BASE));
            success = i2c_status();
            if(success) {
                rxdata[i] = ti_lib_i2c_master_data_get(I2C0_BASE);
                ti_lib_i2c_master_control(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                i++;
            }
        }

        if(success) {
            ti_lib_i2c_master_control(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
            LIMITED_BUSYWAIT(ti_lib_i2c_master_busy(I2C0_BASE));
            success = i2c_status();
            if(success) {
                rxdata[*rxlength - 1] = ti_lib_i2c_master_data_get(I2C0_BASE);
                LIMITED_BUSYWAIT(ti_lib_i2c_master_bus_busy(I2C0_BASE));
            }
        }
    }
    if (!success) {
        return ATCA_COMM_FAIL;
    }
    if (atCheckCrc(rxdata) != ATCA_SUCCESS) {
        return ATCA_COMM_FAIL;
    }
    return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_wake( ATCAIface iface) {
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    int retries = cfg->rx_retries;
    ATCA_STATUS status = !ATCA_SUCCESS;
    uint16_t readlen = 4;
    uint8_t data[4], expected[4] = { 0x04, 0x11, 0x33, 0x43 };

    /* Send the wake up sequence using the GPIO interface for tWLO */
    ti_lib_i2c_master_disable(I2C0_BASE);
    ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_SDA);
    ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_SCL);
    ti_lib_gpio_clear_dio(BOARD_IOID_SDA);
    ti_lib_gpio_clear_dio(BOARD_IOID_SCL);
    atca_delay_us(200);

    /* Restore the value of the SDA and SCL pins and wait tWHI */
    /* First, make sure the SERIAL PD is on */
    ti_lib_prcm_power_domain_on(PRCM_DOMAIN_SERIAL);
    while((ti_lib_prcm_power_domain_status(PRCM_DOMAIN_SERIAL)
                != PRCM_DOMAIN_POWER_ON));

    /* Enable the clock to I2C */
    ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_I2C0);
    ti_lib_prcm_load_set();
    while(!ti_lib_prcm_load_get());

    /* Enable and initialize the I2C master module */
    ti_lib_i2c_master_init_exp_clk(I2C0_BASE, ti_lib_sys_ctrl_clock_get(),
            true);
    ti_lib_i2c_master_disable(I2C0_BASE); 
    ti_lib_ioc_io_port_pull_set(BOARD_IOID_SDA, IOC_NO_IOPULL);
    ti_lib_ioc_io_port_pull_set(BOARD_IOID_SCL, IOC_NO_IOPULL);
    ti_lib_ioc_pin_type_i2c(I2C0_BASE, BOARD_IOID_SDA, BOARD_IOID_SCL);
    /* Set the other I2C bus of the cc2650 as a GPIO input */
    ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_SDA_HP);
    ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_SCL_HP);
    ti_lib_i2c_master_init_exp_clk(I2C0_BASE, ti_lib_sys_ctrl_clock_get(),true);
    atca_delay_us(cfg->wake_delay);

    /* Try to read the wake up value from the device */
    while (retries-- > 0 && status != ATCA_SUCCESS){
        status = hal_i2c_receive(iface, data, &readlen);
    }

    if (status != ATCA_SUCCESS) {
        return ATCA_TOO_MANY_COMM_RETRIES;
    }

    if (memcmp(data, expected, 4) == 0) {
        return ATCA_SUCCESS;
    }
    return ATCA_COMM_FAIL;
}

static ATCA_STATUS send_single(ATCAIface iface, uint8_t single) {
    ATCAIfaceCfg *cfg = atgetifacecfg(iface);
    /* Write slave address */
    ti_lib_i2c_master_slave_addr_set(I2C0_BASE, cfg->atcai2c.slave_address >> 1, false);

    /* Write first byte */
    ti_lib_i2c_master_data_put(I2C0_BASE, single);

    /* Check if another master has access */
    LIMITED_BUSYWAIT(ti_lib_i2c_master_bus_busy(I2C0_BASE));

    /* Assert RUN + START + STOP */
    ti_lib_i2c_master_control(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    LIMITED_BUSYWAIT(ti_lib_i2c_master_busy(I2C0_BASE));
    return i2c_status()? ATCA_SUCCESS: ATCA_COMM_FAIL;
}

/** \brief idle CryptoAuth device using I2C bus
 * \param[in] iface  interface to logical device to idle
 */
ATCA_STATUS hal_i2c_idle( ATCAIface iface )
{
    return send_single(iface, 0x02);
}

/** \brief sleep CryptoAuth device using I2C bus
 * \param[in] iface  interface to logical device to sleep
 */
ATCA_STATUS hal_i2c_sleep( ATCAIface iface )
{
    return send_single(iface, 0x01);
}

/** \brief manages reference count on given bus and releases resource if no more refences exist
 * \param[in] hal_data - opaque pointer to hal data structure - known only to the HAL implementation
 */
ATCA_STATUS hal_i2c_release( void *hal_data )
{
    ti_lib_i2c_master_disable(I2C0_BASE);
    ti_lib_prcm_peripheral_run_disable(PRCM_PERIPH_I2C0);
    ti_lib_prcm_load_set();
    while(!ti_lib_prcm_load_get());
    ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_SDA);
    ti_lib_ioc_io_port_pull_set(BOARD_IOID_SDA, IOC_IOPULL_UP);
    ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_SCL);
    ti_lib_ioc_io_port_pull_set(BOARD_IOID_SCL, IOC_IOPULL_UP);
    return ATCA_SUCCESS;
}
