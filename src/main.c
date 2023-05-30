/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

// WiFi stuff
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>

//
#include "hmc_5883l_driver.h"


/**
 * @file Sample app using the Fujitsu MB85RC256V FRAM through I2C.
 */

/* Macros ========================================================*/
/* WiFi parameters */
#define AP_SSID_ "Funeswifi"
#define AP_PSK_ "00435763252"

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* Private functions =============================================*/
static K_SEM_DEFINE(wifi_connected, 0, 1);
static K_SEM_DEFINE(ipv4_address_obtained, 0, 1);

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
    const struct wifi_status *status = (const struct wifi_status *)cb->info;

    if (status->status)
    {
        printk("Connection request failed (%d)\n", status->status);
    }
    else
    {
        printk("Connected\n");
        k_sem_give(&wifi_connected);
    }
}

static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
{
    const struct wifi_status *status = (const struct wifi_status *)cb->info;

    if (status->status)
    {
        printk("Disconnection request (%d)\n", status->status);
    }
    else
    {
        printk("Disconnected\n");
        k_sem_take(&wifi_connected, K_NO_WAIT);
    }
}

static void handle_ipv4_result(struct net_if *iface)
{
    int i = 0;

    for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {

        char buf[NET_IPV4_ADDR_LEN];

        if (iface->config.ip.ipv4->unicast[i].addr_type != NET_ADDR_DHCP) {
            continue;
        }

        printk("IPv4 address: %s\n",
                net_addr_ntop(AF_INET,
                                &iface->config.ip.ipv4->unicast[i].address.in_addr,
                                buf, sizeof(buf)));
        printk("Subnet: %s\n",
                net_addr_ntop(AF_INET,
                                &iface->config.ip.ipv4->netmask,
                                buf, sizeof(buf)));
        printk("Router: %s\n",
                net_addr_ntop(AF_INET,
                                &iface->config.ip.ipv4->gw,
                                buf, sizeof(buf)));
        }

        k_sem_give(&ipv4_address_obtained);
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
{
    switch (mgmt_event)
    {

        case NET_EVENT_WIFI_CONNECT_RESULT:
            handle_wifi_connect_result(cb);
            break;

        case NET_EVENT_WIFI_DISCONNECT_RESULT:
            handle_wifi_disconnect_result(cb);
            break;

        case NET_EVENT_IPV4_ADDR_ADD:
            handle_ipv4_result(iface);
            break;

        default:
            break;
    }
}

void wifi_connect(void)
{
    struct net_if *iface = net_if_get_default();

    struct wifi_connect_req_params wifi_params = {0};

    wifi_params.ssid = AP_SSID_;
    wifi_params.psk = AP_PSK_;
    wifi_params.ssid_length = strlen(AP_SSID_);
    wifi_params.psk_length = strlen(AP_PSK_);
    wifi_params.channel = WIFI_CHANNEL_ANY;
    wifi_params.security = WIFI_SECURITY_TYPE_PSK;
    wifi_params.band = WIFI_FREQ_BAND_2_4_GHZ; 
    wifi_params.mfp = WIFI_MFP_OPTIONAL;

    printk("Connecting to SSID: %s\n", wifi_params.ssid);

    if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &wifi_params, sizeof(struct wifi_connect_req_params)))
    {
        printk("WiFi Connection Request Failed\n");
    }
	else
	{
		printk("WiFi connect request success.");
	}
}

void wifi_status(void)
{
    struct net_if *iface = net_if_get_default();
    
    struct wifi_iface_status status = {0};

    if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,	sizeof(struct wifi_iface_status)))
    {
        printk("WiFi Status Request Failed\n");
    }

    printk("\n");

    if (status.state >= WIFI_STATE_ASSOCIATED) {
        printk("SSID: %-32s\n", status.ssid);
        printk("Band: %s\n", wifi_band_txt(status.band));
        printk("Channel: %d\n", status.channel);
        printk("Security: %s\n", wifi_security_txt(status.security));
        printk("RSSI: %d\n", status.rssi);
    }
}

void wifi_disconnect(void)
{
    struct net_if *iface = net_if_get_default();

    if (net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0))
    {
        printk("WiFi Disconnection Request Failed\n");
    }
}
/* Threads =======================================================*/

static void hmc_thread(void)
{
	const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	uint8_t hmc_buf[MAX_HMC_FIELDS_];
	uint8_t ret;
	uint8_t command[2] = {0x06, 0x03};

    if (hmc_i2cDriverInit(i2c_dev))
    {
        printk("I2C device successfully initialized.\r\n");
    }
    else
    {
        printk("Error during I2C device initialization.\r\n");
        return;
    }

	hmc_configureContinuosMode();

	for(;;)
	{
		int16_t xx, yy, zz;
		uint8_t cmd;
		k_msleep(100);


		cmd=3;
		ret = i2c_write(i2c_dev, &cmd, 1, HMC5883L_I2C_ADDR);
		if (ret)
			printk("Error on write Mode.");
		ret = i2c_read(i2c_dev, &hmc_buf[3], 3, HMC5883L_I2C_ADDR);
		if (ret)
			printk("Error on write 1st command.\r\n");
		cmd=6;
		ret = i2c_write(i2c_dev, &cmd, 1, HMC5883L_I2C_ADDR);
		if (ret)
			printk("Error on write Mode.");
		ret = i2c_read(i2c_dev, &hmc_buf[6], 3, HMC5883L_I2C_ADDR);
		if (ret)
			printk("Error on write 1st command.\r\n");
		cmd=9;
		ret = i2c_write(i2c_dev, &cmd, 1, HMC5883L_I2C_ADDR);
		if (ret)
			printk("Error on write Mode.");
		ret = i2c_read(i2c_dev, &hmc_buf[9], 4, HMC5883L_I2C_ADDR);
		if (ret)
			printk("Error on write 1st command.\r\n");

		xx = hmc_buf[3] << 8 | hmc_buf[4];
		zz = hmc_buf[5] << 8 | hmc_buf[6];
		yy = hmc_buf[7] << 8 | hmc_buf[8];

		printk("x:(%i) y:(%i) z:(%i) status:(%x)\r\n", xx, yy, zz, hmc_buf[9]);
		printk("idA:(%x) idB:(%x) idC:(%x)", hmc_buf[10], hmc_buf[11], hmc_buf[12]);
	}

#if 0
	hmc_buf[0] = 9;
	ret = i2c_write(i2c_dev, hmc_buf, 1, HMC5883L_I2C_ADDR);
	if (ret)
	{
		printk("Error on 1st write.");
	}

	ret = i2c_read(i2c_dev, hmc_buf, 4, HMC5883L_I2C_ADDR);
	if (ret)
	{
		printk("Error on 1st write.");
	}
	else
	{
		printk("Successfully read all bytes:\r\n");
		for( uint8_t i=0; i<4; ++i)
		{
			printk("field[%d]: (0x%x)\r\n", i, hmc_buf[i]);
		}
	}
#endif
}


void main(void)
{
	int sock;

    printk("WiFi Example\n\n");

    net_mgmt_init_event_callback(&wifi_cb, wifi_mgmt_event_handler,
                                 NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);

    net_mgmt_init_event_callback(&ipv4_cb, wifi_mgmt_event_handler, NET_EVENT_IPV4_ADDR_ADD);

    net_mgmt_add_event_callback(&wifi_cb);
    net_mgmt_add_event_callback(&ipv4_cb);

    wifi_connect();
    k_sem_take(&wifi_connected, K_FOREVER);
    k_sem_take(&ipv4_address_obtained, K_FOREVER);
    printk("Ready...\n\n");
    
	#if 0
    printk("Looking up IP addresses:\n");
    struct zsock_addrinfo *res;
    nslookup("iot.beyondlogic.org", &res);
    print_addrinfo_results(&res);

    printk("Connecting to HTTP Server:\n");
    sock = connect_socket(&res);
    http_get(sock, "iot.beyondlogic.org", "/test.txt");
    zsock_close(sock);
	#endif
    
    // Stay connected for 30 seconds, then disconnect.
    //wifi_disconnect();
    while(1)
    {
        printk("Looping...\r\n");
        k_sleep(K_SECONDS(10)); 
        wifi_status();
    }
}

#if 1
K_THREAD_DEFINE(hmc_id, STACKSIZE, hmc_thread, NULL, NULL, NULL,
		PRIORITY, 0, 0);
        #endif