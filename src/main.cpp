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
#include <zephyr/drivers/adc.h>

// WiFi stuff
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>

// TCP
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/websocket.h>
#include "esp_sleep.h"

// SNTP
#include <zephyr/net/sntp.h>
#ifdef CONFIG_POSIX_API
#include <arpa/inet.h>
#endif

#include <zephyr/posix/time.h>

//
#include "hmc_5883l_driver_port.h"

// CPP
#include <iostream>
#include <string>

// Power Management
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

static bool flagIP = false;

struct hmc_values_s
{
   int x;
   int y;
   int z;
   int sample = 123; 
};

typedef struct hmc_values_s hmc_values_t;
static hmc_values_t hmc_values;

K_SEM_DEFINE(hmc_sem, 0, K_SEM_MAX_LIMIT);
K_SEM_DEFINE(wifi_sem, 0, K_SEM_MAX_LIMIT);


/**
 * 
 * Socket stuff
*/

static int setup_socket(sa_family_t family, const char *server, int port,
			int *sock, struct sockaddr *addr, socklen_t addr_len)
{
	const char *family_str = "IPv4";
	int ret = 0;

	memset(addr, 0, addr_len);

    net_sin(addr)->sin_family = AF_INET;
    net_sin(addr)->sin_port = htons(port);
    inet_pton(AF_INET, server, &net_sin(addr)->sin_addr);

    *sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (*sock < 0) {
		printk("Failed to create %s HTTP socket (%d)", family_str,
			-errno);
	}
    else
    {
        printk("Socket successfully created.\n");
    }

	return ret;
}

static int connect_socket(sa_family_t family, const char *server, int port,
			  int *sock, struct sockaddr *addr, socklen_t addr_len)
{
	int ret;

	ret = setup_socket(family, server, port, sock, addr, addr_len);
	if (ret < 0 || *sock < 0) {
		return -1;
	}

    printk("Connecting...\n");
	ret = connect(*sock, addr, addr_len);
    printk("Done!\n");
	if (ret < 0) {
		printk("Cannot connect to %s remote (%d)\n",
			"IPv4",
			-errno);
		ret = -errno;
	}
    else
    {
        printk("Socket connected.\n");
    }

	return ret;
}

static int connect_cb(int sock, struct http_request *req, void *user_data)
{
	printk("Websocket %d for %s connected.", sock, (char *)user_data);

	return 0;
}


/**
 * @file Sample app using the Fujitsu MB85RC256V FRAM through I2C.
 */

/* Macros ========================================================*/
/* WiFi parameters */
#define AP_SSID_ "Funeswifi"
#define AP_PSK_ "00435763252"

/* size of stack area used by each thread */
#define STACKSIZE 2048

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
        //k_sem_give(&wifi_connected);
    }
}

static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
{
    const struct wifi_status *status = (const struct wifi_status *)cb->info;

    if (status->status)
    {
        printk("Disconnection request (%d)\n", status->status);
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

        //k_sem_give(&ipv4_address_obtained);
        flagIP = true;
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
{
    switch (mgmt_event)
    {

        case NET_EVENT_WIFI_CONNECT_RESULT:
            printk("Received NET_EVENT_WIFI_CONNECT_RESULT.\n");
            handle_wifi_connect_result(cb);
            break;

        case NET_EVENT_WIFI_DISCONNECT_RESULT:
            printk("Received NET_EVENT_WIFI_DISCONNECT_RESULT.\n");
            handle_wifi_disconnect_result(cb);
            break;

        case NET_EVENT_IPV4_ADDR_ADD:
            printk("Received NET_EVENT_IPV4_ADDR_ADD.\n");
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

    wifi_params.ssid = reinterpret_cast<uint8_t *>(const_cast<char*>(AP_SSID_));
    wifi_params.psk = reinterpret_cast<uint8_t*>(const_cast<char*>(AP_PSK_));
    wifi_params.ssid_length = strlen(AP_SSID_);
    wifi_params.psk_length = strlen(AP_PSK_);
    wifi_params.channel = WIFI_CHANNEL_ANY;
    wifi_params.security = WIFI_SECURITY_TYPE_PSK;
    wifi_params.band = WIFI_FREQ_BAND_2_4_GHZ; 
    wifi_params.mfp = WIFI_MFP_DISABLE;
    wifi_params.timeout = SYS_FOREVER_MS;

    printk("Connecting to SSID: %s\n", wifi_params.ssid);

    if (net_mgmt(NET_REQUEST_WIFI_AP_DISABLE, iface, &wifi_params, sizeof(struct wifi_connect_req_params)))
    {
       printk("WiFi in AP Mode Disable Error.\n");
    }
    else
    {
       printk("WiFi in AP Mode Disable.\n");
    }

    if (net_mgmt(NET_REQUEST_WIFI_SCAN, iface, &wifi_params, sizeof(struct wifi_connect_req_params)))
    {
       printk("WiFi Scan Request Error.\n");
    }
    else
    {
       printk("WiFi Scan Request Done!.\n");
    }

    if (net_mgmt_event_wait_on_iface(iface, NET_EVENT_WIFI_SCAN_RESULT, NULL, NULL, NULL, K_FOREVER))
        printk("Scan done!\n");

    if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &wifi_params, sizeof(struct wifi_connect_req_params)))
    {
        printk("WiFi Connection Request Failed\n");
    }
	else
	{
		printk("WiFi Connection Request success.\n");
	}

#if 0
    if (net_mgmt_event_wait_on_iface(iface, NET_EVENT_WIFI_CONNECT_RESULT, NULL, NULL, NULL, K_FOREVER))
        printk("WiFi connected.\n");
#endif
}

void wifi_status(void)
{
    struct net_if *iface = net_if_get_default();

    struct wifi_iface_status status = {0};

    if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,	sizeof(struct wifi_iface_status)))
    {
        printk("WiFi Status Request Failed\n");
    }
    else
    {
        printk("WiFi Status Request Success.\n");
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

extern "C"
{

static void hmc_thread(void)
{
    std::cout << "Running thread...\n";
	const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    hmc_5883l_driver_t hmc_driver;
	uint8_t hmc_buf[20];
	uint8_t ret;
	uint8_t command[2] = {0x06, 0x03};


    // Driver initialization
    hmc_5883l_driverInitPort(&hmc_driver, i2c_dev);
	hmc_configureContinuosMode(&hmc_driver);

	for(;;)
	{
        std::cout << "hmc\r\n";
		int16_t xx, yy, zz;
		uint8_t cmd;
		k_msleep(100);

		cmd=3;
		ret = hmc_driver.write(&hmc_driver, &cmd, 1);
		if (ret)
			printk("Error on write Mode.");
		ret = hmc_driver.read(&hmc_driver, &hmc_buf[3], 3);
		if (ret)
			printk("Error on write 1st command.\r\n");
		cmd=6;
		ret = hmc_driver.write(&hmc_driver, &cmd, 1);
		if (ret)
			printk("Error on write Mode.");
		ret = hmc_driver.read(&hmc_driver, &hmc_buf[6], 3);
		if (ret)
			printk("Error on write 1st command.\r\n");
		cmd=9;
		ret = hmc_driver.write(&hmc_driver, &cmd, 1);
		if (ret)
			printk("Error on write Mode.");
		ret = hmc_driver.read(&hmc_driver, &hmc_buf[9], 4);
		if (ret)
			printk("Error on write 1st command.\r\n");

		xx = hmc_buf[3] << 8 | hmc_buf[4];
		zz = hmc_buf[5] << 8 | hmc_buf[6];
		yy = hmc_buf[7] << 8 | hmc_buf[8];

        hmc_values.x = xx;
        hmc_values.y = yy;
        hmc_values.z = zz;
        //k_sem_give(&hmc_sem);
  //      k_sem_take(&wifi_sem, K_FOREVER);

		//printk("x:(%i) y:(%i) z:(%i) status:(%x)\r\n", xx, yy, zz, hmc_buf[9]);
		//printk("idA:(%x) idB:(%x) idC:(%x)\r\n", hmc_buf[10], hmc_buf[11], hmc_buf[12]);
	}

#if 0
	hmc_buf[0] = 9;
	ret = hmc_driver.write(&hmc_driver, hmc_buf, 1);
	if (ret)
	{
		printk("Error on 1st write.");
	}

	ret = hmc_driver.read(&hmc_driver, hmc_buf, 4);
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
}

static struct net_mgmt_event_callback mgmt_cb;

static void handler(struct net_mgmt_event_callback *cb,
		    uint32_t mgmt_event,
		    struct net_if *iface)
{
	int i = 0;

	if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
		return;
	}

	for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		char buf[NET_IPV4_ADDR_LEN];

		if (iface->config.ip.ipv4->unicast[i].addr_type !=
							NET_ADDR_DHCP) {
			continue;
		}

		printk("Your address: %s",
			net_addr_ntop(AF_INET,
			    &iface->config.ip.ipv4->unicast[i].address.in_addr,
						  buf, sizeof(buf)));
		printk("Lease time: %u seconds",
			 iface->config.dhcpv4.lease_time);
		printk("Subnet: %s",
			net_addr_ntop(AF_INET,
				       &iface->config.ip.ipv4->netmask,
				       buf, sizeof(buf)));
		printk("Router: %s",
			net_addr_ntop(AF_INET,
						 &iface->config.ip.ipv4->gw,
						 buf, sizeof(buf)));
	}
}

#define WAKEUP_TIME_SEC		(20)

int main(void)
{
    return 0;
}

void wifi_thread(void)
{
    k_sleep(K_SECONDS(10)); 
	int sock;
    std::cout << "WiFi Example\n\n";

    net_mgmt_init_event_callback(&wifi_cb, wifi_mgmt_event_handler,
                                 NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);

    net_mgmt_init_event_callback(&ipv4_cb, wifi_mgmt_event_handler, NET_EVENT_IPV4_ADDR_ADD);

    net_mgmt_add_event_callback(&wifi_cb);
    net_mgmt_add_event_callback(&ipv4_cb);
    std::cout << "Callbacks\n";

    k_sleep(K_SECONDS(2)); 
    wifi_connect();
    std::cout << "Ready...\n\n";

    while(!flagIP) k_sleep(K_SECONDS(1));

    //SNTP-----------------------
    struct sntp_ctx ctx;
    struct sockaddr_in addr;
    struct sntp_time sntp_time;
    int rv;
    struct timeval tv;
    struct timespec ts;

    /* ipv4 */
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(123);
    zsock_inet_pton(AF_INET, "170.210.222.10", &addr.sin_addr);

    rv = sntp_init(&ctx, (struct sockaddr *) &addr,
               sizeof(struct sockaddr_in));
    if (rv < 0) {
        std::cout << "Failed to init SNTP IPv4\r\n";
    }

    std::cout << "Sending SNTP IPv4 request...\r\n";
    rv = sntp_query(&ctx, 4 * MSEC_PER_SEC, &sntp_time);
    if (rv < 0) {
        std::cout << "SNTP IPv4 request failed: " << rv << "\r\n";
    }


    std::cout << "status: " << rv << "\r\n";
    printk("time since Epoch: high word: %u, low word: %u",
        (uint32_t)(sntp_time.seconds >> 32), (uint32_t)sntp_time.seconds);

    ts.tv_sec = sntp_time.seconds;

    //-----------------------SNTP


    while(1)
    {
        printk("WiFi\r\n\n");
        // Socket stuff
        int sock4 = -1;
        int websock4 = -1;
        struct sockaddr_in addr4;
        size_t amount;
        int ret;

        (void)connect_socket(AF_INET, "169.55.61.243", 9012,
                        &sock4, (struct sockaddr *)&addr4,
                        sizeof(addr4));
        if (sock4 < 0) {
            printk("Cannot create HTTP connection.");
        }
        else
        {
            printk("Connection created.\n");

            printk("Sending...\n");

            std::string log_str = "Logging from cpp.";
            printk("%s",log_str.c_str());

            std::cout << "Converting...\r\n";
            std::string xdata = "fieldX:" + std::to_string(hmc_values.x);
            std::string ydata = "fieldY:" + std::to_string(hmc_values.y);
            std::string zdata = "fieldZ:" + std::to_string(hmc_values.z);
            std::cout << "Convertion done!\r\n";
            std::string data = "ESP32/1.0|POST|BBFF-DQaxQtnjWZzKDzTsVRtOUfNLulXxqf|64a2425cbe387b5d626506e1:pythonTestDevice=>"+xdata+","+ydata+","+zdata+"|end";

            int rc = send(sock4, data.c_str(), strlen(data.c_str()), 0);

            printk("Data sent!\n");
            
            if (rc < 0)
                printk("Error sending.\n");

#if 1
            std::cout << "Converting...\r\n";
            std::string timestamp_str = "time: " + std::to_string(ts.tv_sec + k_uptime_get()/1000);
            std::cout << "Convertion Done\r\n";
            std::cout << timestamp_str << std::endl;
#endif
            
            close(sock4);
            //k_sem_give(&wifi_sem);
            //k_sem_take(&hmc_sem, K_FOREVER);
        }

        //k_sleep(K_MINUTES(1));
        k_sleep(K_SECONDS(5));
    }
#if 0
    wifi_disconnect();
    while(1)
    {
        printk("Looping...\r\n");
        k_sleep(K_SECONDS(5)); 
        wifi_status();
    }
    #endif
}

// ADC stuff ======================================

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

void adc_thread(void)
{
	int err;
	uint32_t count = 0;
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!device_is_ready(adc_channels[i].dev)) {
			printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			printk("Could not setup channel #%d (%d)\n", i, err);
		}
	}

	while (1) {
		printk("ADC reading[%u]:\n", count++);
		for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
			int32_t val_mv;

			printk("- %s, channel %d: ",
			       adc_channels[i].dev->name,
			       adc_channels[i].channel_id);

			(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

			err = adc_read(adc_channels[i].dev, &sequence);
			if (err < 0) {
				printk("Could not read (%d)\n", err);
				continue;
			}

			/*
			 * If using differential mode, the 16 bit value
			 * in the ADC sample buffer should be a signed 2's
			 * complement value.
			 */
			if (adc_channels[i].channel_cfg.differential) {
				val_mv = (int32_t)((int16_t)buf);
			} else {
				val_mv = (int32_t)buf;
			}
			printk("%"PRId32, val_mv);
			err = adc_raw_to_millivolts_dt(&adc_channels[i],
						       &val_mv);
			/* conversion to mV may not be supported, skip if not */
			if (err < 0) {
				printk(" (value in mV not available)\n");
			} else {
				printk(" = %"PRId32" mV\n", val_mv);
			}
		}

		k_sleep(K_MSEC(1000));
	}
}




#if 1
K_THREAD_DEFINE(hmc_id, STACKSIZE, hmc_thread, NULL, NULL, NULL,
		PRIORITY, 0, 0);
K_THREAD_DEFINE(wifi_id, STACKSIZE, wifi_thread, NULL, NULL, NULL,
		PRIORITY, 0, 0);
K_THREAD_DEFINE(adc_id, STACKSIZE, adc_thread, NULL, NULL, NULL,
		PRIORITY, 0, 0);
#endif