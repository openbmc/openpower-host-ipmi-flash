#include "config.h"

#include <endian.h>
#include <host-ipmid/ipmid-api.h>
#include <string.h>
#include <systemd/sd-bus.h>

#include <fstream>
#include <sdbusplus/bus.hpp>

using namespace sdbusplus;

static void register_openpower_flash_commands() __attribute__((constructor));

namespace openpower
{
namespace flash
{

template <typename T> static inline T get(void *buf)
{
    T t;
    memcpy(&t, buf, sizeof(t));
    return t;
}

template <typename T> static inline void put(void *buf, T &&t)
{
    memcpy(buf, &t, sizeof(t));
}

typedef ipmi_ret_t (*flash_command)(ipmi_request_t req, ipmi_response_t resp,
                                    ipmi_data_len_t data_len,
                                    ipmi_context_t context);

static ipmi_ret_t flash_command_get_info(ipmi_request_t request,
                                         ipmi_response_t response,
                                         ipmi_data_len_t data_len,
                                         ipmi_context_t context)
{
    if (*data_len < 1)
    {
        return IPMI_CC_REQ_DATA_LEN_INVALID;
    }

    uint8_t *reqdata = (uint8_t *)request;
    auto b = bus::new_system();
    auto m = b.new_method_call(
        "xyz.openbmc_project.hiomapd", "/xyz/openbmc_project/hiomapd",
        "xyz.openbmc_project.hiomapd.protocol", "GetInfo");
    m.append(get<uint8_t>(&reqdata[0]));

    /* FIXME: Catch SdBusError and return appropriate CC */
    auto reply = b.call(m);

    uint8_t version;
    uint8_t blockSizeShift;
    uint16_t timeout;
    reply.read(version, blockSizeShift, timeout);

    uint8_t *respdata = (uint8_t *)response;

    /* FIXME: Assumes v2! */
    put(&respdata[0], version);
    put(&respdata[1], blockSizeShift);
    put(&respdata[2], htole16(timeout));

    *data_len = 4;

    return IPMI_CC_OK;
}

static const flash_command flash_commands[] = {
    [0] = NULL, /* 0 is an invalid command ID */
    [1] = NULL, /* RESET */
    [2] = flash_command_get_info,
};

/* FIXME: Define this in the "right" place, wherever that is */
/* FIXME: Double evaluation */
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static ipmi_ret_t ipmi_openpower_flash_command(
    ipmi_netfn_t netfn, ipmi_cmd_t cmd, ipmi_request_t request,
    ipmi_response_t response, ipmi_data_len_t data_len, ipmi_context_t context)
{
    if (*data_len < 2)
    {
        *data_len = 0;
        return IPMI_CC_REQ_DATA_LEN_INVALID;
    }

    uint8_t *ipmi_req = (uint8_t *)request;
    uint8_t *ipmi_resp = (uint8_t *)response;
    uint8_t flash_cmd = ipmi_req[0];

    if (flash_cmd > ARRAY_SIZE(flash_commands) - 1)
    {
        *data_len = 0;
        return IPMI_CC_PARM_OUT_OF_RANGE;
    }
    uint8_t *flash_req = ipmi_req + 2;
    size_t flash_len = *data_len - 2;
    uint8_t *flash_resp = ipmi_resp + 2;

    ipmi_ret_t cc =
        flash_commands[flash_cmd](flash_req, flash_resp, &flash_len, context);
    if (cc != IPMI_CC_OK)
    {
        *data_len = 0;
        return cc;
    }

    /* Populate the response command and sequence */
    put(&ipmi_resp[0], ipmi_req[0]);
    put(&ipmi_resp[1], ipmi_req[1]);

    *data_len = flash_len + 2;

    return cc;
}
} // namespace flash
} // namespace openpower

static void register_openpower_flash_commands()
{
    printf("Registering NetFn:[0x%X], Cmd:[0x%X]\n", NETFUN_OEM, 0x5a);
    ipmi_register_callback(NETFUN_OEM, 0x5a, NULL,
                           openpower::flash::ipmi_openpower_flash_command,
                           SYSTEM_INTERFACE);

    return;
}
