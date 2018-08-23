#include "config.h"

#include <endian.h>
#include <host-ipmid/ipmid-api.h>
#include <string.h>
#include <systemd/sd-bus.h>

#include <fstream>
#include <functional>
#include <host-ipmid/ipmid-host-cmd-utils.hpp>
#include <host-ipmid/ipmid-host-cmd.hpp>
#include <iostream>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>

using namespace sdbusplus;
using namespace phosphor::host::command;

static void register_openpower_flash_commands() __attribute__((constructor));

namespace openpower
{
namespace flash
{

#define BMC_EVENT_DAEMON_READY (1 << 7)
#define BMC_EVENT_FLASH_CTRL_LOST (1 << 6)
#define BMC_EVENT_WINDOW_RESET (1 << 1)
#define BMC_EVENT_PROTOCOL_RESET (1 << 0)

#define IPMI_CMD_HIOMAP_EVENT 0x0f

#define HIOMAPD_SERVICE "xyz.openbmc_project.Hiomapd"
#define HIOMAPD_OBJECT "/xyz/openbmc_project/Hiomapd"
#define HIOMAPD_IFACE "xyz.openbmc_project.Hiomapd.Protocol"
#define HIOMAPD_IFACE_V2 HIOMAPD_IFACE ".V2"

#define DBUS_IFACE_PROPERTIES "org.freedesktop.DBus.Properties"

struct hostflash
{
    bus::bus *bus;

    /* Signals */
    bus::match::match *properties;
    bus::match::match *window_reset;
    bus::match::match *bmc_reboot;

    /* Protocol state */
    std::map<std::string, int> event_lookup;
    uint8_t bmc_events;
};

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

static void ipmi_hiomap_event_response(IpmiCmdData cmd, bool status)
{
    using namespace phosphor::logging;

    if (!status)
    {
        log<level::ERR>("Failed to deliver host command",
                        entry("SEL_COMMAND=%x:%x", cmd.first, cmd.second));
    }
}

static int hiomap_handle_property_update(struct hostflash *ctx,
                                         sdbusplus::message::message &msg)
{
    std::map<std::string, sdbusplus::message::variant<bool>> msgData;

    std::string iface;
    msg.read(iface, msgData);

    for (auto const &x : msgData)
    {
        if (!ctx->event_lookup.count(x.first))
        {
            /* Unsupported event? */
            continue;
        }

        uint8_t mask = ctx->event_lookup[x.first];
        auto value = sdbusplus::message::variant_ns::get<bool>(x.second);

        if (value)
        {
            ctx->bmc_events |= mask;
        }
        else
        {
            ctx->bmc_events &= ~mask;
        }
    }

    auto cmd = std::make_pair(IPMI_CMD_HIOMAP_EVENT, ctx->bmc_events);

    ipmid_send_cmd_to_host(std::make_tuple(cmd, ipmi_hiomap_event_response));

    return 0;
}

static bus::match::match hiomap_match_properties(struct hostflash *ctx)
{
    auto properties =
        bus::match::rules::propertiesChanged(HIOMAPD_OBJECT, HIOMAPD_IFACE_V2);

    bus::match::match match(
        *ctx->bus, properties,
        std::bind(hiomap_handle_property_update, ctx, std::placeholders::_1));

    return match;
}

static int hiomap_handle_signal_v2(struct hostflash *ctx, const char *name)
{
    ctx->bmc_events |= ctx->event_lookup[name];

    auto cmd = std::make_pair(IPMI_CMD_HIOMAP_EVENT, ctx->bmc_events);

    ipmid_send_cmd_to_host(std::make_tuple(cmd, ipmi_hiomap_event_response));

    return 0;
}

static bus::match::match hiomap_match_signal_v2(struct hostflash *ctx,
                                                const char *name)
{
    using namespace bus::match;

    auto signals = rules::type::signal() + rules::path(HIOMAPD_OBJECT) +
                   rules::interface(HIOMAPD_IFACE_V2) + rules::member(name);

    bus::match::match match(*ctx->bus, signals,
                            std::bind(hiomap_handle_signal_v2, ctx, name));

    return match;
}

static ipmi_ret_t flash_command_reset(ipmi_request_t request,
                                      ipmi_response_t response,
                                      ipmi_data_len_t data_len,
                                      ipmi_context_t context)
{
    struct hostflash *ctx = static_cast<struct hostflash *>(context);

    auto m = ctx->bus->new_method_call(HIOMAPD_SERVICE, HIOMAPD_OBJECT,
                                       HIOMAPD_IFACE, "Reset");
    /* FIXME: Catch SdBusError and return appropriate CC */
    ctx->bus->call(m);

    *data_len = 0;

    return IPMI_CC_OK;
}

static ipmi_ret_t flash_command_get_info(ipmi_request_t request,
                                         ipmi_response_t response,
                                         ipmi_data_len_t data_len,
                                         ipmi_context_t context)
{
    struct hostflash *ctx = static_cast<struct hostflash *>(context);

    if (*data_len < 1)
    {
        return IPMI_CC_REQ_DATA_LEN_INVALID;
    }

    uint8_t *reqdata = (uint8_t *)request;
    auto m = ctx->bus->new_method_call(HIOMAPD_SERVICE, HIOMAPD_OBJECT,
                                       HIOMAPD_IFACE, "GetInfo");
    m.append(get<uint8_t>(&reqdata[0]));

    /* FIXME: Catch SdBusError and return appropriate CC */
    auto reply = ctx->bus->call(m);

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

static ipmi_ret_t flash_command_get_flash_info(ipmi_request_t request,
                                               ipmi_response_t response,
                                               ipmi_data_len_t data_len,
                                               ipmi_context_t context)
{
    struct hostflash *ctx = static_cast<struct hostflash *>(context);

    auto m = ctx->bus->new_method_call(HIOMAPD_SERVICE, HIOMAPD_OBJECT,
                                       HIOMAPD_IFACE_V2, "GetFlashInfo");
    /* FIXME: Catch SdBusError and return appropriate CC */
    auto reply = ctx->bus->call(m);

    uint16_t flashSize, eraseSize;
    reply.read(flashSize, eraseSize);

    uint8_t *respdata = (uint8_t *)response;
    put(&respdata[0], htole16(flashSize));
    put(&respdata[2], htole16(eraseSize));

    *data_len = 4;

    return IPMI_CC_OK;
}

static const flash_command flash_commands[] = {
    [0] = NULL, /* 0 is an invalid command ID */
    [1] = flash_command_reset,
    [2] = flash_command_get_info,
    [3] = flash_command_get_flash_info,
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

    /* XXX: Someone help me make this less insane */
    /* FIXME: Clean this up? Can we unregister? */
    struct openpower::flash::hostflash *ctx = new openpower::flash::hostflash();

    /* Initialise mapping from signal and property names to status bit */
    ctx->event_lookup["DaemonReady"] = BMC_EVENT_DAEMON_READY;
    ctx->event_lookup["FlashControlLost"] = BMC_EVENT_FLASH_CTRL_LOST;
    ctx->event_lookup["WindowReset"] = BMC_EVENT_WINDOW_RESET;
    ctx->event_lookup["ProtocolReset"] = BMC_EVENT_PROTOCOL_RESET;

    ctx->bus = new bus::bus(ipmid_get_sd_bus_connection());

    /* Initialise signal handling */

    /*
     * Can't use temporaries here because that causes SEGFAULTs due to slot
     * destruction (!?), so enjoy the weird wrapping.
     */
    ctx->properties =
        new bus::match::match(std::move(hiomap_match_properties(ctx)));
    ctx->bmc_reboot = new bus::match::match(
        std::move(hiomap_match_signal_v2(ctx, "BmcReboot")));
    ctx->window_reset = new bus::match::match(
        std::move(hiomap_match_signal_v2(ctx, "WindowReset")));

    ipmi_register_callback(NETFUN_OEM, 0x5a, ctx,
                           openpower::flash::ipmi_openpower_flash_command,
                           SYSTEM_INTERFACE);
}
