#include "module.h"
#include "lauxlib.h"
#include "platform.h"
#include "user_interface.h"

#define AHT10_I2C_ADDRESS1      (0x38)
#define AHT10_I2C_ADDRESS2      (0x39)

#define AHT10_COMMAND_INIT      (0xe1)
#define AHT10_COMMAND_MEAS      (0xac)
#define AHT10_COMMAND_NORMAL    (0x08)
#define AHT10_COMMAND_SOFTRESET (0xba)

#define AHT10_INIT_NORMAL       (0x00)
#define AHT10_INIT_CYCLE        (0x20)
#define AHT10_INIT_CMD          (0x40)
#define AHT10_INIT_CAL          (0x80)

#define AHT10_DATA_RES          (0x33)
#define AHT10_DATA_NOP          (0x00)

#define AHT10_DELAY_MEASUREMENT 100
#define AHT10_DELAY_INIT        500

static uint8_t aht10_i2c_id = 0;
static uint8_t aht10_i2c_addr = AHT10_I2C_ADDRESS1;
static _Bool aht10_ready = 0;
static _Bool aht10_read_queued = 0;
os_timer_t aht10_timer;

uint32_t aht10_raw_humidity = 0;
uint32_t aht10_raw_temperature = 0;
int lua_connected_readout_ref;

#define CHECK_I2C(x) if(!(x)) { NODE_DBG("i2c failed on line %d\n", __LINE__); }

static uint8_t aht10_read_status()
{
    uint8_t status;
    platform_i2c_send_start(aht10_i2c_id);
    CHECK_I2C(platform_i2c_send_address(aht10_i2c_id, aht10_i2c_addr, PLATFORM_I2C_DIRECTION_RECEIVER));
    status = platform_i2c_recv_byte(aht10_i2c_id, 0);
    platform_i2c_send_stop(aht10_i2c_id);
    return status;
}

static int aht10_read_done(void *arg)
{
    uint8_t buff[6];
    platform_i2c_send_start(aht10_i2c_id);
    CHECK_I2C(platform_i2c_send_address(aht10_i2c_id, aht10_i2c_addr, PLATFORM_I2C_DIRECTION_RECEIVER));
    for(int i = 0; i < 5; i++)
        buff[i] = platform_i2c_recv_byte(aht10_i2c_id, 1);
    buff[5] = platform_i2c_recv_byte(aht10_i2c_id, 1);
    platform_i2c_send_stop(aht10_i2c_id);

    NODE_DBG("Read 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", buff[0], buff[1], buff[2], buff[3], buff[4], buff[5]);
    aht10_raw_humidity = ((uint32_t)buff[1] << 12) | ((uint32_t)buff[2] << 4) | ((uint32_t)buff[3] >> 4);
    aht10_raw_temperature = ((buff[3] & 0xf) << 16) | (buff[4] << 8) | buff[5];

    lua_State *L = lua_getstate();
    lua_rawgeti(L, LUA_REGISTRYINDEX, lua_connected_readout_ref);
    luaL_unref(L, LUA_REGISTRYINDEX, lua_connected_readout_ref);
    os_timer_disarm(&aht10_timer);
    luaL_pcallx(L, 0, 0);
}

static int aht10_perform_read()
{
    platform_i2c_send_start(aht10_i2c_id);
    CHECK_I2C(platform_i2c_send_address(aht10_i2c_id, aht10_i2c_addr, PLATFORM_I2C_DIRECTION_TRANSMITTER));
    CHECK_I2C(platform_i2c_send_byte(aht10_i2c_id, 0xac));
    CHECK_I2C(platform_i2c_send_byte(aht10_i2c_id, 0x33));
    CHECK_I2C(platform_i2c_send_byte(aht10_i2c_id, 0x00));
    platform_i2c_send_stop(aht10_i2c_id);

    os_timer_disarm(&aht10_timer);
    os_timer_setfn(&aht10_timer, (os_timer_func_t *)(aht10_read_done), NULL);
    os_timer_arm(&aht10_timer, AHT10_DELAY_MEASUREMENT, 0);
}

static void aht10_setup_done(void *arg)
{
    uint8_t status = aht10_read_status();
    NODE_DBG("STATUS: %x\n", status);
    aht10_ready = (status & 0x68) == 0x08;
    os_timer_disarm(&aht10_timer);
    if(aht10_ready || aht10_read_queued)
        aht10_perform_read();
}


static void aht10_perform_actual_init(void *arg)
{
    platform_i2c_send_start(aht10_i2c_id);
    CHECK_I2C(platform_i2c_send_address(aht10_i2c_id, aht10_i2c_addr, PLATFORM_I2C_DIRECTION_TRANSMITTER));
    CHECK_I2C(platform_i2c_send_byte(aht10_i2c_id, 0xa8));
    CHECK_I2C(platform_i2c_send_byte(aht10_i2c_id, 0x00));
    CHECK_I2C(platform_i2c_send_byte(aht10_i2c_id, 0x00));
    platform_i2c_send_stop(aht10_i2c_id);

    os_timer_disarm(&aht10_timer);
    os_timer_setfn(&aht10_timer, (os_timer_func_t *)(aht10_setup_done), NULL);
    os_timer_arm(&aht10_timer, 200, 0);
}

static int aht10_lua_setup(lua_State *L)
{
    if(lua_isnumber(L, 1)) // First argument: i2c bus to use
        aht10_i2c_id = luaL_checkinteger(L, 1);

    if(lua_isboolean(L, 2) && luaL_checkinteger(L, 2)) // Second argument: Whether to use alternate address
    {
        NODE_DBG("Using address 2\n");
        aht10_i2c_addr = AHT10_I2C_ADDRESS2;
    }

    aht10_ready = 0;
    os_timer_disarm(&aht10_timer);
    os_timer_setfn(&aht10_timer, (os_timer_func_t *)(aht10_perform_actual_init), NULL);
    os_timer_arm(&aht10_timer, 50, 0);
}


static int aht10_lua_read(lua_State *L)
{
    if(!lua_isnoneornil(L, 1))
    {
        lua_connected_readout_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    }
    else
    {
        lua_connected_readout_ref = LUA_NOREF;
    }
    if(aht10_ready == 0)
    {
        aht10_read_queued = 1;
    }
    else
    {
        aht10_perform_read();
    }
}

static int aht10_lua_humi(lua_State *L)
{
    double humi = (double)aht10_raw_humidity * 100.0 / 1048576.0;
    lua_pushnumber(L, humi);
    return 1;
}

static int aht10_lua_temp(lua_State *L)
{
    double temp = ((200.0 * (double)aht10_raw_temperature) / 1048576.0) - 50.0f;
    lua_pushnumber(L, temp);
    return 1;
}

static int aht10_lua_soft_reset(lua_State *L)
{
    platform_i2c_send_start(aht10_i2c_id);
    platform_i2c_send_address(aht10_i2c_id, aht10_i2c_addr, PLATFORM_I2C_DIRECTION_TRANSMITTER);
    platform_i2c_send_byte(aht10_i2c_id, AHT10_COMMAND_SOFTRESET);
    platform_i2c_send_stop(aht10_i2c_id);
}

static int aht10_lua_is_ready(lua_State *L)
{
    lua_pushboolean(L, aht10_ready);
    return 1;
}

LROT_BEGIN(aht10, NULL, 0)
    LROT_FUNCENTRY( setup, aht10_lua_setup )
    LROT_FUNCENTRY( read, aht10_lua_read )
    LROT_FUNCENTRY( humi, aht10_lua_humi )
    LROT_FUNCENTRY( temp, aht10_lua_temp )
    LROT_FUNCENTRY( soft_reset, aht10_lua_soft_reset )
    LROT_FUNCENTRY( is_ready, aht10_lua_is_ready )
LROT_END(aht10, NULL, 0)

NODEMCU_MODULE(AHT10, "aht10", aht10, NULL);
