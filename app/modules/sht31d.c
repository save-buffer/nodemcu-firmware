#include "module.h"
#include "lauxlib.h"
#include "platform.h"
#include "user_interface.h"

#define SHT31D_I2C_ADDRESS1      (0x44)
#define SHT31D_I2C_ADDRESS2      (0x45)

#define CHECK_I2C(x) if(!(x)) { NODE_DBG("i2c failed on line %d\n", __LINE__); }

#define SHT31D_MEASURE_MSB 0x2c
#define SHT31D_LO_LSB 0x06
#define SHT31D_MI_LSB 0x0d
#define SHT31D_HI_LSB 0x10

#define SHT31D_T_MEAS_LOW   4
#define SHT31D_T_MEAS_MED   6
#define SHT31D_T_MEAS_HIGH 15

static uint8_t sht31d_i2c_id = 0;
static uint8_t sht31d_i2c_addr = SHT31D_I2C_ADDRESS1;
static int lua_connected_readout_ref = LUA_NOREF;

static uint32_t sht31d_raw_temp = 0;
static uint32_t sht31d_raw_humi = 0;
os_timer_t sht31d_timer;

static uint8_t crc8_bits(uint8_t b)
{
    uint8_t crc = 0;
    if(b & 0x01) crc ^= 0x5e;
    if(b & 0x02) crc ^= 0xbc;
    if(b & 0x04) crc ^= 0x61;
    if(b & 0x08) crc ^= 0xc2;
    if(b & 0x10) crc ^= 0x9d;
    if(b & 0x20) crc ^= 0x23;
    if(b & 0x40) crc ^= 0x46;
    if(b & 0x80) crc ^= 0x8c;
    return crc;
}

static uint8_t crc8(uint8_t b1, uint8_t b2)
{
    uint8_t crc = crc8_bits(b1);
    return crc8_bits(b2 ^ crc);
}

static int sht31d_lua_setup(lua_State *L)
{
    if(lua_isnumber(L, 1))
        sht31d_i2c_id = luaL_checkinteger(L, 1);

    if(lua_isboolean(L, 2) && luaL_checkinteger(L, 2))
    {
        NODE_DBG("Using address 2\n");
        sht31d_i2c_addr = SHT31D_I2C_ADDRESS2;
    }
    return 0;
}

static void sht31d_read_done(void *arg)
{
    uint8_t buff[6];
    platform_i2c_send_start(sht31d_i2c_id);
    CHECK_I2C(platform_i2c_send_address(sht31d_i2c_id, sht31d_i2c_addr, PLATFORM_I2C_DIRECTION_RECEIVER));
    for(int i = 0; i < 5; i++)
        buff[i] = platform_i2c_recv_byte(sht31d_i2c_id, 1);
    buff[5] = platform_i2c_recv_byte(sht31d_i2c_id, 0);
    platform_i2c_send_stop(sht31d_i2c_id);

    NODE_DBG("Read 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", buff[0], buff[1], buff[2], buff[3], buff[4], buff[5]);
    sht31d_raw_temp = (buff[0] << 8) | buff[1];
    sht31d_raw_humi = (buff[3] << 8) | buff[4];

    uint8_t crc_temp = crc8(buff[0], buff[1]);
    uint8_t crc_humi = crc8(buff[3], buff[4]);
    if(crc_temp != buff[2])
        NODE_DBG("Temperature CRC mismatch: got 0x%x, expected 0x%x\n", crc_temp, buff[2]);
    if(crc_temp != buff[5])
        NODE_DBG("Humidity CRC mismatch: got 0x%x, expected 0x%x\n", crc_temp, buff[5]);

    os_timer_disarm(&sht31d_timer);
    if(lua_connected_readout_ref != LUA_NOREF)
    {
        lua_State *L = lua_getstate();
        lua_rawgeti(L, LUA_REGISTRYINDEX, lua_connected_readout_ref);
        luaL_unref(L, LUA_REGISTRYINDEX, lua_connected_readout_ref);
        luaL_pcallx(L, 0, 0);
    }
}

static int sht31d_lua_read(lua_State *L)
{
    if(!lua_isnoneornil(L, 1))
        lua_connected_readout_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    else
        lua_connected_readout_ref = LUA_NOREF;
    
    uint8_t meas_lsbs[] = { SHT31D_LO_LSB, SHT31D_MI_LSB, SHT31D_HI_LSB };
    int meas_times[] = { SHT31D_T_MEAS_LOW, SHT31D_T_MEAS_MED, SHT31D_T_MEAS_HIGH };
    int meas_type = sizeof(meas_lsbs) / sizeof(uint8_t) - 1;
    if(lua_isnumber(L, 1))
        meas_type = luaL_checkinteger(L, 1);
    if(meas_type < 0)
        meas_type = 0;
    if(meas_type > 2)
        meas_type = 2;
    platform_i2c_send_start(sht31d_i2c_id);
    CHECK_I2C(platform_i2c_send_address(sht31d_i2c_id, sht31d_i2c_addr, PLATFORM_I2C_DIRECTION_TRANSMITTER));
    CHECK_I2C(platform_i2c_send_byte(sht31d_i2c_id, SHT31D_MEASURE_MSB));
    CHECK_I2C(platform_i2c_send_byte(sht31d_i2c_id, meas_lsbs[meas_type]));
    platform_i2c_send_stop(sht31d_i2c_id);

    os_timer_disarm(&sht31d_timer);
    os_timer_setfn(&sht31d_timer, (os_timer_func_t *)(sht31d_read_done), NULL);
    os_timer_arm(&sht31d_timer, meas_times[meas_type], 0);
    return 0;
}

static int sht31d_lua_humi(lua_State *L)
{
    double humi = (double)sht31d_raw_humi * 100.0 / 65535.0;
    lua_pushnumber(L, humi);
    return 1;
}

static int sht31d_lua_temp(lua_State *L)
{
    double temp = -45.0 + 175.0 * (double)sht31d_raw_temp / 65535.0;
    lua_pushnumber(L, temp);
    return 1;
}

LROT_BEGIN(sht31d, NULL, 0)
    LROT_FUNCENTRY( setup, sht31d_lua_setup )
    LROT_FUNCENTRY( read, sht31d_lua_read )
    LROT_FUNCENTRY( humi, sht31d_lua_humi )
    LROT_FUNCENTRY( temp, sht31d_lua_temp )
LROT_END(sht31d, NULL, 0)

NODEMCU_MODULE(SHT31D, "sht31d", sht31d, NULL);
