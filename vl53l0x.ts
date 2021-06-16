/**
 * makecode VL53L0X Package.
 */

/**
 * VL53L0X block
 */
//% weight=100 color=#303030 icon="\ue115" block="VL53L0X"
namespace VL53L0X {
    let I2C_ADDR = 0x29;

    /**
     * Write byte data to the specified registor
     * @param addr registor address, eg: 0x80
     * @param dat is the data will be write, eg: 0x05
     */
    //% blockId="writeReg" block="write registor address %addr|byte %dat"
    //% advance=true
    export function writeReg(addr: number, dat: number): void {
        let buf = pins.createBuffer(2);
        buf[0] = addr;
        buf[1] = dat;
        pins.i2cWriteBuffer(I2C_ADDR, buf)
    }

    /**
     * Read byte data from the specified registor
     * @param addr registor address, eg: 0x80
     */
    //% blockId="ReadReg" block="read data from registor address %addr"
    export function readByte(addr: number): number {
        pins.i2cWriteNumber(I2C_ADDR, addr, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(I2C_ADDR, NumberFormat.UInt8BE);
    }

    /**
     * Write word data to the specified registor
     * @param addr eeprom address, eg: 0x80
     * @param dat is the data will be write, eg: 0x1234
     */
    //% blockId="WriteReg16" block="write registor address %addr|word %dat"
    export function writeREg16(addr: number, dat: number): void {
        let buf = pins.createBuffer(3);
        buf[0] = addr;
        buf[1] = dat & 0xff;
        buf[2] = dat >> 8;
        pins.i2cWriteBuffer(I2C_ADDR, buf)
    }

    /**
     * Read word data from the specified registor
     * @param addr registor address, eg: 0x80
     */
    //% blockId="ReadReg16" block="read word from registor address %addr"
    export function readWord(addr: number): number {
        pins.i2cWriteNumber(I2C_ADDR, addr, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(I2C_ADDR, NumberFormat.UInt16BE);
    }

    /**
     * Write double word data to the specified registor
     * @param addr registor address, eg: 0x80
     * @param dat is the data will be write, eg: 0x12345678
     */
    //% blockId="WriteReg32" block="write registor address %addr|dword %dat"
    export function writeReg32(addr: number, dat: number): void {
        let buf = pins.createBuffer(5);
        buf[0] = addr;
        buf[1] = dat & 0xff;
        buf[2] = dat >> 8;
        buf[3] = dat >> 16;
        buf[4] = dat >> 24;
        pins.i2cWriteBuffer(I2C_ADDR, buf)
    }

    /**
     * Read double word data from the specified registor
     * @param addr eeprom address, eg: 0x80
     */
    //% blockId="ReadReg32" block="read dword from registor address %addr"
    export function readREg32(addr: number): number {
        pins.i2cWriteNumber(I2C_ADDR, addr, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(I2C_ADDR, NumberFormat.Int32BE);
    }

    /**
     * Write data to the specified registor
     * @param addr registor address, eg: 0x00
     * @param dat is the data will be write, eg: 5
     */
    //% blockId="WriteBuf" block="registor address %addr|write buf %dat"
    export function writeBuf(addr: number, dat: number[]): void {
        let buf = pins.createBuffer(dat.length + 2);
        buf[0] = addr >> 8;
        buf[1] = addr;
        for(let i=0;i<dat.length;i++){
            buf[i + 2] = dat[i] & 0xff;
        }
        pins.i2cWriteBuffer(I2C_ADDR, buf)
    }

    /**
     * Read data from the specified registor
     * @param addr registor address, eg: 0x00
     * @param size read data count, eg: 16
     */
    //% blockId="ReadBuf" block="registor address %addr|read buf %size"
    export function readBuf(addr: number, size: number): number[] {
        let retbuf:number[]=[];

        pins.i2cWriteNumber(I2C_ADDR, addr, NumberFormat.UInt16BE);
        let buf = pins.i2cReadBuffer(I2C_ADDR, size);
        for(let i=0;i<size;i++){
            retbuf.push(buf[i]);
        }
        return retbuf;
    }


    /**
     * set i2c address
     * @param addr i2c address, eg: 0x50
     */
    //% blockId="setI2cAddress" block="i2c address set to %addr"
    export function setI2cAddress(addr: number): void {
        I2C_ADDR = addr
    }
}
