serial.redirectToUSB()
serial.setBaudRate(BaudRate.BaudRate115200)
basic.showIcon(IconNames.Heart)
serial.writeLine("Ready!!")
if (VL53L0X.initialize(true)) {
    serial.writeLine("initialize OK")
    basic.showIcon(IconNames.Happy)
    VL53L0X.startContinuous(0)
} else {
    serial.writeLine("initialize NG")
    basic.showIcon(IconNames.Confused)
}
basic.forever(function () {
    led.toggle(0, 0)
    let dis = VL53L0X.readRangeContinuousMillimeters();
    serial.writeLine("" + dis)
    watchfont.showSorobanNumber(dis, 1, 4)
    basic.pause(100)
})
