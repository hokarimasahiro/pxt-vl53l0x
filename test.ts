basic.showIcon(IconNames.Heart)
if (VL53L0X.initialize(true)){
    basic.showIcon(IconNames.Happy)
}else{
    basic.showIcon(IconNames.Sad)
}
basic.forever(function () {
    basic.showNumber(VL53L0X.readRangeSingleMillimeters())
    basic.pause(1000)
})
