


//% color=#E7734B icon="\uf2db"
namespace sensor {
    //external button
    export enum buttonChannel {
        //% block="A (P20)"
        A,
        //% block="E (P16)"
        E,
        //% block="F (P14)"
        F,
        //% block="G (P2)"
        G,
        //% block="H (P8)"
        H,
    }
    export let buttonChannels: { [key: number]: DigitalPin } = {
        [buttonChannel.A]: DigitalPin.P20,
        [buttonChannel.E]: DigitalPin.P16,
        [buttonChannel.F]: DigitalPin.P14,
        [buttonChannel.G]: DigitalPin.P2,
        [buttonChannel.H]: DigitalPin.P8,
    }

    //external sensor
    export enum sensorChannel {
        //% block="P1"
        P1,
        //% block="P8"
        P8,
        //% block="P12"
        P12,
        //% block="P2"
        P2,
        //% block="P13"
        P13,
        //% block="P14"
        P14,
        //% block="P15"
        P15,
        //% block="P16"
        P16,
    }
    export let sensorChannels: { [key: number]: DigitalPin } = {
        [sensorChannel.P1]: DigitalPin.P1,
        [sensorChannel.P8]: DigitalPin.P8,
        [sensorChannel.P12]: DigitalPin.P12,
        [sensorChannel.P2]: DigitalPin.P2,
        [sensorChannel.P13]: DigitalPin.P13,
        [sensorChannel.P14]: DigitalPin.P14,
        [sensorChannel.P15]: DigitalPin.P15,
        [sensorChannel.P16]: DigitalPin.P16,
    }
    //----------------------------------
    export let sensorAnalogChannels: { [key: number]: AnalogPin } = {
        [sensorChannel.P1]: AnalogPin.P1,
        [sensorChannel.P8]: AnalogPin.P8,
        [sensorChannel.P12]: AnalogPin.P12,
        [sensorChannel.P2]: AnalogPin.P2,
        [sensorChannel.P13]: AnalogPin.P13,
        [sensorChannel.P14]: AnalogPin.P14,
        [sensorChannel.P15]: AnalogPin.P15,
        [sensorChannel.P16]: AnalogPin.P16,
    }

    //----------------------------------
    //% color=#000000
    //% block="read button $pin (0-1)"
    //% group="Read Sensor"
    export function Readbutton(pin: buttonChannel): number {
        let read = buttonChannels[pin];
        pins.setPull(buttonChannels[pin], PinPullMode.PullUp);
        let reading = pins.digitalReadPin(read);
        return (reading);
    }




    //% color=#000000    
    //% block="analog Sensor $pin (0-1023) "
    //% group="Read Sensor"
    export function lightSensor(pin: sensorChannel): number {
        let read = sensorAnalogChannels[pin];
        let reading = pins.analogReadPin(read);
        return Math.round(reading);
    }

    //% color=#000000    
    //% block="digital Sensor $pin (0-1)"
    //% group="Read Sensor"
    export function Sensor(pin: sensorChannel): number {
        let read = sensorChannels[pin];
        let reading = pins.digitalReadPin(read);
        return (reading);
    }


    function signal_dht11(pin: DigitalPin): void {
        pins.digitalWritePin(pin, 0);
        basic.pause(18);
        let i = pins.digitalReadPin(pin);
        pins.setPull(pin, PinPullMode.PullUp);
    }


    //% color=#383838
    //% block="track Line $pin Black Color"
    //% group="Logic Sensor"
    export function isblack(pin: sensorChannel): boolean {
        let read = sensorChannels[pin];
        return pins.digitalReadPin(read) == 1;
    }
    //% color=#636262
    //% block="track Line $pin Not Black"
    //% group="Logic Sensor"
    export function notblack(pin: sensorChannel): boolean {
        let read = sensorChannels[pin];
        return pins.digitalReadPin(read) == 0;
    }


    //% color=#3D3430    
    //% block="on button $pin pressed"
    //% group="Logic Sensor"
    export function isButtonPressed(pin: buttonChannel): boolean {
        pins.setPull(buttonChannels[pin], PinPullMode.PullUp);
        let read = buttonChannels[pin];
        return pins.digitalReadPin(read) == 0;
    }

    //% color=#76dbb1
    //sonar
    //% block="sonar %channel unit %unit"
    //% group="ect"
    //% unit.defl=PingUnit.Centimeters
    export function ping(channel: sonarPort, unit: pingUnit, maxCmDistance = 500): number {
        let trig = trigChanel[channel];
        let echo = echoChanel[channel];
        // send pulse
        pins.setPull(trig, PinPullMode.PullNone);
        pins.digitalWritePin(trig, 0);
        control.waitMicros(2);
        pins.digitalWritePin(trig, 1);
        control.waitMicros(10);
        pins.digitalWritePin(trig, 0);

        // read pulse
        const d = pins.pulseIn(echo, PulseValue.High, maxCmDistance * 58);

        switch (unit) {
            case pingUnit.Centimeters: return Math.idiv(d, 58);
            case pingUnit.Inches: return Math.idiv(d, 148);
            default: return d;
        }
    }

    export enum pingUnit {
        //% block="μs"
        MicroSeconds,
        //% block="cm"
        Centimeters,
        //% block="inches"
        Inches
    }
    export enum sonarPort {
        //% block="E [ trig(P16) echo(P15) ]"
        E,
        //% block="F [ trig(P14) echo(P13) ]"
        F,
        //% block="G [ trig(P2) echo(P12) ]"
        G,
        //% block="H [ trig(P8) echo(P1) ]"
        H,
    }
    export let trigChanel: { [key: number]: DigitalPin } = {
        [sonarPort.E]: DigitalPin.P16,
        [sonarPort.F]: DigitalPin.P14,
        [sonarPort.G]: DigitalPin.P2,
        [sonarPort.H]: DigitalPin.P8,
    }
    export let echoChanel: { [key: number]: DigitalPin } = {
        [sonarPort.E]: DigitalPin.P15,
        [sonarPort.F]: DigitalPin.P13,
        [sonarPort.G]: DigitalPin.P12,
        [sonarPort.H]: DigitalPin.P1,
    }

    //% color=#76dbb1
    //% group="ect"
    //% blockId=sensor_ping block="ultrasonic trig %trig|echo %echo|get distance %unit"
    //% trig.fieldEditor="gridpicker" trig.fieldOptions.columns=4
    //% trig.fieldOptions.tooltips="false" trig.fieldOptions.width="300"
    //% echo.fieldEditor="gridpicker" echo.fieldOptions.columns=4
    //% echo.fieldOptions.tooltips="false" echo.fieldOptions.width="300"
    export function sensor_ping(trig: DigitalPin, echo: DigitalPin, unit: pingUnit, maxCmDistance = 500): number {
        // send pulse
        pins.setPull(trig, PinPullMode.PullNone);
        pins.digitalWritePin(trig, 0);
        control.waitMicros(2);
        pins.digitalWritePin(trig, 1);
        control.waitMicros(10);
        pins.digitalWritePin(trig, 0);

        // read pulse
        const d = pins.pulseIn(echo, PulseValue.High, maxCmDistance * 58);

        switch (unit) {
            case pingUnit.Centimeters: return d / 58;
            case pingUnit.Inches: return d / 148;
            default: return d;
        }
    }


}


//% color=#E7734B icon="\uf48b"
//% groups="['Motor','Servo','Led', 'Read Sensor','Logic Sensor','I2C LCD 1602']"
namespace motor {
    // motor control
    //servo180
    export enum servoChannel {
        //% block="P1"
        P1,
        //% block="P8"
        P8,
        //% block="P12"
        P12,
        //% block="P2"
        P2,
        //% block="P13"
        P13,
        //% block="P14"
        P14,
        //% block="P15"
        P15,
        //% block="P16"
        P16,
    }
    export let servoChannels: { [key: number]: AnalogPin } = {
        [servoChannel.P1]: AnalogPin.P1,
        [servoChannel.P8]: AnalogPin.P8,
        [servoChannel.P12]: AnalogPin.P12,
        [servoChannel.P2]: AnalogPin.P2,
        [servoChannel.P13]: AnalogPin.P13,
        [servoChannel.P14]: AnalogPin.P14,
        [servoChannel.P15]: AnalogPin.P15,
        [servoChannel.P16]: AnalogPin.P16,
    }
    //----------------------------------

    export enum servoconChannel {
        //% block="P1"
        P1,
        //% block="P8"
        P8,
        //% block="P12"
        P12,
        //% block="P2"
        P2,
        //% block="P13"
        P13,
        //% block="P14"
        P14,
        //% block="P15"
        P15,
        //% block="P16"
        P16,
    }
    export let servoconChannels: { [key: number]: AnalogPin } = {
        [servoconChannel.P1]: AnalogPin.P1,
        [servoconChannel.P8]: AnalogPin.P8,
        [servoconChannel.P12]: AnalogPin.P12,
        [servoconChannel.P2]: AnalogPin.P2,
        [servoconChannel.P13]: AnalogPin.P13,
        [servoconChannel.P14]: AnalogPin.P14,
        [servoconChannel.P15]: AnalogPin.P15,
        [servoconChannel.P16]: AnalogPin.P16,
    }
    export enum svconShaft {
        //% block="Right"
        Right = 0,
        //% block="Left"
        Left = 180,
        //% block="Stop"
        Stop = 90,
    }
    export let degreesCon: { [key: number]: number } = {
        [svconShaft.Right]: 0,
        [svconShaft.Left]: 180,
        [svconShaft.Stop]: 90,

    }
    export enum motorChannel {
        //% block="E (P15,P16)"
        E,
        //% block="F (P13,P14)""
        F,
        //% block="G (P12,P2)""
        G,
        //% block="H (P1,P8)""
        H,
    }
    export enum motorShaftDirection {
        //% block="Left"
        LOW,
        //% block="Right"
        HIGH,
    }
    export let motorSpeedPins: { [key: number]: AnalogPin } = {
        [motorChannel.E]: AnalogPin.P16,
        [motorChannel.F]: AnalogPin.P14,
        [motorChannel.G]: AnalogPin.P2,
        [motorChannel.H]: AnalogPin.P8,
    }
    export let motorChannels: { [key: number]: DigitalPin } = {
        [motorChannel.E]: DigitalPin.P15,
        [motorChannel.F]: DigitalPin.P13,
        [motorChannel.G]: DigitalPin.P12,
        [motorChannel.H]: DigitalPin.P1,
    }
    //% color=#E7734B
    //% direction.defl=motorShaftDirection.HIGH
    //% block="stop motor $channel"
    //% group="Motor"
    export function motorStop1(channel: motorChannel): void {
        let dirPin = motorChannels[channel];
        let speedPin = motorSpeedPins[channel];
        pins.digitalWritePin(dirPin, 0);
        pins.analogWritePin(speedPin, 0);
    }

    //% color=#E7734B
    //% block="motor $channel direction $direction speed $speed"
    //% speed.min=0 speed.max=255
    //% speed.defl=100
    //% direction.min=0 direction.max=1
    //% group="Motor"
    //% color=#E7734B
    export function motorControl1(channel: motorChannel, direction: number, speed: number): void {
        let dirPin = motorChannels[channel];
        let speedPin = motorSpeedPins[channel];

        pins.digitalWritePin(dirPin, direction);
        pins.analogWritePin(speedPin, pins.map(speed, 0, 255, 0, 1023));
    }

    //% color=#E7734B
    //% block="motor $channel direction $direction speed $speed"
    //% speed.min=0 speed.max=255
    //% speed.defl=100
    //% direction.defl=motorShaftDirection.HIGH
    //% group="Motor"
    //% color=#E7734B
    export function motorControl2(channel: motorChannel, direction: motorShaftDirection, speed: number): void {
        let dirPin = motorChannels[channel];
        let speedPin = motorSpeedPins[channel];

        pins.digitalWritePin(dirPin, direction);
        pins.analogWritePin(speedPin, pins.map(speed, 0, 255, 0, 1023));
    }
    //% color=#E84E19
    //% block"servo180 $pinSmini degrees $degrees"
    //% degrees.min=20 degrees.max=160
    //% degrees.defl=90
    //% group="Servo"
    export function MiniServo(pinSmini: servoChannel, degrees: number): void {
        let pinsmini = servoChannels[pinSmini];
        pins.servoWritePin(pinsmini, degrees);

    }
    //% color=#E84E19
    //% block"continuous Servo $pinSV direction $direction"
    //% direction.defl=90
    //% group="Servo"
    export function ContinuousServo(pinSV: servoChannel, direction: svconShaft): void {
        let pinservo = servoChannels[pinSV];
        pins.servoWritePin(pinservo, direction);

    }
}
//% weight=5 color=#E7734B icon="\uf110"
namespace gigoLED {
    //led
    export enum lEDChannel {
        //% block="A (P19)"
        A,
        //% block="B (P14)"
        B,
        //% block="C (P2)"
        C,
        //% block="D (P8)"
        D,
        //% block="E (P15)"
        E,
        //% block="F (P13)"
        F,
        //% block="G (P12)"
        G,
        //% block="H (P1)"
        H,
    }
    export let lEDChannels: { [key: number]: DigitalPin } = {
        [lEDChannel.A]: DigitalPin.P19,
        [lEDChannel.B]: DigitalPin.P14,
        [lEDChannel.C]: DigitalPin.P2,
        [lEDChannel.D]: DigitalPin.P8,
        [lEDChannel.E]: DigitalPin.P15,
        [lEDChannel.F]: DigitalPin.P13,
        [lEDChannel.G]: DigitalPin.P12,
        [lEDChannel.H]: DigitalPin.P1,
    }
    export enum lEDShaftonoff {
        //% block="off"
        LOW,
        //% block="on"
        HIGH,

    }


    //% color=#FACB09
    //% block="led $leds Status $Status"
    //% Status.min=0 Status.max=1
    //% leds.defl=lEDChannel.D
    //% group="Led"
    export function ledtest(leds: lEDChannel, Status: number): void {
        let ledg = lEDChannels[leds];

        pins.digitalWritePin(ledg, Status);

    }

    //% color=#FACB09
    //% block="led $leds Status $Status"
    //% Status.defl=lEDShaftonoff.HIGH*
    //% leds.defl=lEDChannel.D
    //% group="Led"
    export function led(leds: lEDChannel, Status: lEDShaftonoff): void {
        let ledg = lEDChannels[leds];

        pins.digitalWritePin(ledg, Status);

    }
    //% color=#FACB09
    //toggle led
    //% blockId=led block="led %pin $ledstate"
    //% ledstate.shadow="toggleOnOff"
    //% expandableArgumentMode="toggle"
    //% pin.defl=lEDChannel.D
    //% group="Led"
    export function ledBrightness(pin: lEDChannel, ledstate: boolean): void {
        if (ledstate) {
            let pinled = lEDChannels[pin];
            pins.digitalWritePin(pinled, 1);

        }
        else {
            let pinled = lEDChannels[pin];
            pins.digitalWritePin(pinled, 0);

        }
    }
}

//% color=#E7734B icon="\uf26c"
namespace lcd1602 {
    //LCD i2c


    let i2cAddr: number // 0x3F: PCF8574A, 0x27: PCF8574
    let BK: number      // backlight control
    let RS: number      // command/data

    // set LCD reg
    function setreg(d: number) {
        pins.i2cWriteNumber(i2cAddr, d, NumberFormat.Int8LE)
        basic.pause(1)
    }

    // send data to I2C bus
    function set(d: number) {
        d = d & 0xF0
        d = d + BK + RS
        setreg(d)
        setreg(d + 4)
        setreg(d)
    }

    // send command
    function cmd(d: number) {
        RS = 0
        set(d)
        set(d << 4)
    }

    // send data
    function dat(d: number) {
        RS = 1
        set(d)
        set(d << 4)
    }

    // auto get LCD address
    function AutoAddr() {
        let k = true
        let addr = 0x20
        let d1 = 0, d2 = 0
        while (k && (addr < 0x28)) {
            pins.i2cWriteNumber(addr, -1, NumberFormat.Int32LE)
            d1 = pins.i2cReadNumber(addr, NumberFormat.Int8LE) % 16
            pins.i2cWriteNumber(addr, 0, NumberFormat.Int16LE)
            d2 = pins.i2cReadNumber(addr, NumberFormat.Int8LE)
            if ((d1 == 7) && (d2 == 0)) k = false
            else addr += 1
        }
        if (!k) return addr

        addr = 0x38
        while (k && (addr < 0x40)) {
            pins.i2cWriteNumber(addr, -1, NumberFormat.Int32LE)
            d1 = pins.i2cReadNumber(addr, NumberFormat.Int8LE) % 16
            pins.i2cWriteNumber(addr, 0, NumberFormat.Int16LE)
            d2 = pins.i2cReadNumber(addr, NumberFormat.Int8LE)
            if ((d1 == 7) && (d2 == 0)) k = false
            else addr += 1
        }
        if (!k) return addr
        else return 0

    }


    //% color=#045F14
    //% blockId="i2c_LCD1620_SET_ADDRESS" block="lcd (A) Address %addr"
    //% weight=100 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function LcdInit(Addr: number) {
        if (Addr == 0) i2cAddr = AutoAddr()
        else i2cAddr = Addr
        BK = 8
        RS = 0
        cmd(0x33)       // set 4bit mode
        basic.pause(5)
        set(0x30)
        basic.pause(5)
        set(0x20)
        basic.pause(5)
        cmd(0x28)       // set mode
        cmd(0x0C)
        cmd(0x06)
        cmd(0x01)       // clear
    }

    //% color=#045F14
    //% blockId="i2c_LCD1620_SHOW_NUMBER" block="show number %n|at x %x|y %y"
    //% weight=90 blockGap=8
    //% x.min=0 x.max=15
    //% y.min=0 y.max=1
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function ShowNumber(n: number, x: number, y: number): void {
        let s = n.toString()
        ShowString(s, x, y)
    }

    //% color=#045F14
    //% blockId="i2c_LCD1620_SHOW_STRING" block="show string %s|at x %x|y %y"
    //% weight=90 blockGap=8
    //% x.min=0 x.max=15
    //% y.min=0 y.max=1
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function ShowString(s: string, x: number, y: number): void {
        let a: number

        if (y > 0)
            a = 0xC0
        else
            a = 0x80
        a += x
        cmd(a)

        for (let i = 0; i < s.length; i++) {
            dat(s.charCodeAt(i))
        }
    }

    //% color=#045F14
    //% blockId="i2c_LCD1620_ON" block="lcd on"
    //% weight=81 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function on(): void {
        cmd(0x0C)
    }

    //% color=#045F14
    //% blockId="i2c_LCD1620_OFF" block="lcd off"
    //% weight=80 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function off(): void {
        cmd(0x08)
    }

    //% color=#045F14
    //% blockId="i2c_LCD1620_CLEAR" block="clear"
    //% weight=85 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function clear(): void {
        cmd(0x01)
    }

    //% color=#045F14
    //% blockId="i2c_LCD1620_BACKLIGHT_ON" block="light on"
    //% weight=71 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function BacklightOn(): void {
        BK = 8
        cmd(0)
    }

    //% color=#045F14
    //% blockId="i2c_LCD1620_BACKLIGHT_OFF" block="light off "
    //% weight=70 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function BacklightOff(): void {
        BK = 0
        cmd(0)
    }



}


//% color=#E7734B icon="\uf11b"
namespace joystick {
    // joystick control
    export enum joymove {
        //% blockId="Nostate" block="stop"
        Stop,
        //% block="Up"
        Up,
        //% block="Down"
        Down,
        //% block="Left"
        Left,
        //% block="Right"
        Right,
    }

    export enum dfButton {
        //% block="red"
        P15,
        //% block="blue"
        P16,
        //% block="green"
        P13,
        //% block="yellow"
        P14,
    }
    export let enButtonChannels: { [key: number]: DigitalPin } = {
        [dfButton.P15]: DigitalPin.P15,
        [dfButton.P16]: DigitalPin.P16,
        [dfButton.P13]: DigitalPin.P13,
        [dfButton.P14]: DigitalPin.P14,
    }

    export enum enRocker {
        //% blockId="Nostate" block="Nostate"
        Nostate = 0,
        //% blockId="Up" block="Up"
        Up,
        //% blockId="Down" block="Down"
        Down,
        //% blockId="Left" block="Left"
        Left,
        //% blockId="Right" block="Right"
        Right,
        //% blockId="Press" block="Press"
        Press
    }

    export enum enButtonState {
        //% blockId="Press" block="Press"
        Press = 0,
        //% blockId="Realse" block="Realse"
        Realse = 1
    }

    export enum enButton {
        B1 = 0,
        B2,
        B3,
        B4
    }


    //% group="DFrobot"
    //% color=#383838
    //% block="button %pin is pressed"
    export function joystickbuttonpressed(pin: dfButton): boolean {
        pins.setPull(enButtonChannels[pin], PinPullMode.PullUp);
        let read = enButtonChannels[pin];
        return pins.digitalReadPin(read) == 0;
    }
    //% color=#383838
    //% block="joystick is pressed"
    //% group="DFrobot"
    export function joypressed(): boolean {
        pins.setPull(DigitalPin.P8, PinPullMode.PullUp);
        let read = DigitalPin.P8;
        return pins.digitalReadPin(read) == 0;
    }
    //% color=#383838
    //% block="joystick is %pin"
    //% group="DFrobot"
    export function joystickmove(pin: joymove): boolean {
        let x = pins.analogReadPin(AnalogPin.P1);
        let y = pins.analogReadPin(AnalogPin.P2);
        let now_state = joymove.Stop;
        if (x < 300) {

            now_state = joymove.Left;

        }
        else if (x > 600) {

            now_state = joymove.Right;
        }
        else {
            if (y < 300) {
                now_state = joymove.Down;
            }
            else if (y > 600) {
                now_state = joymove.Up;
            }
        }

        if (now_state == pin)
            return true;
        else
            return false;

    }

    //---------------------------------------------------
    //% group="GHBit"
    //% blockId=ghBit_Rocker block="rocker|value %value"
    //% weight=96
    //% blockGap=10
    //% color="#C814B8"
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=6
    export function Rocker(value: enRocker): boolean {

        pins.setPull(DigitalPin.P8, PinPullMode.PullUp);
        let x = pins.analogReadPin(AnalogPin.P1);
        let y = pins.analogReadPin(AnalogPin.P2);
        let z = pins.digitalReadPin(DigitalPin.P8);
        let now_state = enRocker.Nostate;

        if (x < 200) {

            now_state = enRocker.Up;

        }
        else if (x > 730) {

            now_state = enRocker.Down;
        }
        else {
            if (y < 200) {
                now_state = enRocker.Right;
            }
            else if (y > 730) {
                now_state = enRocker.Left;
            }
        }
        if (z == 0)
            now_state = enRocker.Press;
        if (now_state == value)
            return true;
        else
            return false;

    }
    //% group="GHBit"
    //% blockId=ghBit_Button block="button| %num|value %value"
    //% weight=95
    //% blockGap=10
    //% color="#C814B8"
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=5
    export function Button(num: enButton, value: enButtonState): boolean {
        let temp = false;
        switch (num) {
            case enButton.B1: {
                pins.setPull(DigitalPin.P13, PinPullMode.PullUp);
                if (pins.digitalReadPin(DigitalPin.P13) == value) {
                    temp = true;
                }
                else {
                    temp = false;
                }
                break;
            }
            case enButton.B2: {
                pins.setPull(DigitalPin.P14, PinPullMode.PullUp);
                if (pins.digitalReadPin(DigitalPin.P14) == value) {
                    temp = true;
                }
                else {
                    temp = false;
                }
                break;
            }
            case enButton.B3: {
                pins.setPull(DigitalPin.P15, PinPullMode.PullUp);
                if (pins.digitalReadPin(DigitalPin.P15) == value) {
                    temp = true;
                }
                else {
                    temp = false;
                }
                break;
            }
            case enButton.B4: {
                pins.setPull(DigitalPin.P16, PinPullMode.PullUp);
                if (pins.digitalReadPin(DigitalPin.P16) == value) {
                    temp = true;
                }
                else {
                    temp = false;
                }
                break;
            }
        }
        return temp;
    }



}


//-----------------------------------
