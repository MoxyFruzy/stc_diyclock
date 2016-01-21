// DS1302 RTC IC
// http://datasheets.maximintegrated.com/en/ds/DS1302.pdf
//

#include "ds1302.h"

void ds_writebit(__bit bit) {
    _nop_; _nop_;
    DS_IO = bit;
    DS_SCLK = 1;
    _nop_; _nop_;
    DS_SCLK = 0;
}

__bit ds_readbit() {
    __bit b;
    _nop_; _nop_;
    b = DS_IO;
    DS_SCLK = 1;
    _nop_; _nop_;
    DS_SCLK = 0;
    return b;
}

uint8_t ds_readbyte(uint8_t addr) {
    // ds1302 single-byte read
    uint8_t i, b = 0;
    b = DS_CMD | DS_CMD_CLOCK | addr << 1 | DS_CMD_READ;
    DS_CE = 0;
    DS_SCLK = 0;
    DS_CE = 1;
    // send cmd byte
    for (i=0; i < 8; i++) {
        ds_writebit((b >> i) & 0x01);
        _nop_; _nop_;
    }
    // read byte
    for (i=0; i < 8; i++) {
        if (ds_readbit()) 
            b |= 1 << i; // set
        else
            b &= ~(1 << i); // clear  
        _nop_; _nop_;              
    }
    DS_CE = 0;
    return b;
}

void ds_readburst(uint8_t time[8]) {
    // ds1302 burst-read 8 bytes into struct
    uint8_t i, j, b = 0;
    b = DS_CMD | DS_CMD_CLOCK | DS_BURST_MODE << 1 | DS_CMD_READ;
    DS_CE = 0;
    DS_SCLK = 0;
    DS_CE = 1;
    // send cmd byte
    for (i=0; i < 8; i++) {
        ds_writebit((b >> i) & 0x01);
        _nop_; _nop_;
    }
    // read bytes
    for (j=0; j < 8; j++) {
        for (i=0; i < 8; i++) {
            if (ds_readbit()) 
                b |= 1 << i; // set
            else
                b &= ~(1 << i); // clear  
            _nop_; _nop_;              
        }
        time[j] = b;
    }
    DS_CE = 0;
}

uint8_t ds_writebyte(uint8_t addr, uint8_t data) {
    // ds1302 single-byte write
    uint8_t i, b = 0;
    b = DS_CMD | DS_CMD_CLOCK | addr << 1 | DS_CMD_WRITE;
    DS_CE = 0;
    DS_SCLK = 0;
    DS_CE = 1;
    // send cmd byte
    for (i=0; i < 8; i++) {
        ds_writebit((b >> i) & 0x01);
        _nop_; _nop_;
    }
    // send data byte
    for (i=0; i < 8; i++) {
        ds_writebit((data >> i) & 0x01);
        _nop_; _nop_;
    }

    DS_CE = 0;
    return b;
}


// set hours
void ds_set_hours(struct ds1302_rtc* rtc) {
    uint8_t b = ds_readbyte(DS_ADDR_HOUR);
    b = (b & 0b11000000) | rtc->h12.tenhour << 4 | rtc->h12.hour;
    ds_writebyte(DS_ADDR_HOUR, b);
}

// set minutes
void ds_set_minutes(struct ds1302_rtc* rtc) {
    uint8_t b = ds_readbyte(DS_ADDR_MINUTES);
    b = (b & 0b10000000) | rtc->tenminutes << 4 | rtc->minutes;
    ds_writebyte(DS_ADDR_MINUTES, b);
}

// increment hours
void ds_hours_incr(struct ds1302_rtc* rtc) {
    if (rtc->h12.hour_12_24) {
        // 12 hour
        if (rtc->h12.hour < 2 || (rtc->h12.tenhour < 1 && rtc->h12.hour < 9)) {
            rtc->h12.hour++;
        } else {
            rtc->h12.hour = 0;
            if (rtc->h12.tenhour < 1)
                rtc->h12.tenhour++;
            else
                rtc->h12.tenhour = 0;
        }
    } else {
        // TODO: 24 hour
    }
}

// increment minutes
void ds_minutes_incr(struct ds1302_rtc* rtc) {
    if (rtc->minutes < 9) {
        rtc->minutes++;
    } else {
        rtc->minutes = 0;
        if (rtc->tenminutes < 5)
            rtc->tenminutes++;
        else
            rtc->tenminutes = 0;
    }
}
