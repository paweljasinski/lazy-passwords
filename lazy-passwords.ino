/*
    MIT License

    Copyright (c) 2019 Pawel Jasinski

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOAFTWARE.
*/

#include <LiquidCrystal.h>
#include <Keyboard.h>
#include <Mouse.h>
#include <EEPROM.h>

#include "unlock_code.h"
#include "labels.h"

// #define INCLUDE_DUMP_EEPROM

const byte LED1 = LED_BUILTIN;
const byte LED2 = LED_BUILTIN_RX;
const byte LED3 = LED_BUILTIN_TX;
const char SPACE = ' ';
// LCD
const byte LCD_COLS = 16;
const byte LCD_ROWS = 2;
const byte PIN_RS = 8, PIN_EN = 9, PIN_D4 = 4, PIN_D5 = 5, PIN_D6 = 6, PIN_D7 = 7, PIN_BL = 10;
LiquidCrystal lcd(PIN_RS,  PIN_EN,  PIN_D4,  PIN_D5,  PIN_D6,  PIN_D7);


// Working variables for the rotary encoder interrupt routines
volatile byte int0signal = 0;
volatile byte int1signal = 0;
volatile byte int0history = 0;
volatile byte int1history = 0;

// circular buffer queue of events
const byte QUEUE_SIZE = 32;
volatile byte queue[QUEUE_SIZE];
volatile byte write_idx = 0;
volatile byte read_idx = 0;
volatile byte qsize = 0;
volatile byte dropped = 0;

// rotary PIN definitions
// CLK
const byte PIN_A = 1, BIT_A = 3, INT_A = 3;
#define PORT_A PIND

// DATA
const byte PIN_B = 2, BIT_B = 1, INT_B = 1;
#define PORT_B PIND

// SW
const byte KEY = 3, BIT_KEY = 0, INT_KEY = 0;
#define PORT_KEY PIND

// key up and key down events for rotary build in switch
const byte KEY_DOWN = 0x80;

#ifdef GENERATE_KEY_UP
const byte KEY_UP   = 0x40;
#endif

const byte N_PASS = 5; // number of passwords
const byte PASSWORDS_EEPROM_BASE_ADDR = 16;
const byte PASSWORD_MAX_LEN = 48; // we waste last byte in eeprom to store \0 so it is 47

const byte EEPROM_MOUSE_DELTA_ADDR = 4; // persist mouse jump value
const byte MOUSE_DELTA_INIT_VALUE = 5;  // default value used when eeprom is not initialized
int8_t mouse_delta; // how much mouse is moved
unsigned long next_screen_saver_action; // millis when we need next key/mouse
unsigned long end_of_mouse_calibration; // millis when to stop calibrating

const int16_t CALIBRATION_TIMEOUT = 5000; // ms, 5 secons of no activity during mouse calibration

unsigned long now; // holds value returned by last call to millis

byte shifter; // collect rotary clk/data bits, 4 x 2 bits
const byte ROTARY_EVENT_MASK = 0x0F; // only 4 bits get into event

const byte CALIBRATION_STATE = 2;
const byte OPERATION_STATE   = 1;
const byte LOCKED_STATE      = 0;
byte state = LOCKED_STATE;

void setup() {

    lcd.begin(LCD_COLS, LCD_ROWS);

    // Not sure if it is needed
    // pinMode(LED1, OUTPUT);
    // pinMode(LED2, OUTPUT);
    // pinMode(LED3, OUTPUT);

    pinMode(PIN_A, INPUT);
    attachInterrupt(INT_A, int0, CHANGE);
    pinMode(PIN_B, INPUT);
    attachInterrupt(INT_B, int1, CHANGE);

    pinMode(KEY, INPUT_PULLUP);
    attachInterrupt(INT_KEY, int2, CHANGE);


    // initial content for rotary encoder
    int0signal = bitRead(PORT_A, PIN_A);
    int1signal = bitRead(PORT_B, PIN_B);
    shifter = int0signal << 1 | int1signal;

    Keyboard.begin();
    Mouse.begin();
    Serial.begin(115200);
    // delay(2000);
}


void int0() {
    int0history = int0signal;
    int0signal = bitRead(PORT_A, BIT_A);
    if (int0history == int0signal) {
        return; // yes, it can interrupt and during read it is already back to original value
        // most debouncing is happening here
    }
    shifter = shifter << 2 | int1signal << 1 | int0signal;
    enque(shifter & ROTARY_EVENT_MASK);
}

void int1() {
    int1history = int1signal;
    int1signal = bitRead(PORT_B, BIT_B);
    if (int1history == int1signal) {
        return;
    }
    shifter = shifter << 2 | int1signal << 1 | int0signal;
    // trere are 2 encoder steps per grove, intentionally skip one step to match grove on the rotary
    // enque(shifter & ROTARY_EVENT_MASK);
}

const byte KEY_DEBOUNCE = 50; // ms
void int2() {
    static unsigned long last_key_interrupt_time = 0;
    static int key_down = 0;
    now = millis();
    if (now - last_key_interrupt_time > KEY_DEBOUNCE) {
        if (key_down) {
            key_down = 0;
#ifdef GENERATE_KEY_UP
            enque(KEY_UP);
#endif
        } else {
            key_down = 1;
            enque(KEY_DOWN);
        }
    }
    last_key_interrupt_time = now;
}

/*
    queue is implemented as cicular buffer
    queue state can be changed in interrupt routine or with blocked interrupts
*/
void enque(byte b) {
    if (qsize == QUEUE_SIZE) {
        dropped++;
        return;
    }
    queue[write_idx++] = b;
    if (write_idx == QUEUE_SIZE) {
        write_idx = 0;
    }
    qsize++;
}

/*
    return number of dequed bytes
    out - buffer where the bytes are written out
    n - maximum number of bytes to deque
*/
byte deque(byte * out, byte n) {
    if (qsize == 0) {
        return 0;
    }
    byte start_n = n;
    noInterrupts();
    while (qsize > 0 && n > 0) {
        *out++ = queue[read_idx++];
        if (read_idx == QUEUE_SIZE) {
            read_idx = 0;
        }
        qsize--;
        n--;
    }
    interrupts();
    return start_n - n;
}


/*
    type n'th password
*/
void send_password(byte n) {
    // send n'th password as keyboard
    uint16_t src_addr = PASSWORDS_EEPROM_BASE_ADDR + n * PASSWORD_MAX_LEN;
    uint16_t end_addr = src_addr + PASSWORD_MAX_LEN;
    byte c;
    do {
        c = EEPROM.read(src_addr++);
        Keyboard.press(c);
        Keyboard.releaseAll();
    } while (c != 0 && src_addr != end_addr);
}

/*
    Retrieve mouse delta
    If this is first time use of EEPROM, initialize it
*/
int get_mouse_delta() {
    char id[4];
    for (int addr = 0; addr < 4; addr++) {
        id[addr] = EEPROM.read(addr);
    }
    if (strncmp(id, "ssk1", 4) != 0) {
        // one time init
        EEPROM.write(0, 's'); // screen
        EEPROM.write(1, 's'); // saver
        EEPROM.write(2, 'k'); // killer
        EEPROM.write(3, '1'); // version 1 stores mouse delta
        EEPROM.write(EEPROM_MOUSE_DELTA_ADDR, MOUSE_DELTA_INIT_VALUE);
    }
    return EEPROM.read(EEPROM_MOUSE_DELTA_ADDR);
}

// index into table is constrict from DATA << 1 | CLK of the rotary
const int8_t rot_enc_table[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};


// this is a list which is periodically filled up from the circular buffer
const byte EVENT_LST_SIZE = 16;
byte events[EVENT_LST_SIZE];

// event processing variables, no reason to be global other than saving stack space
byte event_count;
byte event;

void loop() {
    if (state == CALIBRATION_STATE) {
        calibration_loop();
    } else if (state == OPERATION_STATE) {
        shortcuts_loop();
    } else {
        unlock_loop();
    }
}

void shortcuts_loop() {
    screen_saver();
    process_shortcut_events();
    process_password_update();
}

unsigned long KEY_UP_DELAY = 5; // ms
unsigned long SCREEN_SAVER_TIMEOUT = 58000; // ms, just under a minute

void screen_saver() {
    now = millis();
    if (now > next_screen_saver_action) {
        mouse_delta = -mouse_delta;
        Mouse.move(mouse_delta, 0, 0);
        Keyboard.press(KEY_RIGHT_SHIFT);
        delay(KEY_UP_DELAY);
        Keyboard.release(KEY_RIGHT_SHIFT);
        next_screen_saver_action = now + KEY_UP_DELAY + SCREEN_SAVER_TIMEOUT;
    }
}


const byte N_LAYOUT = 2;
const byte EN_US_LAYOUT = 0;
const byte DE_CH_LAYOUT = 1;
byte layout = EN_US_LAYOUT; // track currnt layout

#ifdef INCLUDE_DUMP_EEPROM
const byte N_LABELS = N_PASS + N_LAYOUT + 1;
#else
const byte N_LABELS = N_PASS + N_LAYOUT;
#endif


const char labels[N_LABELS][LCD_COLS + 1] = { PASS0_LABEL, PASS1_LABEL, PASS2_LABEL, PASS3_LABEL, PASS4_LABEL,
                                              "US Layout", "Swiss Layout"
#ifdef INCLUDE_DUMP_EEPROM
                                              , "dump eeprom"
#endif
                                            };

int8_t pos = 0;                         // possition of the rotary
int8_t last_pos = -1;                   // track change, trigger initial update

void mark_encoding_error() {
    // never got here
    lcd.setCursor(5, 1);
    lcd.print('E');
}

void process_shortcut_events() {
    while (0 != (event_count = deque(events, EVENT_LST_SIZE))) {
        // as long as there are events, process process process
        for (byte i = 0; i < event_count; i++) {
            event = events[i];
            if (event & KEY_DOWN) {
                if (pos < N_PASS) {
                    send_password(pos);
                } else if (pos < N_PASS + N_LAYOUT) {
                    layout = pos - N_PASS;
                    Keyboard.switchLayout(layout);
#ifdef INCLUDE_DUMP_EEPROM
                } else {
                    dump_eeprom(32 * 8);
#endif
                }
                update_display();
#ifdef GENERATE_KEY_UP
            } else if (event & KEY_UP) {
#endif
            } else {
                // everything else is rotary event
                int8_t delta = rot_enc_table[event];
                if (delta) {
                    pos += delta;
                } else {
                    mark_encoding_error();
                }
            }
        }
    }

    if (pos < 0) {
        pos = N_LABELS - 1;
    } else if (pos >= N_LABELS) {
        pos = 0;
    }

    // no more events to process
    // update display
    if (last_pos != pos) {
        update_display();
        last_pos = pos;
    }
}

char layout_labels[2][6] = {"en-US", "de-CH"};
void update_display() {
    lcd.setCursor(0, 0);
    const char * label = labels[pos];
    lcd.print(label);
    for (byte i = 0; i < LCD_COLS - strnlen(label, LCD_COLS); i++) {
        lcd.write(SPACE);
    }
    lcd.setCursor(0, 1);
    lcd.print(layout_labels[layout]);
}

void process_password_update() {
    if (!Serial.available()) {
        return;
    }
    // example content for update:
    // 1:whatever\n\0
    // 2:somethingelse\n\0
    byte buffer[PASSWORD_MAX_LEN + 2];
    // The terminator character is discarded!
    byte count = Serial.readBytesUntil('\0', buffer, PASSWORD_MAX_LEN + 2);
    if (count == 0) {
        return; // timeout, error or just a termination character
    }
    byte password_idx = buffer[0] - '0';
    if (password_idx >= N_PASS) {
        return; // wrong index, ignore
    }
    uint16_t dest_addr = PASSWORDS_EEPROM_BASE_ADDR + PASSWORD_MAX_LEN * password_idx;
    byte i;
    for (i = 0; i < count - 2; i++) {
        EEPROM.update(dest_addr + i, buffer[2 + i]);
    }
    // zero rest of the password, add removed termination \0
    for (byte j = i; j < PASSWORD_MAX_LEN; j++) {
        EEPROM.update(dest_addr + j, '\0');
    }
    lcd.setCursor(0, 1);
    lcd.print("updated ");
    lcd.print(password_idx);
}

void lcdPrintPaddedByte(byte n) {
    if (n < 0x10) {
        lcd.print('0');
    }
    lcd.print(n, HEX);
}

/*
     unlock
*/
void unlock_loop() {
    static byte number = 0;
    static byte code[CODE_SIZE];
    while (0 != (event_count = deque(events, EVENT_LST_SIZE))) {
        // as long as there are events, process process process
        for (byte i = 0; i < event_count; i++) {
            event = events[i];
            if (event & KEY_DOWN) {
                // shift codes
                for (byte j = CODE_SIZE - 1; j > 0; j--) {
                    code[j] = code[j - 1];
                }
                code[0] = number;
                if (0 == memcmp(code, CODE, CODE_SIZE)) {
                    state = CALIBRATION_STATE;
                    lcd.clear();
                    end_of_mouse_calibration = millis() + CALIBRATION_TIMEOUT;
                    mouse_delta = get_mouse_delta();
                    lcd.setCursor(0, 0);
                    lcd.print("mouse calib: ");
                    lcd.print(mouse_delta);
                    return;
                }
                break;
#ifdef GENERATE_KEY_UP
            } else if (event & KEY_UP) {
                // not much
#endif
            } else {
                // everything else is rotary event
                int8_t delta = rot_enc_table[event];
                if (delta) {
                    number += delta;
                } else {
                    mark_encoding_error();
                }
            }
        }
    }
    lcd.setCursor(0, 0);
    lcdPrintPaddedByte(number);
    //lcd.print(number, HEX);
    lcd.setCursor(0, 1);
    for (int i = 0; i < CODE_SIZE; i++) {
        lcdPrintPaddedByte(code[i]);
        lcd.print(SPACE);
    }
}


/*
    calibration
*/
void calibration_loop() {
    unsigned long now = millis();
    static unsigned long last_move_mouse = 0;
    while (0 != (event_count = deque(events, EVENT_LST_SIZE))) {
        // as long as there are events, process process process
        for (byte i = 0; i < event_count; i++) {
            event = events[i];
            if (event & KEY_DOWN) {
                state = OPERATION_STATE;
                break;
#ifdef GENERATE_KEY_UP
            } else if (event & KEY_UP) {
                // not much
#endif
            } else {
                // everything else is rotary event
                int8_t delta = rot_enc_table[event];
                if (delta) {
                    mouse_delta += delta;
                    end_of_mouse_calibration = now + CALIBRATION_TIMEOUT;
                } else {
                    mark_encoding_error();
                }
            }
        }
    }

    if (now > end_of_mouse_calibration) {
        state = OPERATION_STATE; // end of calibration loop
    }
    if (state == CALIBRATION_STATE) {
        lcd.setCursor(13, 0);
        lcd.print(mouse_delta);
        lcd.print("   ");
        if (now > last_move_mouse + 100) {
            Mouse.move(mouse_delta, 0, 0);
            last_move_mouse = now;
        }
    } else {
        lcd.clear();
    }
}



#ifdef INCLUDE_DUMP_EEPROM

const byte DUMP_BUFFER_SIZE = 16;
const byte BYTES_PER_ROW = 16;
const char * const CRLF = "\r\n";
void dump_eeprom(uint16_t n) {
    char buffer[DUMP_BUFFER_SIZE];
    Serial.write("000000 ", 7);
    for (uint16_t i = 0; i < n; i++) {
        if (i && (i % BYTES_PER_ROW == 0)) {
            sprintf(buffer, "%06x ", i);
            Serial.write(buffer, 7);
        }
        sprintf(buffer, "%02x", EEPROM.read(i));
        Serial.write(buffer, 2);
        if ((i + 1) % BYTES_PER_ROW != 0) {
            Serial.write(SPACE);
        } else {
            Serial.write(CRLF);
        }
    }
    Serial.write(CRLF);
}

#endif
