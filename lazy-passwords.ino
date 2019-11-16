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


// uncomment if needed
// #define INCLUDE_DUMP_EEPROM

const byte LED1 = LED_BUILTIN;
const byte LED2 = LED_BUILTIN_RX;
const byte LED3 = LED_BUILTIN_TX;
const char SPACE = ' ';


/******************************************************************************

    LCD 1602

*******************************************************************************/
const byte LCD_COLS = 16;
const byte LCD_ROWS = 2;
const byte PIN_RS = 8, PIN_EN = 9, PIN_D4 = 4, PIN_D5 = 5, PIN_D6 = 6, PIN_D7 = 7, PIN_BL = 10;
LiquidCrystal lcd(PIN_RS,  PIN_EN,  PIN_D4,  PIN_D5,  PIN_D6,  PIN_D7);


/******************************************************************************

    Rotary Encoder

*******************************************************************************/
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

byte shifter; // collect rotary clk/data bits, 4 x 2 bits
const byte ROTARY_EVENT_MASK = 0x0F; // only 4 bits get into event

/******************************************************************************

    Label and password

*******************************************************************************/
const byte N_PASS = 5; // number of passwords/labels
const byte LABEL_BASE_ADDR = 16; // EEPROM
const byte PASSWORDS_EEPROM_BASE_ADDR = LABEL_BASE_ADDR + LCD_COLS * N_PASS;
const byte PASSWORD_MAX_LEN = 48;

/******************************************************************************

    Mouse and screen saver

*******************************************************************************/
const byte EEPROM_MOUSE_DELTA_ADDR = 4; // persist mouse jump value
const byte MOUSE_DELTA_INIT_VALUE = 5;  // default value used when eeprom is not initialized
int8_t mouse_delta; // how much mouse is moved
uint32_t next_screen_saver_action; // millis when we need next key/mouse
uint32_t end_of_mouse_calibration; // millis when to stop calibrating
const int16_t CALIBRATION_TIMEOUT = 5000; // ms, 5 secons of no activity during mouse calibration

/******************************************************************************

    PIN

*******************************************************************************/
const byte PIN_ADDR = 5; // pin in EEPROM
const byte PIN_SIZE = 5;

/******************************************************************************

    KEYBOARD LAYOUT

*******************************************************************************/
const byte KEYBOARD_LAYOUT_ADDR = PIN_ADDR + PIN_SIZE;
const byte N_LAYOUT = 2;
const byte EN_US_LAYOUT = 0;
const byte DE_CH_LAYOUT = 1;
byte layout; // track current layout


/******************************************************************************

    CAPS LOCK BUZZER

*******************************************************************************/
#define BUZZER A1
#define ANALOG_KEYBOARD A0



uint32_t now; // holds value returned by last call to millis

/******************************************************************************

    STATE

*******************************************************************************/
const byte CHANGE_PIN_STATE  = 3;
const byte CALIBRATION_STATE = 2;
const byte MENU_STATE        = 1;
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

    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, 0);

    pinMode(ANALOG_KEYBOARD, INPUT);

    // initial content for rotary encoder
    int0signal = bitRead(PORT_A, PIN_A);
    int1signal = bitRead(PORT_B, PIN_B);
    shifter = int0signal << 1 | int1signal;

    Keyboard.begin();
    Mouse.begin();
    Serial.begin(115200);
    // delay(2000);
    start_locked();
    eeprom_one_time_init();
    mouse_delta = EEPROM.read(EEPROM_MOUSE_DELTA_ADDR);
    layout = EEPROM.read(KEYBOARD_LAYOUT_ADDR);
}


/*
    If this is first time use of EEPROM, initialize it
*/
void eeprom_one_time_init() {
    uint16_t addr = 0;
    if (EEPROM.read(addr++) == 's' &&
        EEPROM.read(addr++) == 's' &&
        EEPROM.read(addr++) == 'k' &&
        EEPROM.read(addr++) == '1') {
            return;
    }
    // one time init
    addr = 0;
    EEPROM.write(addr++, 's'); // screen
    EEPROM.write(addr++, 's'); // saver
    EEPROM.write(addr++, 'k'); // killer
    EEPROM.write(addr++, '1'); // version 1
    EEPROM.write(addr++, MOUSE_DELTA_INIT_VALUE);
    EEPROM.write(addr++, 0xff); // PIN
    EEPROM.write(addr++, 0xff); // PIN
    EEPROM.write(addr++, 0xff); // PIN
    EEPROM.write(addr++, 0xff); // PIN
    EEPROM.write(addr++, 0x00); // PIN
    EEPROM.write(addr++, EN_US_LAYOUT); // Keyboard layout
}

/******************************************************************************

    Util

*******************************************************************************/
uint32_t short_message_cleanup = 0; // when to remove short message
void show_short_message(const char * const msg) {
    clean_short_message();
    lcd.setCursor(LCD_COLS - 3, LCD_ROWS - 1);
    lcd.print(msg);
    short_message_cleanup = millis() + 3000;
    // Serial.println(msg);
}

void clean_short_message() {
    lcd.setCursor(LCD_COLS - 3, LCD_ROWS - 1);
    lcd.print("   ");
}

void process_short_message_cleanup() {
    if (0 == short_message_cleanup) {
        return;
    }
    if (short_message_cleanup < millis()) {
        short_message_cleanup = 0;
        clean_short_message();
        lcd.setCursor(0, 0);
    }
}

void lcd_print_padded_byte(byte n) {
    if (n < 0x10) {
        lcd.print('0');
    }
    lcd.print(n, HEX);
}


/******************************************************************************

    Interrupts

*******************************************************************************/
void int0() {
    // CLK
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
    // DATA
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
    // KEY
    static uint32_t last_key_interrupt_time = 0;
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



// index into table is constructed from DATA << 1 | CLK of the rotary
const int8_t rot_enc_table[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

void mark_encoding_error() {
    // never got here
    show_short_message("ERE");
}

/*
    translate rotary event to delta
    returns either -1, 1, or 0
*/
byte rotary_delta(byte event) {
    int8_t delta = rot_enc_table[event];
    if (!delta) {
        mark_encoding_error();
    }
    return delta;
}


/*
    type n'th password
*/
void send_password(byte n) {
    // send n'th password as keyboard
    uint16_t src_addr = PASSWORDS_EEPROM_BASE_ADDR + n * PASSWORD_MAX_LEN;
    uint16_t end_addr = src_addr + PASSWORD_MAX_LEN;
    byte c;
    while (src_addr != end_addr && 0 != (c = EEPROM.read(src_addr++))) {
        Keyboard.press(c);
        Keyboard.releaseAll();
    };
}


// this is a list which is periodically filled up from the circular buffer
const byte EVENT_LST_SIZE = 16;
byte events[EVENT_LST_SIZE];

// event processing variables, no reason to be global other than saving stack space
byte event_count;
byte event;

/******************************************************************************

    LOOP

*******************************************************************************/
void loop() {
    if (state == CALIBRATION_STATE) {
        calibration_loop();
    } else if (state == MENU_STATE) {
        screen_saver();
        process_shortcut_events();
        process_config_update();
        process_short_message_cleanup();
        process_caps_lock();
    } else if (state == CHANGE_PIN_STATE) {
        change_pin_loop();
    } else {
        locked_loop();
    }
}

// any key on the LCD Keypad Shield can mute/unmute caps lock
byte loud_caps_lock = 1; // should it be loud
byte last_caps_mute = 0;
void process_caps_lock() {
    // state
    if (analogRead(ANALOG_KEYBOARD) < 1000) {
        if (! last_caps_mute) {
            last_caps_mute = 1;
            loud_caps_lock = ! loud_caps_lock;
        }
    } else {
        last_caps_mute = 0;
    }
    // display
    lcd.setCursor(7, 1);
    if (Keyboard.getLedStatus() & LED_CAPS_LOCK) {
        lcd.print("CAPS");
        digitalWrite(BUZZER, loud_caps_lock);
    } else {
        lcd.print("    ");
        digitalWrite(BUZZER, 0);
    }
}


#ifdef INCLUDE_DUMP_EEPROM
const byte N_LABELS = N_PASS + N_LAYOUT + 2;
#else
const byte N_LABELS = N_PASS + N_LAYOUT + 1;
#endif


const char static_labels[N_LABELS - N_PASS][13] = { "US Layout", "Swiss Layout", "Change Pin"
#ifdef INCLUDE_DUMP_EEPROM
                                                    , "dump eeprom"
#endif
                                                  };

/******************************************************************************

    MENU_STATE

*******************************************************************************/
int8_t pos = 0;                         // possition of the rotary
int8_t last_pos = -1;                   // track change, trigger initial update

void start_menu() {
    state = MENU_STATE;
    lcd.clear();
    update_display();
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
                    EEPROM.write(KEYBOARD_LAYOUT_ADDR, layout);
                } else if (pos == N_PASS + N_LAYOUT) {
                    start_change_pin();
                    return;
#ifdef INCLUDE_DUMP_EEPROM
                } else {
                    dump_eeprom(PASSWORDS_EEPROM_BASE_ADDR + N_PASS * PASSWORD_MAX_LEN);
#endif
                }
                update_display();
#ifdef GENERATE_KEY_UP
            } else if (event & KEY_UP) {
#endif
            } else {
                pos += rotary_delta(event);
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
    const char * label;
    if (pos < N_PASS) {
        uint16_t addr = LABEL_BASE_ADDR + pos * LCD_COLS;
        char lc;
        byte i = 0;
        while (0 != (lc = EEPROM.read(addr + i)) && i < LCD_COLS) {
            lcd.write(lc);
            i++;
        }
        for (byte j = LCD_COLS - i; j > 0; j--) {
            lcd.write(SPACE);
        }
    } else {
        label = static_labels[pos - N_PASS];
        lcd.print(label);
        for (byte i = 0; i < LCD_COLS - strnlen(label, LCD_COLS); i++) {
            lcd.write(SPACE);
        }
    }
    lcd.setCursor(0, 1);
    lcd.print(layout_labels[layout]);
}


/******************************************************************************

    MENU_STATE / Screen Saver BLocker

*******************************************************************************/

uint32_t KEY_UP_DELAY = 5; // ms
uint32_t SCREEN_SAVER_TIMEOUT = 58000; // ms, just under a minute

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


const byte PREFIX_SIZE = 3; // cmd address comma
const byte CMD_BUFFER_SIZE = PREFIX_SIZE + PASSWORD_MAX_LEN;

/******************************************************************************

    MENU_STATE / Configuration Update

*******************************************************************************/
void process_config_update() {
    if (!Serial.available()) {
        return;
    }
    // example content for update:
    // l0,label1\0
    // l1,1234567890123456\0
    //             1
    // p0,something\n\0
    // p1,123456789012345678901234567890123456789012345678\0
    //             1         2         3         4
    byte buffer[CMD_BUFFER_SIZE];
    Serial.setTimeout(250);
    // The terminator character is discarded!
    byte count = Serial.readBytesUntil('\0', buffer, CMD_BUFFER_SIZE);
    if (count == 0) {
        return;
    }
    if (count < PREFIX_SIZE) {
        show_short_message("E00");
        return;
    }
    byte idx = buffer[1] - '0';
    if (idx >= N_PASS) {
        show_short_message("E01");
        return; // wrong index, ignore
    }
    byte end_count;
    uint16_t dest_addr;
    byte i;
    if (buffer[0] == 'p') { // password
        dest_addr = PASSWORDS_EEPROM_BASE_ADDR + PASSWORD_MAX_LEN * idx;
        end_count = PASSWORD_MAX_LEN;
    } else if (buffer[0] == 'l') {
        if (count > PREFIX_SIZE + LCD_COLS) {
            show_short_message("E03");
            return; // too many characters for label
        }
        dest_addr =  LABEL_BASE_ADDR + LCD_COLS * idx;
        end_count = LCD_COLS;
    } else {
        show_short_message("E02");
        return; // invalid command
    }

    for (i = 0; i < count - PREFIX_SIZE; i++) {
        EEPROM.update(dest_addr + i, buffer[PREFIX_SIZE + i]);
    }
    // zero rest, in case of prefix only message it has an effect of erasing label or password
    for (byte j = i; j < end_count; j++) {
        EEPROM.update(dest_addr + j, '\0');
    }

    char msg[4];
    msg[0] = SPACE;
    msg[1] = buffer[0];
    msg[2] = buffer[1];
    msg[3] = '\0';
    show_short_message(msg);
}


/******************************************************************************

    CHANGE_PIN_STATE

*******************************************************************************/

byte change_pin_number = 0;
byte change_pin_code[PIN_SIZE];
byte change_pin_count = 0;
byte change_pin_last_number = 0;

void start_change_pin() {
    state = CHANGE_PIN_STATE;
    change_pin_number = 0;
    memset(change_pin_code, 0, PIN_SIZE);
    change_pin_count = 0;
    change_pin_last_number = 1; // trigger display update
    lcd.clear();
    lcd.print("Enter new pin");
}

void update_pin() {
    for (byte i = 0; i < PIN_SIZE; i++) {
        EEPROM.update(PIN_ADDR + i, change_pin_code[i]);
    }
}

void change_pin_loop() {
    while (0 != (event_count = deque(events, EVENT_LST_SIZE))) {
        // as long as there are events, process process process
        for (byte i = 0; i < event_count; i++) {
            event = events[i];
            if (event & KEY_DOWN) {
                if (change_pin_count < PIN_SIZE) {
                    change_pin_code[change_pin_count++] = change_pin_number;
                    change_pin_last_number = !change_pin_number; // trigger refresh
                    if (change_pin_count == PIN_SIZE) {
                        lcd.setCursor(12, 0);
                        lcd.print("    ");
                        change_pin_number = 0; // cancel
                        change_pin_last_number = 1; // force refresh
                    }
                } else {
                    if (change_pin_number % 2) { // even is cancel, odd is ok
                        // update pin
                        update_pin();
                    }
                    start_menu();
                    return;
                }
                break;
#ifdef GENERATE_KEY_UP
            } else if (event & KEY_UP) {
                // not much
#endif
            } else {
                change_pin_last_number = change_pin_number;
                change_pin_number += rotary_delta(event);
            }
        }
    }
    if (change_pin_number != change_pin_last_number) {
        if (change_pin_count < PIN_SIZE) {
            lcd.setCursor(change_pin_count * 3, 1);
            lcd_print_padded_byte(change_pin_number);
        } else {
            lcd.setCursor(0, 0);
            if (change_pin_number % 2) {
                lcd.print(" Cancel >OK<");
            } else {
                lcd.print(">Cancel< OK ");
            }
        }
    }
}

/******************************************************************************

    LOCKED_STATE

*******************************************************************************/

byte code[PIN_SIZE]; // pin entered by a user

byte pin_match() {
    for (int i = 0; i < PIN_SIZE; i++) {
        if (code[i] != EEPROM.read(PIN_ADDR + i)) {
            return 0;
        }
    }
    return 1;
}

void start_locked() {
    state = LOCKED_STATE;
    lcd.clear();
    lcd.print("Enter pin");
}

/*
     unlock / enter pin
*/
void locked_loop() {
    static byte number = 0;
    static byte count = 0;
    static byte last_unlock_number = 1;
    while (0 != (event_count = deque(events, EVENT_LST_SIZE))) {
        // as long as there are events, process process process
        for (byte i = 0; i < event_count; i++) {
            event = events[i];
            if (event & KEY_DOWN) {
                if (count < PIN_SIZE) {
                    code[count++] = number;
                } else {
                    code[PIN_SIZE - 1] = number;
                }
                if (count == PIN_SIZE) {
                    if (pin_match()) {
                        start_calibration();
                        return;
                    }
                    // shift left
                    for (byte i = 1; i < PIN_SIZE; i++) {
                        code[i - 1] = code[i];
                    }
                    lcd.setCursor(0, 1);
                    for (byte i = 0; i < PIN_SIZE; i++) {
                        lcd_print_padded_byte(code[i]);
                        lcd.print(" ");
                    }
                }
#ifdef GENERATE_KEY_UP
            } else if (event & KEY_UP) {
                // not much
#endif
            } else {
                last_unlock_number = number;
                number += rotary_delta(event);
            }
        }
    }
    if (last_unlock_number != number) {
        lcd.setCursor(count < PIN_SIZE ? count * 3 : (PIN_SIZE - 1) * 3, 1);
        lcd_print_padded_byte(number);
    }
}

/******************************************************************************

    CALIBRATION_STATE

*******************************************************************************/

void start_calibration() {
    state = CALIBRATION_STATE;
    lcd.clear();
    end_of_mouse_calibration = millis() + CALIBRATION_TIMEOUT;
    lcd.setCursor(0, 0);
    lcd.print("mouse calib: ");
    lcd.print(mouse_delta);
}

void calibration_loop() {
    uint32_t now = millis();
    static uint32_t last_move_mouse = 0;
    while (0 != (event_count = deque(events, EVENT_LST_SIZE))) {
        // as long as there are events, process process process
        for (byte i = 0; i < event_count; i++) {
            event = events[i];
            if (event & KEY_DOWN) {
                state = MENU_STATE;
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
        state = MENU_STATE; // end of calibration loop
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
        // persist mouse delta
        EEPROM.write(EEPROM_MOUSE_DELTA_ADDR, mouse_delta);
    }
}


/******************************************************************************

    EEPROM_DUMP

*******************************************************************************/

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
