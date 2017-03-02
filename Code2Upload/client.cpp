/* Changes to client.cpp:
  - PathMaker Structure
  - Additions to the select event.
*/
#include <Arduino.h>
#include <Adafruit_ST7735.h>
#include <SD.h>
#include <mem_syms.h>

#include "map.h"
#include "serial_handling.h"
#include "dprintf.h"


// map scrolling
// #define DEBUG_SCROLLING

// path drawing
//#define DEBUG_PATH

// to display initial memory and when getting low.
#define DEBUG_MEMORY
// If memory gets below this we want a warning, and we will truncate
// a waypoint path allocation that will bring us below that mark.
size_t memory_low_mark = 1024;

// SD card related
#define DEBUG_SD_CARD

// Pins and interrupt lines for the zoom in and out buttons.
const uint8_t zoom_in_interrupt = 1;     // Digital pin 3.
const uint8_t zoom_in_pin = 3;

const uint8_t zoom_out_interrupt = 0;    // Digital pin 2.
const uint8_t zoom_out_pin = 2;

// the pins used to connect to the AdaFruit display
const uint8_t sd_cs = 5;
const uint8_t tft_cs = 6;
const uint8_t tft_dc = 7;
const uint8_t tft_rst = 8;

// Arduino analog input pin for the horizontal on the joystick.
const uint8_t joy_pin_x = 1;
// Arduino analog input pin for the vertical on the joystick.
const uint8_t joy_pin_y = 0;
// Digital pin for the joystick button on the Arduino.
const uint8_t joy_pin_button = 4;

// forward function declarations
void initialize_sd_card();
void initialize_screen();
void initialize_joystick();
uint8_t process_joystick(int16_t *dx, int16_t *dy);
void status_msg(char *msg);
void clear_status_msg();
void flush_incSerial();

// Interrupt routines for zooming in and out, and their shared
// state
extern volatile uint8_t shared_new_map_num;
volatile uint8_t shared_current_map_num;
void handle_zoom_in();
void handle_zoom_out();

// global state variables

// globally accessible screen
Adafruit_ST7735 tft = Adafruit_ST7735(tft_cs, tft_dc, tft_rst);

// Map number (zoom level) currently selected.
extern uint8_t current_map_num;

// First time flag for loop, set to cause actions for the first time only
uint8_t first_time;

// Storage for the current set of waypoints
uint16_t max_path_len;
uint16_t cur_path_len;
LonLat32* waypoints;

struct path_maker {
/*
Decided to make a struct to organize all the functions used in this assignment.
NOTE: Incase we needed extra memory, I decided that the waypoints will only change
      values rather than storing the new converted waypoint values into another array.
*/
  bool prev_exist; // variable used to see if there is already a path printed on the screen

  path_maker() {
    this->prev_exist = false;
  }

  void convert_wp_to_map_coord(LonLat32* waypoints) {
    /* Converts to the given waypoints to map coordinates for client use
        Args:
          waypoints:  A pointer to an array holding the waypoint values. These
                      values must be the "graphical" values from the csv file that
                      was given by the server.

        Returns:      This function does not return anything but rather changes
                      the "graphical" values of the waypoints (from the csv file),
                      to "map coordinate" values based on the zoom level of the map.
    */
    for (int index = 0; index < cur_path_len; index++) {
      waypoints[index].lon = longitude_to_x(current_map_num, waypoints[index].lon);
      waypoints[index].lat = latitude_to_y(current_map_num, waypoints[index].lat);
      dprintf("%ld, %ld, %d, %d", waypoints[index].lon, waypoints[index].lat, screen_map_x, screen_map_y);
    }
  }

  void convert_wp_to_graph_coord(LonLat32* waypoints) {
    /* Converts to the given waypoints to map coordinates for client use
        Args:
          waypoints:  A pointer to an array holding the waypoint values. These
                      values must be in "map coordinate" form.

        Returns:      This function does not return anything but rather changes
                      the "map coordinate" values of the waypoints, to its
                      equivalent "graphical" values.
    */
    for (int index = 0; index < cur_path_len; index++) {
      waypoints[index].lon = x_to_longitude(shared_current_map_num, waypoints[index].lon);
      waypoints[index].lat = y_to_latitude(shared_current_map_num, waypoints[index].lat);
      dprintf("%ld, %ld, %d, %d", waypoints[index].lon, waypoints[index].lat, screen_map_x, screen_map_y);
    }
  }

  bool is_cursor_on_route(LonLat32 wp0, LonLat32 wp1, uint16_t cursor_screen_x, uint16_t cursor_screen_y) {
    /*
    the points (a,c) is wp0 and (b,d) is wp1. There are 3 special cases.
    Case 1:
      wp0.lon == wp1.lon (infinite slope);
      Then cursor is on the route iff c =< y =< d if c <= d or vice-versa
    Case 2:
      wp0.lat == wp1.lat (0 slope);
      Then cursor is on the route iff a =< x =< b if a <= b or vice-versa
    Case 3:
      Let m be the slope, then m = (d-c)/(b-a) and y(x) = m(x-a) + c.
      Let cursor_screen_x = x. Then if y(x) == cursor_screen_y, then the cursor
      must be on the line.
    */
    int32_t a = wp0.lon, b = wp1.lon, c = wp0.lat, d = wp1.lat;
    int32_t x = cursor_screen_x;
    // case 1
    if ((a == b) && ((c <= cursor_screen_y && cursor_screen_y <= d) || (d <= cursor_screen_y && cursor_screen_y <= c)))
      return true;
    // case 2
    if ((c == d) && ((a <= cursor_screen_y && cursor_screen_y <= b) || (b <= cursor_screen_y && cursor_screen_y <= a)))
      return true;
    // case 3
    int32_t y = (x-a)*(d-c)/(b-a) + c;
    if (cursor_screen_y == y)
      return true;
    return false;
  }

  bool is_route_on_screen(LonLat32 wp0, LonLat32 wp1) {
    /*  Checks to see whether or not the edge consisting of the verticies wp0
        and wp1 can be displayed.

        Args:
          wp0:  The waypoint that is adjacent to wp1. The value of this waypoint
                must be in "map coordinate" form.
          wp1:  The waypoint that is adjacent to wp2. The value of this waypoint
                must be in "map coordinate" form.

        Returns:
          (bool) True:  if the edge can be displayed on the LCD screen.
          (bool) False: if the edge cannot be displayed on the LCD screen.

        Others:
          The point (a,c) is wp0 and (b,d) is wp1.
    */
    int32_t a = wp0.lon, b = wp1.lon, c = wp0.lat, d = wp1.lat;
    int32_t boundary_x0 = screen_map_x, boundary_x1 = screen_map_x + 127;
    int32_t boundary_y0 = screen_map_y, boundary_y1 = screen_map_y + 159;

    if ((boundary_x0 < a && a < boundary_x1) && (boundary_x0 < b && b < boundary_x1)) {
      return true;
    }
    else if ((boundary_y0 < c && c < boundary_x1) && (boundary_x0 < d && d < boundary_x1)) {
      return true;
    }
    else {
      return false;
    }
  }

  int32_t wp_to_screen_x(int32_t wp_x) {
    /*  Converts the longitude (x value) from the map coordinates to the
        coordinate position on the screen.

        Args:
          int32_t wp_x: The map coordinate value in the x direction.

        Returns: X coordinate of the waypoint ON THE SCREEN
    */
    int32_t r = wp_x - screen_map_x;
    return constrain(r, 0, 128);
  }

  int32_t wp_to_screen_y(int32_t wp_y) {
    /*  Converts the latitude (y value) from the map coordinates to the
        coordinate position on the screen.

        Args:
          int32_t wp_y: The map coordinate value in the y direction.

        Returns: Y coordinate of the waypoint ON THE SCREEN
    */    int32_t r = wp_y - screen_map_y;
    return constrain(r, 0, 160);
  }

  void print_to_lcd(LonLat32* wp) {
    /* Prints the path onto the lcd screen

        Args:
          wp: A pointer to an array holding the waypoint values. These
              values must be in "map coordinate" form.

        Retuns: (Nothing)
    */
    for (int i = 0; i < cur_path_len-1; i++) {
      if (is_route_on_screen(wp[i], wp[i+1])) {
        //x0 = wp_to_screen_x(wp[i].lon), x1 = wp_to_screen_x(wp[i+1].lon)
        //y0 = wp_to_screen_y(wp[i].lat), y1 = wp_to_screen_y(wp[i+1].lat)
         tft.drawLine(
           wp_to_screen_x(wp[i].lon), wp_to_screen_y(wp[i].lat),
           wp_to_screen_x(wp[i+1].lon), wp_to_screen_y(wp[i+1].lat),
           0x0000); // color = black
      }
    }
    this->prev_exist = true;
  }

  void print_only_edge(LonLat32* wp) {
    //yet to be tested, unnessisary function
    uint16_t cursor_screen_x, cursor_screen_y;
    get_cursor_screen_x_y(&cursor_screen_x, &cursor_screen_y);
    for (int i = 0; i < cur_path_len-1; i++) {
      if (is_cursor_on_route(wp[i], wp[i+1], cursor_screen_x, cursor_screen_y)) {
        tft.drawLine(
          wp_to_screen_x(wp[i].lon), wp_to_screen_y(wp[i].lat),
          wp_to_screen_x(wp[i+1].lon), wp_to_screen_y(wp[i+1].lat),
          0x0000); // color = black
      }
    }
  }

} path;

void setup() {
    Serial.begin(9600);
    Serial.println("Starting...");
    Serial.flush();    // There can be nasty leftover bits.

    initialize_screen();

    initialize_sd_card();

    initialize_joystick();

    initialize_map();

    // Want to start viewing window in the center of the map
    move_window(
        (map_box[current_map_num].W + map_box[current_map_num].E) / 2,
        (map_box[current_map_num].N + map_box[current_map_num].S) / 2
    );

    // with cursor in the middle of the window
    move_cursor_to( screen_map_x + display_window_width / 2
                  , screen_map_y + display_window_height / 2
    );

    // Draw the initial screen and cursor
    first_time = 1;

    // Now initialize and enable the zoom buttons.
    pinMode(zoom_out_pin, INPUT);    // Zoom out.
    digitalWrite(zoom_out_pin, HIGH);

    pinMode(zoom_in_pin, INPUT);    // Zoom in.
    digitalWrite(zoom_in_pin, HIGH);

    // Initialize interrupt routines attached to zoom buttons.
    attachInterrupt(zoom_in_interrupt, handle_zoom_in, FALLING);
    attachInterrupt(zoom_out_interrupt, handle_zoom_out, FALLING);

    // wait a bit for display to settle down.
    delay(1000);

    // allocate the waypoints buffer
    max_path_len = 300;

    /* guard against too many waypoints taking up all available memory */
    if ( max_path_len * sizeof(LonLat32) > AVAIL_MEM - memory_low_mark ) {
        max_path_len = (AVAIL_MEM - memory_low_mark) / sizeof(LonLat32);
        }

    waypoints = (LonLat32*) malloc(max_path_len * sizeof(LonLat32));
    dprintf("Max %d waypoints", max_path_len);

    // no current path
    cur_path_len = 0;

    #ifdef DEBUG_MEMORY
        dprintf("Free mem:%d", AVAIL_MEM);
    #endif

}

const uint16_t screen_scroll_delta = 32;
const uint16_t screen_left_margin = 10;
const uint16_t screen_right_margin = 117;
const uint16_t screen_top_margin = 10;
const uint16_t screen_bottom_margin = 117;

// the path request, start and stop lat and lon
typedef enum {
    RS_WAIT_FOR_START = 0,
    RS_WAIT_FOR_STOP,
    RS_WAIT_LAST
} RequestState;

RequestState request_state = RS_WAIT_FOR_START;
LonLat32 start = LonLat32(0,0);
LonLat32 end = LonLat32(0,0);

void loop() {

    #ifdef DEBUG_MEMORY
        // if memory is getting low, generate a warning
        if ( AVAIL_MEM < memory_low_mark ) {
            dprintf("Free mem %d < %d", AVAIL_MEM, memory_low_mark);
            }
    #endif

    // Make sure we don't update the map tile on screen when we don't need to!
    uint8_t update_display_window = 0;

    if (first_time) {
        first_time = 0;
        update_display_window = 1;
    }

    // Joystick displacement.
    int16_t dx = 0;
    int16_t dy = 0;
    uint8_t select_button_event = 0;

    // Path drawing
    LonLat32 p_prev;
    int32_t x_prev;
    int32_t y_prev;

    LonLat32 p_cur;
    int32_t x_cur;
    int32_t y_cur;

    /*
        Zooming is an expensive operation.

        If the map is zoomed in or out we need to do a redraw, and will
        center the display window about the cursor. Thus a zoom in-out
        will re-center over a mis-positioned cursor!

        We also want to redraw the path, if any that is on the screen.

        We detect requests for zooming through the interrupt routines,
        but just mark the zoom as pending, and the do the zooming at this
        point.

        zoom_pending means we need to zoom the map, and no further
        zooms will be pending until we updated the shared state.
    */

    /*
        Critical section to determined if zoom is pending on the basis
        of the shared variables.   Critical sections start by turning
        off interrupts, and then enabling them after the critical operation.
        Critical sections do not nest.
    */
    uint8_t zoom_pending;
    noInterrupts();
    zoom_pending = shared_new_map_num != shared_current_map_num;
    interrupts();

    if ( zoom_pending ) {
        #ifdef DEBUG_SCROLLING
            dprintf("Zoom from map %d, x %d y %d",
                current_map_num, cursor_map_x, cursor_map_y);
        #endif

        /*
            It is safe to access shared_new_map_num outside of critical
            section, since it is one byte, and access is atomic.  Also,
            the interrupt routines will not modify the shared variables
            until they are equal again.
        */
        if ( current_map_num != shared_new_map_num ) {
            dprintf("Zoom from map %d to %d",
                current_map_num, shared_new_map_num);
            }
        uint8_t zoomed_map_num = set_zoom();

        // center the display window around the cursor
        move_window_to( cursor_map_x - display_window_width/2
                      , cursor_map_y - display_window_height/2
            );

        #ifdef DEBUG_SCROLLING
            dprintf("Zoom to map %d, x %d y %d",
                current_map_num, cursor_map_x, cursor_map_y);
        #endif

        // Changed the zoom level, so we want to redraw the window
        update_display_window = 1;

        }


    // Now, see if the joystick has moved, in which case we want to
    // also want to move the visible cursor on the screen.

    // Process joystick input.
    select_button_event = process_joystick(&dx, &dy);

    // the joystick routine filters out small changes, so anything non-0
    // is a real movement
    if (abs(dx) > 0 || abs(dy) > 0) {
        // Is the cursor getting near the edge of the screen?  If so
        // then scroll the map over by re-centering the window.

        uint16_t new_screen_map_x = screen_map_x;
        uint16_t new_screen_map_y = screen_map_y;
        uint8_t need_to_move = 0;

        uint16_t cursor_screen_x;
        uint16_t cursor_screen_y;
        if (get_cursor_screen_x_y(&cursor_screen_x, &cursor_screen_y)) {
            // if the cursor is visible, then adjust the display to
            // to scroll if near the edge.

            if (cursor_screen_x < screen_left_margin) {
                new_screen_map_x = screen_map_x - screen_scroll_delta;
                need_to_move = 1;
            }
            else if (cursor_screen_x > screen_right_margin) {
                new_screen_map_x = screen_map_x + screen_scroll_delta;
                need_to_move = 1;
            }

            if (cursor_screen_y < screen_top_margin) {
                new_screen_map_y = screen_map_y - screen_scroll_delta;
                need_to_move = 1;
            }
            else if (cursor_screen_y > screen_bottom_margin) {
                new_screen_map_y = screen_map_y + screen_scroll_delta;
                need_to_move = 1;
            }

            if (need_to_move) {
                // move the display window, leaving cursor at same lat-lon
                move_window_to(new_screen_map_x, new_screen_map_y);
                update_display_window = 1;
            }
            else {
                // erase old cursor, move, and draw new one, no need to
                // redraw the underlying map tile
                erase_cursor();
                move_cursor_by(dx, dy);
                draw_cursor();
            }
        }

    }

    /*
        At this point the screen is updated, with a new tile window and
        cursor position if necessary.
    */

    // process the select button being pressed on the joystick.
    if (select_button_event) {
        /*
        Button was pressed, we are selecting a point!
        which press is this, the start or the stop selection?

        If we are making a request to find a shortest path, we will send out
        the request on the serial port and then wait for a response from the
        server.  While this is happening, the client user interface is
        suspended.

        If the stop point, then we send out the server request and wait.

        */

        // NOTE: swapped lon lat for the server!
        dprintf("Button press @ lat %ld lon %ld", cursor_lat, cursor_lon);


        // create a LonLat32 structure initialized to current cursor
        // position in lon lat.

        LonLat32 p(cursor_lon, cursor_lat);

        if (request_state==RS_WAIT_FOR_START) {
            dprintf("Start point lon %ld lat %ld", p.lon, p.lat);
            start = p;
            request_state = RS_WAIT_FOR_STOP;
            }
        else { // request_state==RS_WAIT_FOR_STOP
            dprintf("End point lon %ld lat %ld", p.lon, p.lat);
            end = p;
            request_state = RS_WAIT_FOR_START;

            // start the server communication with a path request
            dprintf("Requesting lat %ld lon %ld to lat %ld lon %ld",
                start.lat, start.lon, end.lat, end.lon);

            Serial.print("R "); //Printing request.
            Serial.print(start.lat);
            Serial.print(" ");
            Serial.print(start.lon);
            Serial.print(" ");
            Serial.print(end.lat);
            Serial.print(" ");
            Serial.println(end.lon);

            int16_t path_len = srv_get_pathlen(start, end); //Get's path length.
            dprintf("Path Length: %d", path_len);

            if ( path_len > 0 ) { // If no error occurs with path finding.
                update_display_window = 1;

                // Ensures the #waypoints is < #max_waypoints -->(300)
                if ( path_len > max_path_len ) {
                    cur_path_len = max_path_len;
                } else {
                    cur_path_len = path_len;
                }

                // If an error does not occur with getting waypoints
                if (srv_get_waypoints(waypoints, path_len, max_path_len) >= 0) {
                    //Check if values are correctly assigned.
                    dprintf("Waypoints (lat, lon):");
                    // for (int16_t i=0; i < cur_path_len; i++) {
                    //     dprintf("%d: %ld %ld",
                    //         i, waypoints[i].lat, waypoints[i].lon);
                    // }
                    path.convert_wp_to_map_coord(waypoints);
                } else {
                  flush_incSerial();
                }

              } else {
                flush_incSerial();
              }

            }  // end request path
            flush_incSerial();
            Serial.flush();
        } // end of select_button_event processing

    // do we have to redraw the map tile?
    if (update_display_window) {
        #ifdef DEBUG_SCROLLING
            dprintf("Screen update map %d lon %ld lat %ld",
                current_map_num, cursor_lon, cursor_lat);
        #endif

        draw_map_screen();
        draw_cursor();

        // find the waypoints on the current tile and draw as lines
        if ( cur_path_len > 1 ) {
            path.print_to_lcd(waypoints);
        }

        // force a redisplay of status message
        clear_status_msg();

        if ( zoom_pending ) {
            /*
              Finally, after this round of display processing, enable
              zooming occur again. NOTE: critical section, update the shared
              map num variable to indicate zoom processed.
            */

            path.convert_wp_to_graph_coord(waypoints);
            noInterrupts();
            shared_new_map_num = current_map_num;
            shared_current_map_num = current_map_num;
            interrupts();
            dprintf("Zoom re-enabled");
            if (path.prev_exist) {
              path.convert_wp_to_map_coord(waypoints);
              path.print_to_lcd(waypoints);
            }

        }
    }

    // always update the status message area if message changes
    // Indicate which point we are waiting for
    if (request_state == RS_WAIT_FOR_START) {
        status_msg("FROM?");
    }
    else {
        status_msg("TO?");
    }
}

char* prev_status_msg = 0;

// Flushes the incoming stream of the serial port
void flush_incSerial() {
  while(Serial.available()) {
    Serial.read();
  }
}

void clear_status_msg() {
    status_msg("");
}

void status_msg(char *msg) {
    // messages are strings, so we assume constant, and if they are the
    // same pointer then the contents are the same.  You can force by
    // setting prev_status_msg = 0

    if (prev_status_msg != msg) {
        prev_status_msg = msg;
        tft.fillRect(0, 148, 128, 12, GREEN);

        tft.setTextSize(1);
        tft.setTextColor(MAGENTA);
        tft.setCursor(0, 150);
        tft.setTextSize(1);

        tft.println(msg);
    }
}


void initialize_screen() {

    tft.initR(INITR_REDTAB);

    tft.setRotation(0);

    tft.setCursor(0, 0);
    tft.setTextColor(0x0000);
    tft.setTextSize(1);
    tft.fillScreen(BLUE);
}

void initialize_sd_card() {
    if (!SD.begin(sd_cs)) {
        #ifdef DEBUG_SD_CARD
            dprintf("SD card init has failed");
            dprintf("Is a card inserted?");
            dprintf("Is your wiring correct?");
            dprintf("Is the chipSelect pin the one for your shield or module?");
        #endif

        // Just wait, stuff exploded.
        while (1) {}
        }

    #ifdef DEBUG_SD_CARD
        dprintf("SD card ready.");
    #endif
    }


// Center point of the joystick - analog reads from the Arduino.
int16_t joy_center_x = 512;
int16_t joy_center_y = 512;

void initialize_joystick() {
    // Initialize the button pin, turn on pullup resistor
    pinMode(joy_pin_button, INPUT);
    digitalWrite(joy_pin_button, HIGH);

    // Center Joystick
    joy_center_x = analogRead(joy_pin_x);
    joy_center_y = analogRead(joy_pin_y);
}


// button state: 0 not pressed, 1 pressed
uint8_t prev_button_state = 0;

// time of last sampling of button state
uint32_t button_prev_time = millis();

// only after this much time has passed is the state sampled.
uint32_t button_sample_delay = 200;

/*
    Read the joystick position, and return the x, y displacement from
    the zero position. The joystick has to be at least 4 units away from
    zero before a non-zero displacement is returned. This filters out
    the centering errors that occur when the joystick is released.

    Also, return 1 if the joystick button has been pushed, held for a
    minimum amount of time, and then released. That is, a 1 is returned
    if a button select action has occurred.

*/
uint8_t process_joystick(int16_t *dx, int16_t *dy) {
    int16_t joy_x;
    int16_t joy_y;
    uint8_t button_state;

    joy_x = (analogRead(joy_pin_y) - joy_center_x);
    joy_y = (analogRead(joy_pin_x) - joy_center_y);

    if (abs(joy_x) <= 4) {
        joy_x = 0;
    }

    if (abs(joy_y) <= 4) {
        joy_y = 0;
    }

    *dx = joy_x / 128;
    *dy = joy_y / 128;

    uint8_t button_press_event = 0; // no event by default
    // check for suitable time delay since the last time we read the joystick

    uint32_t cur_time = millis();

    // time inversion caused by wraparound?
    if (cur_time < button_prev_time) {
        // this is a hack.. but first this happens once in a million years
        // and the user won't notice either, but this could be made nicer

        button_prev_time = cur_time;
    }
    if (cur_time > button_prev_time + button_sample_delay) {
        button_prev_time = cur_time;

        // true if button is pressed
        button_state = (LOW == digitalRead(joy_pin_button));

        // if a press is followed by a release, we will have an event:
        button_press_event = (prev_button_state && !button_state);
        prev_button_state = button_state;
    }
    return button_press_event;
}


/*
    Zooming in and out button handlers

    You should never be doing complex operations inside an
    interrupt handler.

    There are two shared variables, which must only be updated in
    critical sections precisely located in the code flow.

    When a zoom in or out button is pressed, if
        shared_new_map_num == shared_current_map_num, then
    it is safe to change shared_new_map_num to reflect the new zoom
    level. No further changes can occur until the current_map_num is
    finally set to the shared one.

    All that the interrup handlers do is to set shared_new_map_num
    on interrupt, and then handling of the zooming is done in the
    main loop.
*/


void handle_zoom_in() {
    // critical section begins, safe to manipulate shared variables

    if ( shared_current_map_num == shared_new_map_num ) {
        // only affects shared_new_map_num
        zoom_in();
        }
    }

void handle_zoom_out() {
    // critical section begins, safe to manipulate shared variables

    if ( shared_current_map_num == shared_new_map_num ) {
        // only affects shared_new_map_num
        zoom_out();
        }
    }
