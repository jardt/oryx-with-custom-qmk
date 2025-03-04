#include QMK_KEYBOARD_H
#include "version.h"
#include "i18n.h"
#define MOON_LED_LEVEL LED_LEVEL
#define ML_SAFE_RANGE SAFE_RANGE

enum custom_keycodes {
  RGB_SLD = ML_SAFE_RANGE,
  HSV_0_255_255,
  HSV_74_255_255,
  HSV_169_255_255,
  ST_MACRO_0,
  ST_MACRO_1,
};



enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
  DANCE_4,
  DANCE_5,
  DANCE_6,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    LT(4,KC_ESCAPE),KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_Q,           KC_7,           KC_8,           KC_9,           KC_0,           TD(DANCE_1),    
    LT(3,KC_TAB),   KC_Q,           KC_Y,           KC_O,           KC_U,           KC_SCLN,                                        KC_X,           KC_L,           KC_D,           KC_W,           KC_Z,           KC_BSPC,        
    KC_B,           MT(MOD_LALT, KC_C),MT(MOD_LGUI, KC_I),MT(MOD_LCTL, KC_E),MT(MOD_LSFT, KC_A),KC_COMMA,                                       KC_K,           MT(MOD_RSFT, KC_H),MT(MOD_RCTL, KC_T),MT(MOD_RGUI, KC_N),MT(MOD_RALT, KC_S),KC_V,           
    KC_LEFT_SHIFT,  KC_QUOTE,       KC_MINUS,       KC_EQUAL,       KC_DOT,         KC_SLASH,                                       KC_J,           KC_M,           KC_G,           KC_P,           KC_F,           KC_RIGHT_SHIFT, 
                                                    LT(2,KC_SPACE), TD(DANCE_0),                                    KC_MEH,         LT(1,KC_R)
  ),
  [1] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_LCBR,        KC_LPRN,        KC_RPRN,        KC_RCBR,        KC_QUES,                                        KC_EXLM,        KC_TILD,        KC_LBRC,        KC_RBRC,        ST_MACRO_0,     KC_TRANSPARENT, 
    KC_HASH,        KC_CIRC,        KC_EQUAL,       KC_UNDS,        KC_DLR,         KC_ASTR,                                        KC_AT,          KC_PERC,        KC_PLUS,        KC_AMPR,        KC_COLN,        KC_SCLN,        
    KC_TRANSPARENT, KC_LABK,        KC_PIPE,        KC_MINUS,       KC_RABK,        KC_SLASH,                                       KC_BSLS,        KC_DQUO,        KC_QUOTE,       KC_GRAVE,       ST_MACRO_1,     KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_DOT,                                         KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [2] = LAYOUT_voyager(
    LALT(LGUI(KC_ESCAPE)),KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, LGUI(KC_Q),     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_G,           KC_7,           KC_8,           KC_9,           KC_MINUS,       KC_SLASH,       
    KC_TRANSPARENT, LGUI(KC_A),     KC_TRANSPARENT, KC_TRANSPARENT, LGUI(KC_F),     KC_TRANSPARENT,                                 KC_K,           KC_4,           KC_5,           KC_6,           KC_PLUS,        KC_ASTR,        
    KC_TRANSPARENT, KC_MAC_UNDO,    KC_MAC_CUT,     KC_MAC_COPY,    KC_MAC_PASTE,   KC_TRANSPARENT,                                 KC_J,           KC_1,           KC_2,           KC_3,           KC_DOT,         KC_EQUAL,       
                                                    KC_TRANSPARENT, TO(5),                                          KC_TRANSPARENT, KC_0
  ),
  [3] = LAYOUT_voyager(
    KC_F1,          KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,                                          KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         
    KC_TRANSPARENT, TOGGLE_LAYER_COLOR,RGB_MODE_FORWARD,RGB_SLD,        RGB_VAD,        RGB_VAI,                                        KC_PAGE_UP,     KC_PAGE_UP,     KC_TRANSPARENT, KC_END,         KC_TRANSPARENT, KC_F12,         
    KC_TRANSPARENT, KC_MEDIA_PREV_TRACK,KC_MEDIA_NEXT_TRACK,KC_PGDN,        KC_MEDIA_PLAY_PAUSE,TD(DANCE_2),                                    KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, HSV_0_255_255,  HSV_74_255_255, HSV_169_255_255,                                KC_TRANSPARENT, KC_AUDIO_MUTE,  KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_BRIGHTNESS_DOWN,KC_BRIGHTNESS_UP,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [4] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_WH_UP,    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_UP,       KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_ACCEL0,   KC_MS_ACCEL1,   KC_MS_ACCEL2,   KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_RIGHT,    KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_WH_DOWN,  KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_MS_BTN1,     KC_TRANSPARENT,                                 TO(5),          KC_MS_BTN2
  ),
  [5] = LAYOUT_voyager(
    TD(DANCE_3),    KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           TD(DANCE_5),    TD(DANCE_6),    TO(4),          
    KC_TAB,         KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,                                           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           RALT(KC_BSPC),  
    KC_LEFT_CTRL,   KC_A,           KC_S,           KC_D,           KC_F,           KC_G,                                           KC_H,           KC_J,           KC_K,           KC_L,           KC_SCLN,        KC_QUOTE,       
    OSM(MOD_LSFT),  KC_Z,           KC_X,           MT(MOD_LALT, KC_C),MT(MOD_LGUI, KC_V),KC_B,                                           KC_N,           MT(MOD_RGUI, KC_M),MT(MOD_RALT, KC_COMMA),KC_DOT,         KC_SLASH,       OSM(MOD_HYPR),  
                                                    KC_SPACE,       TD(DANCE_4),                                    MEH_T(KC_ESCAPE),KC_TRANSPARENT
  ),
};

const uint16_t PROGMEM combo0[] = { KC_TILD, KC_LBRC, COMBO_END};
const uint16_t PROGMEM combo1[] = { KC_7, KC_8, COMBO_END};
const uint16_t PROGMEM combo2[] = { KC_L, KC_D, COMBO_END};
const uint16_t PROGMEM combo3[] = { KC_L, KC_D, KC_W, COMBO_END};
const uint16_t PROGMEM combo4[] = { KC_D, KC_W, COMBO_END};
const uint16_t PROGMEM combo5[] = { KC_U, KC_O, COMBO_END};
const uint16_t PROGMEM combo6[] = { MT(MOD_RCTL, KC_T), MT(MOD_RSFT, KC_H), COMBO_END};
const uint16_t PROGMEM combo7[] = { MT(MOD_LSFT, KC_A), MT(MOD_LCTL, KC_E), COMBO_END};

combo_t key_combos[COMBO_COUNT] = {
    COMBO(combo0, KC_BSPC),
    COMBO(combo1, KC_BSPC),
    COMBO(combo2, KC_BSPC),
    COMBO(combo3, RCTL(KC_W)),
    COMBO(combo4, KC_DELETE),
    COMBO(combo5, LCTL(KC_Y)),
    COMBO(combo6, KC_ENTER),
    COMBO(combo7, KC_ESCAPE),
};

uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case LT(2,KC_SPACE):
            return TAPPING_TERM -55;
        case TD(DANCE_0):
            return TAPPING_TERM -55;
        case LT(1,KC_R):
            return TAPPING_TERM -55;
        case MT(MOD_LGUI, KC_V):
            return TAPPING_TERM + 20;
        case TD(DANCE_4):
            return TAPPING_TERM -55;
        case MT(MOD_RALT, KC_COMMA):
            return TAPPING_TERM + 20;
        default:
            return TAPPING_TERM;
    }
}

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249} },

    [1] = { {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243} },

    [2] = { {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227}, {82,74,227} },

    [3] = { {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250}, {155,115,250} },

    [5] = { {155,115,250}, {165,49,134}, {165,49,134}, {165,49,134}, {165,49,134}, {165,49,134}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {226,53,245}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {246,109,243}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {246,109,243}, {82,74,227}, {165,49,134}, {165,49,134}, {165,49,134}, {165,49,134}, {165,49,134}, {155,115,250}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {246,109,243}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {29,75,249}, {246,109,243}, {82,74,227}, {29,75,249} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (keyboard_config.disable_layer_led) { return false; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 1:
      set_layer_color(1);
      break;
    case 2:
      set_layer_color(2);
      break;
    case 3:
      set_layer_color(3);
      break;
    case 5:
      set_layer_color(5);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_LSFT(SS_TAP(X_SCLN)) SS_DELAY(100) SS_LSFT(SS_TAP(X_SCLN)));
    }
    break;
    case ST_MACRO_1:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_DOT) SS_DELAY(100) SS_TAP(X_DOT) SS_DELAY(100) SS_TAP(X_SLASH));
    }
    break;

    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
    case HSV_0_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(0,255,255);
      }
      return false;
    case HSV_74_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(74,255,255);
      }
      return false;
    case HSV_169_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(169,255,255);
      }
      return false;
  }
  return true;
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[7];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LCTL(KC_F));
        tap_code16(LCTL(KC_F));
        tap_code16(LCTL(KC_F));
    }
    if(state->count > 3) {
        tap_code16(LCTL(KC_F));
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(LCTL(KC_F)); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_CTRL); break;
        case DOUBLE_TAP: register_code16(LCTL(KC_F)); register_code16(LCTL(KC_F)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LCTL(KC_F)); register_code16(LCTL(KC_F));
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(LCTL(KC_F)); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_CTRL); break;
        case DOUBLE_TAP: unregister_code16(LCTL(KC_F)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LCTL(KC_F)); break;
    }
    dance_state[0].step = 0;
}
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case DOUBLE_TAP: layer_move(5); break;
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
    }
    dance_state[1].step = 0;
}
void on_dance_2(tap_dance_state_t *state, void *user_data);
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void on_dance_2(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_HOME);
        tap_code16(KC_HOME);
        tap_code16(KC_HOME);
    }
    if(state->count > 3) {
        tap_code16(KC_HOME);
    }
}

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(KC_HOME); break;
        case SINGLE_HOLD: register_code16(KC_END); break;
        case DOUBLE_TAP: register_code16(KC_HOME); register_code16(KC_HOME); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_HOME); register_code16(KC_HOME);
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(KC_HOME); break;
        case SINGLE_HOLD: unregister_code16(KC_END); break;
        case DOUBLE_TAP: unregister_code16(KC_HOME); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_HOME); break;
    }
    dance_state[2].step = 0;
}
void on_dance_3(tap_dance_state_t *state, void *user_data);
void dance_3_finished(tap_dance_state_t *state, void *user_data);
void dance_3_reset(tap_dance_state_t *state, void *user_data);

void on_dance_3(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_ESCAPE);
        tap_code16(KC_ESCAPE);
        tap_code16(KC_ESCAPE);
    }
    if(state->count > 3) {
        tap_code16(KC_ESCAPE);
    }
}

void dance_3_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[3].step = dance_step(state);
    switch (dance_state[3].step) {
        case SINGLE_TAP: register_code16(KC_ESCAPE); break;
        case SINGLE_HOLD: register_code16(KC_BSLS); break;
        case DOUBLE_TAP: register_code16(KC_ESCAPE); register_code16(KC_ESCAPE); break;
        case DOUBLE_HOLD: register_code16(KC_SYSTEM_SLEEP); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_ESCAPE); register_code16(KC_ESCAPE);
    }
}

void dance_3_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[3].step) {
        case SINGLE_TAP: unregister_code16(KC_ESCAPE); break;
        case SINGLE_HOLD: unregister_code16(KC_BSLS); break;
        case DOUBLE_TAP: unregister_code16(KC_ESCAPE); break;
        case DOUBLE_HOLD: unregister_code16(KC_SYSTEM_SLEEP); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_ESCAPE); break;
    }
    dance_state[3].step = 0;
}
void on_dance_4(tap_dance_state_t *state, void *user_data);
void dance_4_finished(tap_dance_state_t *state, void *user_data);
void dance_4_reset(tap_dance_state_t *state, void *user_data);

void on_dance_4(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LCTL(KC_F));
        tap_code16(LCTL(KC_F));
        tap_code16(LCTL(KC_F));
    }
    if(state->count > 3) {
        tap_code16(LCTL(KC_F));
    }
}

void dance_4_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[4].step = dance_step(state);
    switch (dance_state[4].step) {
        case SINGLE_TAP: register_code16(LCTL(KC_F)); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_SHIFT); break;
        case DOUBLE_TAP: register_code16(LCTL(KC_F)); register_code16(LCTL(KC_F)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LCTL(KC_F)); register_code16(LCTL(KC_F));
    }
}

void dance_4_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[4].step) {
        case SINGLE_TAP: unregister_code16(LCTL(KC_F)); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_SHIFT); break;
        case DOUBLE_TAP: unregister_code16(LCTL(KC_F)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LCTL(KC_F)); break;
    }
    dance_state[4].step = 0;
}
void on_dance_5(tap_dance_state_t *state, void *user_data);
void dance_5_finished(tap_dance_state_t *state, void *user_data);
void dance_5_reset(tap_dance_state_t *state, void *user_data);

void on_dance_5(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_9);
        tap_code16(KC_9);
        tap_code16(KC_9);
    }
    if(state->count > 3) {
        tap_code16(KC_9);
    }
}

void dance_5_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[5].step = dance_step(state);
    switch (dance_state[5].step) {
        case SINGLE_TAP: register_code16(KC_9); break;
        case SINGLE_HOLD: register_code16(KC_MINUS); break;
        case DOUBLE_TAP: register_code16(KC_9); register_code16(KC_9); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_9); register_code16(KC_9);
    }
}

void dance_5_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[5].step) {
        case SINGLE_TAP: unregister_code16(KC_9); break;
        case SINGLE_HOLD: unregister_code16(KC_MINUS); break;
        case DOUBLE_TAP: unregister_code16(KC_9); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_9); break;
    }
    dance_state[5].step = 0;
}
void on_dance_6(tap_dance_state_t *state, void *user_data);
void dance_6_finished(tap_dance_state_t *state, void *user_data);
void dance_6_reset(tap_dance_state_t *state, void *user_data);

void on_dance_6(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_0);
        tap_code16(KC_0);
        tap_code16(KC_0);
    }
    if(state->count > 3) {
        tap_code16(KC_0);
    }
}

void dance_6_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[6].step = dance_step(state);
    switch (dance_state[6].step) {
        case SINGLE_TAP: register_code16(KC_0); break;
        case SINGLE_HOLD: register_code16(KC_PLUS); break;
        case DOUBLE_TAP: register_code16(KC_0); register_code16(KC_0); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_0); register_code16(KC_0);
    }
}

void dance_6_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[6].step) {
        case SINGLE_TAP: unregister_code16(KC_0); break;
        case SINGLE_HOLD: unregister_code16(KC_PLUS); break;
        case DOUBLE_TAP: unregister_code16(KC_0); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_0); break;
    }
    dance_state[6].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
        [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_3, dance_3_finished, dance_3_reset),
        [DANCE_4] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_4, dance_4_finished, dance_4_reset),
        [DANCE_5] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_5, dance_5_finished, dance_5_reset),
        [DANCE_6] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_6, dance_6_finished, dance_6_reset),
};
